--------------------------------------------------------------------------------
-- spi_master.vhd
--
-- ADMV48281 icin SPI master cekirdegi (CPOL=0, CPHA=0, MSB once).
-- Veriyi SCLK'in dusen kenarinda surer, cip yukselen kenarda ornekler.
--
-- Dort komut modu:
--   C_MODE_WRITE32 : 32 bitlik standart ADI frame'i (cmd_hdr)
--   C_MODE_STREAM  : 24 bit header + cmd_len byte veri (str_* arayuzunden)
--   C_MODE_READ    : 24 bit header + cmd_len byte bos clock (MOSI = 0).
--                    Donen veri spi_slave modulu tarafindan yakalanir;
--                    rd_arm cikisi okuma penceresini isaretler.
--   C_MODE_BURST   : CS pasif iken cmd_len adet SCLK darbesi
--                    (NVM merge icin gereken >=10354 clock).
--
-- Streaming veri kaynagi: str_idx cikisi bir sonraki byte'in indeksini gosterir,
-- str_data girisi o indeksin verisini tasir. Indeks bir byte suresi (8 SCLK)
-- once yayinlandigi icin kayitli (registered) BRAM cikisi dogrudan baglanabilir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.admv48281_pkg.all;

entity spi_master is
  generic (
    -- f_sclk = f_clk / (2 * G_CLK_DIV_x). ADMV48281 maksimum 133 MHz.
    G_CLK_DIV_WR       : natural := 2;   -- yazma hizi
    G_CLK_DIV_RD       : natural := 8;   -- okuma hizi (ring gecikmesi icin dusuk)
    G_CS_SETUP_CYCLES  : natural := 4;   -- CS dustukten sonra ilk SCLK'a kadar (tS)
    G_CS_HOLD_CYCLES   : natural := 4;   -- son SCLK'tan sonra CS yuksek olana kadar (tH)
    G_CS_GAP_CYCLES    : natural := 8;   -- iki frame arasi CS yuksek suresi
    G_RX_SETTLE_CYCLES : natural := 32   -- ring donus gecikmesi payi (okuma)
  );
  port (
    clk   : in std_logic;
    rst_n : in std_logic;

    -- komut arayuzu (tek islem, handshake)
    cmd_valid : in  std_logic;
    cmd_ready : out std_logic;
    cmd_mode  : in  std_logic_vector(1 downto 0);
    cmd_hdr   : in  std_logic_vector(31 downto 0);
    cmd_len   : in  std_logic_vector(15 downto 0);

    -- streaming veri kaynagi
    str_idx  : out std_logic_vector(8 downto 0);
    str_data : in  std_logic_vector(7 downto 0);

    -- SPI pinleri (kontrolcu tarafi)
    spi_sclk : out std_logic;
    spi_mosi : out std_logic;
    spi_cs_n : out std_logic;

    -- okuma penceresi: spi_slave bu sinyal '1' iken donen clock ile ornekler
    rd_arm : out std_logic;

    busy : out std_logic;
    done : out std_logic   -- islem bitisinde 1 clock darbe
  );
end entity spi_master;

architecture rtl of spi_master is

  type t_state is (ST_IDLE, ST_SETUP, ST_SHIFT, ST_HOLD, ST_SETTLE, ST_GAP);
  signal state : t_state := ST_IDLE;

  signal mode_r    : std_logic_vector(1 downto 0) := (others => '0');
  signal hdr_r     : std_logic_vector(31 downto 0) := (others => '0');
  signal bits_left : unsigned(15 downto 0) := (others => '0');

  signal div_max : unsigned(7 downto 0) := (others => '0');
  signal div_cnt : unsigned(7 downto 0) := (others => '0');

  signal sclk_r : std_logic := '0';
  signal cs_n_r : std_logic := '1';
  signal mosi_r : std_logic := '0';

  signal sh8         : std_logic_vector(7 downto 0) := (others => '0');
  signal bit_in_byte : unsigned(2 downto 0) := (others => '0');
  signal byte_idx    : unsigned(11 downto 0) := (others => '0');
  signal str_idx_r   : unsigned(8 downto 0) := (others => '0');

  signal wait_cnt : unsigned(7 downto 0) := (others => '0');
  signal rd_arm_r : std_logic := '0';
  signal done_r   : std_logic := '0';

  -- byte_idx'e gore siradaki byte'i sec
  function f_byte_sel(mode : std_logic_vector(1 downto 0);
                      hdr  : std_logic_vector(31 downto 0);
                      idx  : unsigned(11 downto 0);
                      sd   : std_logic_vector(7 downto 0))
                      return std_logic_vector is
  begin
    if mode = C_MODE_WRITE32 then
      case to_integer(idx) is
        when 0      => return hdr(31 downto 24);
        when 1      => return hdr(23 downto 16);
        when 2      => return hdr(15 downto 8);
        when others => return hdr(7 downto 0);
      end case;
    else
      case to_integer(idx) is
        when 0      => return hdr(23 downto 16);
        when 1      => return hdr(15 downto 8);
        when 2      => return hdr(7 downto 0);
        when others =>
          if mode = C_MODE_READ then
            return x"00";           -- okuma fazinda MOSI bos
          else
            return sd;              -- streaming veri
          end if;
      end case;
    end if;
  end function;

begin

  spi_sclk  <= sclk_r;
  spi_mosi  <= mosi_r;
  spi_cs_n  <= cs_n_r;
  rd_arm    <= rd_arm_r;
  str_idx   <= std_logic_vector(str_idx_r);
  cmd_ready <= '1' when state = ST_IDLE else '0';
  busy      <= '0' when state = ST_IDLE else '1';
  done      <= done_r;

  process (clk)
    variable nb : std_logic_vector(7 downto 0);
  begin
    if rising_edge(clk) then
      done_r <= '0';

      if rst_n = '0' then
        state       <= ST_IDLE;
        sclk_r      <= '0';
        cs_n_r      <= '1';
        mosi_r      <= '0';
        rd_arm_r    <= '0';
        div_cnt     <= (others => '0');
        byte_idx    <= (others => '0');
        str_idx_r   <= (others => '0');
        bit_in_byte <= (others => '0');
        bits_left   <= (others => '0');
        wait_cnt    <= (others => '0');

      else
        case state is

          --------------------------------------------------------------------
          when ST_IDLE =>
            sclk_r <= '0';
            cs_n_r <= '1';
            mosi_r <= '0';
            if cmd_valid = '1' then
              mode_r      <= cmd_mode;
              hdr_r       <= cmd_hdr;
              byte_idx    <= (others => '0');
              str_idx_r   <= (others => '0');
              bit_in_byte <= (others => '0');
              div_cnt     <= (others => '0');
              wait_cnt    <= (others => '0');

              if cmd_mode = C_MODE_READ then
                div_max  <= to_unsigned(G_CLK_DIV_RD, 8);
                rd_arm_r <= '1';
              else
                div_max  <= to_unsigned(G_CLK_DIV_WR, 8);
                rd_arm_r <= '0';
              end if;

              case cmd_mode is
                when C_MODE_WRITE32 =>
                  bits_left <= to_unsigned(32, 16);
                when C_MODE_STREAM | C_MODE_READ =>
                  bits_left <= to_unsigned(24, 16)
                               + shift_left(unsigned(cmd_len), 3);
                when others =>
                  bits_left <= unsigned(cmd_len);   -- ham clock sayisi
              end case;

              state <= ST_SETUP;
            end if;

          --------------------------------------------------------------------
          -- CS'i indir (burst haric), ilk byte'i yukle, tS kadar bekle
          when ST_SETUP =>
            if mode_r = C_MODE_BURST then
              cs_n_r <= '1';        -- NVM merge clock'lari CS pasif iken verilir
              mosi_r <= '0';
            else
              cs_n_r <= '0';
              nb     := f_byte_sel(mode_r, hdr_r, to_unsigned(0, 12), str_data);
              sh8    <= nb;
              mosi_r <= nb(7);
            end if;

            if wait_cnt = to_unsigned(G_CS_SETUP_CYCLES, 8) then
              wait_cnt <= (others => '0');
              state    <= ST_SHIFT;
            else
              wait_cnt <= wait_cnt + 1;
            end if;

          --------------------------------------------------------------------
          when ST_SHIFT =>
            if div_cnt = div_max - 1 then
              div_cnt <= (others => '0');

              if sclk_r = '0' then
                -- yukselen kenar: cip MOSI'yi ornekliyor
                sclk_r    <= '1';
                bits_left <= bits_left - 1;
              else
                -- dusen kenar: siradaki biti sur
                sclk_r <= '0';
                if bits_left = 0 then
                  wait_cnt <= (others => '0');
                  state    <= ST_HOLD;
                elsif mode_r /= C_MODE_BURST then
                  if bit_in_byte = 7 then
                    bit_in_byte <= (others => '0');
                    byte_idx    <= byte_idx + 1;
                    nb          := f_byte_sel(mode_r, hdr_r, byte_idx + 1, str_data);
                    sh8         <= nb;
                    mosi_r      <= nb(7);
                    -- streaming veri fazina girildiginde bir sonraki byte'i iste
                    if mode_r = C_MODE_STREAM and byte_idx >= 2 then
                      str_idx_r <= str_idx_r + 1;
                    end if;
                  else
                    bit_in_byte <= bit_in_byte + 1;
                    sh8         <= sh8(6 downto 0) & '0';
                    mosi_r      <= sh8(6);
                  end if;
                end if;
              end if;

            else
              div_cnt <= div_cnt + 1;
            end if;

          --------------------------------------------------------------------
          -- son SCLK ile CS'in yukselmesi arasindaki tutma suresi (tH)
          when ST_HOLD =>
            sclk_r <= '0';
            if wait_cnt = to_unsigned(G_CS_HOLD_CYCLES, 8) then
              wait_cnt <= (others => '0');
              if mode_r = C_MODE_READ then
                state <= ST_SETTLE;
              else
                cs_n_r <= '1';
                state  <= ST_GAP;
              end if;
            else
              wait_cnt <= wait_cnt + 1;
            end if;

          --------------------------------------------------------------------
          -- Ring uzerinden donen SCLK/SDO kontrolcuye gec ulasir; son bitler
          -- yakalanana kadar CS'i dusuk ve okuma penceresini acik tutuyoruz.
          when ST_SETTLE =>
            if wait_cnt = to_unsigned(G_RX_SETTLE_CYCLES, 8) then
              wait_cnt <= (others => '0');
              rd_arm_r <= '0';        -- dusen kenar: spi_slave veriyi gecerli yapar
              cs_n_r   <= '1';
              state    <= ST_GAP;
            else
              wait_cnt <= wait_cnt + 1;
            end if;

          --------------------------------------------------------------------
          when ST_GAP =>
            cs_n_r <= '1';
            if wait_cnt = to_unsigned(G_CS_GAP_CYCLES, 8) then
              wait_cnt <= (others => '0');
              done_r   <= '1';
              state    <= ST_IDLE;
            else
              wait_cnt <= wait_cnt + 1;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;

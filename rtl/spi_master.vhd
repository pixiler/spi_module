--------------------------------------------------------------------------------
-- spi_master.vhd
-- SPI master cekirdegi (CPOL=0, CPHA=0, MSB once) - ADMV48281 uyumlu.
--
-- Ozellikler:
--  * Yazma: MOSI, dahili uretilen sclk_out'a senkron surulur
--    (falling edge'de bit degisir, slave rising edge'de ornekler).
--  * Okuma: MISO, geri donen/looplanan sclk_in'in RISING EDGE'i tespit
--    edilerek orneklenir. sclk_in dogrudan clock olarak KULLANILMAZ; sistem
--    saati ile oversample edilip kenar tespiti yapilir. Boylece sclk_in'in
--    clock-capable (MRCC/SRCC) bir pine baglanmasi gerekmez, BUFG/clock
--    routing hatasi olusmaz. Kart/kablo gecikmesi yine telafi edilir.
--  * clk_div girisi ile her transferin SCLK hizi ayri secilebilir:
--        f_sclk = f_clk / (2 * clk_div)
--  * Bir transfer = tx_len bit header + stream_nbytes ek byte (streaming
--    mode) + rx_len bit okuma; hepsi ayni CS penceresinde.
--      - Standart yazma      : tx_len=32, stream_nbytes=0, rx_len=0
--      - Standart okuma      : tx_len=24, stream_nbytes=0, rx_len=8
--      - Streaming yazma     : tx_len=24 (header), stream_nbytes=N
--        Master her byte'i tuketince stream_taken pulse'i uretir; ust modul
--        bir sonraki byte'i stream_byte girisine koyar (8 SCLK bit suresi
--        kadar zamani vardir).
--  * start_burst ile CS aktif edilmeden N adet SCLK pulse'i uretilir
--    (ADMV48281 NVM merge islemi icin gereken >=10354 clock).
--
-- Notlar:
--  * tx_data sag hizalidir: ornegin tx_len=24 icin header tx_data(23 downto 0)
--    icine konur, MSB once gonderilir.
--  * rx_data'nin gecerli kismi alt rx_len bit'tir.
--  * Kenar tespiti oversampling ile yapildigi icin okuma SCLK'si sistem
--    saatinden yeterince yavas olmalidir: rd icin clk_div >= 4 onerilir
--    (f_sclk_rd <= f_clk/8).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_master is
  generic (
    CS_SETUP_CYCLES  : natural := 4;   -- CS dusmesi ile ilk SCLK arasi bekleme (clk)
    CS_HOLD_CYCLES   : natural := 4;   -- son SCLK ile CS kalkmasi arasi bekleme (clk)
    RX_SETTLE_CYCLES : natural := 12   -- son kenardan sonra sclk_in/MISO'nun
                                       -- oturmasi + senkronizer/kenar tespiti
                                       -- gecikmesi icin beklenen clk sayisi
  );
  port (
    clk      : in  std_logic;
    rst_n    : in  std_logic;

    -- kontrol arayuzu
    start    : in  std_logic;                      -- 1 clk'lik pulse
    clk_div  : in  unsigned(15 downto 0);          -- SCLK yarim periyodu (clk cinsinden, min 1)
    tx_data  : in  std_logic_vector(31 downto 0);  -- sag hizali header/veri
    tx_len   : in  unsigned(5 downto 0);           -- header bit sayisi (0..32)
    rx_len   : in  unsigned(5 downto 0);           -- okunacak bit sayisi (0..32)
    rx_data  : out std_logic_vector(31 downto 0);  -- alt rx_len bit gecerli

    -- streaming uzantisi
    stream_nbytes : in  unsigned(7 downto 0);          -- header sonrasi ek byte sayisi
    stream_byte   : in  std_logic_vector(7 downto 0);  -- siradaki byte (surekli sunulur)
    stream_taken  : out std_logic;                     -- pulse: byte alindi, yenisini sun

    -- CS'siz saat uretimi (NVM merge)
    start_burst  : in  std_logic;                  -- 1 clk'lik pulse
    burst_cycles : in  unsigned(15 downto 0);      -- uretilecek SCLK pulse sayisi

    busy     : out std_logic;
    done     : out std_logic;                      -- 1 clk'lik pulse

    -- SPI pinleri
    sclk_out : out std_logic;
    sclk_in  : in  std_logic;                      -- geri donen (loopback) SPI saati
    mosi     : out std_logic;
    miso     : in  std_logic;
    cs_n     : out std_logic
  );
end entity spi_master;

architecture rtl of spi_master is

  type t_state is (ST_IDLE, ST_CS_SETUP, ST_XFER, ST_SETTLE, ST_CS_HOLD, ST_BURST);
  signal state : t_state := ST_IDLE;

  signal tx_shift   : std_logic_vector(31 downto 0) := (others => '0');
  signal tx_end     : unsigned(11 downto 0) := (others => '0');  -- header bit sayisi
  signal stream_end : unsigned(11 downto 0) := (others => '0');  -- header + stream bitleri
  signal total_bits : unsigned(11 downto 0) := (others => '0');
  signal bit_cnt    : unsigned(11 downto 0) := (others => '0');
  signal div_r      : unsigned(15 downto 0) := (others => '0');
  signal div_cnt    : unsigned(15 downto 0) := (others => '0');
  signal wait_cnt   : unsigned(7 downto 0) := (others => '0');

  signal cur_byte   : std_logic_vector(7 downto 0) := (others => '0');
  signal stream_bit : unsigned(2 downto 0) := (others => '0');

  signal burst_cnt  : unsigned(15 downto 0) := (others => '0');
  signal burst_tgt  : unsigned(15 downto 0) := (others => '0');

  signal sclk_i     : std_logic := '0';
  signal cs_n_i     : std_logic := '1';
  signal mosi_i     : std_logic := '0';
  signal done_r     : std_logic := '0';
  signal taken_r    : std_logic := '0';

  -- okuma yolu (tamami clk domain'inde calisir)
  signal rx_gate    : std_logic := '0';
  signal rx_shift   : std_logic_vector(31 downto 0) := (others => '0');
  signal rx_data_r  : std_logic_vector(31 downto 0) := (others => '0');

  -- sclk_in / miso senkronizerleri (asenkron girisler icin 2-FF)
  signal sclk_in_meta : std_logic := '0';
  signal sclk_in_sync : std_logic := '0';
  signal sclk_in_prev : std_logic := '0';
  signal miso_meta    : std_logic := '0';
  signal miso_sync    : std_logic := '0';
  signal sclk_in_rise : std_logic;

  attribute ASYNC_REG : string;
  attribute ASYNC_REG of sclk_in_meta : signal is "TRUE";
  attribute ASYNC_REG of sclk_in_sync : signal is "TRUE";
  attribute ASYNC_REG of miso_meta    : signal is "TRUE";
  attribute ASYNC_REG of miso_sync    : signal is "TRUE";

begin

  sclk_out     <= sclk_i;
  cs_n         <= cs_n_i;
  mosi         <= mosi_i;
  rx_data      <= rx_data_r;
  done         <= done_r;
  stream_taken <= taken_r;
  busy         <= '0' when state = ST_IDLE else '1';

  ------------------------------------------------------------------------------
  -- MISO yakalama: sclk_in clock olarak kullanilmaz (clock-capable pin
  -- gerektirmemesi icin). Bunun yerine sclk_in ve miso 2-FF senkronizerden
  -- gecirilir, sclk_in'in rising edge'i clk domain'inde tespit edilir ve o
  -- anda miso orneklenir. Her iki sinyal ayni senkronizer gecikmesine sahip
  -- oldugu icin hizalari korunur. rx_gate yalnizca okuma fazinda '1' oldugu
  -- icin tam olarak rx_len bit kaydirilir.
  ------------------------------------------------------------------------------
  p_sync : process(clk)
  begin
    if rising_edge(clk) then
      sclk_in_meta <= sclk_in;
      sclk_in_sync <= sclk_in_meta;
      sclk_in_prev <= sclk_in_sync;
      miso_meta    <= miso;
      miso_sync    <= miso_meta;
    end if;
  end process p_sync;

  sclk_in_rise <= '1' when (sclk_in_prev = '0' and sclk_in_sync = '1') else '0';

  p_rx : process(clk)
  begin
    if rising_edge(clk) then
      if rx_gate = '1' and sclk_in_rise = '1' then
        rx_shift <= rx_shift(30 downto 0) & miso_sync;
      end if;
    end if;
  end process p_rx;

  ------------------------------------------------------------------------------
  -- Ana FSM (clk domain'i): SCLK uretimi, MOSI kaydirma, CS kontrolu.
  ------------------------------------------------------------------------------
  p_fsm : process(clk)
    variable v_shift : integer;
    variable v_next  : unsigned(11 downto 0);
  begin
    if rising_edge(clk) then
      done_r  <= '0';
      taken_r <= '0';

      if rst_n = '0' then
        state   <= ST_IDLE;
        sclk_i  <= '0';
        cs_n_i  <= '1';
        mosi_i  <= '0';
        rx_gate <= '0';
      else
        case state is

          when ST_IDLE =>
            sclk_i <= '0';
            if start = '1' and (tx_len /= 0 or rx_len /= 0 or stream_nbytes /= 0) then
              -- header'i sola hizala ki MSB'den gonderilsin
              v_shift := 32 - to_integer(tx_len);
              if v_shift < 0 then
                v_shift := 0;
              end if;
              tx_shift   <= std_logic_vector(shift_left(unsigned(tx_data), v_shift));
              tx_end     <= resize(tx_len, 12);
              stream_end <= resize(tx_len, 12)
                            + shift_left(resize(stream_nbytes, 12), 3);
              total_bits <= resize(tx_len, 12)
                            + shift_left(resize(stream_nbytes, 12), 3)
                            + resize(rx_len, 12);
              if clk_div = 0 then
                div_r <= to_unsigned(1, 16);
              else
                div_r <= clk_div;
              end if;
              bit_cnt  <= (others => '0');
              div_cnt  <= (others => '0');
              cs_n_i   <= '0';
              wait_cnt <= to_unsigned(CS_SETUP_CYCLES, 8);
              state    <= ST_CS_SETUP;
            elsif start_burst = '1' and burst_cycles /= 0 then
              if clk_div = 0 then
                div_r <= to_unsigned(1, 16);
              else
                div_r <= clk_div;
              end if;
              burst_tgt <= burst_cycles;
              burst_cnt <= (others => '0');
              div_cnt   <= (others => '0');
              state     <= ST_BURST;
            end if;

          when ST_CS_SETUP =>
            -- ilk bit'i CS dustukten hemen sonra sur
            if tx_end /= 0 then
              mosi_i <= tx_shift(31);
            elsif stream_end /= 0 then
              mosi_i     <= stream_byte(7);
              cur_byte   <= stream_byte(6 downto 0) & '0';
              stream_bit <= "001";
            else
              mosi_i  <= '0';
              rx_gate <= '1';        -- transfer tamamen okuma ise gate hemen acilir
            end if;
            if wait_cnt = 0 then
              div_cnt <= (others => '0');
              if tx_end = 0 and stream_end /= 0 then
                taken_r <= '1';      -- ilk byte alindi
              end if;
              state <= ST_XFER;
            else
              wait_cnt <= wait_cnt - 1;
            end if;

          when ST_XFER =>
            if div_cnt = div_r - 1 then
              div_cnt <= (others => '0');
              if sclk_i = '0' then
                -- rising edge: slave MOSI'yi, biz (sclk_in uzerinden) MISO'yu
                -- orneklemis oluruz
                sclk_i <= '1';
              else
                -- falling edge: bit ilerlet, siradaki MOSI bit'ini sur
                sclk_i <= '0';
                if bit_cnt = total_bits - 1 then
                  wait_cnt <= to_unsigned(RX_SETTLE_CYCLES, 8);
                  state    <= ST_SETTLE;
                else
                  bit_cnt <= bit_cnt + 1;
                  v_next  := bit_cnt + 1;
                  if v_next < tx_end then
                    -- header fazi
                    tx_shift <= tx_shift(30 downto 0) & '0';
                    mosi_i   <= tx_shift(30);
                  elsif v_next < stream_end then
                    -- streaming fazi
                    if v_next = tx_end or stream_bit = "000" then
                      -- yeni byte'i al (ust modul stream_byte'i sunmus olmali)
                      mosi_i     <= stream_byte(7);
                      cur_byte   <= stream_byte(6 downto 0) & '0';
                      stream_bit <= "001";
                      taken_r    <= '1';
                    else
                      mosi_i     <= cur_byte(7);
                      cur_byte   <= cur_byte(6 downto 0) & '0';
                      stream_bit <= stream_bit + 1;
                    end if;
                  else
                    -- okuma fazi
                    mosi_i  <= '0';
                    rx_gate <= '1';
                  end if;
                end if;
              end if;
            else
              div_cnt <= div_cnt + 1;
            end if;

          when ST_SETTLE =>
            -- geciken son sclk_in kenarinin gelmesini ve senkronizer/kenar
            -- tespiti gecikmesini bekle, sonra rx_shift'i kaydet
            if wait_cnt = 0 then
              rx_data_r <= rx_shift;
              rx_gate   <= '0';
              wait_cnt  <= to_unsigned(CS_HOLD_CYCLES, 8);
              state     <= ST_CS_HOLD;
            else
              wait_cnt <= wait_cnt - 1;
            end if;

          when ST_CS_HOLD =>
            if wait_cnt = 0 then
              cs_n_i <= '1';
              done_r <= '1';
              state  <= ST_IDLE;
            else
              wait_cnt <= wait_cnt - 1;
            end if;

          when ST_BURST =>
            -- CS aktif edilmeden SCLK pulse'lari uret (NVM merge icin)
            if div_cnt = div_r - 1 then
              div_cnt <= (others => '0');
              if sclk_i = '0' then
                sclk_i    <= '1';
                burst_cnt <= burst_cnt + 1;
              else
                sclk_i <= '0';
                if burst_cnt = burst_tgt then
                  done_r <= '1';
                  state  <= ST_IDLE;
                end if;
              end if;
            else
              div_cnt <= div_cnt + 1;
            end if;

        end case;
      end if;
    end if;
  end process p_fsm;

end architecture rtl;

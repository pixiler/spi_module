--------------------------------------------------------------------------------
-- spi_ddr_top.vhd
-- ADMV48281 SPI + DDR ust modulu.
--
-- Calisma sirasi (enable='1' sonrasi):
--   1) INIT   : C_INIT_TABLE'daki registerlar sirayla yazilir (Band 0 bias
--               ayarlari dahil). LOAD gerektiren frame'lerden sonra spi_load
--               pini toggle edilir.
--   2) FAZ    : Global faz SRAM'i doldurulur: DDR'daki 256 byte'lik tablo
--               (G_PHASE_TABLE_ADDR) okunur; ilk 128 byte 0x480-0x4FF'e (RX),
--               sonraki 128 byte 0x580-0x5FF'e (TX) tek tek yazilir.
--   3) NVM    : 0x05F<-0x01 (merge baslat), SCLK'da G_NVM_BURST_CYCLES pulse
--               uretilir (>=10354), 0x01A bit[6]=1 olana kadar okunarak
--               beklenir, 0x05F<-0x00 ile cikilir. Ardindan init_done='1'.
--   4) RUN    : Her G_PERIOD_CYCLES'ta bir idx_a..idx_e'den eleman adresi
--               hesaplanir (eleman = 32 byte), DDR'dan 8 kelime okunur ve
--               streaming write ile stream_reg_addr'den baslayarak 32
--               register'a yazilir (direct beam: RX=0x200, TX=0x240).
--               Frame sonunda LOAD toggle edilir.
--               user_rd_req ile istenildiginde SPI register okumasi yapilir.
--
-- DDR yerlesimi (MicroBlaze little-endian, C ile birebir):
--   * Beam elemani : uint8_t beam[A][B][C][D][E][32];
--       eleman adresi = G_BASE_ADDR + 32*((((a*B+b)*C+c)*D+d)*E+e)
--       byte j -> register (stream_reg_addr + j)  (ascending mod)
--   * Faz tablosu  : uint8_t phase[256] @ G_PHASE_TABLE_ADDR;
--       byte 0..127 -> 0x480..0x4FF, byte 128..255 -> 0x580..0x5FF
--
-- Block design: "Add Module" ile eklenir; m_axi_* portlari otomatik AXI4
-- master olarak taninir ve SmartConnect'e baglanir. G_BASE_ADDR /
-- G_PHASE_TABLE_ADDR, Address Editor'deki DDR araligiyla uyumlu olmalidir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.spi_pkg.all;

entity spi_ddr_top is
  generic (
    G_BASE_ADDR        : std_logic_vector(31 downto 0) := x"80000000";
    G_PHASE_TABLE_ADDR : std_logic_vector(31 downto 0) := x"80100000";
    G_DIM_A            : positive := 4;  -- dizinin ilk boyutu (yalnizca bilgi amacli)
    G_DIM_B            : positive := 4;
    G_DIM_C            : positive := 4;
    G_DIM_D            : positive := 4;
    G_DIM_E            : positive := 4;
    G_PERIOD_CYCLES    : positive := 100_000_000;  -- periyodik yazma araligi (clk)
    G_NVM_BURST_CYCLES : positive := 10500;        -- NVM merge SCLK sayisi (>=10354)
    G_LOAD_WIDTH       : positive := 4             -- LOAD pulse genisligi (clk)
  );
  port (
    clk    : in  std_logic;
    rst_n  : in  std_logic;
    enable : in  std_logic;

    -- dizi indisleri (beam[a][b][c][d][e][32])
    idx_a : in std_logic_vector(7 downto 0);
    idx_b : in std_logic_vector(7 downto 0);
    idx_c : in std_logic_vector(7 downto 0);
    idx_d : in std_logic_vector(7 downto 0);
    idx_e : in std_logic_vector(7 downto 0);

    -- SPI hizlari: f_sclk = f_clk / (2 * div)
    wr_clk_div : in std_logic_vector(15 downto 0);
    rd_clk_div : in std_logic_vector(15 downto 0);

    -- streaming yazmanin baslangic registeri:
    -- RX direct beam = 0x200 (varsayilan), TX direct beam = 0x240
    stream_reg_addr : in std_logic_vector(13 downto 0) := "00001000000000";

    -- istege bagli SPI register okuma arayuzu
    user_rd_req     : in  std_logic;                      -- 1 clk'lik pulse
    user_rd_cmd     : in  std_logic_vector(31 downto 0);  -- sag hizali komut (f_spi_rd)
    user_rd_cmd_len : in  std_logic_vector(5 downto 0);   -- komut bit sayisi (24)
    user_rd_len     : in  std_logic_vector(5 downto 0);   -- okunacak bit sayisi (8)
    user_rd_data    : out std_logic_vector(31 downto 0);
    user_rd_done    : out std_logic;                      -- 1 clk'lik pulse

    -- durum
    init_done : out std_logic;   -- init + faz SRAM + NVM kalibrasyon bitti
    busy      : out std_logic;
    axi_err   : out std_logic;

    -- AXI4 read master (SmartConnect'e baglanir)
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arcache : out std_logic_vector(3 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rlast   : in  std_logic;
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic;

    -- SPI / ADMV48281 pinleri (1.8V CMOS! seviye donusturucu gerekir)
    spi_sclk_out : out std_logic;   -- SCLK
    spi_sclk_in  : in  std_logic;   -- loopback / ring modda son cipin CLK_OUT'u
    spi_mosi     : out std_logic;   -- SDIO (4-wire modda giris)
    spi_miso     : in  std_logic;   -- SDO
    spi_cs_n     : out std_logic;   -- CS
    spi_load     : out std_logic    -- LOAD
  );
end entity spi_ddr_top;

architecture rtl of spi_ddr_top is

  type t_state is (S_IDLE,
                   S_INIT_START, S_INIT_WAIT,
                   S_PH_FETCH, S_PH_FETCH_WAIT, S_PH_WR, S_PH_WR_WAIT,
                   S_NVM_FILL, S_NVM_FILL_WAIT,
                   S_NVM_BURST, S_NVM_BURST_WAIT,
                   S_NVM_POLL, S_NVM_POLL_WAIT,
                   S_NVM_EXIT, S_NVM_EXIT_WAIT,
                   S_RUN,
                   S_CALC, S_ELEM_ADDR, S_BEAM_FETCH, S_BEAM_FETCH_WAIT,
                   S_STREAM_START, S_STREAM_WAIT,
                   S_LOAD,
                   S_RD_START, S_RD_WAIT);
  signal state      : t_state := S_IDLE;
  signal after_load : t_state := S_RUN;   -- LOAD pulse'i sonrasi gidilecek durum

  -- Horner yontemi icin boyutlar: acc = ((((a*B)+b)*C+c)*D+d)*E+e
  type t_dim_arr is array (1 to 4) of positive;
  constant C_DIMS : t_dim_arr := (1 => G_DIM_B, 2 => G_DIM_C,
                                  3 => G_DIM_D, 4 => G_DIM_E);

  type t_idx_arr is array (0 to 4) of unsigned(7 downto 0);
  signal idx_lat : t_idx_arr := (others => (others => '0'));

  type t_buf is array (0 to 7) of std_logic_vector(31 downto 0);
  signal ddr_buf : t_buf := (others => (others => '0'));

  signal init_idx   : integer range 0 to C_INIT_TABLE'length - 1 := 0;
  signal calc_i     : integer range 1 to 4 := 1;
  signal acc        : unsigned(31 downto 0) := (others => '0');
  signal elem_base  : unsigned(31 downto 0) := (others => '0');
  signal period_cnt : unsigned(31 downto 0) := (others => '0');
  signal rd_pending : std_logic := '0';

  signal word_cnt  : unsigned(2 downto 0) := (others => '0');
  signal byte_idx  : unsigned(4 downto 0) := (others => '0');
  signal phase_idx : unsigned(7 downto 0) := (others => '0');
  signal ph_word   : std_logic_vector(31 downto 0) := (others => '0');

  signal load_r   : std_logic := '0';
  signal load_cnt : unsigned(7 downto 0) := (others => '0');

  signal init_done_r    : std_logic := '0';
  signal user_rd_done_r : std_logic := '0';
  signal user_rd_data_r : std_logic_vector(31 downto 0) := (others => '0');

  -- spi_master baglantilari
  signal spi_start    : std_logic := '0';
  signal spi_burst    : std_logic := '0';
  signal spi_div      : unsigned(15 downto 0) := (others => '0');
  signal spi_tx       : std_logic_vector(31 downto 0) := (others => '0');
  signal spi_tx_len   : unsigned(5 downto 0) := (others => '0');
  signal spi_rx_len   : unsigned(5 downto 0) := (others => '0');
  signal spi_nbytes   : unsigned(7 downto 0) := (others => '0');
  signal spi_sbyte    : std_logic_vector(7 downto 0);
  signal spi_taken    : std_logic;
  signal spi_rx       : std_logic_vector(31 downto 0);
  signal spi_done     : std_logic;
  signal spi_busy     : std_logic;

  -- axi_ddr_reader baglantilari
  signal ddr_req   : std_logic := '0';
  signal ddr_addr  : std_logic_vector(31 downto 0) := (others => '0');
  signal ddr_rdata : std_logic_vector(31 downto 0);
  signal ddr_done  : std_logic;

  constant C_NVM_FILL_FRAME : std_logic_vector(31 downto 0) := f_spi_wr(16#05F#, 16#01#);
  constant C_NVM_EXIT_FRAME : std_logic_vector(31 downto 0) := f_spi_wr(16#05F#, 16#00#);
  constant C_NVM_POLL_FRAME : std_logic_vector(31 downto 0) := f_spi_rd(16#01A#);

begin

  init_done    <= init_done_r;
  user_rd_done <= user_rd_done_r;
  user_rd_data <= user_rd_data_r;
  spi_load     <= load_r;
  busy         <= '0' when (state = S_RUN or state = S_IDLE) else '1';

  -- streaming byte'i: DDR buffer'indan little-endian sirayla sun
  spi_sbyte <= f_lane32(ddr_buf(to_integer(byte_idx(4 downto 2))),
                        byte_idx(1 downto 0));

  u_spi : entity work.spi_master
    port map (
      clk           => clk,
      rst_n         => rst_n,
      start         => spi_start,
      clk_div       => spi_div,
      tx_data       => spi_tx,
      tx_len        => spi_tx_len,
      rx_len        => spi_rx_len,
      rx_data       => spi_rx,
      stream_nbytes => spi_nbytes,
      stream_byte   => spi_sbyte,
      stream_taken  => spi_taken,
      start_burst   => spi_burst,
      burst_cycles  => to_unsigned(G_NVM_BURST_CYCLES, 16),
      busy          => spi_busy,
      done          => spi_done,
      sclk_out      => spi_sclk_out,
      sclk_in       => spi_sclk_in,
      mosi          => spi_mosi,
      miso          => spi_miso,
      cs_n          => spi_cs_n
    );

  u_ddr : entity work.axi_ddr_reader
    port map (
      clk           => clk,
      rst_n         => rst_n,
      req           => ddr_req,
      addr          => ddr_addr,
      rdata         => ddr_rdata,
      done          => ddr_done,
      axi_err       => axi_err,
      m_axi_araddr  => m_axi_araddr,
      m_axi_arlen   => m_axi_arlen,
      m_axi_arsize  => m_axi_arsize,
      m_axi_arburst => m_axi_arburst,
      m_axi_arcache => m_axi_arcache,
      m_axi_arprot  => m_axi_arprot,
      m_axi_arvalid => m_axi_arvalid,
      m_axi_arready => m_axi_arready,
      m_axi_rdata   => m_axi_rdata,
      m_axi_rresp   => m_axi_rresp,
      m_axi_rlast   => m_axi_rlast,
      m_axi_rvalid  => m_axi_rvalid,
      m_axi_rready  => m_axi_rready
    );

  p_fsm : process(clk)
    variable v_next : t_state;
    variable v_reg  : std_logic_vector(13 downto 0);
  begin
    if rising_edge(clk) then
      spi_start      <= '0';
      spi_burst      <= '0';
      ddr_req        <= '0';
      user_rd_done_r <= '0';

      if rst_n = '0' then
        state       <= S_IDLE;
        init_done_r <= '0';
        rd_pending  <= '0';
        load_r      <= '0';
        period_cnt  <= (others => '0');
      else
        -- init tamamlandiysa okuma istegini her durumda kaydet
        if user_rd_req = '1' and init_done_r = '1' then
          rd_pending <= '1';
        end if;

        case state is

          ----------------------------------------------------------------
          -- 1) Initial registerlarin yuklenmesi (Band 0 tablosu)
          ----------------------------------------------------------------
          when S_IDLE =>
            if enable = '1' then
              init_idx <= 0;
              state    <= S_INIT_START;
            end if;

          when S_INIT_START =>
            spi_tx     <= C_INIT_TABLE(init_idx).data;
            spi_tx_len <= to_unsigned(C_INIT_TABLE(init_idx).len, 6);
            spi_rx_len <= (others => '0');
            spi_nbytes <= (others => '0');
            spi_div    <= unsigned(wr_clk_div);
            spi_start  <= '1';
            state      <= S_INIT_WAIT;

          when S_INIT_WAIT =>
            if spi_done = '1' then
              if init_idx = C_INIT_TABLE'length - 1 then
                phase_idx <= (others => '0');
                v_next    := S_PH_FETCH;
              else
                init_idx <= init_idx + 1;
                v_next   := S_INIT_START;
              end if;
              if C_INIT_TABLE(init_idx).load then
                after_load <= v_next;
                load_cnt   <= to_unsigned(G_LOAD_WIDTH, 8);
                state      <= S_LOAD;
              else
                state <= v_next;
              end if;
            end if;

          ----------------------------------------------------------------
          -- 2) Global faz SRAM doldurma (DDR'daki 256 byte'lik tablo)
          --    byte 0..127 -> 0x480..0x4FF, byte 128..255 -> 0x580..0x5FF
          ----------------------------------------------------------------
          when S_PH_FETCH =>
            if phase_idx(1 downto 0) = "00" then
              ddr_addr <= std_logic_vector(unsigned(G_PHASE_TABLE_ADDR)
                                           + resize(phase_idx, 32));
              ddr_req  <= '1';
              state    <= S_PH_FETCH_WAIT;
            else
              state <= S_PH_WR;
            end if;

          when S_PH_FETCH_WAIT =>
            if ddr_done = '1' then
              ph_word <= ddr_rdata;
              state   <= S_PH_WR;
            end if;

          when S_PH_WR =>
            if phase_idx(7) = '0' then
              v_reg := "0001001" & std_logic_vector(phase_idx(6 downto 0));  -- 0x480+i
            else
              v_reg := "0001011" & std_logic_vector(phase_idx(6 downto 0));  -- 0x580+i
            end if;
            spi_tx     <= f_spi_wr_u(unsigned(v_reg),
                                     f_lane32(ph_word, phase_idx(1 downto 0)));
            spi_tx_len <= to_unsigned(32, 6);
            spi_rx_len <= (others => '0');
            spi_nbytes <= (others => '0');
            spi_div    <= unsigned(wr_clk_div);
            spi_start  <= '1';
            state      <= S_PH_WR_WAIT;

          when S_PH_WR_WAIT =>
            if spi_done = '1' then
              if phase_idx = 255 then
                state <= S_NVM_FILL;
              else
                phase_idx <= phase_idx + 1;
                state     <= S_PH_FETCH;
              end if;
            end if;

          ----------------------------------------------------------------
          -- 3) NVM kalibrasyon merge (UG-2293, NVM Calibration)
          ----------------------------------------------------------------
          when S_NVM_FILL =>
            spi_tx     <= C_NVM_FILL_FRAME;         -- 0x05F <- 0x01
            spi_tx_len <= to_unsigned(32, 6);
            spi_rx_len <= (others => '0');
            spi_nbytes <= (others => '0');
            spi_div    <= unsigned(wr_clk_div);
            spi_start  <= '1';
            state      <= S_NVM_FILL_WAIT;

          when S_NVM_FILL_WAIT =>
            if spi_done = '1' then
              state <= S_NVM_BURST;
            end if;

          when S_NVM_BURST =>
            spi_div   <= unsigned(wr_clk_div);
            spi_burst <= '1';                       -- >=10354 SCLK pulse
            state     <= S_NVM_BURST_WAIT;

          when S_NVM_BURST_WAIT =>
            if spi_done = '1' then
              state <= S_NVM_POLL;
            end if;

          when S_NVM_POLL =>
            spi_tx     <= C_NVM_POLL_FRAME;         -- 0x01A oku
            spi_tx_len <= to_unsigned(24, 6);
            spi_rx_len <= to_unsigned(8, 6);
            spi_nbytes <= (others => '0');
            spi_div    <= unsigned(rd_clk_div);
            spi_start  <= '1';
            state      <= S_NVM_POLL_WAIT;

          when S_NVM_POLL_WAIT =>
            if spi_done = '1' then
              if spi_rx(6) = '1' then               -- kalibrasyon bitti
                state <= S_NVM_EXIT;
              else
                state <= S_NVM_POLL;                -- her poll 32 clock daha saglar
              end if;
            end if;

          when S_NVM_EXIT =>
            spi_tx     <= C_NVM_EXIT_FRAME;         -- 0x05F <- 0x00
            spi_tx_len <= to_unsigned(32, 6);
            spi_rx_len <= (others => '0');
            spi_nbytes <= (others => '0');
            spi_div    <= unsigned(wr_clk_div);
            spi_start  <= '1';
            state      <= S_NVM_EXIT_WAIT;

          when S_NVM_EXIT_WAIT =>
            if spi_done = '1' then
              init_done_r <= '1';
              period_cnt  <= (others => '0');
              state       <= S_RUN;
            end if;

          ----------------------------------------------------------------
          -- 4) Bekleme: periyot sayaci + okuma istegi onceligi
          ----------------------------------------------------------------
          when S_RUN =>
            period_cnt <= period_cnt + 1;
            if rd_pending = '1' then
              rd_pending <= '0';
              state      <= S_RD_START;
            elsif period_cnt >= to_unsigned(G_PERIOD_CYCLES - 1, 32) then
              period_cnt <= (others => '0');
              idx_lat(0) <= unsigned(idx_a);
              idx_lat(1) <= unsigned(idx_b);
              idx_lat(2) <= unsigned(idx_c);
              idx_lat(3) <= unsigned(idx_d);
              idx_lat(4) <= unsigned(idx_e);
              acc        <= resize(unsigned(idx_a), 32);
              calc_i     <= 1;
              state      <= S_CALC;
            end if;

          ----------------------------------------------------------------
          -- 5) Eleman adresi hesabi + DDR'dan 32 byte (8 kelime) okuma
          ----------------------------------------------------------------
          when S_CALC =>
            acc <= resize(acc * to_unsigned(C_DIMS(calc_i), 16), 32)
                   + resize(idx_lat(calc_i), 32);
            if calc_i = 4 then
              state <= S_ELEM_ADDR;
            else
              calc_i <= calc_i + 1;
            end if;

          when S_ELEM_ADDR =>
            elem_base <= unsigned(G_BASE_ADDR) + shift_left(acc, 5);  -- eleman = 32 byte
            word_cnt  <= (others => '0');
            state     <= S_BEAM_FETCH;

          when S_BEAM_FETCH =>
            ddr_addr <= std_logic_vector(elem_base
                                         + shift_left(resize(word_cnt, 32), 2));
            ddr_req  <= '1';
            state    <= S_BEAM_FETCH_WAIT;

          when S_BEAM_FETCH_WAIT =>
            if ddr_done = '1' then
              ddr_buf(to_integer(word_cnt)) <= ddr_rdata;
              if word_cnt = 7 then
                byte_idx <= (others => '0');
                state    <= S_STREAM_START;
              else
                word_cnt <= word_cnt + 1;
                state    <= S_BEAM_FETCH;
              end if;
            end if;

          ----------------------------------------------------------------
          -- 6) Streaming write: 24 bit header + 32 byte, sonra LOAD toggle
          ----------------------------------------------------------------
          when S_STREAM_START =>
            spi_tx     <= x"00" & "0" & "0000" & "00000" & stream_reg_addr;
            spi_tx_len <= to_unsigned(24, 6);
            spi_rx_len <= (others => '0');
            spi_nbytes <= to_unsigned(32, 8);
            spi_div    <= unsigned(wr_clk_div);
            spi_start  <= '1';
            state      <= S_STREAM_WAIT;

          when S_STREAM_WAIT =>
            if spi_taken = '1' then
              byte_idx <= byte_idx + 1;   -- siradaki byte'i sun
            end if;
            if spi_done = '1' then
              after_load <= S_RUN;        -- 0x200-0x25F LOAD toggle gerektirir
              load_cnt   <= to_unsigned(G_LOAD_WIDTH, 8);
              state      <= S_LOAD;
            end if;

          ----------------------------------------------------------------
          -- LOAD pulse'i (CS kalktiktan sonra)
          ----------------------------------------------------------------
          when S_LOAD =>
            if load_cnt = 0 then
              load_r <= '0';
              state  <= after_load;
            else
              load_r   <= '1';
              load_cnt <= load_cnt - 1;
            end if;

          ----------------------------------------------------------------
          -- 7) Istege bagli SPI register okuma
          ----------------------------------------------------------------
          when S_RD_START =>
            spi_tx     <= user_rd_cmd;
            spi_tx_len <= unsigned(user_rd_cmd_len);
            spi_rx_len <= unsigned(user_rd_len);
            spi_nbytes <= (others => '0');
            spi_div    <= unsigned(rd_clk_div);
            spi_start  <= '1';
            state      <= S_RD_WAIT;

          when S_RD_WAIT =>
            if spi_done = '1' then
              user_rd_data_r <= spi_rx;
              user_rd_done_r <= '1';
              state          <= S_RUN;
            end if;

        end case;
      end if;
    end if;
  end process p_fsm;

end architecture rtl;

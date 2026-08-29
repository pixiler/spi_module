--------------------------------------------------------------------------------
-- admv48281_top.vhd
--
-- 7 SPI bus uzerinde toplam 52 ADMV48281 beamformer'i (6 x 8 + 1 x 4, ring
-- konfigurasyonu) yoneten ust modul.
--
-- Akis:
--   enable = '1'  -> 7 bus paralel olarak init + faz SRAM + NVM kalibrasyon
--                    sekansini yurutur, hepsi bitince init_done = '1'
--   init_done     -> modul IDLE'da trig bekler
--   trig darbesi  -> rx_tx_sel ornekleir, s_axis_tready = '1' yapar
--   AXI-Stream    -> 416 x 32 bit (1664 byte) beam paketi bus RAM'lerine dagitilir
--   paket biter   -> her bus kendi ciplerine 32'ser byte streaming ile yazar
--   hepsi biter   -> load_pending = '1', disaridan load_trig beklenir
--                    (load_trig daha erken geldiyse kuyrukta tutulur)
--   load_trig     -> tum LOAD hatlari es zamanli toggle edilir; darbe genisligi
--                    UG-2293 Table 2'den turetilir (toggle periyodu >= 7.5 ns),
--                    ardindan load_done darbesi
--   LOAD sonrasi  -> G_TRX_DELAY_CYCLES beklenir, TRX_x pini rx_tx_sel'e
--                    guncellenir (trx_updated), sonra beam_done
--
-- TRX sirasi onemlidir: beam verisi once SPI ile yazilir, LOAD ile calisan
-- registerlara aktarilir, TRX pini ancak ondan sonra yon degistirir.
--
-- AXI-Stream paket duzeni (TDATA 32 bit, little-endian byte sirasi):
--   kelime   0.. 63 : bus 0, cip 0..7   (cip basina 8 kelime = 32 byte)
--   kelime  64..127 : bus 1
--   kelime 128..191 : bus 2
--   kelime 192..255 : bus 3
--   kelime 256..319 : bus 4
--   kelime 320..383 : bus 5
--   TLAST 384. kelimede
--
-- Bus 6 STATIK'tir (C_BEAM_ON_TRIG): pakette yer almaz. Acilista
-- C_STATIC_BEAM_RX/TX tablolari bir kez yazilip LOAD'lanir; trig'lerde SPI
-- yazmasi yapilmaz ama senkron LOAD darbesi ve TRX pini yine surulur.
--
-- Bir cipin 32 byte'i, direct beam register blogunun (0x200 veya 0x240) icerigidir:
--   byte  0 : ch0V gain    byte  1 : ch0V phase
--   byte  2 : ch0H gain    byte  3 : ch0H phase
--   ...
--   byte 30 : ch7H gain    byte 31 : ch7H phase
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.admv48281_pkg.all;

entity admv48281_top is
  generic (
    G_CLK_FREQ_HZ      : natural := 100000000;  -- sistem saati (LOAD zamanlamasi icin)
    G_CLK_DIV_WR       : natural := 2;      -- f_sclk_wr = f_clk / (2*div)
    G_CLK_DIV_RD       : natural := 8;      -- okuma daha yavas (ring gecikmesi)
    G_NVM_BURST        : natural := 10500;  -- UG-2293: en az 10354
    -- LOAD darbesinin yuksek (ve dusuk) yarisi, clock cinsinden.
    -- 0 = UG-2293 Table 2'den turet (toggle periyodu min 7.5 ns).
    G_LOAD_CYCLES      : natural := 0;
    -- LOAD darbesi ile TRX gecisi arasindaki bekleme. UG-2293'te bu spec
    -- yoktur; ADMV48281 datasheet'ine gore ayarlayin.
    G_TRX_DELAY_CYCLES : natural := 100;    -- 100 MHz'de 1 us
    G_RESET_WAIT       : natural := 1000;
    G_POLL_LIMIT       : natural := 2000;
    G_RX_SETTLE_CYCLES : natural := 64;
    G_RX_FILTER_LEN    : natural := 3       -- donen CLK_OUT/SDO giris filtresi
  );
  port (
    clk   : in std_logic;
    rst_n : in std_logic;

    enable : in std_logic;   -- ADMV power-up tamamlandiktan sonra verilir

    -- AXI4-Stream slave: beam verisi
    s_axis_tvalid : in  std_logic;
    s_axis_tready : out std_logic;
    s_axis_tdata  : in  std_logic_vector(31 downto 0);
    s_axis_tlast  : in  std_logic;

    -- tetik
    trig      : in std_logic;   -- yukselen kenar, beam turunu baslatir
    rx_tx_sel : in std_logic;   -- '0' = RX direct beam, '1' = TX direct beam

    -- LOAD tetigi: SPI yazma bittikten sonra LOAD pinini toggle eder.
    -- Erken gelirse kilitlenir ve yazma biter bitmez uygulanir (kuyruk).
    load_trig : in std_logic;   -- yukselen kenar

    -- TRX_x donanim pini (bus basina). '0' = receive, '1' = transmit.
    -- Acilista '0' (UG-2293: cip receive modda baslamali) ve her beam turunda
    -- LOAD darbesinden G_TRX_DELAY_CYCLES sonra guncellenir.
    trx_out : out std_logic_vector(C_NUM_BUS-1 downto 0);

    -- kullanici SPI okumasi (init_done sonrasi, beam yazmadigi anlarda)
    rd_req   : in  std_logic;                      -- yukselen kenar
    rd_bus   : in  std_logic_vector(2 downto 0);   -- 0..6
    rd_chip  : in  std_logic_vector(2 downto 0);   -- zincirdeki sira
    rd_addr  : in  std_logic_vector(13 downto 0);
    rd_data  : out std_logic_vector(7 downto 0);
    rd_valid : out std_logic;                      -- 1 clock darbe
    -- kalici teshis: bir okumada beklenen bit sayisi yakalanamadi
    -- (ring gecikmesi / G_RX_SETTLE_CYCLES yetersiz ya da hatta glitch)
    rd_short_err : out std_logic_vector(C_NUM_BUS-1 downto 0);

    -- durum
    init_done    : out std_logic;
    init_err     : out std_logic;
    busy         : out std_logic;
    load_pending : out std_logic;  -- seviye: SPI yazma bitti, load_trig bekleniyor
    load_done    : out std_logic;  -- 1 clock darbe: LOAD toggle tamamlandi
    trx_updated  : out std_logic;  -- 1 clock darbe: TRX pini guncellendi
    beam_done    : out std_logic;  -- 1 clock darbe: beam turu tamamen bitti
    axis_err     : out std_logic;  -- paket uzunlugu / TLAST uyumsuzlugu

    -- SPI pinleri (bus basina)
    spi_sclk_out : out std_logic_vector(C_NUM_BUS-1 downto 0);
    spi_mosi     : out std_logic_vector(C_NUM_BUS-1 downto 0);
    spi_cs_n     : out std_logic_vector(C_NUM_BUS-1 downto 0);
    spi_load     : out std_logic_vector(C_NUM_BUS-1 downto 0);
    spi_sclk_in  : in  std_logic_vector(C_NUM_BUS-1 downto 0);
    spi_miso     : in  std_logic_vector(C_NUM_BUS-1 downto 0)
  );
end entity admv48281_top;

architecture rtl of admv48281_top is

  constant C_TOTAL_WORDS : natural := f_total_words;   -- 384 (sadece veri buslari)

  constant C_ALL_ONES : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '1');
  constant C_ALL_ZERO : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '0');

  -- LOAD darbesinin yarim periyodu (UG-2293 Table 2: toggle periyodu >= 7.5 ns)
  constant C_LOAD_MIN  : natural := f_load_half_cycles(G_CLK_FREQ_HZ);
  constant C_LOAD_HALF : natural := f_load_cycles(G_CLK_FREQ_HZ, G_LOAD_CYCLES);

  type t_word_array is array (0 to C_NUM_BUS-1) of std_logic_vector(31 downto 0);
  type t_addr_array is array (0 to C_NUM_BUS-1) of std_logic_vector(5 downto 0);

  signal bus_init_done : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal bus_init_err  : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal bus_busy      : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal bus_beam_rdy  : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal bus_load      : std_logic_vector(C_NUM_BUS-1 downto 0);

  signal bram_we   : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '0');
  signal bram_addr : t_addr_array := (others => (others => '0'));
  signal bram_data : t_word_array := (others => (others => '0'));

  type t_tstate is (T_IDLE, T_AXIS, T_SPI, T_WAIT_LOAD,
                    T_LOAD_HI, T_LOAD_LO, T_TRX_DLY, T_TRX, T_DONE);
  signal tstate : t_tstate := T_IDLE;

  signal trig_d      : std_logic := '0';
  signal trig_rise   : std_logic;
  signal ltrig_d     : std_logic := '0';
  signal ltrig_rise  : std_logic;
  signal ltrig_pend  : std_logic := '0';   -- kuyruk: erken gelen load_trig
  signal rx_tx_lat   : std_logic := '0';
  signal beam_start  : std_logic := '0';
  signal load_glb    : std_logic := '0';
  signal load_cnt    : unsigned(31 downto 0) := (others => '0');
  signal trx_r       : std_logic := '0';   -- acilista receive
  signal beam_done_r : std_logic := '0';
  signal load_done_r : std_logic := '0';
  signal trx_upd_r   : std_logic := '0';
  signal axis_err_r  : std_logic := '0';

  signal bus_idx     : natural range 0 to C_NUM_BUS-1 := 0;
  signal word_in_bus : unsigned(5 downto 0) := (others => '0');
  signal total_cnt   : unsigned(9 downto 0) := (others => '0');

  signal tready_r : std_logic := '0';
  signal accept   : std_logic;

  signal all_init : std_logic;
  signal all_rdy  : std_logic;
  signal any_busy : std_logic;

  -- kullanici okuma yonlendirmesi
  type t_byte_vec is array (0 to C_NUM_BUS-1) of std_logic_vector(7 downto 0);
  signal bus_rd_req   : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '0');
  signal bus_rd_data  : t_byte_vec;
  signal bus_rd_valid : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal rd_bus_lat   : natural range 0 to C_NUM_BUS-1 := 0;
  signal rd_chip_lat  : std_logic_vector(2 downto 0) := (others => '0');
  signal rd_addr_lat  : std_logic_vector(13 downto 0) := (others => '0');
  signal rd_req_d     : std_logic := '0';
  signal rd_pend      : std_logic := '0';

begin

  ------------------------------------------------------------------------------
  -- Durum toplama
  ------------------------------------------------------------------------------
  all_init <= '1' when bus_init_done = C_ALL_ONES else '0';
  all_rdy  <= '1' when bus_beam_rdy  = C_ALL_ONES else '0';
  any_busy <= '0' when bus_busy      = C_ALL_ZERO else '1';

  init_done    <= all_init;
  init_err     <= '0' when bus_init_err = C_ALL_ZERO else '1';
  busy         <= '1' when (any_busy = '1' or tstate /= T_IDLE) else '0';
  load_pending <= '1' when tstate = T_WAIT_LOAD else '0';
  load_done    <= load_done_r;
  trx_updated  <= trx_upd_r;
  beam_done    <= beam_done_r;
  axis_err     <= axis_err_r;

  trx_out <= (others => trx_r);

  s_axis_tready <= tready_r;
  accept        <= s_axis_tvalid and tready_r;

  trig_rise  <= trig and not trig_d;
  ltrig_rise <= load_trig and not ltrig_d;

  -- UG-2293 Table 2: LOAD toggle periyodu en az 7.5 ns olmalidir
  assert C_LOAD_HALF >= C_LOAD_MIN
    report "G_LOAD_CYCLES, UG-2293 Table 2 minimumunun altinda "
           & "(LOAD toggle periyodu >= 7.5 ns)"
    severity failure;

  ------------------------------------------------------------------------------
  -- Bus ornekleri. LOAD hatti = bus'in kendi init darbesi VEYA global beam darbesi
  ------------------------------------------------------------------------------
  gen_bus : for i in 0 to C_NUM_BUS-1 generate
    u_bus : entity work.admv_bus_ctrl
      generic map (
        G_BUS_ID           => i,
        G_STATIC_BEAM      => not C_BEAM_ON_TRIG(i),
        G_CLK_DIV_WR       => G_CLK_DIV_WR,
        G_CLK_DIV_RD       => G_CLK_DIV_RD,
        G_NVM_BURST        => G_NVM_BURST,
        -- init LOAD darbeleri de spec'ten turetilen genisligi kullanir
        G_LOAD_CYCLES      => C_LOAD_HALF,
        G_RESET_WAIT       => G_RESET_WAIT,
        G_POLL_LIMIT       => G_POLL_LIMIT,
        G_RX_SETTLE_CYCLES => G_RX_SETTLE_CYCLES,
        G_RX_FILTER_LEN    => G_RX_FILTER_LEN
      )
      port map (
        clk          => clk,
        rst_n        => rst_n,
        enable       => enable,
        bram_we      => bram_we(i),
        bram_addr    => bram_addr(i),
        bram_data    => bram_data(i),
        beam_start   => beam_start,
        rx_tx_sel    => rx_tx_lat,
        beam_ready   => bus_beam_rdy(i),

        user_rd_req   => bus_rd_req(i),
        user_rd_chip  => rd_chip_lat,
        user_rd_addr  => rd_addr_lat,
        user_rd_data  => bus_rd_data(i),
        user_rd_valid => bus_rd_valid(i),

        init_done    => bus_init_done(i),
        init_err     => bus_init_err(i),
        busy         => bus_busy(i),
        dbg_rd_data  => open,
        dbg_rd_bits  => open,
        rd_short_err => rd_short_err(i),
        spi_sclk_out => spi_sclk_out(i),
        spi_mosi     => spi_mosi(i),
        spi_cs_n     => spi_cs_n(i),
        spi_load     => bus_load(i),
        spi_sclk_in  => spi_sclk_in(i),
        spi_miso     => spi_miso(i)
      );

    spi_load(i) <= bus_load(i) or load_glb;
  end generate gen_bus;

  ------------------------------------------------------------------------------
  -- Kullanici okuma istegini ilgili bus'a yonlendir
  ------------------------------------------------------------------------------
  rd_data  <= bus_rd_data(rd_bus_lat);
  rd_valid <= bus_rd_valid(rd_bus_lat);

  process (clk)
  begin
    if rising_edge(clk) then
      bus_rd_req <= (others => '0');

      if rst_n = '0' then
        rd_req_d    <= '0';
        rd_pend     <= '0';
        rd_bus_lat  <= 0;
        rd_chip_lat <= (others => '0');
        rd_addr_lat <= (others => '0');

      else
        rd_req_d <= rd_req;

        -- okuma sadece beam turu disinda kabul edilir; aksi halde araya giren
        -- bir okuma, LOAD oncesindeki "son yazma beam register'i olmali"
        -- kosulunu bozabilir (UG-2293 Figure 10)
        if rd_req = '1' and rd_req_d = '0' and tstate = T_IDLE then
          -- istek parametrelerini kilitle (aralik disi bus -> 0)
          if to_integer(unsigned(rd_bus)) < C_NUM_BUS then
            rd_bus_lat <= to_integer(unsigned(rd_bus));
          else
            rd_bus_lat <= 0;
          end if;
          rd_chip_lat <= rd_chip;
          rd_addr_lat <= rd_addr;
          rd_pend     <= '1';

        elsif rd_pend = '1' then
          bus_rd_req(rd_bus_lat) <= '1';
          rd_pend                <= '0';
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- AXI-Stream demux + tetik sekansi
  ------------------------------------------------------------------------------
  process (clk)
  begin
    if rising_edge(clk) then
      beam_done_r <= '0';
      load_done_r <= '0';
      trx_upd_r   <= '0';
      bram_we     <= (others => '0');

      if rst_n = '0' then
        tstate      <= T_IDLE;
        trig_d      <= '0';
        ltrig_d     <= '0';
        ltrig_pend  <= '0';
        tready_r    <= '0';
        beam_start  <= '0';
        load_glb    <= '0';
        load_cnt    <= (others => '0');
        trx_r       <= '0';          -- UG-2293: cip receive modda baslamali
        rx_tx_lat   <= '0';
        bus_idx     <= 0;
        word_in_bus <= (others => '0');
        total_cnt   <= (others => '0');
        axis_err_r  <= '0';

      else
        trig_d  <= trig;
        ltrig_d <= load_trig;

        -- load_trig kuyrugu: beam turu basladiktan sonra gelen her yukselen
        -- kenar kilitlenir; SPI yazma bitince tuketilir
        if ltrig_rise = '1' then
          ltrig_pend <= '1';
        end if;

        case tstate is

          --------------------------------------------------------------------
          when T_IDLE =>
            tready_r   <= '0';
            beam_start <= '0';
            if trig_rise = '1' and all_init = '1' then
              rx_tx_lat   <= rx_tx_sel;
              bus_idx     <= f_first_data_bus;
              word_in_bus <= (others => '0');
              total_cnt   <= (others => '0');
              axis_err_r  <= '0';
              tready_r    <= '1';
              ltrig_pend  <= '0';   -- onceki turdan kalan tetigi tasima
              tstate      <= T_AXIS;
            end if;

          --------------------------------------------------------------------
          -- Paketi bus RAM'lerine dagit
          when T_AXIS =>
            if accept = '1' then
              bram_we(bus_idx)   <= '1';
              bram_addr(bus_idx) <= std_logic_vector(word_in_bus);
              bram_data(bus_idx) <= s_axis_tdata;

              if total_cnt = C_TOTAL_WORDS - 1 then
                -- son kelime: TLAST bekleniyor
                if s_axis_tlast = '0' then
                  axis_err_r <= '1';
                end if;
                tready_r <= '0';
                tstate   <= T_SPI;
              else
                if s_axis_tlast = '1' then
                  -- paket erken bitti: hatayi isaretle ve yine de yaz
                  axis_err_r <= '1';
                  tready_r   <= '0';
                  tstate     <= T_SPI;
                end if;

                total_cnt <= total_cnt + 1;
                if word_in_bus = C_CHIPS_PER_BUS(bus_idx) * C_BEAM_WORDS_PER_CHIP - 1 then
                  word_in_bus <= (others => '0');
                  -- statik buslar pakette yer almaz: siradaki veri bus'ina atla
                  bus_idx <= f_next_data_bus(bus_idx);
                else
                  word_in_bus <= word_in_bus + 1;
                end if;
              end if;
            end if;

          --------------------------------------------------------------------
          -- Tum buslar kendi ciplerine yaziyor
          when T_SPI =>
            beam_start <= '1';
            if all_rdy = '1' then
              tstate <= T_WAIT_LOAD;
            end if;

          --------------------------------------------------------------------
          -- SPI yazma bitti. LOAD'i dizi genelinde es zamanli vermek icin
          -- disaridan load_trig bekleniyor (erken gelmisse kuyruktan alinir).
          -- beam_start bu sure boyunca '1' kalir; bu sayede buslar mesgul
          -- gorunur ve araya kullanici SPI okumasi giremez (UG-2293 Figure 10:
          -- LOAD oncesi son yazma beam ile ilgili bir register olmalidir).
          when T_WAIT_LOAD =>
            if ltrig_pend = '1' then
              ltrig_pend <= '0';
              load_glb   <= '1';
              load_cnt   <= (others => '0');
              tstate     <= T_LOAD_HI;
            end if;

          --------------------------------------------------------------------
          -- LOAD darbesi: yuksek yarim periyot (UG-2293 Table 2, >= 3.75 ns)
          when T_LOAD_HI =>
            if load_cnt = to_unsigned(C_LOAD_HALF - 1, 32) then
              load_glb <= '0';
              load_cnt <= (others => '0');
              tstate   <= T_LOAD_LO;
            else
              load_cnt <= load_cnt + 1;
            end if;

          --------------------------------------------------------------------
          -- Dusuk yarim periyot: toggle periyodunun tamamlanmasi garanti edilir
          when T_LOAD_LO =>
            if load_cnt = to_unsigned(C_LOAD_HALF - 1, 32) then
              load_done_r <= '1';
              load_cnt    <= (others => '0');
              tstate      <= T_TRX_DLY;
            else
              load_cnt <= load_cnt + 1;
            end if;

          --------------------------------------------------------------------
          -- TRX gecisi LOAD'dan sonra gelir (sira garantisi + bekleme)
          when T_TRX_DLY =>
            if load_cnt = to_unsigned(G_TRX_DELAY_CYCLES, 32) then
              tstate <= T_TRX;
            else
              load_cnt <= load_cnt + 1;
            end if;

          --------------------------------------------------------------------
          when T_TRX =>
            trx_r     <= rx_tx_lat;   -- '0' = receive, '1' = transmit
            trx_upd_r <= '1';
            tstate    <= T_DONE;

          --------------------------------------------------------------------
          when T_DONE =>
            beam_start <= '0';
            if all_rdy = '0' then
              beam_done_r <= '1';
              tstate      <= T_IDLE;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;

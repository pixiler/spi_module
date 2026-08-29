--------------------------------------------------------------------------------
-- tb_admv48281_vunit.vhd
--
-- admv48281_top icin VUnit testbench'i. 7 bus'in her birine bir
-- admv48281_ring_model baglanir (6 x 8 cip + 1 x 4 cip = 52 cip).
--
-- Test durumlari (her biri ayri simulasyonda kosar):
--   test_init_sequence      init tablosu + faz SRAM + NVM merge + LOAD toggle
--   test_rx_beam            RX direct beam (0x200-0x21F) yazma
--   test_tx_beam            TX direct beam (0x240-0x25F) yazma, RX bozulmamali
--   test_back_to_back_beams ard arda iki beam turu, ikincisi birincinin ustune
--   test_spi_read           ring uzerinden okuma (SRAM icin cift okuma dahil)
--   test_axis_short_packet  erken TLAST -> axis_err
--   test_load_trig_gating   load_trig gelmeden LOAD toggle edilmemeli
--   test_load_trig_early    SPI yazma bitmeden gelen load_trig kuyruga alinir
--   test_load_pulse_width   LOAD darbesi UG-2293 Table 2 minimumunu saglar
--   test_trx_sequence       TRX pini LOAD'dan sonra ve gecikmeyle degisir
--   test_static_bus         statik bus (bus 6): acilis beam'i yuklenir,
--                           trig'lerde yazilmaz, LOAD/TRX yine takip eder
--   test_rx_window_too_short okuma penceresi yetersizken veri sessizce kaymaz,
--                           short_err isaretlenir
--   test_nvm_timeout        NVM bit6 hic gelmezse init_err
--
-- Kosturma:  cd sim/vunit && python run.py
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library vunit_lib;
context vunit_lib.vunit_context;

library work;
use work.admv48281_pkg.all;
use work.admv48281_tb_pkg.all;

entity tb_admv48281_vunit is
  generic (
    runner_cfg : string;

    -- Simulasyon suresini kisaltmak icin NVM merge varsayilan olarak kucuktur;
    -- test_init_sequence'in "real_nvm" konfigurasyonu gercek degeri kullanir.
    G_NVM_BURST   : natural := 400;
    G_NVM_CLOCKS  : natural := 300;
    G_POLL_LIMIT  : natural := 2000;
    G_RING_DLY_NS : natural := 8;
    G_CLK_DIV_WR  : natural := 2;
    G_CLK_DIV_RD  : natural := 4;

    -- Sistem saati. TB saat periyodu bundan turetilir ve ayni deger DUT'a
    -- verilir; boylece LOAD darbe genisligi turetmesi ucdan uca sinanir.
    G_CLK_FREQ_HZ : natural := 100000000;

    G_RX_SETTLE_CYCLES : natural := 64;
    G_RX_FILTER_LEN    : natural := 3;

    -- LOAD darbe genisligi: 0 = UG-2293 Table 2'den turet
    G_LOAD_CYCLES      : natural := 0;
    -- LOAD -> TRX gecikmesi (100 MHz'de 100 clock = 1 us)
    G_TRX_DELAY_CYCLES : natural := 100
  );
end entity tb_admv48281_vunit;

architecture tb of tb_admv48281_vunit is

  constant C_CLK_PER : time := 1 sec / G_CLK_FREQ_HZ;

  signal clk    : std_logic := '0';
  signal rst_n  : std_logic := '0';
  signal enable : std_logic := '0';

  signal s_axis_tvalid : std_logic := '0';
  signal s_axis_tready : std_logic;
  signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
  signal s_axis_tlast  : std_logic := '0';

  signal trig      : std_logic := '0';
  signal rx_tx_sel : std_logic := '0';
  signal load_trig : std_logic := '0';
  signal trx_out   : std_logic_vector(C_NUM_BUS-1 downto 0);

  signal rd_req   : std_logic := '0';
  signal rd_bus   : std_logic_vector(2 downto 0) := (others => '0');
  signal rd_chip  : std_logic_vector(2 downto 0) := (others => '0');
  signal rd_addr  : std_logic_vector(13 downto 0) := (others => '0');
  signal rd_data  : std_logic_vector(7 downto 0);
  signal rd_valid : std_logic;
  signal rd_short_err : std_logic_vector(C_NUM_BUS-1 downto 0);

  signal init_done    : std_logic;
  signal init_err     : std_logic;
  signal busy         : std_logic;
  signal load_pending : std_logic;
  signal load_done    : std_logic;
  signal trx_updated  : std_logic;
  signal beam_done    : std_logic;
  signal axis_err     : std_logic;

  -- LOAD darbe genisligi ve LOAD -> TRX sirasini olcen monitor
  signal t_load_rise : time := 0 ns;
  signal t_load_fall : time := 0 ns;
  signal t_trx_edge  : time := 0 ns;
  signal load_width  : time := 0 ns;
  signal load_to_trx : time := 0 ns;
  signal load_pulses : natural := 0;

  signal spi_sclk_out : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_mosi     : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_cs_n     : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_load     : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_sclk_in  : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_miso     : std_logic_vector(C_NUM_BUS-1 downto 0);

  constant C_ALL_ZERO_TB : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '0');
  constant C_ALL_ONES_TB : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '1');

  -- bus basina CS dusme sayaci: statik busta trig sirasinda SPI islemi
  -- olmadigini kanitlamak icin
  signal cs_falls : t_natvec := (others => 0);

  signal peek_chip : natural := 0;
  signal peek_addr : std_logic_vector(15 downto 0) := (others => '0');
  signal peek_data : t_peek_data;
  signal load_tog  : t_natvec;

begin

  clk <= not clk after C_CLK_PER / 2;

  ------------------------------------------------------------------------------
  dut : entity work.admv48281_top
    generic map (
      G_CLK_FREQ_HZ      => G_CLK_FREQ_HZ,
      G_CLK_DIV_WR       => G_CLK_DIV_WR,
      G_CLK_DIV_RD       => G_CLK_DIV_RD,
      G_NVM_BURST        => G_NVM_BURST,
      G_LOAD_CYCLES      => G_LOAD_CYCLES,
      G_TRX_DELAY_CYCLES => G_TRX_DELAY_CYCLES,
      G_RESET_WAIT       => 20,
      G_POLL_LIMIT       => G_POLL_LIMIT,
      G_RX_SETTLE_CYCLES => G_RX_SETTLE_CYCLES,
      G_RX_FILTER_LEN    => G_RX_FILTER_LEN
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      enable        => enable,
      s_axis_tvalid => s_axis_tvalid,
      s_axis_tready => s_axis_tready,
      s_axis_tdata  => s_axis_tdata,
      s_axis_tlast  => s_axis_tlast,
      trig          => trig,
      rx_tx_sel     => rx_tx_sel,
      load_trig     => load_trig,
      trx_out       => trx_out,
      rd_req        => rd_req,
      rd_bus        => rd_bus,
      rd_chip       => rd_chip,
      rd_addr       => rd_addr,
      rd_data       => rd_data,
      rd_valid      => rd_valid,
      rd_short_err  => rd_short_err,
      init_done     => init_done,
      init_err      => init_err,
      busy          => busy,
      load_pending  => load_pending,
      load_done     => load_done,
      trx_updated   => trx_updated,
      beam_done     => beam_done,
      axis_err      => axis_err,
      spi_sclk_out  => spi_sclk_out,
      spi_mosi      => spi_mosi,
      spi_cs_n      => spi_cs_n,
      spi_load      => spi_load,
      spi_sclk_in   => spi_sclk_in,
      spi_miso      => spi_miso
    );

  ------------------------------------------------------------------------------
  gen_model : for b in 0 to C_NUM_BUS-1 generate
    u_model : entity work.admv48281_ring_model
      generic map (
        G_BUS_ID     => b,
        G_NVM_CLOCKS => G_NVM_CLOCKS,
        G_RING_DLY   => G_RING_DLY_NS * 1 ns
      )
      port map (
        sclk         => spi_sclk_out(b),
        mosi         => spi_mosi(b),
        cs_n         => spi_cs_n(b),
        load         => spi_load(b),
        sclk_ret     => spi_sclk_in(b),
        miso_ret     => spi_miso(b),
        peek_chip    => peek_chip,
        peek_addr    => peek_addr,
        peek_data    => peek_data(b),
        load_toggles => load_tog(b)
      );
  end generate gen_model;

  ------------------------------------------------------------------------------
  -- CS aktivite sayaclari
  ------------------------------------------------------------------------------
  gen_csmon : for b in 0 to C_NUM_BUS-1 generate
    process (spi_cs_n(b))
      variable n : natural := 0;
    begin
      if falling_edge(spi_cs_n(b)) then
        n := n + 1;
      end if;
      cs_falls(b) <= n;
    end process;
  end generate gen_csmon;

  ------------------------------------------------------------------------------
  -- LOAD darbe genisligini ve LOAD -> TRX gecikmesini olc (bus 0 uzerinden)
  ------------------------------------------------------------------------------
  monitor : process (spi_load(0), trx_out(0))
  begin
    if rising_edge(spi_load(0)) then
      t_load_rise <= now;
      load_pulses <= load_pulses + 1;
    end if;
    if falling_edge(spi_load(0)) then
      t_load_fall <= now;
      load_width  <= now - t_load_rise;
    end if;
    if trx_out(0)'event then
      t_trx_edge  <= now;
      load_to_trx <= now - t_load_fall;
    end if;
  end process monitor;

  ------------------------------------------------------------------------------
  main : process

    ----------------------------------------------------------------------------
    -- yardimcilar
    ----------------------------------------------------------------------------
    procedure reset_dut is
    begin
      rst_n  <= '0';
      enable <= '0';
      wait for 200 ns;
      wait until rising_edge(clk);
      rst_n <= '1';
      wait for 100 ns;
    end procedure;

    -- init sekansini calistir ve bitmesini bekle
    procedure run_init is
    begin
      wait until rising_edge(clk);
      enable <= '1';
      wait until (init_done = '1' or init_err = '1') and rising_edge(clk);
    end procedure;

    -- bir register'i tum buslarin tum ciplerinde kontrol et
    procedure chk_all(constant a   : in natural;
                      constant exp : in std_logic_vector(7 downto 0);
                      constant msg : in string) is
    begin
      for b in 0 to C_NUM_BUS-1 loop
        for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
          peek_chip <= c;
          peek_addr <= std_logic_vector(to_unsigned(a, 16));
          wait for 1 ns;
          check_equal(peek_data(b), exp,
                      msg & " bus=" & integer'image(b)
                      & " cip=" & integer'image(c)
                      & " adr=0x" & f_hex16(a));
        end loop;
      end loop;
    end procedure;

    -- direct beam blogunu kontrol et (sadece paketten beslenen buslar;
    -- statik buslarin blogu chk_static_beam ile kontrol edilir)
    procedure chk_beam(constant base : in natural;
                       constant p    : in natural;
                       constant msg  : in string) is
    begin
      for b in 0 to C_NUM_BUS-1 loop
        next when not C_BEAM_ON_TRIG(b);
        for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
          for j in 0 to C_BEAM_BYTES_PER_CHIP-1 loop
            peek_chip <= c;
            peek_addr <= std_logic_vector(to_unsigned(base + j, 16));
            wait for 1 ns;
            check_equal(peek_data(b), f_exp(b, c, j, p),
                        msg & " bus=" & integer'image(b)
                        & " cip=" & integer'image(c)
                        & " byte=" & integer'image(j));
          end loop;
        end loop;
      end loop;
    end procedure;

    -- statik buslarin direct beam bloklari acilis tablolariyla eslesmeli
    procedure chk_static_beam(constant msg : in string) is
    begin
      for b in 0 to C_NUM_BUS-1 loop
        next when C_BEAM_ON_TRIG(b);
        for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
          for j in 0 to C_BEAM_BYTES_PER_CHIP-1 loop
            peek_chip <= c;
            peek_addr <= std_logic_vector(to_unsigned(16#200# + j, 16));
            wait for 1 ns;
            check_equal(peek_data(b), C_STATIC_BEAM_RX(c)(j),
                        msg & " statik RX bus=" & integer'image(b)
                        & " cip=" & integer'image(c)
                        & " byte=" & integer'image(j));
            peek_addr <= std_logic_vector(to_unsigned(16#240# + j, 16));
            wait for 1 ns;
            check_equal(peek_data(b), C_STATIC_BEAM_TX(c)(j),
                        msg & " statik TX bus=" & integer'image(b)
                        & " cip=" & integer'image(c)
                        & " byte=" & integer'image(j));
          end loop;
        end loop;
      end loop;
    end procedure;

    -- disaridan LOAD tetigi ver (1 clock darbe)
    procedure pulse_load_trig is
    begin
      wait until rising_edge(clk);
      load_trig <= '1';
      wait until rising_edge(clk);
      load_trig <= '0';
    end procedure;

    -- bir beam turu: trig -> paket -> (SPI yazma) -> load_trig -> LOAD -> TRX
    -- early_load = true ise load_trig paketten hemen sonra, SPI yazma daha
    -- bitmeden gonderilir (kuyruk davranisi sinanir)
    procedure do_beam(constant sel        : in std_logic;
                      constant p          : in natural;
                      constant early_load : in boolean) is
    begin
      rx_tx_sel <= sel;
      wait until rising_edge(clk);
      trig <= '1';
      wait until rising_edge(clk);
      trig <= '0';

      send_beam_packet(clk, s_axis_tready, s_axis_tvalid,
                       s_axis_tdata, s_axis_tlast, p, 0);

      if early_load then
        pulse_load_trig;                                  -- SPI yazma surerken
      else
        wait until load_pending = '1' and rising_edge(clk);
        pulse_load_trig;
      end if;

      wait until beam_done = '1' and rising_edge(clk);
    end procedure;

    procedure do_beam(constant sel : in std_logic;
                      constant p   : in natural) is
    begin
      do_beam(sel, p, false);
    end procedure;

    -- SPI uzerinden tek register okumasi
    procedure chk_read(constant b   : in natural;
                       constant c   : in natural;
                       constant a   : in natural;
                       constant exp : in std_logic_vector(7 downto 0);
                       constant msg : in string) is
    begin
      rd_bus  <= std_logic_vector(to_unsigned(b, 3));
      rd_chip <= std_logic_vector(to_unsigned(c, 3));
      rd_addr <= std_logic_vector(to_unsigned(a, 14));
      wait until rising_edge(clk);
      rd_req <= '1';
      wait until rising_edge(clk);
      rd_req <= '0';

      wait until rd_valid = '1' and rising_edge(clk);

      check_equal(rd_data, exp,
                  msg & " bus=" & integer'image(b)
                  & " cip=" & integer'image(c)
                  & " adr=0x" & f_hex16(a));
    end procedure;

    variable tog : t_natvec;
    variable csb : t_natvec;

  begin
    test_runner_setup(runner, runner_cfg);
    show(get_logger(default_checker), display_handler, pass);

    while test_suite loop

      --------------------------------------------------------------------------
      if run("test_init_sequence") then
        reset_dut;
        run_init;

        check_equal(init_err, '0', "init_err aktif olmamali");
        check_equal(init_done, '1', "init_done gelmedi");
        info("init tamamlandi, t = " & time'image(now));

        -- init tablosundan ornekler (broadcast ile tum ciplere gitmis olmali)
        chk_all(16#000#,  x"BD", "SPI_CONFIG");
        chk_all(16#07E#,  x"54", "NVM_RESET");
        chk_all(16#0C2#,  x"F7", "CM_TC_CTRL");
        chk_all(16#2FF#,  x"80", "TEMP_COMP_BYPASS");
        chk_all(16#1021#, x"00", "POWER_DOWN_BLOCKS_2");
        chk_all(16#1075#, x"33", "BIAS_1075");
        chk_all(16#0C0#,  x"00", "NVM_CTRL Band 0");
        chk_all(16#0C1#,  x"0F", "MANUAL_BEAM_BYPASS");
        chk_all(16#289#,  x"00", "TX_DIRECT_DVGA1_GAIN_H");
        chk_all(16#05F#,  x"00", "RAM_FILL_LD merge kapandi");

        -- global faz SRAM (UG-2293 Table 11)
        for j in 0 to 127 loop
          chk_all(16#480# + j, C_PHASE_STREAM(j), "RX faz SRAM");
          chk_all(16#580# + j, C_PHASE_STREAM(j), "TX faz SRAM");
        end loop;

        -- init sirasinda LOAD toggle edilmis olmali (0x2FF, 0x1021, 0x289;
        -- statik busta ek olarak acilis beam yuklemesi)
        for b in 0 to C_NUM_BUS-1 loop
          check(load_tog(b) >= 3,
                "bus " & integer'image(b) & " init LOAD toggle sayisi = "
                & integer'image(load_tog(b)) & ", >=3 bekleniyor");
        end loop;

        -- statik bus acilis beam'i init icinde yazilip yuklenmis olmali
        chk_static_beam("init sonrasi");

      --------------------------------------------------------------------------
      elsif run("test_rx_beam") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        for b in 0 to C_NUM_BUS-1 loop
          tog(b) := load_tog(b);
        end loop;

        do_beam('0', 0);
        check_equal(axis_err, '0', "axis_err aktif olmamali");
        chk_beam(16#200#, 0, "RX direct beam");

        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b) + 1,
                      "bus " & integer'image(b) & " beam basina bir LOAD toggle");
        end loop;

      --------------------------------------------------------------------------
      elsif run("test_tx_beam") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        do_beam('0', 0);          -- once RX yaz

        -- LOAD sayacini TX turundan hemen once ornekle ki artis dogrudan
        -- TX beam'e atfedilebilsin
        for b in 0 to C_NUM_BUS-1 loop
          tog(b) := load_tog(b);
        end loop;

        do_beam('1', 1);          -- sonra TX

        check_equal(axis_err, '0', "axis_err aktif olmamali");
        chk_beam(16#240#, 1, "TX direct beam");
        -- TX turu RX blogunu bozmamali (ayri register bloklari)
        chk_beam(16#200#, 0, "RX blogu TX turunda korunmali");

        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b) + 1,
                      "bus " & integer'image(b) & " TX beam basina bir LOAD toggle");
        end loop;

      --------------------------------------------------------------------------
      elsif run("test_back_to_back_beams") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        for b in 0 to C_NUM_BUS-1 loop
          tog(b) := load_tog(b);
        end loop;

        do_beam('0', 0);
        chk_beam(16#200#, 0, "birinci RX beam");

        do_beam('0', 2);          -- ayni blogu farkli veriyle tekrar yaz
        chk_beam(16#200#, 2, "ikinci RX beam ustune yazmali");

        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b) + 2,
                      "bus " & integer'image(b) & " iki beam = iki LOAD toggle");
        end loop;

      --------------------------------------------------------------------------
      elsif run("test_spi_read") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        do_beam('0', 0);
        do_beam('1', 1);

        -- regular registerlar (tek okuma)
        chk_read(0, 0, 16#1075#, x"33", "bias 0x1075");
        chk_read(3, 5, 16#1075#, x"33", "bias 0x1075");
        chk_read(3, 5, 16#0C1#,  x"0F", "MANUAL_BEAM_BYPASS");
        chk_read(6, 3, 16#000#,  x"BD", "SPI_CONFIG");

        -- SRAM bolgesi: komut iki kez gonderilir (UG-2293)
        chk_read(1, 2, 16#480#, C_PHASE_STREAM(0),  "faz SRAM Q[0]");
        chk_read(1, 2, 16#4C0#, C_PHASE_STREAM(64), "faz SRAM I[0]");
        chk_read(4, 7, 16#5BF#, C_PHASE_STREAM(63), "TX faz SRAM Q[63]");

        -- direct beam geri okuma
        chk_read(2, 4, 16#240#, f_exp(2, 4, 0, 1),  "TX direct beam");
        chk_read(2, 4, 16#21F#, f_exp(2, 4, 31, 0), "RX direct beam");

      --------------------------------------------------------------------------
      elsif run("test_axis_short_packet") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        rx_tx_sel <= '0';
        wait until rising_edge(clk);
        trig <= '1';
        wait until rising_edge(clk);
        trig <= '0';

        -- 416 yerine 100 kelime gonderip TLAST ile erken bitir
        send_beam_packet(clk, s_axis_tready, s_axis_tvalid,
                         s_axis_tdata, s_axis_tlast, 0, 100);

        wait until load_pending = '1' and rising_edge(clk);
        pulse_load_trig;
        wait until beam_done = '1' and rising_edge(clk);
        check_equal(axis_err, '1', "erken TLAST axis_err vermeli");

        -- modul kilitlenmemeli: sonraki tam paket dogru yazilmali
        do_beam('0', 3);
        chk_beam(16#200#, 3, "kisa paket sonrasi tam paket");

      --------------------------------------------------------------------------
      elsif run("test_load_trig_gating") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        for b in 0 to C_NUM_BUS-1 loop
          tog(b) := load_tog(b);
        end loop;

        -- trig ver ve paketi gonder, ama load_trig VERME
        rx_tx_sel <= '0';
        wait until rising_edge(clk);
        trig <= '1';
        wait until rising_edge(clk);
        trig <= '0';
        send_beam_packet(clk, s_axis_tready, s_axis_tvalid,
                         s_axis_tdata, s_axis_tlast, 0, 0);

        wait until load_pending = '1' and rising_edge(clk);
        info("SPI yazma bitti, load_pending = 1, t = " & time'image(now));

        -- load_trig gelmeden LOAD toggle edilmemeli ve beam_done olmamali
        for i in 0 to 999 loop
          wait until rising_edge(clk);
          check_equal(beam_done, '0', "load_trig yokken beam_done gelmemeli");
        end loop;

        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b),
                      "bus " & integer'image(b)
                      & " load_trig yokken LOAD toggle edilmemeli");
        end loop;
        check_equal(trx_out(0), '0', "load_trig yokken TRX degismemeli");

        -- simdi load_trig ver
        pulse_load_trig;
        wait until beam_done = '1' and rising_edge(clk);

        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b) + 1,
                      "bus " & integer'image(b) & " load_trig sonrasi bir LOAD");
        end loop;

        -- veri yine dogru yazilmis olmali
        chk_beam(16#200#, 0, "gecikmeli LOAD sonrasi RX beam");

      --------------------------------------------------------------------------
      elsif run("test_load_trig_early") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        for b in 0 to C_NUM_BUS-1 loop
          tog(b) := load_tog(b);
        end loop;

        -- load_trig'i SPI yazma daha bitmeden gonder: kuyrukta tutulmali
        do_beam('0', 0, true);

        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b) + 1,
                      "bus " & integer'image(b)
                      & " erken load_trig kuyruga alinip bir kez uygulanmali");
        end loop;
        chk_beam(16#200#, 0, "erken load_trig ile RX beam");

      --------------------------------------------------------------------------
      elsif run("test_load_pulse_width") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        do_beam('0', 0);

        -- UG-2293 Table 2: LOAD toggle periyodu (tHIGH + tLOW) >= 7.5 ns,
        -- dolayisiyla yuksek yari >= 3.75 ns
        info("f_clk = " & integer'image(G_CLK_FREQ_HZ / 1000000) & " MHz, "
             & "saat periyodu = " & time'image(C_CLK_PER) & ", "
             & "olculen LOAD darbe genisligi = " & time'image(load_width));
        check(load_pulses > 0, "LOAD hic toggle edilmedi");
        check(load_width >= 3750 ps,
              "LOAD yuksek suresi " & time'image(load_width)
              & ", UG-2293 Table 2 minimumu 3.75 ns");
        -- darbe gereksiz genis de olmamali: spec minimumunu saglayan en kucuk
        -- clock sayisi kadar olmali
        check(load_width < 3750 ps + C_CLK_PER,
              "LOAD darbesi gereginden genis: " & time'image(load_width)
              & " (spec minimumu saglayan en kucuk deger bekleniyor)");

      --------------------------------------------------------------------------
      elsif run("test_trx_sequence") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        -- acilista receive modda olmali (UG-2293 TRX_x pin aciklamasi)
        check_equal(trx_out, C_ALL_ZERO_TB, "acilista TRX = receive olmali");

        -- RX beam: TRX '0' kalmali
        do_beam('0', 0);
        check_equal(trx_out, C_ALL_ZERO_TB, "RX beam sonrasi TRX = 0");

        -- TX beam: TRX '1' olmali, LOAD'dan SONRA ve gecikmeyle
        do_beam('1', 1);
        check_equal(trx_out, C_ALL_ONES_TB, "TX beam sonrasi TRX = 1");

        info("olculen LOAD -> TRX gecikmesi = " & time'image(load_to_trx));
        check(t_trx_edge > t_load_fall,
              "TRX gecisi LOAD darbesinden SONRA olmali");
        check(load_to_trx >= (G_TRX_DELAY_CYCLES - 1) * C_CLK_PER,
              "LOAD -> TRX gecikmesi " & time'image(load_to_trx)
              & ", beklenen >= "
              & time'image((G_TRX_DELAY_CYCLES - 1) * C_CLK_PER));

        -- geri RX'e don
        do_beam('0', 2);
        check_equal(trx_out, C_ALL_ZERO_TB, "tekrar RX beam sonrasi TRX = 0");
        chk_beam(16#200#, 2, "RX'e donusteki beam verisi");

      --------------------------------------------------------------------------
      elsif run("test_static_bus") then
        reset_dut;
        run_init;
        check_equal(init_done, '1', "init_done gelmedi");

        -- acilis beam'i yuklenmis olmali (model sentinel 0xAA ile basladigi
        -- icin bu kontrol yazmanin GERCEKTEN yapildigini kanitlar)
        chk_static_beam("init sonrasi");

        -- statik bus init'te bir ek LOAD yapmali (acilis beam yuklemesi)
        for b in 0 to C_NUM_BUS-1 loop
          if not C_BEAM_ON_TRIG(b) then
            check_equal(load_tog(b), load_tog(f_first_data_bus) + 1,
                        "statik bus " & integer'image(b)
                        & " init'te acilis beam LOAD'i yapmali");
          end if;
        end loop;

        for b in 0 to C_NUM_BUS-1 loop
          tog(b) := load_tog(b);
          csb(b) := cs_falls(b);
        end loop;

        -- RX + TX beam turlari: statik bus'a SPI yazilmamali
        do_beam('0', 0);
        do_beam('1', 1);

        -- statik busta trig sirasinda hicbir SPI islemi (CS dusmesi) olmamali
        for b in 0 to C_NUM_BUS-1 loop
          if not C_BEAM_ON_TRIG(b) then
            check_equal(cs_falls(b), csb(b),
                        "statik bus " & integer'image(b)
                        & " trig'lerde SPI islemi yapmamali");
          end if;
        end loop;

        -- veri buslari paket verisini almis olmali
        chk_beam(16#200#, 0, "RX beam (veri buslari)");
        chk_beam(16#240#, 1, "TX beam (veri buslari)");

        -- statik bus'in beam bloklari DEGISMEMIS olmali
        chk_static_beam("iki beam turu sonrasi");

        -- senkron LOAD statik bus dahil tum hatlara gider (dizi lockstep)
        for b in 0 to C_NUM_BUS-1 loop
          check_equal(load_tog(b), tog(b) + 2,
                      "bus " & integer'image(b)
                      & " iki beam turu = iki senkron LOAD");
        end loop;

        -- TRX pini statik busta da rx_tx_sel'i takip etmeli
        check_equal(trx_out, C_ALL_ONES_TB,
                    "TX turu sonrasi TRX tum buslarda (statik dahil) = 1");
        do_beam('0', 2);
        check_equal(trx_out, C_ALL_ZERO_TB,
                    "RX turu sonrasi TRX tum buslarda (statik dahil) = 0");

      --------------------------------------------------------------------------
      -- Donen CLK_OUT'un son kenari okuma penceresinden sonra gelirse, veri
      -- SESSIZCE KAYMAMALI; short_err ile isaretlenmeli. Donanimda gorulen
      -- "okumada bit kaymasi" belirtisinin teshis yoludur.
      elsif run("test_rx_window_too_short") then
        reset_dut;
        wait until rising_edge(clk);
        enable <= '1';

        -- init NVM polling'i okuma yapar; pencere yetersizse tamamlanamaz
        for i in 0 to 400000 loop
          wait until rising_edge(clk);
          exit when init_done = '1' or init_err = '1'
                 or rd_short_err /= C_ALL_ZERO_TB;
        end loop;

        check(rd_short_err /= C_ALL_ZERO_TB,
              "yetersiz okuma penceresinde short_err bekleniyor "
              & "(sessiz bit kaymasi yerine)");
        check_equal(init_done, '0',
                    "eksik bit yakalanmisken init tamamlanmis gorunmemeli");
        info("short_err dogru sekilde isaretlendi, t = " & time'image(now));

      --------------------------------------------------------------------------
      elsif run("test_nvm_timeout") then
        -- G_NVM_CLOCKS ulasilamaz seviyede, G_POLL_LIMIT kucuk
        reset_dut;
        run_init;

        check_equal(init_err, '1', "NVM bit6 gelmezse init_err beklenir");
        check_equal(init_done, '0', "init_done gelmemeli");

      end if;

    end loop;

    test_runner_cleanup(runner);
  end process main;

  test_runner_watchdog(runner, 50 ms);

end architecture tb;

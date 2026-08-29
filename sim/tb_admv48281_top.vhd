--------------------------------------------------------------------------------
-- tb_admv48281_top.vhd
--
-- admv48281_top icin self-checking testbench. 7 bus'in her birine bir
-- admv48281_ring_model baglanir (6 x 8 cip + 1 x 4 cip = 52 cip).
--
-- Dogrulanan sekanslar:
--   1) Init tablosu tum ciplere broadcast ile yazildi mi
--   2) Global faz SRAM (0x480-0x4FF, 0x580-0x5FF) Table 11 ile birebir mi
--   3) NVM merge sekansi tamamlandi mi (0x05F 1 -> 0 ve 0x01A bit6 polling)
--   4) LOAD pini init sirasinda toggle edildi mi
--   5) RX direct beam (0x200-0x21F) AXI-Stream verisiyle byte-byte esit mi
--   6) TX direct beam (0x240-0x25F) ikinci tetikte dogru yazildi mi
--   7) Her beam turundan sonra LOAD tum buslarda toggle edildi mi
--   8) Statik bus (bus 6): acilis beam tablolari yuklendi mi, trig'lerde
--      degismeden kaldi mi
--
-- NVM merge clock sayisi gercek degerdedir (10500 uretilir, model 10354 bekler);
-- bu adim tek basina yaklasik 420 us simulasyon suresi alir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.admv48281_pkg.all;
use work.admv48281_tb_pkg.all;

entity tb_admv48281_top is
end entity tb_admv48281_top;

architecture sim of tb_admv48281_top is

  constant C_CLK_PER : time := 10 ns;   -- 100 MHz

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

  constant C_ZERO_BUS : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '0');
  constant C_ONES_BUS : std_logic_vector(C_NUM_BUS-1 downto 0) := (others => '1');

  signal spi_sclk_out : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_mosi     : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_cs_n     : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_load     : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_sclk_in  : std_logic_vector(C_NUM_BUS-1 downto 0);
  signal spi_miso     : std_logic_vector(C_NUM_BUS-1 downto 0);

  -- t_peek_data / t_natvec, f_exp, f_hex, f_hex16 ve send_beam_packet
  -- admv48281_tb_pkg icinde; VUnit testbench'i ile paylasilir.
  signal peek_chip : natural := 0;
  signal peek_addr : std_logic_vector(15 downto 0) := (others => '0');
  signal peek_data : t_peek_data;
  signal load_tog  : t_natvec;

  signal sim_done : boolean := false;

begin

  ------------------------------------------------------------------------------
  clk <= not clk after C_CLK_PER / 2 when not sim_done else '0';

  ------------------------------------------------------------------------------
  -- DUT
  ------------------------------------------------------------------------------
  dut : entity work.admv48281_top
    generic map (
      G_CLK_DIV_WR       => 2,     -- sclk = 25 MHz
      G_CLK_DIV_RD       => 4,     -- okuma 12.5 MHz
      G_NVM_BURST        => 10500, -- UG-2293: en az 10354
      G_LOAD_CYCLES      => 0,     -- UG-2293 Table 2'den turet
      G_TRX_DELAY_CYCLES => 100,   -- 100 MHz'de 1 us
      G_RESET_WAIT       => 20,
      G_POLL_LIMIT       => 2000,
      G_RX_SETTLE_CYCLES => 64
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
  -- Her bus icin bir ring modeli
  ------------------------------------------------------------------------------
  gen_model : for b in 0 to C_NUM_BUS-1 generate
    u_model : entity work.admv48281_ring_model
      generic map (
        G_BUS_ID     => b,
        G_NVM_CLOCKS => 10354,
        G_RING_DLY   => 8 ns
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
  -- Uyarici + kontrol
  ------------------------------------------------------------------------------
  stim : process

    variable nerr : natural := 0;

    -- bir register'i tum buslarda kontrol et
    procedure chk_all(a : natural; exp : std_logic_vector(7 downto 0);
                      msg : string) is
    begin
      for b in 0 to C_NUM_BUS-1 loop
        for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
          peek_chip <= c;
          peek_addr <= std_logic_vector(to_unsigned(a, 16));
          wait for 1 ns;
          if peek_data(b) /= exp then
            report "HATA " & msg & ": bus=" & integer'image(b)
                   & " cip=" & integer'image(c)
                   & " adr=0x" & f_hex16(a)
                   & " beklenen=0x" & f_hex(exp)
                   & " okunan=0x" & f_hex(peek_data(b))
              severity error;
            nerr := nerr + 1;
          end if;
        end loop;
      end loop;
    end procedure;

    -- direct beam blogunu kontrol et (sadece paketten beslenen buslar)
    procedure chk_beam(base : natural; p : natural) is
    begin
      for b in 0 to C_NUM_BUS-1 loop
        next when not C_BEAM_ON_TRIG(b);
        for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
          for j in 0 to C_BEAM_BYTES_PER_CHIP-1 loop
            peek_chip <= c;
            peek_addr <= std_logic_vector(to_unsigned(base + j, 16));
            wait for 1 ns;
            if peek_data(b) /= f_exp(b, c, j, p) then
              report "HATA beam: bus=" & integer'image(b)
                     & " cip=" & integer'image(c)
                     & " byte=" & integer'image(j)
                     & " beklenen=0x" & f_hex(f_exp(b, c, j, p))
                     & " okunan=0x" & f_hex(peek_data(b))
                severity error;
              nerr := nerr + 1;
            end if;
          end loop;
        end loop;
      end loop;
    end procedure;


    -- statik buslarin beam bloklari acilis tablolariyla eslesmeli
    procedure chk_static_beam(msg : string) is
    begin
      for b in 0 to C_NUM_BUS-1 loop
        next when C_BEAM_ON_TRIG(b);
        for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
          for j in 0 to C_BEAM_BYTES_PER_CHIP-1 loop
            peek_chip <= c;
            peek_addr <= std_logic_vector(to_unsigned(16#200# + j, 16));
            wait for 1 ns;
            if peek_data(b) /= C_STATIC_BEAM_RX(c)(j) then
              report "HATA statik RX " & msg & ": bus=" & integer'image(b)
                     & " cip=" & integer'image(c) & " byte=" & integer'image(j)
                severity error;
              nerr := nerr + 1;
            end if;
            peek_addr <= std_logic_vector(to_unsigned(16#240# + j, 16));
            wait for 1 ns;
            if peek_data(b) /= C_STATIC_BEAM_TX(c)(j) then
              report "HATA statik TX " & msg & ": bus=" & integer'image(b)
                     & " cip=" & integer'image(c) & " byte=" & integer'image(j)
                severity error;
              nerr := nerr + 1;
            end if;
          end loop;
        end loop;
      end loop;
    end procedure;

    -- SPI uzerinden okuma yap ve beklenen degerle karsilastir
    procedure chk_read(b : natural; c : natural; a : natural;
                       exp : std_logic_vector(7 downto 0); msg : string) is
    begin
      rd_bus  <= std_logic_vector(to_unsigned(b, 3));
      rd_chip <= std_logic_vector(to_unsigned(c, 3));
      rd_addr <= std_logic_vector(to_unsigned(a, 14));
      wait until rising_edge(clk);
      rd_req <= '1';
      wait until rising_edge(clk);
      rd_req <= '0';

      for i in 0 to 100000 loop
        wait until rising_edge(clk);
        exit when rd_valid = '1';
        if i = 100000 then
          report "HATA okuma zaman asimi: " & msg severity error;
          nerr := nerr + 1;
        end if;
      end loop;

      if rd_data /= exp then
        report "HATA okuma " & msg & ": bus=" & integer'image(b)
               & " cip=" & integer'image(c)
               & " adr=0x" & f_hex16(a)
               & " beklenen=0x" & f_hex(exp)
               & " okunan=0x" & f_hex(rd_data)
          severity error;
        nerr := nerr + 1;
      end if;
    end procedure;

    variable tog_before : t_natvec;

  begin
    ----------------------------------------------------------------------------
    report "ADMV48281 testbench basliyor";
    rst_n  <= '0';
    enable <= '0';
    wait for 200 ns;
    wait until rising_edge(clk);
    rst_n <= '1';
    wait for 100 ns;

    ----------------------------------------------------------------------------
    -- 1) Init + faz SRAM + NVM kalibrasyon
    ----------------------------------------------------------------------------
    wait until rising_edge(clk);
    enable <= '1';

    for i in 0 to 2000000 loop
      wait until rising_edge(clk);
      exit when init_done = '1' or init_err = '1';
      if i = 2000000 then
        report "HATA: init_done zaman asimi" severity error;
        nerr := nerr + 1;
      end if;
    end loop;

    if init_err = '1' then
      report "HATA: init_err aktif (NVM polling basarisiz)" severity error;
      nerr := nerr + 1;
    end if;

    report "init tamamlandi, t = " & time'image(now);

    -- init tablosundan ornekler
    chk_all(16#000#,  x"BD", "SPI_CONFIG");
    chk_all(16#07E#,  x"54", "NVM_RESET");
    chk_all(16#0C2#,  x"F7", "CM_TC_CTRL");
    chk_all(16#2FF#,  x"80", "TEMP_COMP_BYPASS");
    chk_all(16#1021#, x"00", "POWER_DOWN_BLOCKS_2");
    chk_all(16#1075#, x"33", "BIAS_1075");
    chk_all(16#0C0#,  x"00", "NVM_CTRL (Band 0)");
    chk_all(16#0C1#,  x"0F", "MANUAL_BEAM_BYPASS");
    chk_all(16#289#,  x"00", "TX_DIRECT_DVGA1_GAIN_H");
    chk_all(16#05F#,  x"00", "RAM_FILL_LD (merge kapandi)");

    -- 2) Global faz SRAM (Table 11)
    for j in 0 to 127 loop
      chk_all(16#480# + j, C_PHASE_STREAM(j), "RX faz SRAM");
      chk_all(16#580# + j, C_PHASE_STREAM(j), "TX faz SRAM");
    end loop;
    report "faz SRAM dogrulandi";

    -- statik bus acilis beam'i init icinde yuklenmis olmali
    chk_static_beam("init");

    -- 4) LOAD init sirasinda toggle edilmis olmali
    for b in 0 to C_NUM_BUS-1 loop
      if load_tog(b) < 3 then
        report "HATA: bus " & integer'image(b) & " init LOAD toggle sayisi = "
               & integer'image(load_tog(b)) & " (>=3 bekleniyor)"
          severity error;
        nerr := nerr + 1;
      end if;
      tog_before(b) := load_tog(b);
    end loop;

    ----------------------------------------------------------------------------
    -- 5) RX direct beam
    ----------------------------------------------------------------------------
    rx_tx_sel <= '0';
    wait until rising_edge(clk);
    trig <= '1';
    wait until rising_edge(clk);
    trig <= '0';

    send_beam_packet(clk, s_axis_tready, s_axis_tvalid,
                     s_axis_tdata, s_axis_tlast, 0, 0);

    -- SPI yazma bitince disaridan LOAD tetigi ver
    wait until load_pending = '1' and rising_edge(clk);
    if trx_out /= C_ZERO_BUS then
      report "HATA: LOAD oncesi TRX zaten degismis" severity error;
      nerr := nerr + 1;
    end if;
    load_trig <= '1';
    wait until rising_edge(clk);
    load_trig <= '0';

    for i in 0 to 2000000 loop
      wait until rising_edge(clk);
      exit when beam_done = '1';
      if i = 2000000 then
        report "HATA: RX beam_done zaman asimi" severity error;
        nerr := nerr + 1;
      end if;
    end loop;
    report "RX beam yazildi, t = " & time'image(now);

    if trx_out /= C_ZERO_BUS then
      report "HATA: RX beam sonrasi TRX receive (0) olmali" severity error;
      nerr := nerr + 1;
    end if;

    if axis_err = '1' then
      report "HATA: axis_err aktif (RX paketi)" severity error;
      nerr := nerr + 1;
    end if;

    chk_beam(16#200#, 0);

    for b in 0 to C_NUM_BUS-1 loop
      if load_tog(b) /= tog_before(b) + 1 then
        report "HATA: bus " & integer'image(b)
               & " RX beam sonrasi LOAD toggle sayisi = "
               & integer'image(load_tog(b)) & " (bir artis bekleniyor)"
          severity error;
        nerr := nerr + 1;
      end if;
      tog_before(b) := load_tog(b);
    end loop;

    ----------------------------------------------------------------------------
    -- 6) TX direct beam
    ----------------------------------------------------------------------------
    rx_tx_sel <= '1';
    wait until rising_edge(clk);
    trig <= '1';
    wait until rising_edge(clk);
    trig <= '0';

    send_beam_packet(clk, s_axis_tready, s_axis_tvalid,
                     s_axis_tdata, s_axis_tlast, 1, 0);

    wait until load_pending = '1' and rising_edge(clk);
    load_trig <= '1';
    wait until rising_edge(clk);
    load_trig <= '0';

    for i in 0 to 2000000 loop
      wait until rising_edge(clk);
      exit when beam_done = '1';
      if i = 2000000 then
        report "HATA: TX beam_done zaman asimi" severity error;
        nerr := nerr + 1;
      end if;
    end loop;
    report "TX beam yazildi, t = " & time'image(now);

    if trx_out /= C_ONES_BUS then
      report "HATA: TX beam sonrasi TRX transmit (1) olmali" severity error;
      nerr := nerr + 1;
    end if;

    if axis_err = '1' then
      report "HATA: axis_err aktif (TX paketi)" severity error;
      nerr := nerr + 1;
    end if;

    chk_beam(16#240#, 1);

    -- RX blogu TX turunda bozulmamis olmali
    chk_beam(16#200#, 0);

    -- statik bus iki beam turunda da DEGISMEMIS olmali
    chk_static_beam("beam turlari sonrasi");

    for b in 0 to C_NUM_BUS-1 loop
      if load_tog(b) /= tog_before(b) + 1 then
        report "HATA: bus " & integer'image(b)
               & " TX beam sonrasi LOAD toggle sayisi = "
               & integer'image(load_tog(b))
          severity error;
        nerr := nerr + 1;
      end if;
    end loop;

    ----------------------------------------------------------------------------
    -- 8) Ring uzerinden SPI okumasi (spi_slave yolu)
    ----------------------------------------------------------------------------
    chk_read(0, 0, 16#1075#, x"33", "bias 0x1075 bus0 cip0");
    chk_read(3, 5, 16#1075#, x"33", "bias 0x1075 bus3 cip5");
    chk_read(3, 5, 16#0C1#,  x"0F", "MANUAL_BEAM_BYPASS bus3 cip5");
    chk_read(6, 3, 16#000#,  x"BD", "SPI_CONFIG bus6 cip3");
    -- SRAM bolgesi: komut iki kez gonderilir (UG-2293)
    chk_read(1, 2, 16#480#,  C_PHASE_STREAM(0),  "faz SRAM 0x480 bus1 cip2");
    chk_read(1, 2, 16#4C0#,  C_PHASE_STREAM(64), "faz SRAM 0x4C0 bus1 cip2");
    -- direct beam geri okuma (son yazilan TX turu)
    chk_read(2, 4, 16#240#,  f_exp(2, 4, 0, 1),  "TX direct beam 0x240 bus2 cip4");
    chk_read(2, 4, 16#21F#,  f_exp(2, 4, 31, 0), "RX direct beam 0x21F bus2 cip4");
    report "SPI okuma yolu dogrulandi";

    ----------------------------------------------------------------------------
    if nerr = 0 then
      report "TUM TESTLER BASARILI (t = " & time'image(now) & ")" severity note;
    else
      report "TEST BASARISIZ: " & integer'image(nerr) & " hata" severity failure;
    end if;

    sim_done <= true;
    wait;
  end process stim;

end architecture sim;

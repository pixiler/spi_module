--------------------------------------------------------------------------------
-- tb_spi_ddr_top.vhd
-- spi_ddr_top icin self-checking testbench (ADMV48281 davranis modeli ile).
--
-- Modeller:
--  * AXI slave (DDR)   : okunan adrese gore deterministik veri dondurur
--                        (data = addr xor 0xA5A5A5A5).
--  * ADMV48281 modeli  : 24 bit header + N byte'lik frame'leri cozer;
--                        yazmalari register dosyasina isler (ascending),
--                        okumalarda SDO verisini falling edge'de kaydirir.
--                        0x05F(0)=1 iken SCLK sayar; >=10354 pulse sonrasi
--                        0x01A okumasi 0x40 dondurur (NVM merge tamam).
--  * sclk_in           : sclk_out'un 12 ns geciktirilmis kopyasi.
--
-- Test adimlari:
--  1) init_done beklenir; init registerlari, faz SRAM (0x480-0x5FF) icerigi
--     ve NVM sekansi (0x05F=0) dogrulanir; LOAD sayisi = 2 kontrol edilir.
--  2) Ilk periyodik beam stream'i: 0x200-0x21F'e yazilan 32 byte, DDR'daki
--     beklenen elemanla karsilastirilir; LOAD toggle dogrulanir.
--  3) SPI okuma: 0x00E registeri okunur (model 0x5A dondurur).
--  4) Indisler degistirilir, ikinci stream yeni eleman adresiyle dogrulanir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.spi_pkg.all;

entity tb_spi_ddr_top is
end entity tb_spi_ddr_top;

architecture sim of tb_spi_ddr_top is

  constant CLK_PERIOD    : time := 10 ns;  -- 100 MHz
  constant C_BASE_ADDR   : std_logic_vector(31 downto 0) := x"80000000";
  constant C_PHASE_ADDR  : std_logic_vector(31 downto 0) := x"80100000";
  constant C_PERIOD_CYC  : positive := 3000;   -- 30 us (simulasyon icin kisa)
  constant C_DIM_B       : positive := 3;
  constant C_DIM_C       : positive := 4;
  constant C_DIM_D       : positive := 5;
  constant C_DIM_E       : positive := 6;
  constant C_LOOP_DELAY  : time := 12 ns;
  constant C_NVM_CLOCKS  : integer := 10354;

  signal clk    : std_logic := '0';
  signal rst_n  : std_logic := '0';
  signal enable : std_logic := '0';

  signal idx_a, idx_b, idx_c, idx_d, idx_e : std_logic_vector(7 downto 0) := (others => '0');

  signal wr_clk_div : std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(2, 16));  -- 25 MHz
  signal rd_clk_div : std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(4, 16));  -- 12.5 MHz

  signal stream_reg_addr : std_logic_vector(13 downto 0) := "00001000000000";  -- 0x200

  signal user_rd_req     : std_logic := '0';
  signal user_rd_cmd     : std_logic_vector(31 downto 0) := (others => '0');
  signal user_rd_cmd_len : std_logic_vector(5 downto 0) := (others => '0');
  signal user_rd_len     : std_logic_vector(5 downto 0) := (others => '0');
  signal user_rd_data    : std_logic_vector(31 downto 0);
  signal user_rd_done    : std_logic;

  signal init_done : std_logic;
  signal busy      : std_logic;
  signal axi_err   : std_logic;

  signal m_araddr  : std_logic_vector(31 downto 0);
  signal m_arlen   : std_logic_vector(7 downto 0);
  signal m_arsize  : std_logic_vector(2 downto 0);
  signal m_arburst : std_logic_vector(1 downto 0);
  signal m_arcache : std_logic_vector(3 downto 0);
  signal m_arprot  : std_logic_vector(2 downto 0);
  signal m_arvalid : std_logic;
  signal m_arready : std_logic;
  signal m_rdata   : std_logic_vector(31 downto 0);
  signal m_rresp   : std_logic_vector(1 downto 0);
  signal m_rlast   : std_logic;
  signal m_rvalid  : std_logic;
  signal m_rready  : std_logic;

  signal spi_sclk : std_logic;
  signal sclk_in  : std_logic := '0';
  signal spi_mosi : std_logic;
  signal spi_miso : std_logic;
  signal spi_cs_n : std_logic;
  signal spi_load : std_logic;

  -- AXI slave (DDR) modeli
  signal arready_i : std_logic := '1';
  signal rvalid_i  : std_logic := '0';
  signal rdata_i   : std_logic_vector(31 downto 0) := (others => '0');
  signal ar_addr_q : std_logic_vector(31 downto 0) := (others => '0');
  signal lat_cnt   : integer := 0;

  -- ADMV48281 modeli
  type t_regs is array (0 to 16#3FFF#) of std_logic_vector(7 downto 0);
  signal slave_regs  : t_regs := (16#00E# => x"5A", others => x"00");
  signal miso_out    : std_logic := '0';
  signal cal_clk_cnt : integer := 0;
  signal merge_on    : std_logic;
  signal nvm_ready   : std_logic;

  -- monitorler
  signal mon_len      : integer := 0;
  signal mon_count    : integer := 0;
  signal stream_count : integer := 0;
  signal load_pulses  : integer := 0;

  function f_ddr_word(addr : unsigned(31 downto 0)) return std_logic_vector is
  begin
    return std_logic_vector(addr) xor x"A5A5A5A5";
  end function;

begin

  clk <= not clk after CLK_PERIOD / 2;

  -- kart uzerindeki saat geri donus (loopback) hattinin modeli
  sclk_in <= transport spi_sclk after C_LOOP_DELAY;

  ------------------------------------------------------------------------------
  -- DUT
  ------------------------------------------------------------------------------
  dut : entity work.spi_ddr_top
    generic map (
      G_BASE_ADDR        => C_BASE_ADDR,
      G_PHASE_TABLE_ADDR => C_PHASE_ADDR,
      G_DIM_A            => 2,
      G_DIM_B            => C_DIM_B,
      G_DIM_C            => C_DIM_C,
      G_DIM_D            => C_DIM_D,
      G_DIM_E            => C_DIM_E,
      G_PERIOD_CYCLES    => C_PERIOD_CYC,
      G_NVM_BURST_CYCLES => 10500,
      G_LOAD_WIDTH       => 4
    )
    port map (
      clk    => clk,
      rst_n  => rst_n,
      enable => enable,
      idx_a  => idx_a,
      idx_b  => idx_b,
      idx_c  => idx_c,
      idx_d  => idx_d,
      idx_e  => idx_e,
      wr_clk_div      => wr_clk_div,
      rd_clk_div      => rd_clk_div,
      stream_reg_addr => stream_reg_addr,
      user_rd_req     => user_rd_req,
      user_rd_cmd     => user_rd_cmd,
      user_rd_cmd_len => user_rd_cmd_len,
      user_rd_len     => user_rd_len,
      user_rd_data    => user_rd_data,
      user_rd_done    => user_rd_done,
      init_done => init_done,
      busy      => busy,
      axi_err   => axi_err,
      m_axi_araddr  => m_araddr,
      m_axi_arlen   => m_arlen,
      m_axi_arsize  => m_arsize,
      m_axi_arburst => m_arburst,
      m_axi_arcache => m_arcache,
      m_axi_arprot  => m_arprot,
      m_axi_arvalid => m_arvalid,
      m_axi_arready => m_arready,
      m_axi_rdata   => m_rdata,
      m_axi_rresp   => m_rresp,
      m_axi_rlast   => m_rlast,
      m_axi_rvalid  => m_rvalid,
      m_axi_rready  => m_rready,
      spi_sclk_out => spi_sclk,
      spi_sclk_in  => sclk_in,
      spi_mosi     => spi_mosi,
      spi_miso     => spi_miso,
      spi_cs_n     => spi_cs_n,
      spi_load     => spi_load
    );

  ------------------------------------------------------------------------------
  -- AXI slave (DDR) modeli
  ------------------------------------------------------------------------------
  m_arready <= arready_i;
  m_rvalid  <= rvalid_i;
  m_rdata   <= rdata_i;
  m_rresp   <= "00";
  m_rlast   <= rvalid_i;  -- tek beat

  p_ddr_model : process(clk)
  begin
    if rising_edge(clk) then
      if rvalid_i = '1' and m_rready = '1' then
        rvalid_i  <= '0';
        arready_i <= '1';
      end if;

      if lat_cnt > 0 then
        lat_cnt <= lat_cnt - 1;
        if lat_cnt = 1 then
          rdata_i  <= f_ddr_word(unsigned(ar_addr_q));
          rvalid_i <= '1';
        end if;
      end if;

      if m_arvalid = '1' and arready_i = '1' then
        arready_i <= '0';
        ar_addr_q <= m_araddr;
        lat_cnt   <= 6;
      end if;
    end if;
  end process p_ddr_model;

  ------------------------------------------------------------------------------
  -- ADMV48281 davranis modeli:
  --  * rising edge'de MOSI'yi kaydirir; 24. bitte header'i cozer
  --  * yazma frame'inde her tamamlanan byte'i register'a isler, adresi
  --    artirir (0x000=0xBD ascending secildigi icin)
  --  * okuma frame'inde veriyi falling edge'de SDO'ya kaydirir
  ------------------------------------------------------------------------------
  p_admv : process
    variable v_shift : std_logic_vector(31 downto 0);
    variable v_cnt   : integer;
    variable v_read  : boolean;
    variable v_addr  : integer;
    variable v_out   : std_logic_vector(7 downto 0);
    variable v_bbits : integer;
  begin
    miso_out <= '0';
    wait until falling_edge(spi_cs_n);
    v_shift := (others => '0');
    v_cnt   := 0;
    v_read  := false;
    v_addr  := 0;
    v_bbits := 0;
    loop
      wait on spi_sclk, spi_cs_n;
      if spi_cs_n = '1' then
        exit;
      end if;
      if spi_sclk'event and spi_sclk = '1' then
        v_shift := v_shift(30 downto 0) & spi_mosi;
        v_cnt   := v_cnt + 1;
        if v_cnt = 24 then
          v_read := (v_shift(23) = '1');
          v_addr := to_integer(unsigned(v_shift(13 downto 0)));
          if v_read then
            if v_addr = 16#01A# then
              -- NVM merge durumu: bit6 = kalibrasyon tamam
              if nvm_ready = '1' then
                v_out := x"40";
              else
                v_out := x"00";
              end if;
            else
              v_out := slave_regs(v_addr);
            end if;
          end if;
          v_bbits := 0;
        elsif v_cnt > 24 then
          if not v_read then
            v_bbits := v_bbits + 1;
            if v_bbits = 8 then
              slave_regs(v_addr) <= v_shift(7 downto 0);
              v_addr  := v_addr + 1;  -- ascending streaming
              v_bbits := 0;
            end if;
          end if;
        end if;
      end if;
      if spi_sclk'event and spi_sclk = '0' then
        -- ADMV48281 okuma verisini falling edge'de disari kaydirir
        if v_read and v_cnt >= 24 then
          miso_out <= v_out(7);
          v_out    := v_out(6 downto 0) & '0';
        end if;
      end if;
    end loop;
  end process p_admv;

  spi_miso <= miso_out when spi_cs_n = '0' else '0';

  -- NVM merge saat sayaci: 0x05F(0)=1 iken SCLK rising edge'lerini sayar
  merge_on  <= slave_regs(16#05F#)(0);
  nvm_ready <= '1' when cal_clk_cnt >= C_NVM_CLOCKS else '0';

  p_cal_cnt : process(spi_sclk, merge_on)
  begin
    if merge_on = '0' then
      cal_clk_cnt <= 0;
    elsif rising_edge(spi_sclk) then
      cal_clk_cnt <= cal_clk_cnt + 1;
    end if;
  end process p_cal_cnt;

  ------------------------------------------------------------------------------
  -- Frame monitoru: CS penceresi basina bit sayar; 280 bitlik frame = stream
  ------------------------------------------------------------------------------
  p_monitor : process
    variable v_n : integer;
  begin
    wait until falling_edge(spi_cs_n);
    v_n := 0;
    loop
      wait on spi_sclk, spi_cs_n;
      if spi_cs_n = '1' then
        exit;
      end if;
      if spi_sclk'event and spi_sclk = '1' then
        v_n := v_n + 1;
      end if;
    end loop;
    mon_len   <= v_n;
    mon_count <= mon_count + 1;
    if v_n = 280 then
      stream_count <= stream_count + 1;
    end if;
  end process p_monitor;

  p_load_cnt : process(spi_load)
  begin
    if rising_edge(spi_load) then
      load_pulses <= load_pulses + 1;
    end if;
  end process p_load_cnt;

  ------------------------------------------------------------------------------
  -- Ana test surecisi
  ------------------------------------------------------------------------------
  p_main : process
    variable v_elem_idx  : integer;
    variable v_elem_addr : unsigned(31 downto 0);
    variable v_word      : std_logic_vector(31 downto 0);
    variable v_exp       : std_logic_vector(7 downto 0);
    variable v_reg       : integer;
  begin
    rst_n <= '0';
    wait for 200 ns;
    wait until rising_edge(clk);
    rst_n <= '1';
    wait for 100 ns;

    -- indisler: beam[1][2][3][4][5]
    idx_a <= std_logic_vector(to_unsigned(1, 8));
    idx_b <= std_logic_vector(to_unsigned(2, 8));
    idx_c <= std_logic_vector(to_unsigned(3, 8));
    idx_d <= std_logic_vector(to_unsigned(4, 8));
    idx_e <= std_logic_vector(to_unsigned(5, 8));
    enable <= '1';

    ----------------------------------------------------------------
    -- 1) init + faz SRAM + NVM kalibrasyon
    ----------------------------------------------------------------
    wait until init_done = '1' for 3 ms;
    assert init_done = '1'
      report "HATA: init_done zaman asimi" severity failure;

    -- init tablosu spot kontrolleri
    assert slave_regs(16#000#) = x"BD"
      report "HATA: 0x000 SPI_CONFIG yanlis" severity error;
    assert slave_regs(16#07E#) = x"54"
      report "HATA: 0x07E NVM reset degeri yanlis" severity error;
    assert slave_regs(16#0C2#) = x"F7"
      report "HATA: 0x0C2 bias degeri yanlis" severity error;
    assert slave_regs(16#1054#) = x"12"
      report "HATA: 0x1054 bias degeri yanlis" severity error;
    assert slave_regs(16#0C0#) = x"00"
      report "HATA: 0x0C0 Band 0 secimi yanlis" severity error;
    assert slave_regs(16#05F#) = x"00"
      report "HATA: NVM merge'den cikilmamis (0x05F /= 0)" severity error;
    assert load_pulses = 2
      report "HATA: init LOAD toggle sayisi 2 degil: "
             & integer'image(load_pulses) severity error;

    -- faz SRAM icerigi: 256 byte'in tamamini dogrula
    for i in 0 to 255 loop
      v_word := f_ddr_word(unsigned(C_PHASE_ADDR) + to_unsigned((i / 4) * 4, 32));
      v_exp  := f_lane32(v_word, to_unsigned(i mod 4, 2));
      if i < 128 then
        v_reg := 16#480# + i;
      else
        v_reg := 16#580# + (i - 128);
      end if;
      assert slave_regs(v_reg) = v_exp
        report "HATA: faz SRAM 0x" & integer'image(v_reg) & " yanlis"
        severity error;
    end loop;
    report "OK: init + faz SRAM + NVM kalibrasyon dogru tamamlandi";

    ----------------------------------------------------------------
    -- 2) ilk beam stream'i: beam[1][2][3][4][5] -> 0x200..0x21F
    ----------------------------------------------------------------
    v_elem_idx  := (((1 * C_DIM_B + 2) * C_DIM_C + 3) * C_DIM_D + 4) * C_DIM_E + 5;
    v_elem_addr := unsigned(C_BASE_ADDR) + to_unsigned(v_elem_idx * 32, 32);

    wait until stream_count = 1 for 200 us;
    assert stream_count = 1
      report "HATA: beam stream zaman asimi" severity failure;
    wait for 1 us;   -- LOAD pulse'inin tamamlanmasi icin

    assert mon_len = 280
      report "HATA: stream frame 280 bit degil" severity error;
    for j in 0 to 31 loop
      v_word := f_ddr_word(v_elem_addr + to_unsigned((j / 4) * 4, 32));
      v_exp  := f_lane32(v_word, to_unsigned(j mod 4, 2));
      assert slave_regs(16#200# + j) = v_exp
        report "HATA: beam byte " & integer'image(j) & " yanlis"
        severity error;
    end loop;
    assert load_pulses = 3
      report "HATA: stream sonrasi LOAD toggle yok" severity error;
    report "OK: beam stream #1 dogru (32 byte + LOAD)";

    ----------------------------------------------------------------
    -- 3) SPI okuma: 0x00E (model 0x5A dondurur)
    ----------------------------------------------------------------
    user_rd_cmd     <= f_spi_rd(16#00E#);
    user_rd_cmd_len <= std_logic_vector(to_unsigned(24, 6));
    user_rd_len     <= std_logic_vector(to_unsigned(8, 6));
    wait until rising_edge(clk);
    user_rd_req <= '1';
    wait until rising_edge(clk);
    user_rd_req <= '0';

    wait until user_rd_done = '1' for 200 us;
    assert user_rd_done = '1'
      report "HATA: SPI okuma zaman asimi" severity error;
    assert user_rd_data(7 downto 0) = x"5A"
      report "HATA: SPI okuma verisi yanlis" severity error;
    report "OK: SPI okuma dogru (0x00E = 0x5A, sclk_in ile alindi)";

    ----------------------------------------------------------------
    -- 4) indis degisikligi + ikinci stream: beam[0][1][0][2][1]
    ----------------------------------------------------------------
    idx_a <= std_logic_vector(to_unsigned(0, 8));
    idx_b <= std_logic_vector(to_unsigned(1, 8));
    idx_c <= std_logic_vector(to_unsigned(0, 8));
    idx_d <= std_logic_vector(to_unsigned(2, 8));
    idx_e <= std_logic_vector(to_unsigned(1, 8));

    v_elem_idx  := (((0 * C_DIM_B + 1) * C_DIM_C + 0) * C_DIM_D + 2) * C_DIM_E + 1;
    v_elem_addr := unsigned(C_BASE_ADDR) + to_unsigned(v_elem_idx * 32, 32);

    wait until stream_count = 2 for 200 us;
    assert stream_count = 2
      report "HATA: 2. beam stream zaman asimi" severity failure;
    wait for 1 us;

    for j in 0 to 31 loop
      v_word := f_ddr_word(v_elem_addr + to_unsigned((j / 4) * 4, 32));
      v_exp  := f_lane32(v_word, to_unsigned(j mod 4, 2));
      assert slave_regs(16#200# + j) = v_exp
        report "HATA: 2. stream beam byte " & integer'image(j) & " yanlis"
        severity error;
    end loop;
    assert load_pulses = 4
      report "HATA: 2. stream sonrasi LOAD toggle yok" severity error;
    report "OK: beam stream #2 dogru (yeni indislerle)";

    report "TUM TESTLER BASARILI" severity note;
    assert false report "SIMULASYON SONU (basarili)" severity failure;
  end process p_main;

  ------------------------------------------------------------------------------
  -- Watchdog
  ------------------------------------------------------------------------------
  p_watchdog : process
  begin
    wait for 5 ms;
    assert false report "HATA: test zaman asimina ugradi" severity failure;
  end process p_watchdog;

end architecture sim;

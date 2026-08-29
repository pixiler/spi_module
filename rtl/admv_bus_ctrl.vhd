--------------------------------------------------------------------------------
-- admv_bus_ctrl.vhd
--
-- Tek bir SPI bus'inin (ring modunda 4 veya 8 ADMV48281) kontrolcusu.
-- Icinde bir spi_master, bir spi_slave ve bu bus'a ait beam RAM'i barindirir.
--
-- Calisma sirasi (enable = '1' sonrasi):
--
--   1) INIT      : C_INIT_TABLE broadcast (chip addr 0000) ile yazilir.
--                  0x000 <- 0xBD sonrasi soft reset icin bekleme eklenir.
--                  Tabloda load='1' olan girdilerden sonra LOAD toggle edilir.
--   2) FAZ SRAM  : 0x480'den 128 byte, 0x580'den 128 byte streaming ile
--                  yazilir (UG-2293 Table 11, broadcast).
--   3) NVM CAL   : 0x05F <- 0x01, CS pasif iken >=10354 SCLK darbesi, ardindan
--                  HER CIPTEN 0x01A okunup bit[6] beklenir (broadcast okuma
--                  gecersizdir, bu yuzden cip cip yapilir), sonra 0x05F <- 0x00.
--   4) READY     : init_done = '1', beam_start bekler.
--   5) BEAM      : Her cip icin ayri streaming transaction:
--                  24 bit header (o cipin chip address'i, 0x200 veya 0x240) +
--                  32 byte veri = 280 SCLK. LOAD toggle'i tum buslar bitince
--                  ust modul tarafindan es zamanli verilir.
--
-- Ring uzerindeki ciplerin hepsi ayni veriyi gorur; ayrismayi sadece header'daki
-- chip address saglar. Bu yuzden cip basina farkli gain/phase yazmak N ayri
-- transaction gerektirir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.admv48281_pkg.all;

entity admv_bus_ctrl is
  generic (
    G_BUS_ID           : natural := 0;
    -- true -> STATIK bus: trig'lerde beam yazmaz; init sonunda
    -- C_STATIC_BEAM_RX/TX tablolarini CIP CIP bir kez yazip LOAD'lar
    G_STATIC_BEAM      : boolean := false;
    G_CLK_DIV_WR       : natural := 2;
    G_CLK_DIV_RD       : natural := 8;
    G_NVM_BURST        : natural := 10500;  -- UG-2293: en az 10354
    G_LOAD_CYCLES      : natural := 4;
    G_RESET_WAIT       : natural := 1000;   -- soft reset sonrasi bekleme (clk)
    G_POLL_LIMIT       : natural := 2000;   -- NVM poll deneme siniri
    G_RX_SETTLE_CYCLES : natural := 64;
    G_RX_FILTER_LEN    : natural := 3       -- donen CLK_OUT/SDO giris filtresi
  );
  port (
    clk   : in std_logic;
    rst_n : in std_logic;

    enable : in std_logic;

    -- beam RAM yazma portu (ust moduldeki AXI-Stream demux surer)
    bram_we   : in std_logic;
    bram_addr : in std_logic_vector(5 downto 0);
    bram_data : in std_logic_vector(31 downto 0);

    -- beam yazma kontrolu
    beam_start : in  std_logic;   -- seviye
    rx_tx_sel  : in  std_logic;   -- '0' = RX 0x200, '1' = TX 0x240
    beam_ready : out std_logic;   -- seviye: bu bus turu bitirdi

    -- kullanici SPI okuma istegi (sadece init_done sonrasi, IDLE'da kabul edilir)
    user_rd_req   : in  std_logic;                      -- 1 clock darbe
    user_rd_chip  : in  std_logic_vector(2 downto 0);   -- zincirdeki sira
    user_rd_addr  : in  std_logic_vector(13 downto 0);
    user_rd_data  : out std_logic_vector(7 downto 0);
    user_rd_valid : out std_logic;                      -- 1 clock darbe

    -- durum
    init_done : out std_logic;
    init_err  : out std_logic;
    busy      : out std_logic;

    -- teshis
    dbg_rd_data : out std_logic_vector(7 downto 0);
    dbg_rd_bits : out std_logic_vector(7 downto 0);  -- son okumada yakalanan kenar
    rd_short_err : out std_logic;                    -- kalici: eksik bit yakalandi

    -- SPI pinleri
    spi_sclk_out : out std_logic;
    spi_mosi     : out std_logic;
    spi_cs_n     : out std_logic;
    spi_load     : out std_logic;
    spi_sclk_in  : in  std_logic;
    spi_miso     : in  std_logic
  );
end entity admv_bus_ctrl;

architecture rtl of admv_bus_ctrl is

  constant C_NCHIPS   : natural := C_CHIPS_PER_BUS(G_BUS_ID);
  constant C_RAM_SIZE : natural := C_MAX_CHIPS * C_BEAM_WORDS_PER_CHIP;  -- 64 kelime
  constant C_INIT_LAST : natural := C_INIT_TABLE'high;

  -- spi_master arayuzu
  signal cmd_valid : std_logic := '0';
  signal cmd_ready : std_logic;
  signal cmd_mode  : std_logic_vector(1 downto 0) := C_MODE_WRITE32;
  signal cmd_hdr   : std_logic_vector(31 downto 0) := (others => '0');
  signal cmd_len   : std_logic_vector(15 downto 0) := (others => '0');
  signal m_done    : std_logic;
  signal m_busy    : std_logic;
  signal str_idx   : std_logic_vector(8 downto 0);
  signal str_data  : std_logic_vector(7 downto 0);
  signal rd_arm    : std_logic;

  -- spi_slave arayuzu
  signal rd_data   : std_logic_vector(7 downto 0);
  signal rd_valid  : std_logic;
  signal rd_bits   : std_logic_vector(7 downto 0);
  signal rd_serr   : std_logic;
  signal rd_nbits  : std_logic_vector(7 downto 0);
  signal serr_lat  : std_logic := '0';
  signal rd_bits_r : std_logic_vector(7 downto 0) := (others => '0');

  -- beam RAM (basit iki portlu)
  type t_ram is array (0 to C_RAM_SIZE-1) of std_logic_vector(31 downto 0);
  signal ram     : t_ram := (others => (others => '0'));
  signal ram_q   : std_logic_vector(31 downto 0) := (others => '0');
  signal ram_ra  : unsigned(5 downto 0) := (others => '0');
  signal phase_q  : std_logic_vector(7 downto 0) := (others => '0');
  signal static_q : std_logic_vector(7 downto 0) := (others => '0');

  -- streaming veri kaynagi secimi:
  --   "00" = faz ROM, "01" = beam RAM,
  --   "10" = statik RX tablosu, "11" = statik TX tablosu
  signal str_sel : std_logic_vector(1 downto 0) := "00";

  type t_state is (
    S_IDLE,
    S_ISSUE, S_WAIT, S_LOADP, S_DELAY,
    S_INIT, S_INIT_NEXT,
    S_PH_RX, S_PH_TX,
    S_NVM_SET, S_NVM_BURST, S_NVM_POLL, S_NVM_CHK, S_NVM_CLR,
    S_SB_RX, S_SB_TX, S_SB_NEXT,
    S_READY,
    S_BEAM, S_BEAM_NEXT, S_BEAM_END,
    S_URD1, S_URD2, S_URD_END,
    S_ERR
  );
  signal state     : t_state := S_IDLE;
  signal ret_state : t_state := S_IDLE;
  signal dly_ret   : t_state := S_IDLE;

  signal init_idx  : natural range 0 to C_INIT_LAST := 0;
  signal chip_idx  : natural range 0 to C_MAX_CHIPS-1 := 0;
  signal poll_cnt  : unsigned(15 downto 0) := (others => '0');
  signal dly_cnt   : unsigned(15 downto 0) := (others => '0');
  signal load_pend : std_logic := '0';
  signal load_cnt  : unsigned(7 downto 0) := (others => '0');

  signal load_r      : std_logic := '0';
  signal init_done_r : std_logic := '0';
  signal init_err_r  : std_logic := '0';
  signal beam_rdy_r  : std_logic := '0';
  signal rd_data_r   : std_logic_vector(7 downto 0) := (others => '0');
  signal urd_addr    : std_logic_vector(13 downto 0) := (others => '0');
  signal urd_vld_r   : std_logic := '0';

  -- o an adreslenen cipin chip address'i
  signal chip_addr : unsigned(3 downto 0);

begin

  ------------------------------------------------------------------------------
  -- Alt moduller
  ------------------------------------------------------------------------------
  u_master : entity work.spi_master
    generic map (
      G_CLK_DIV_WR       => G_CLK_DIV_WR,
      G_CLK_DIV_RD       => G_CLK_DIV_RD,
      G_RX_SETTLE_CYCLES => G_RX_SETTLE_CYCLES
    )
    port map (
      clk       => clk,
      rst_n     => rst_n,
      cmd_valid => cmd_valid,
      cmd_ready => cmd_ready,
      cmd_mode  => cmd_mode,
      cmd_hdr   => cmd_hdr,
      cmd_len   => cmd_len,
      str_idx   => str_idx,
      str_data  => str_data,
      spi_sclk  => spi_sclk_out,
      spi_mosi  => spi_mosi,
      spi_cs_n  => spi_cs_n,
      rd_arm    => rd_arm,
      rd_nbits  => rd_nbits,
      busy      => m_busy,
      done      => m_done
    );

  u_slave : entity work.spi_slave
    generic map (
      G_FILTER_LEN  => G_RX_FILTER_LEN,
      G_SAMPLE_RISE => true
    )
    port map (
      clk       => clk,
      rst_n     => rst_n,
      arm       => rd_arm,
      n_bits    => rd_nbits,
      sclk_in   => spi_sclk_in,
      sdi_in    => spi_miso,
      data      => rd_data,
      bit_cnt   => rd_bits,
      valid     => rd_valid,
      short_err => rd_serr
    );

  ------------------------------------------------------------------------------
  -- Beam RAM: yazma ust modulden, okuma spi_master'in istedigi byte icin.
  -- str_idx bir byte suresi (>= 8 SCLK) boyunca sabit kaldigindan kayitli RAM
  -- cikisi kullanilmadan cok once oturur.
  ------------------------------------------------------------------------------
  ram_ra <= to_unsigned(chip_idx * C_BEAM_WORDS_PER_CHIP, 6)
            + resize(unsigned(str_idx(4 downto 2)), 6);

  process (clk)
  begin
    if rising_edge(clk) then
      if bram_we = '1' then
        ram(to_integer(unsigned(bram_addr))) <= bram_data;
      end if;
      -- reset sirasinda str_idx henuz surulmedigi icin indeksleme yapilmaz
      if rst_n = '0' then
        ram_q    <= (others => '0');
        phase_q  <= (others => '0');
        static_q <= (others => '0');
      else
        ram_q   <= ram(to_integer(ram_ra));
        phase_q <= C_PHASE_STREAM(to_integer(unsigned(str_idx(6 downto 0))));
        -- statik tablolar cip basina bir satir tasir; chip_idx transaction
        -- boyunca sabittir
        if str_sel = "11" then
          static_q <= C_STATIC_BEAM_TX(chip_idx)(to_integer(unsigned(str_idx(4 downto 0))));
        else
          static_q <= C_STATIC_BEAM_RX(chip_idx)(to_integer(unsigned(str_idx(4 downto 0))));
        end if;
      end if;
    end if;
  end process;

  -- streaming veri kaynagi mux'i
  str_data <= f_lane32(ram_q, unsigned(str_idx(1 downto 0))) when str_sel = "01"
              else static_q when str_sel(1) = '1'
              else phase_q;

  chip_addr <= to_unsigned(C_CHIP_ADDR(G_BUS_ID, chip_idx), 4);

  spi_load    <= load_r;
  init_done   <= init_done_r;
  init_err    <= init_err_r;
  beam_ready  <= beam_rdy_r;
  busy        <= '0' when (state = S_IDLE or state = S_READY) else '1';
  dbg_rd_data  <= rd_data_r;
  dbg_rd_bits  <= rd_bits_r;
  rd_short_err <= serr_lat;

  user_rd_data  <= rd_data_r;
  user_rd_valid <= urd_vld_r;

  ------------------------------------------------------------------------------
  -- Ana sekans
  ------------------------------------------------------------------------------
  process (clk)
  begin
    if rising_edge(clk) then
      urd_vld_r <= '0';

      if rst_n = '0' then
        state       <= S_IDLE;
        urd_addr    <= (others => '0');
        ret_state   <= S_IDLE;
        dly_ret     <= S_IDLE;
        cmd_valid   <= '0';
        init_idx    <= 0;
        chip_idx    <= 0;
        poll_cnt    <= (others => '0');
        dly_cnt     <= (others => '0');
        load_pend   <= '0';
        load_cnt    <= (others => '0');
        load_r      <= '0';
        init_done_r <= '0';
        init_err_r  <= '0';
        beam_rdy_r  <= '0';
        str_sel     <= "00";
        rd_data_r   <= (others => '0');
        rd_bits_r   <= (others => '0');
        serr_lat    <= '0';

      else
        if rd_valid = '1' then
          rd_data_r <= rd_data;
          rd_bits_r <= rd_bits;
          if rd_serr = '1' then
            serr_lat <= '1';   -- kalici: donanimda teshis icin
          end if;
        end if;

        case state is

          --------------------------------------------------------------------
          when S_IDLE =>
            init_done_r <= '0';
            beam_rdy_r  <= '0';
            if enable = '1' then
              init_idx <= 0;
              state    <= S_INIT;
            end if;

          --------------------------------------------------------------------
          -- Ortak komut gonderme adimlari
          --------------------------------------------------------------------
          when S_ISSUE =>
            -- cmd_valid kayitli oldugu icin handshake'i gercek deger uzerinden
            -- kontrol et; aksi halde master komutu hic gormeden gecilir
            cmd_valid <= '1';
            if cmd_valid = '1' and cmd_ready = '1' then
              cmd_valid <= '0';
              state     <= S_WAIT;
            end if;

          when S_WAIT =>
            if m_done = '1' then
              if load_pend = '1' then
                load_pend <= '0';
                load_cnt  <= (others => '0');
                load_r    <= '1';
                state     <= S_LOADP;
              else
                state <= ret_state;
              end if;
            end if;

          when S_LOADP =>
            -- LOAD darbesi: global SPI blogundan kanal registerlarina aktarim
            if load_cnt = to_unsigned(G_LOAD_CYCLES, 8) then
              load_r <= '0';
              state  <= ret_state;
            else
              load_cnt <= load_cnt + 1;
            end if;

          when S_DELAY =>
            if dly_cnt = 0 then
              state <= dly_ret;
            else
              dly_cnt <= dly_cnt - 1;
            end if;

          --------------------------------------------------------------------
          -- 1) Init tablosu (broadcast)
          --------------------------------------------------------------------
          when S_INIT =>
            cmd_mode  <= C_MODE_WRITE32;
            cmd_hdr   <= f_wr_frame(C_CHIP_ADDR_BCAST,
                                    C_INIT_TABLE(init_idx).addr,
                                    C_INIT_TABLE(init_idx).data);
            cmd_len   <= (others => '0');
            load_pend <= C_INIT_TABLE(init_idx).load;
            ret_state <= S_INIT_NEXT;
            state     <= S_ISSUE;

          when S_INIT_NEXT =>
            if init_idx = 0 then
              -- 0x000 <- 0xBD soft reset yapar; sonraki yazmadan once bekle
              dly_cnt  <= to_unsigned(G_RESET_WAIT, 16);
              dly_ret  <= S_INIT;
              init_idx <= init_idx + 1;
              state    <= S_DELAY;
            elsif init_idx = C_INIT_LAST then
              state <= S_PH_RX;
            else
              init_idx <= init_idx + 1;
              state    <= S_INIT;
            end if;

          --------------------------------------------------------------------
          -- 2) Global faz SRAM (Table 11), broadcast, streaming
          --------------------------------------------------------------------
          when S_PH_RX =>
            str_sel   <= "00";
            cmd_mode  <= C_MODE_STREAM;
            cmd_hdr   <= f_stream_hdr(to_unsigned(C_CHIP_ADDR_BCAST, 4),
                                      C_REG_RX_PHASE_SRAM);
            cmd_len   <= std_logic_vector(to_unsigned(128, 16));
            load_pend <= '0';
            ret_state <= S_PH_TX;
            state     <= S_ISSUE;

          when S_PH_TX =>
            str_sel   <= "00";
            cmd_mode  <= C_MODE_STREAM;
            cmd_hdr   <= f_stream_hdr(to_unsigned(C_CHIP_ADDR_BCAST, 4),
                                      C_REG_TX_PHASE_SRAM);
            cmd_len   <= std_logic_vector(to_unsigned(128, 16));
            load_pend <= '0';
            ret_state <= S_NVM_SET;
            state     <= S_ISSUE;

          --------------------------------------------------------------------
          -- 3) NVM kalibrasyon (merge)
          --------------------------------------------------------------------
          when S_NVM_SET =>
            cmd_mode  <= C_MODE_WRITE32;
            cmd_hdr   <= f_wr_frame(C_CHIP_ADDR_BCAST, C_REG_RAM_FILL_LD, x"01");
            cmd_len   <= (others => '0');
            load_pend <= '0';
            ret_state <= S_NVM_BURST;
            state     <= S_ISSUE;

          when S_NVM_BURST =>
            -- CS pasif iken >= 10354 SCLK darbesi
            cmd_mode  <= C_MODE_BURST;
            cmd_hdr   <= (others => '0');
            cmd_len   <= std_logic_vector(to_unsigned(G_NVM_BURST, 16));
            load_pend <= '0';
            chip_idx  <= 0;
            poll_cnt  <= (others => '0');
            ret_state <= S_NVM_POLL;
            state     <= S_ISSUE;

          when S_NVM_POLL =>
            -- broadcast okuma gecersiz: her cip tek tek sorgulanir
            cmd_mode  <= C_MODE_READ;
            cmd_hdr   <= f_rd_hdr(chip_addr, C_REG_SRAM_FILL);
            cmd_len   <= std_logic_vector(to_unsigned(1, 16));
            load_pend <= '0';
            ret_state <= S_NVM_CHK;
            state     <= S_ISSUE;

          when S_NVM_CHK =>
            if rd_data_r(C_NVM_DONE_BIT) = '1' then
              poll_cnt <= (others => '0');
              if chip_idx = C_NCHIPS - 1 then
                state <= S_NVM_CLR;
              else
                chip_idx <= chip_idx + 1;
                state    <= S_NVM_POLL;
              end if;
            elsif poll_cnt = to_unsigned(G_POLL_LIMIT, 16) then
              state <= S_ERR;
            else
              poll_cnt <= poll_cnt + 1;
              state    <= S_NVM_POLL;
            end if;

          when S_NVM_CLR =>
            cmd_mode  <= C_MODE_WRITE32;
            cmd_hdr   <= f_wr_frame(C_CHIP_ADDR_BCAST, C_REG_RAM_FILL_LD, x"00");
            cmd_len   <= (others => '0');
            load_pend <= '0';
            if G_STATIC_BEAM then
              chip_idx  <= 0;
              ret_state <= S_SB_RX;
            else
              ret_state <= S_READY;
            end if;
            state <= S_ISSUE;

          --------------------------------------------------------------------
          -- 3b) Statik bus: acilis beam'ini CIP CIP yaz (her cip kendi chip
          --     address'i ile kendi tablolarini alir; NVM kalibrasyonundan
          --     SONRA ki LOAD merge edilmis SRAM'lerden gecsin). Tum cipler
          --     yazildiktan sonra TEK LOAD toggle: hepsi ayni anda yuklenir.
          --------------------------------------------------------------------
          when S_SB_RX =>
            str_sel   <= "10";
            cmd_mode  <= C_MODE_STREAM;
            cmd_hdr   <= f_stream_hdr(chip_addr, C_REG_RX_DIRECT_BASE);
            cmd_len   <= std_logic_vector(to_unsigned(C_BEAM_BYTES_PER_CHIP, 16));
            load_pend <= '0';
            ret_state <= S_SB_TX;
            state     <= S_ISSUE;

          when S_SB_TX =>
            str_sel  <= "11";
            cmd_mode <= C_MODE_STREAM;
            cmd_hdr  <= f_stream_hdr(chip_addr, C_REG_TX_DIRECT_BASE);
            cmd_len  <= std_logic_vector(to_unsigned(C_BEAM_BYTES_PER_CHIP, 16));
            if chip_idx = C_NCHIPS - 1 then
              load_pend <= '1';   -- son cip: hepsi yazildi, simdi yukle
              ret_state <= S_READY;
            else
              load_pend <= '0';
              ret_state <= S_SB_NEXT;
            end if;
            state <= S_ISSUE;

          when S_SB_NEXT =>
            chip_idx <= chip_idx + 1;
            state    <= S_SB_RX;

          --------------------------------------------------------------------
          -- 4) Hazir: disaridan tetik bekle
          --------------------------------------------------------------------
          when S_READY =>
            init_done_r <= '1';
            beam_rdy_r  <= '0';
            if beam_start = '1' then
              if G_STATIC_BEAM then
                state <= S_BEAM_END;   -- yazma yok, dogrudan hazir bildir
              else
                chip_idx <= 0;
                state    <= S_BEAM;
              end if;
            elsif user_rd_req = '1' then
              -- istenen cip zincirde yoksa 0'a klemple
              if to_integer(unsigned(user_rd_chip)) < C_NCHIPS then
                chip_idx <= to_integer(unsigned(user_rd_chip));
              else
                chip_idx <= 0;
              end if;
              urd_addr <= user_rd_addr;
              state    <= S_URD1;
            end if;

          --------------------------------------------------------------------
          -- 5) Direct beam yazma: cip basina bir streaming transaction
          --------------------------------------------------------------------
          when S_BEAM =>
            str_sel  <= "01";
            cmd_mode <= C_MODE_STREAM;
            if rx_tx_sel = '0' then
              cmd_hdr <= f_stream_hdr(chip_addr, C_REG_RX_DIRECT_BASE);
            else
              cmd_hdr <= f_stream_hdr(chip_addr, C_REG_TX_DIRECT_BASE);
            end if;
            cmd_len   <= std_logic_vector(to_unsigned(C_BEAM_BYTES_PER_CHIP, 16));
            load_pend <= '0';   -- LOAD'i ust modul tum buslar icin es zamanli verir
            ret_state <= S_BEAM_NEXT;
            state     <= S_ISSUE;

          when S_BEAM_NEXT =>
            if chip_idx = C_NCHIPS - 1 then
              state <= S_BEAM_END;
            else
              chip_idx <= chip_idx + 1;
              state    <= S_BEAM;
            end if;

          when S_BEAM_END =>
            beam_rdy_r <= '1';
            if beam_start = '0' then
              beam_rdy_r <= '0';
              state      <= S_READY;
            end if;

          --------------------------------------------------------------------
          -- Kullanici okumasi. UG-2293: SRAM registerlari (>= 0x400) icin
          -- okuma komutu IKI KEZ gonderilmeli, gecerli veri ikincisinde gelir.
          --------------------------------------------------------------------
          when S_URD1 =>
            cmd_mode  <= C_MODE_READ;
            cmd_hdr   <= f_rd_hdr(chip_addr, "00" & urd_addr);
            cmd_len   <= std_logic_vector(to_unsigned(1, 16));
            load_pend <= '0';
            ret_state <= S_URD2;
            state     <= S_ISSUE;

          when S_URD2 =>
            if unsigned(urd_addr) >= to_unsigned(16#400#, 14) then  -- SRAM bolgesi
              cmd_mode  <= C_MODE_READ;
              cmd_hdr   <= f_rd_hdr(chip_addr, "00" & urd_addr);
              cmd_len   <= std_logic_vector(to_unsigned(1, 16));
              load_pend <= '0';
              ret_state <= S_URD_END;
              state     <= S_ISSUE;
            else
              state <= S_URD_END;
            end if;

          when S_URD_END =>
            urd_vld_r <= '1';
            state     <= S_READY;

          --------------------------------------------------------------------
          when S_ERR =>
            init_err_r <= '1';
            if enable = '0' then
              init_err_r <= '0';
              state      <= S_IDLE;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;

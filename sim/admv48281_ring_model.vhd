--------------------------------------------------------------------------------
-- admv48281_ring_model.vhd  (sadece simulasyon)
--
-- Tek bir SPI bus'indaki ADMV48281 ring zincirinin davranis modeli.
--
-- Ring bir repeater zinciri oldugundan (SDO = SDIO, CLK_OUT = SCLK) zincirdeki
-- tum cipler ayni veriyi gorur; ayrisma header'daki chip address ile olur.
-- Bu yuzden model tek bir decoder ile N cipin register dosyasini yonetir.
--
-- Destekledigi davranislar:
--   * 24 bit header (R/W# + chip + channel + adres) + N x 8 bit veri
--     (standart 32 bit frame = 24 + 8, streaming = 24 + 8*N ayni yoldan islenir)
--   * ascending adres artisi (0x000 bit5 = 1 varsayimi)
--   * broadcast (chip addr 0000) tum ciplere yazar
--   * okuma: adreslenen cip veriyi SCLK'in DUSEN kenarinda surer
--   * ring gecikmesi: CLK_OUT/SDO, SCLK/SDIO'ya gore G_RING_DLY kadar geciktirilir
--   * NVM merge: 0x05F bit0 = 1 iken CS pasif SCLK darbeleri sayilir,
--     G_NVM_CLOCKS asilinca 0x01A bit6 = 1 olur; 0x05F <- 0x00 ile temizlenir
--   * LOAD pini toggle sayaci
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.admv48281_pkg.all;

entity admv48281_ring_model is
  generic (
    G_BUS_ID     : natural := 0;
    G_NVM_CLOCKS : natural := 300;      -- gercek cipte 10354
    G_RING_DLY   : time    := 8 ns      -- 8 hop toplam gecikme
  );
  port (
    -- kontrolcuden zincire
    sclk : in std_logic;
    mosi : in std_logic;
    cs_n : in std_logic;
    load : in std_logic;

    -- zincirden kontrolcuye donen
    sclk_ret : out std_logic;
    miso_ret : out std_logic;

    -- testbench inceleme arayuzu
    peek_chip : in  natural;
    peek_addr : in  std_logic_vector(15 downto 0);
    peek_data : out std_logic_vector(7 downto 0);

    load_toggles : out natural
  );
end entity admv48281_ring_model;

architecture sim of admv48281_ring_model is

  constant C_NCHIPS  : natural := C_CHIPS_PER_BUS(G_BUS_ID);
  constant C_MEM_TOP : natural := 16#10FF#;

  type t_mem     is array (0 to C_MEM_TOP) of std_logic_vector(7 downto 0);
  type t_chipmem is array (0 to C_NCHIPS-1) of t_mem;

  -- Direct beam bolgesi (0x200-0x25F) kasitli olarak 0xAA sentineli ile
  -- baslatilir. Gercek cip bu registerlari 0x00'a resetler; sentinel,
  -- "0 yazildi" ile "hic yazilmadi" durumlarini testte ayirt edebilmek
  -- icindir (acilis beam tablolari tamamen 0 oldugu icin sart).
  function f_mem_init return t_chipmem is
    variable m : t_chipmem := (others => (others => (others => '0')));
  begin
    for c in 0 to C_NCHIPS-1 loop
      for a in 16#200# to 16#25F# loop
        m(c)(a) := x"AA";
      end loop;
    end loop;
    return m;
  end function;

  signal sdo_int : std_logic := '0';

begin

  -- ring gecikmesi: donen clock ve veri
  sclk_ret <= transport sclk    after G_RING_DLY;
  miso_ret <= transport sdo_int after G_RING_DLY;

  ------------------------------------------------------------------------------
  -- LOAD toggle sayaci
  ------------------------------------------------------------------------------
  process (load)
    variable n : natural := 0;
  begin
    if rising_edge(load) then
      n := n + 1;
    end if;
    load_toggles <= n;
  end process;

  ------------------------------------------------------------------------------
  -- SPI decoder + register dosyalari
  ------------------------------------------------------------------------------
  process (sclk, cs_n, peek_chip, peek_addr)
    variable mem     : t_chipmem := f_mem_init;
    variable bitcnt  : natural := 0;
    variable sh      : std_logic_vector(23 downto 0) := (others => '0');
    variable rw      : std_logic := '0';
    variable chip    : integer := 0;
    variable addr    : integer := 0;
    variable dbits   : natural := 0;
    variable dbyte   : std_logic_vector(7 downto 0) := (others => '0');
    variable rd_sh   : std_logic_vector(7 downto 0) := (others => '0');
    variable rd_idx  : integer := -1;
    variable nvm_on  : boolean := false;
    variable nvm_cnt : natural := 0;

    -- chip address -> zincirdeki dizin (-1 = bu buste yok)
    function f_idx_of(a : integer) return integer is
    begin
      for c in 0 to C_NCHIPS-1 loop
        if C_CHIP_ADDR(G_BUS_ID, c) = a then
          return c;
        end if;
      end loop;
      return -1;
    end function;

  begin
    ----------------------------------------------------------------------------
    if cs_n = '1' then
      -- CS pasif: transaction sifirlanir, NVM merge clock'lari burada sayilir
      bitcnt := 0;
      dbits  := 0;
      rd_idx := -1;

      if rising_edge(sclk) and nvm_on then
        if nvm_cnt < G_NVM_CLOCKS then
          nvm_cnt := nvm_cnt + 1;
          if nvm_cnt = G_NVM_CLOCKS then
            for c in 0 to C_NCHIPS-1 loop
              mem(c)(16#01A#)(C_NVM_DONE_BIT) := '1';
            end loop;
          end if;
        end if;
      end if;

    elsif rising_edge(sclk) then
      ------------------------------------------------------------------------
      -- kontrolcuden gelen bitler
      if bitcnt < 24 then
        sh     := sh(22 downto 0) & mosi;
        bitcnt := bitcnt + 1;
        if bitcnt = 24 then
          rw    := sh(23);
          chip  := to_integer(unsigned(sh(22 downto 19)));
          addr  := to_integer(unsigned(sh(13 downto 0)));
          dbits := 0;
          if rw = '1' then
            rd_idx := f_idx_of(chip);   -- broadcast okuma gecersiz -> -1
            if rd_idx >= 0 then
              rd_sh := mem(rd_idx)(addr);
            else
              rd_sh := (others => '0');
            end if;
          end if;
        end if;

      else
        dbyte := dbyte(6 downto 0) & mosi;
        dbits := dbits + 1;
        if dbits = 8 then
          dbits := 0;
          if rw = '0' then
            for c in 0 to C_NCHIPS-1 loop
              if chip = C_CHIP_ADDR_BCAST or chip = C_CHIP_ADDR(G_BUS_ID, c) then
                if addr <= C_MEM_TOP then
                  mem(c)(addr) := dbyte;
                end if;
              end if;
            end loop;

            -- NVM merge kontrolu (0x05F bit0)
            if addr = 16#05F# then
              if dbyte(0) = '1' then
                nvm_on  := true;
                nvm_cnt := 0;
              else
                nvm_on := false;
                for c in 0 to C_NCHIPS-1 loop
                  mem(c)(16#01A#)(C_NVM_DONE_BIT) := '0';
                end loop;
              end if;
            end if;
          end if;
          addr := addr + 1;   -- ascending streaming
        end if;
      end if;

    elsif falling_edge(sclk) then
      ------------------------------------------------------------------------
      -- okuma fazi: veri dusen kenarda surulur
      if bitcnt = 24 and rw = '1' then
        sdo_int <= rd_sh(7);
        rd_sh   := rd_sh(6 downto 0) & '0';
      else
        sdo_int <= '0';
      end if;
    end if;

    ----------------------------------------------------------------------------
    -- testbench inceleme
    if peek_chip < C_NCHIPS
       and to_integer(unsigned(peek_addr)) <= C_MEM_TOP then
      peek_data <= mem(peek_chip)(to_integer(unsigned(peek_addr)));
    else
      peek_data <= (others => 'X');
    end if;

  end process;

end architecture sim;

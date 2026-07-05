--------------------------------------------------------------------------------
-- spi_pkg.vhd
-- ADMV48281 SPI modulu icin ortak tipler, frame yardimci fonksiyonlari ve
-- acilista SPI uzerinden yazilacak initial register tablosu.
--
-- ADMV48281 standart SPI frame'i (32 bit, MSB once):
--   bit 31    : R/W#      (0 = yazma, 1 = okuma)
--   bit 30:27 : CHIP_ADDR (tek cip / broadcast = 0000)
--   bit 26:22 : CHANNEL   (00000 = global / tum kanallara broadcast)
--   bit 21:8  : REG ADDR  (A13:A0)
--   bit 7:0   : DATA
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package spi_pkg is

  -- Bir init kaydi: 32 bitlik frame, bit sayisi ve frame sonrasi LOAD pini
  -- toggle gereksinimi (UG-2293 Table 172).
  type t_init_entry is record
    data : std_logic_vector(31 downto 0);
    len  : natural;
    load : boolean;
  end record;

  type t_init_table is array (natural range <>) of t_init_entry;

  -- yazma frame'i olustur (chip addr = 0, channel = broadcast)
  function f_spi_wr(addr : natural; data : natural) return std_logic_vector;
  function f_spi_wr_u(addr : unsigned(13 downto 0);
                      data : std_logic_vector(7 downto 0))
                      return std_logic_vector;

  -- okuma komutu olustur: sag hizali 24 bitlik header (R/W#=1 + adres);
  -- tx_len=24, rx_len=8 ile kullanilir
  function f_spi_rd(addr : natural) return std_logic_vector;

  -- 32 bitlik little-endian DDR kelimesinden byte sec (0 = en dusuk adres)
  function f_lane32(w   : std_logic_vector(31 downto 0);
                    sel : unsigned(1 downto 0))
                    return std_logic_vector;

  ------------------------------------------------------------------------------
  -- ADMV48281 SPI init dizisi (UG-2293, SPI Initialization, sayfa 28):
  --   1) 0x000 <- 0xBD : soft reset + 4-wire mod + streaming ascending
  --   2) 0x07E <- 0x40, 0x54 : NVM devresi reseti
  --   3) Table 12 bias ayarlari, Band 0 / genis bant (>=400 MHz mod. BW) sutunu
  --      (dusuk bant genisligi icin 0x102B/0x102C degerlerini guncelleyin)
  --   4) 0x0C0 <- 0x00 : NVM Band 0 secimi (bit6 = 0)
  -- Kanal registerlari (0x1021+) channel=00000 broadcast ile tum 16 kanala
  -- ayni anda yazilir. 0x2FF ve 0x1021 LOAD toggle gerektirir.
  ------------------------------------------------------------------------------
  constant C_INIT_TABLE : t_init_table := (
    (data => x"000000BD", len => 32, load => false),  -- SPI_CONFIG: 4-wire, ascending
    (data => x"00007E40", len => 32, load => false),  -- NVM reset adim 1
    (data => x"00007E54", len => 32, load => false),  -- NVM reset adim 2
    -- Table 12 bias ayarlari (Band 0, genis bant):
    (data => x"0000C2F7", len => 32, load => false),
    (data => x"0000C32A", len => 32, load => false),
    (data => x"0000C42A", len => 32, load => false),
    (data => x"0000C614", len => 32, load => false),
    (data => x"0000D105", len => 32, load => false),
    (data => x"0000D214", len => 32, load => false),
    (data => x"0000D305", len => 32, load => false),
    (data => x"0000D414", len => 32, load => false),
    (data => x"0000D606", len => 32, load => false),
    (data => x"0000D80E", len => 32, load => false),
    (data => x"0000D900", len => 32, load => false),
    (data => x"0002FF80", len => 32, load => true),   -- TEMP_COMP_BYPASS + LOAD
    (data => x"00102100", len => 32, load => true),   -- POWER_DOWN_BLOCKS_2 + LOAD
    (data => x"00102217", len => 32, load => false),
    (data => x"00102417", len => 32, load => false),
    (data => x"0010260C", len => 32, load => false),
    (data => x"00102807", len => 32, load => false),
    (data => x"00102B09", len => 32, load => false),
    (data => x"00102C1B", len => 32, load => false),
    (data => x"00102D12", len => 32, load => false),
    (data => x"00102E1E", len => 32, load => false),
    (data => x"00102F0E", len => 32, load => false),
    (data => x"00103100", len => 32, load => false),
    (data => x"00103206", len => 32, load => false),
    (data => x"00103306", len => 32, load => false),
    (data => x"00103E23", len => 32, load => false),
    (data => x"0010401C", len => 32, load => false),
    (data => x"0010415C", len => 32, load => false),
    (data => x"0010420A", len => 32, load => false),
    (data => x"00104503", len => 32, load => false),
    (data => x"0010464F", len => 32, load => false),
    (data => x"00104707", len => 32, load => false),
    (data => x"00104819", len => 32, load => false),
    (data => x"00104A0E", len => 32, load => false),
    (data => x"00104B17", len => 32, load => false),
    (data => x"00104C01", len => 32, load => false),
    (data => x"00104F07", len => 32, load => false),
    (data => x"00105346", len => 32, load => false),
    (data => x"00105412", len => 32, load => false),
    (data => x"0010700C", len => 32, load => false),
    (data => x"00107116", len => 32, load => false),
    (data => x"0010720D", len => 32, load => false),
    (data => x"00107312", len => 32, load => false),
    (data => x"00107410", len => 32, load => false),
    (data => x"00107533", len => 32, load => false),
    -- NVM band secimi: Band 0 (0x0C0 bit6 = 0)
    (data => x"0000C000", len => 32, load => false)
  );

end package spi_pkg;

package body spi_pkg is

  function f_spi_wr(addr : natural; data : natural) return std_logic_vector is
  begin
    return "0" & "0000" & "00000"
           & std_logic_vector(to_unsigned(addr, 14))
           & std_logic_vector(to_unsigned(data, 8));
  end function;

  function f_spi_wr_u(addr : unsigned(13 downto 0);
                      data : std_logic_vector(7 downto 0))
                      return std_logic_vector is
  begin
    return "0" & "0000" & "00000" & std_logic_vector(addr) & data;
  end function;

  function f_spi_rd(addr : natural) return std_logic_vector is
  begin
    return x"00" & "1" & "0000" & "00000"
           & std_logic_vector(to_unsigned(addr, 14));
  end function;

  function f_lane32(w   : std_logic_vector(31 downto 0);
                    sel : unsigned(1 downto 0))
                    return std_logic_vector is
  begin
    case sel is
      when "00"   => return w(7 downto 0);
      when "01"   => return w(15 downto 8);
      when "10"   => return w(23 downto 16);
      when others => return w(31 downto 24);
    end case;
  end function;

end package body spi_pkg;

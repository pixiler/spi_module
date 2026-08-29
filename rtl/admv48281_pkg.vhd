--------------------------------------------------------------------------------
-- admv48281_pkg.vhd
--
-- ADMV48281 (UG-2293 Rev. Sp0) icin register haritasi, frame yardimci
-- fonksiyonlari, dizi konfigurasyonu ve acilis tablolari.
--
-- SPI frame formati (standart ADI protokolu, 32 bit, MSB once, CPOL=0/CPHA=0):
--   bit 31    : R/W#      0 = yazma, 1 = okuma
--   bit 30:27 : CHIP_ADDR 0000 = broadcast, 1..15 = tek cip
--   bit 26:22 : CHANNEL   00000 = global / tum kanallar
--   bit 21:8  : REG ADDR  A13:A0
--   bit 7:0   : DATA
--
-- Streaming frame: 24 bit header (R/W# + chip + channel + ilk adres) + N x 8 bit
-- veri. Adres artis yonu 0x000 bit5 (ADDR_ASCN) ile belirlenir; bu tasarim
-- ascending (0xBD) kullanir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package admv48281_pkg is

  ------------------------------------------------------------------------------
  -- Ortak tipler
  ------------------------------------------------------------------------------
  type t_byte_array    is array (natural range <>) of std_logic_vector(7 downto 0);
  type t_natural_array is array (natural range <>) of natural;
  type t_bool_array    is array (natural range <>) of boolean;

  ------------------------------------------------------------------------------
  -- spi_master komut modlari
  ------------------------------------------------------------------------------
  constant C_MODE_WRITE32 : std_logic_vector(1 downto 0) := "00"; -- 32 bit standart frame
  constant C_MODE_STREAM  : std_logic_vector(1 downto 0) := "01"; -- 24 bit header + N byte
  constant C_MODE_READ    : std_logic_vector(1 downto 0) := "10"; -- 24 bit header + N byte okuma
  constant C_MODE_BURST   : std_logic_vector(1 downto 0) := "11"; -- CS pasif, N adet SCLK

  ------------------------------------------------------------------------------
  -- DIZI KONFIGURASYONU
  -- 7 SPI bus. Bus 0..5 ring modunda 8 cip, bus 6 ring modunda 4 cip icerir.
  -- Toplam 6*8 + 4 = 52 ADMV48281.
  ------------------------------------------------------------------------------
  constant C_NUM_BUS   : natural := 7;
  constant C_MAX_CHIPS : natural := 8;

  constant C_CHIPS_PER_BUS : t_natural_array(0 to C_NUM_BUS-1) :=
    (8, 8, 8, 8, 8, 8, 4);

  ------------------------------------------------------------------------------
  -- Bus basina beam davranisi:
  --   true  -> her trig'de AXI-Stream'den gelen beam verisi bu bus'a yazilir
  --   false -> STATIK bus: acilista C_STATIC_BEAM_RX/TX tablolari bir kez
  --            yazilir, sonraki trig'lerde SPI yazmasi yapilmaz. TRX pini ve
  --            senkron LOAD darbesi yine de surulur.
  -- AXI-Stream paketi sadece true olan buslarin verisini tasir.
  ------------------------------------------------------------------------------
  constant C_BEAM_ON_TRIG : t_bool_array(0 to C_NUM_BUS-1) :=
    (true, true, true, true, true, true, false);   -- bus 6 statik

  ------------------------------------------------------------------------------
  -- CIP ADRESLERI (CHIP_ADD0..CHIP_ADD3 pinlerinden kart uzerinde set edilir)
  --
  -- Ring modunda chip address ZORUNLU: 0000 sadece broadcast icin ayrilmistir,
  -- ring uzerindeki hicbir cipe 0000 verilemez (UG-2293, Ring Configuration).
  -- Asagidaki tablo karttaki hardwired degerlerle BIREBIR ayni olmalidir.
  -- Kullanilmayan slotlar 0 birakilir (C_CHIPS_PER_BUS ile maskelenir).
  --
  -- Satir = bus, sutun = zincirdeki sira (0 = kontrolcuye en yakin cip).
  ------------------------------------------------------------------------------
  type t_chip_addr_map is array (0 to C_NUM_BUS-1, 0 to C_MAX_CHIPS-1)
       of integer range 0 to 15;

  constant C_CHIP_ADDR : t_chip_addr_map := (
    (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 0
    (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 1
    (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 2
    (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 3
    (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 4
    (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 5
    (1, 2, 3, 4, 0, 0, 0, 0)    -- bus 6: sadece 4 cip
  );

  constant C_CHIP_ADDR_BCAST : natural := 0;   -- broadcast (sadece yazma)

  ------------------------------------------------------------------------------
  -- REGISTER ADRESLERI (UG-2293 Register Information)
  ------------------------------------------------------------------------------
  constant C_REG_SPI_CONFIG    : std_logic_vector(15 downto 0) := x"0000";
  constant C_REG_SRAM_FILL     : std_logic_vector(15 downto 0) := x"001A"; -- bit6 = NVM cal bitti
  constant C_REG_RAM_FILL_LD   : std_logic_vector(15 downto 0) := x"005F"; -- bit0 = LD_CH
  constant C_REG_NVM_RESET     : std_logic_vector(15 downto 0) := x"007E";
  constant C_REG_NVM_CTRL      : std_logic_vector(15 downto 0) := x"00C0"; -- bit6 = band sec
  constant C_REG_MANUAL_BYPASS : std_logic_vector(15 downto 0) := x"00C1";

  -- Direct beam (bypass mode) kanal register bloklari. Her blok 32 byte:
  --   +0  ch0V gain, +1  ch0V phase, +2  ch0H gain, +3  ch0H phase, ...
  --   +30 ch7H gain, +31 ch7H phase
  constant C_REG_RX_DIRECT_BASE : std_logic_vector(15 downto 0) := x"0200"; -- 0x200..0x21F
  constant C_REG_TX_DIRECT_BASE : std_logic_vector(15 downto 0) := x"0240"; -- 0x240..0x25F

  -- Direct beam ortak (common) kazanc registerlari
  constant C_REG_RX_DIR_COM_V : std_logic_vector(15 downto 0) := x"0280";
  constant C_REG_RX_DIR_COM_H : std_logic_vector(15 downto 0) := x"0281";
  constant C_REG_TX_DIR_COM_V : std_logic_vector(15 downto 0) := x"0288";
  constant C_REG_TX_DIR_COM_H : std_logic_vector(15 downto 0) := x"0289";

  -- Global faz SRAM taban adresleri (her biri 128 byte: 64 Q + 64 I)
  constant C_REG_RX_PHASE_SRAM : std_logic_vector(15 downto 0) := x"0480"; -- 0x480..0x4FF
  constant C_REG_TX_PHASE_SRAM : std_logic_vector(15 downto 0) := x"0580"; -- 0x580..0x5FF

  constant C_NVM_DONE_BIT : natural := 6;   -- 0x01A bit[6]

  ------------------------------------------------------------------------------
  -- BEAM VERI DUZENI
  -- Bir cipin direct beam guncellemesi = 32 byte (16 kanal x gain+phase).
  ------------------------------------------------------------------------------
  constant C_BEAM_BYTES_PER_CHIP : natural := 32;
  constant C_BEAM_WORDS_PER_CHIP : natural := C_BEAM_BYTES_PER_CHIP / 4;  -- 8

  -- AXI-Stream paketinde bus b'nin ilk 32-bit kelimesinin indeksi
  -- (statik buslar pakette yer almaz)
  function f_bus_word_offset(b : natural) return natural;
  -- Paketteki toplam 32-bit kelime sayisi. Sadece C_BEAM_ON_TRIG = true olan
  -- buslar sayilir: 6 bus x 8 cip x 8 kelime = 384 kelime (1536 byte).
  function f_total_words return natural;
  -- Pakette veri tasiyan ilk bus / b'den sonraki ilk veri tasiyan bus
  -- (yoksa b dondurur)
  function f_first_data_bus return natural;
  function f_next_data_bus(b : natural) return natural;

  ------------------------------------------------------------------------------
  -- FRAME YARDIMCILARI
  ------------------------------------------------------------------------------
  -- 32 bitlik standart yazma frame'i
  function f_wr_frame(chip : unsigned(3 downto 0);
                      addr : std_logic_vector(15 downto 0);
                      data : std_logic_vector(7 downto 0))
                      return std_logic_vector;

  function f_wr_frame(chip : natural;
                      addr : std_logic_vector(15 downto 0);
                      data : std_logic_vector(7 downto 0))
                      return std_logic_vector;

  -- 24 bitlik streaming yazma header'i (32 bit vektorde sag hizali)
  function f_stream_hdr(chip : unsigned(3 downto 0);
                        addr : std_logic_vector(15 downto 0))
                        return std_logic_vector;

  -- 24 bitlik okuma header'i (32 bit vektorde sag hizali)
  function f_rd_hdr(chip : unsigned(3 downto 0);
                    addr : std_logic_vector(15 downto 0))
                    return std_logic_vector;

  -- 32 bitlik little-endian kelimeden byte sec (0 = en dusuk adres)
  function f_lane32(w : std_logic_vector(31 downto 0);
                    s : unsigned(1 downto 0))
                    return std_logic_vector;

  ------------------------------------------------------------------------------
  -- LOAD PINI ZAMANLAMASI (UG-2293 Table 2)
  --
  -- Tablodaki tek LOAD speci "Load Line Toggle Period (tCLK)": min 7.5 ns,
  -- typ 8 ns. SCLK'te oldugu gibi periyot = tHIGH + tLOW oldugundan, darbenin
  -- yuksek ve dusuk yarilarinin her biri en az 3.75 ns olmalidir.
  --
  -- f_load_half_cycles, verilen sistem saati frekansi icin bu 3.75 ns'i
  -- saglayan en kucuk clock sayisini dondurur (en az 1).
  --   100 MHz -> 1 clock (10 ns)   500 MHz -> 2 clock (4 ns)
  ------------------------------------------------------------------------------
  constant C_LOAD_HALF_PERIOD_PS : natural := 3750;   -- 3.75 ns

  function f_load_half_cycles(clk_freq_hz : natural) return natural;

  -- override = 0 ise spec'ten turet, degilse verilen degeri kullan
  function f_load_cycles(clk_freq_hz : natural; override : natural) return natural;

  ------------------------------------------------------------------------------
  -- FAZ SRAM TABLOSU (UG-2293 Table 11)
  --
  -- 64 faz durumu (0, -5.625, -11.25, ... -354.375 derece); her durum icin bir
  -- Q ve bir I katsayisi (7 bit).
  -- Yerlesim (ascending streaming, taban 0x480 / 0x580):
  --   byte 0..63   -> Q[0..63]   (0x480..0x4BF / 0x580..0x5BF)
  --   byte 64..127 -> I[0..63]   (0x4C0..0x4FF / 0x5C0..0x5FF)
  -- I egrisi Q egrisinin 16 adim (90 derece) otelenmis halidir: I[k]=Q[(k+16) mod 64].
  -- Ayni tablo hem TX hem RX icin gecerlidir.
  ------------------------------------------------------------------------------
  constant C_PHASE_STREAM : t_byte_array(0 to 127) := (
    -- Q katsayilari (0x480..0x4BF / 0x580..0x5BF)
    x"00", x"46", x"4C", x"52", x"58", x"5E", x"63", x"68",
    x"6D", x"71", x"74", x"78", x"7A", x"7C", x"7E", x"7F",
    x"7F", x"7F", x"7E", x"7C", x"7A", x"78", x"74", x"71",
    x"6D", x"68", x"63", x"5E", x"58", x"52", x"4C", x"46",
    x"00", x"06", x"0C", x"12", x"18", x"1E", x"23", x"28",
    x"2D", x"31", x"34", x"38", x"3A", x"3C", x"3E", x"3F",
    x"3F", x"3F", x"3E", x"3C", x"3A", x"38", x"34", x"31",
    x"2D", x"28", x"23", x"1E", x"18", x"12", x"0C", x"06",
    -- I katsayilari (0x4C0..0x4FF / 0x5C0..0x5FF)
    x"7F", x"7F", x"7E", x"7C", x"7A", x"78", x"74", x"71",
    x"6D", x"68", x"63", x"5E", x"58", x"52", x"4C", x"46",
    x"00", x"06", x"0C", x"12", x"18", x"1E", x"23", x"28",
    x"2D", x"31", x"34", x"38", x"3A", x"3C", x"3E", x"3F",
    x"3F", x"3F", x"3E", x"3C", x"3A", x"38", x"34", x"31",
    x"2D", x"28", x"23", x"1E", x"18", x"12", x"0C", x"06",
    x"00", x"46", x"4C", x"52", x"58", x"5E", x"63", x"68",
    x"6D", x"71", x"74", x"78", x"7A", x"7C", x"7E", x"7F"
  );

  ------------------------------------------------------------------------------
  -- STATIK BUS ACILIS BEAM TABLOLARI (bus 6)
  --
  -- C_BEAM_ON_TRIG = false olan bus, init sonunda (NVM kalibrasyonundan sonra)
  -- bu tablolari CIP CIP yazar: her cip kendi chip address'i ile adreslenir ve
  -- kendi satirini alir. Tum cipler yazildiktan sonra TEK LOAD toggle ile
  -- hepsi ayni anda yuklenir. Sonraki trig'lerde beam yazilmaz.
  --
  -- Tablo yapisi: cip basina 32 byte'lik bir satir. Satir indeksi zincirdeki
  -- siradir (0 = kontrolcuye en yakin cip; chip address = C_CHIP_ADDR(bus, c)).
  -- Kullanilmayan satirlar (bus 6'da 4..7) yazilmaz, sadece tanimli durur.
  --
  -- Satir ici indeks j = direct beam register offseti:
  --   C_STATIC_BEAM_RX(c)(j) -> cip c, register 0x200 + j  (RX blogu)
  --   C_STATIC_BEAM_TX(c)(j) -> cip c, register 0x240 + j  (TX blogu)
  --
  -- Byte anlamlari (gain/phase indeksleri bit[5:0], UG-2293 Table 79-82):
  --   j=0  ch0V gain   j=1  ch0V phase   j=2  ch0H gain   j=3  ch0H phase
  --   j=4  ch1V gain   j=5  ch1V phase   j=6  ch1H gain   j=7  ch1H phase
  --   j=8  ch2V gain   j=9  ch2V phase   j=10 ch2H gain   j=11 ch2H phase
  --   j=12 ch3V gain   j=13 ch3V phase   j=14 ch3H gain   j=15 ch3H phase
  --   j=16 ch4V gain   j=17 ch4V phase   j=18 ch4H gain   j=19 ch4H phase
  --   j=20 ch5V gain   j=21 ch5V phase   j=22 ch5H gain   j=23 ch5H phase
  --   j=24 ch6V gain   j=25 ch6V phase   j=26 ch6H gain   j=27 ch6H phase
  --   j=28 ch7V gain   j=29 ch7V phase   j=30 ch7H gain   j=31 ch7H phase
  --
  -- Bir cipe farkli beam vermek icin satirini acikca yazin, ornek:
  --   2 => (0 => x"3F", 1 => x"20", others => x"00"),  -- cip 2: ch0V ozel
  ------------------------------------------------------------------------------
  type t_static_beam is array (0 to C_MAX_CHIPS-1) of t_byte_array(0 to 31);

  -- varsayilan satir: tum kanallar gain indeksi 0, phase indeksi 0
  constant C_STATIC_ROW_ZERO : t_byte_array(0 to 31) := (others => x"00");

  constant C_STATIC_BEAM_RX : t_static_beam := (
    0      => C_STATIC_ROW_ZERO,   -- cip 0 (chip addr C_CHIP_ADDR(6,0))
    1      => C_STATIC_ROW_ZERO,   -- cip 1
    2      => C_STATIC_ROW_ZERO,   -- cip 2
    3      => C_STATIC_ROW_ZERO,   -- cip 3
    others => C_STATIC_ROW_ZERO    -- kullanilmayan slotlar (4..7)
  );

  constant C_STATIC_BEAM_TX : t_static_beam := (
    0      => C_STATIC_ROW_ZERO,   -- cip 0 (chip addr C_CHIP_ADDR(6,0))
    1      => C_STATIC_ROW_ZERO,   -- cip 1
    2      => C_STATIC_ROW_ZERO,   -- cip 2
    3      => C_STATIC_ROW_ZERO,   -- cip 3
    others => C_STATIC_ROW_ZERO    -- kullanilmayan slotlar (4..7)
  );

  ------------------------------------------------------------------------------
  -- ACILIS REGISTER TABLOSU
  ------------------------------------------------------------------------------
  type t_init_entry is record
    addr : std_logic_vector(15 downto 0);
    data : std_logic_vector(7 downto 0);
    load : std_logic;                    -- '1' -> frame sonrasi LOAD toggle
  end record;

  type t_init_table is array (natural range <>) of t_init_entry;

  -- Direct beam ortak kazanc baslangic degerleri (0 = kazanc offseti yok)
  constant C_RX_COM_GAIN_V : std_logic_vector(7 downto 0) := x"00";
  constant C_RX_COM_GAIN_H : std_logic_vector(7 downto 0) := x"00";
  constant C_TX_COM_GAIN_V : std_logic_vector(7 downto 0) := x"00";
  constant C_TX_COM_GAIN_H : std_logic_vector(7 downto 0) := x"00";

  ------------------------------------------------------------------------------
  -- UG-2293 "SPI Initialization" (sayfa 28) + Table 12 bias ayarlari.
  -- Sutun: ADMV48281 Band 0 (b3.0.13), genis bant (>= 400 MHz mod. BW).
  --
  -- Diger sutunlar icin degisen degerler:
  --   Band 1 genis bant : 0x102B=0x15 0x102C=0x97 0x102D=0x15 0x102E=0x0C
  --                       0x103E=0x25 0x1041=0x7C 0x1054=0x13 0x1072=0x12
  --                       0x1073=0x14 0x1074=0x0D
  --   Band 0 dar bant   : 0x102C=0x12 0x102D=0x1D
  --   Band 1 dar bant   : Band 1 farklari + 0x102C=0x18 0x102D=0x9C 0x102E=0x1C
  --
  -- Tablo tum ciplere broadcast (chip addr 0000, channel 00000) yazilir.
  ------------------------------------------------------------------------------
  constant C_INIT_TABLE : t_init_table := (
    -- 1) SPI konfigurasyonu: soft reset + 4-wire + streaming ascending.
    --    Palindrom yazilmasi zorunlu. Ring modunda RING_EN='1' oldugu icin
    --    SDO zaten daima aktiftir; bit4 etkisizdir ama zararsizdir.
    (x"0000", x"BD", '0'),

    -- 2) NVM devresi reseti
    (x"007E", x"40", '0'),
    (x"007E", x"54", '0'),

    -- 3) Table 12 bias ayarlari (Band 0, genis bant)
    (x"00C2", x"F7", '0'),   -- CM_TC_CTRL: ortak yol sicaklik komp. devresi kapali
    (x"00C3", x"2A", '0'),
    (x"00C4", x"2A", '0'),
    (x"00C6", x"14", '0'),
    (x"00D1", x"05", '0'),
    (x"00D2", x"14", '0'),
    (x"00D3", x"05", '0'),
    (x"00D4", x"14", '0'),
    (x"00D6", x"06", '0'),
    (x"00D8", x"0E", '0'),
    (x"00D9", x"00", '0'),
    (x"02FF", x"80", '1'),   -- TEMP_COMP_BYPASS - LOAD toggle gerekir
    (x"1021", x"00", '1'),   -- POWER_DOWN_BLOCKS_2 - LOAD toggle gerekir
    (x"1022", x"17", '0'),
    (x"1024", x"17", '0'),
    (x"1026", x"0C", '0'),
    (x"1028", x"07", '0'),
    (x"102B", x"09", '0'),
    (x"102C", x"1B", '0'),
    (x"102D", x"12", '0'),
    (x"102E", x"1E", '0'),
    (x"102F", x"0E", '0'),
    (x"1031", x"00", '0'),
    (x"1032", x"06", '0'),
    (x"1033", x"06", '0'),
    (x"103E", x"23", '0'),
    (x"1040", x"1C", '0'),
    (x"1041", x"5C", '0'),
    (x"1042", x"0A", '0'),
    (x"1045", x"03", '0'),
    (x"1046", x"4F", '0'),
    (x"1047", x"07", '0'),
    (x"1048", x"19", '0'),
    (x"104A", x"0E", '0'),
    (x"104B", x"17", '0'),
    (x"104C", x"01", '0'),
    (x"104F", x"07", '0'),   -- bit4=0 -> TX guc dedektorleri aktif
    (x"1053", x"46", '0'),
    (x"1054", x"12", '0'),
    (x"1070", x"0C", '0'),
    (x"1071", x"16", '0'),
    (x"1072", x"0D", '0'),
    (x"1073", x"12", '0'),
    (x"1074", x"10", '0'),
    (x"1075", x"33", '0'),

    -- 4) NVM band secimi: bit6=0 -> Band 0, bit6=1 -> Band 1
    (x"00C0", x"00", '0'),

    -- 5) Beam kaynagi: kanal ve ortak SRAM icin "manuel direct beam" (b'11).
    --    Varsayilan 0x00 "son yazilan adrese gore otomatik secim"dir; sadece
    --    direct beam kullandigimiz icin secimi sabitliyoruz.
    (x"00C1", x"0F", '0'),

    -- 6) Direct beam ortak kazanc registerlari baslangic degeri (+LOAD)
    (x"0280", C_RX_COM_GAIN_V, '0'),
    (x"0281", C_RX_COM_GAIN_H, '0'),
    (x"0288", C_TX_COM_GAIN_V, '0'),
    (x"0289", C_TX_COM_GAIN_H, '1')
  );

end package admv48281_pkg;


package body admv48281_pkg is

  function f_bus_word_offset(b : natural) return natural is
    variable v : natural := 0;
  begin
    for i in integer range 0 to C_NUM_BUS-1 loop
      exit when i >= b;
      if C_BEAM_ON_TRIG(i) then
        v := v + C_CHIPS_PER_BUS(i) * C_BEAM_WORDS_PER_CHIP;
      end if;
    end loop;
    return v;
  end function;

  function f_total_words return natural is
    variable v : natural := 0;
  begin
    for i in integer range 0 to C_NUM_BUS-1 loop
      if C_BEAM_ON_TRIG(i) then
        v := v + C_CHIPS_PER_BUS(i) * C_BEAM_WORDS_PER_CHIP;
      end if;
    end loop;
    return v;
  end function;

  function f_first_data_bus return natural is
  begin
    for i in integer range 0 to C_NUM_BUS-1 loop
      if C_BEAM_ON_TRIG(i) then
        return i;
      end if;
    end loop;
    return 0;
  end function;

  function f_next_data_bus(b : natural) return natural is
  begin
    for i in integer range 0 to C_NUM_BUS-1 loop
      if i > b and C_BEAM_ON_TRIG(i) then
        return i;
      end if;
    end loop;
    return b;   -- b son veri bus'i: demux zaten toplam sayacla durur
  end function;

  function f_wr_frame(chip : unsigned(3 downto 0);
                      addr : std_logic_vector(15 downto 0);
                      data : std_logic_vector(7 downto 0))
                      return std_logic_vector is
  begin
    return '0' & std_logic_vector(chip) & "00000" & addr(13 downto 0) & data;
  end function;

  function f_wr_frame(chip : natural;
                      addr : std_logic_vector(15 downto 0);
                      data : std_logic_vector(7 downto 0))
                      return std_logic_vector is
  begin
    return f_wr_frame(to_unsigned(chip, 4), addr, data);
  end function;

  function f_stream_hdr(chip : unsigned(3 downto 0);
                        addr : std_logic_vector(15 downto 0))
                        return std_logic_vector is
  begin
    return x"00" & '0' & std_logic_vector(chip) & "00000" & addr(13 downto 0);
  end function;

  function f_rd_hdr(chip : unsigned(3 downto 0);
                    addr : std_logic_vector(15 downto 0))
                    return std_logic_vector is
  begin
    return x"00" & '1' & std_logic_vector(chip) & "00000" & addr(13 downto 0);
  end function;

  function f_lane32(w : std_logic_vector(31 downto 0);
                    s : unsigned(1 downto 0))
                    return std_logic_vector is
  begin
    case s is
      when "00"   => return w(7 downto 0);
      when "01"   => return w(15 downto 8);
      when "10"   => return w(23 downto 16);
      when others => return w(31 downto 24);
    end case;
  end function;

  function f_load_half_cycles(clk_freq_hz : natural) return natural is
    -- MHz uzerinden calisilir; boylece 32 bit integer tasmasi olmaz
    constant C_MHZ : natural := clk_freq_hz / 1000000;
    variable n     : natural;
  begin
    -- ceil(3.75 ns * f) = ceil(C_MHZ * 3750 / 1000000) = ceil(C_MHZ * 15 / 4000)
    n := (C_MHZ * 15 + 3999) / 4000;
    if n < 1 then
      n := 1;
    end if;
    return n;
  end function;

  function f_load_cycles(clk_freq_hz : natural; override : natural) return natural is
  begin
    if override = 0 then
      return f_load_half_cycles(clk_freq_hz);
    else
      return override;
    end if;
  end function;

end package body admv48281_pkg;

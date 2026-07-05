# ADMV4828 SPI + DDR Modülü (Zybo Z7 / MicroBlaze)

DDR'dan SmartConnect üzerinden okuyup ADMV4828 beamformer'a SPI ile yazan,
SPI'dan okuma da yapabilen SPI master modülü. ADMV4828 initialize + NVM
kalibrasyon sekansını donanımda kendisi yürütür (UG-2293'e göre).

## Dosyalar

| Dosya | Açıklama |
|---|---|
| `rtl/spi_pkg.vhd` | Frame yardımcı fonksiyonları + Band 0 init tablosu (`C_INIT_TABLE`) |
| `rtl/spi_master.vhd` | SPI master çekirdeği: standart / streaming / clock-burst |
| `rtl/axi_ddr_reader.vhd` | Tek-beat AXI4 read master (SmartConnect'e bağlanır) |
| `rtl/spi_ddr_top.vhd` | Üst modül: init → faz SRAM → NVM kalibrasyon → periyodik beam stream |
| `sim/tb_spi_ddr_top.vhd` | Self-checking testbench (ADMV4828 davranış modeli dahil) |

## ADMV4828 frame formatı

Standart frame 32 bit, MSB önce (CPOL=0, CPHA=0):

```
bit 31    : R/W̄        (0 = yazma, 1 = okuma)
bit 30:27 : CHIP_ADDR  (tek çip / broadcast = 0000)
bit 26:22 : CHANNEL    (00000 = global / tüm kanallara broadcast)
bit 21:8  : REG ADDR   (A13:A0)
bit 7:0   : DATA
```

`spi_pkg`'daki `f_spi_wr(addr, data)` / `f_spi_rd(addr)` fonksiyonları bu
frame'leri üretir. Okuma = 24 bit header + 8 bit veri (aynı CS penceresi).

## Çalışma sırası (`enable='1'` sonrası)

1. **Init** — `C_INIT_TABLE`: `0x000←0xBD` (4-wire + streaming ascending),
   `0x07E←0x40/0x54` (NVM reset), Table 12 **Band 0 / geniş bant** bias
   ayarları (45 register), `0x0C0←0x00` (Band 0 seçimi). `0x2FF` ve `0x1021`
   sonrası LOAD otomatik toggle edilir.
2. **Faz SRAM** — DDR'daki 256 byte'lık tablo (`G_PHASE_TABLE_ADDR`) okunur;
   byte 0–127 → `0x480–0x4FF` (RX), byte 128–255 → `0x580–0x5FF` (TX).
   Tablo değerleri UG-2293 Table 11'den alınıp MicroBlaze ile DDR'a yazılır.
3. **NVM kalibrasyon** — `0x05F←0x01`, SCLK'da `G_NVM_BURST_CYCLES` pulse
   (≥10354, CS pasif), `0x01A` bit[6]=1 olana kadar poll, `0x05F←0x00`.
   Bitince `init_done='1'`.
4. **Periyodik beam stream** — her `G_PERIOD_CYCLES`'ta bir `idx_a..idx_e`'den
   eleman adresi hesaplanır, DDR'dan 32 byte okunur ve **tek streaming
   frame'de** (24 bit header + 32 byte = 280 clock) `stream_reg_addr`'dan
   başlayarak yazılır, ardından LOAD toggle edilir.
   `stream_reg_addr`: RX direct beam = `0x200` (varsayılan), TX = `0x240`.
5. **SPI okuma** — `user_rd_req` ile istenildiğinde: `user_rd_cmd = f_spi_rd(addr)`,
   `cmd_len=24`, `rd_len=8`. SRAM registerlarını (0x400+, 0x1400+) okurken
   komutu iki kez gönderin (UG-2293 gereksinimi).

## DDR yerleşimi (MicroBlaze, little-endian, C ile birebir)

```c
uint8_t beam[A][B][C][D][E][32];   // G_BASE_ADDR        (eleman = 32 byte)
uint8_t phase_table[256];          // G_PHASE_TABLE_ADDR (Table 11 değerleri)
```

Eleman adresi = `G_BASE_ADDR + 32*((((a*B+b)*C+c)*D+d)*E+e)`; byte j →
register `stream_reg_addr + j` (ascending mod).

## Saatleme

- Yazma: MOSI, dahili üretilen `spi_sclk_out`'a senkron sürülür.
- Okuma: MISO, `spi_sclk_in`'in **rising edge**'inde örneklenir; `sclk_in`
  clock olarak kullanılmaz (oversampling + kenar tespiti), dolayısıyla
  clock-capable pin/BUFG gerekmez. Ring modda `spi_sclk_in` ← son çipin
  `CLK_OUT`u; tek çipte kart üzerinde `sclk_out` loopback'i (normal modda
  ADMV4828 `CLK_OUT` boşta bırakılır).
- Hızlar ayrı: `f_sclk = f_clk / (2·div)`. Okuma için `rd_clk_div ≥ 4`
  (`f_sclk_rd ≤ f_clk/8`); ADMV4828 maks 133 MHz.
- Loopback gecikmesi SCLK yarım periyodundan uzunsa `RX_SETTLE_CYCLES`
  generic'ini büyütün.

## Donanım bağlantısı — ÖNEMLİ

- **ADMV4828 SPI pinleri 1.8V CMOS'tur** (V_IH maks 1.8V). Zybo Z7 Pmod
  bankları 3.3V — doğrudan bağlamayın, seviye dönüştürücü (ör. TXB0104)
  kullanın. Her SPI hattına seri 33Ω önerilir (UG-2293).
- Pin eşleşmesi (4-wire mod): `spi_mosi`→SDIO, `spi_miso`→SDO, `spi_cs_n`→CS,
  `spi_sclk_out`→SCLK, `spi_load`→LOAD. `RING_EN` ve `CHIP_ADD0–3`
  kullanılmıyorsa topraklanmalı (boşta bırakılamaz).
- `RST`, `CHIP_EN`, `RF_PD`, `TRX_V/H` pinleri ve power-up sırası ADMV4828
  datasheet'ine göre MicroBlaze GPIO'sundan yönetilmelidir; `enable`,
  power-up tamamlandıktan sonra verilmelidir.

## Block design entegrasyonu

1. `rtl/*.vhd` dosyalarını projeye ekleyin, BD'de **Add Module** →
   `spi_ddr_top`; `m_axi_*` otomatik AXI4 master olarak tanınır.
2. `M_AXI`'yi SmartConnect'in slave portuna bağlayın; Address Editor'de DDR
   aralığını map'leyin (`G_BASE_ADDR`/`G_PHASE_TABLE_ADDR` ile uyumlu).
3. `clk` = AXI saati, `rst_n` = aktif-düşük reset.

## Simülasyon

```
xvhdl rtl\spi_pkg.vhd rtl\spi_master.vhd rtl\axi_ddr_reader.vhd rtl\spi_ddr_top.vhd sim\tb_spi_ddr_top.vhd
xelab work.tb_spi_ddr_top -s tb_snap
xsim tb_snap -runall
```

Testbench ADMV4828 davranış modeli içerir (register dosyası, ascending
streaming, falling-edge SDO, NVM merge SCLK sayacı) ve şunları doğrular:
init tablosu, 256 byte'lık faz SRAM içeriği, NVM sekansı (0x05F/0x01A),
LOAD toggle sayısı, iki beam stream'inin (280 bit) DDR içeriğiyle byte-byte
eşleşmesi ve `sclk_in` üzerinden SPI okuması. Sonunda `TUM TESTLER BASARILI`
raporu basılır.

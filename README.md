# ADMV48281 SPI Sürücüsü (7 bus × ring, 52 çip, direct beam)

7 SPI bus üzerinde toplam **52 adet ADMV48281** beamformer'ı (6 × 8 çip + 1 × 4 çip,
**ring konfigürasyonu**) yöneten VHDL sürücü. Bus 0..5 her trig'de beam alır;
**bus 6 statiktir**: açılışta tek bir beam yüklenir, sonrasında beam yazılmaz
(TRX ve senkron LOAD yine sürülür). Açılışta UG-2293'teki init + faz SRAM +
NVM kalibrasyon sekansını donanımda kendisi yürütür, sonra idle'a geçip dışarıdan trig
bekler. Trig gelince beam verisini AXI4-Stream'den alıp **direct beam (bypass mode)**
register'larına streaming SPI ile yazar. LOAD pini dışarıdan `load_trig` ile toggle
edilir, ardından TRX_x pinleri sürülür.

Referans: *ADMV48281 Programming Reference Manual, UG-2293 Rev. Sp0*.

## Dosyalar

| Dosya | Açıklama |
|---|---|
| `rtl/admv48281_pkg.vhd` | Register haritası, frame fonksiyonları, dizi konfigürasyonu, **çip adresleri**, `C_BEAM_ON_TRIG`, **statik beam tabloları**, Table 12 init tablosu, Table 11 faz tablosu |
| `rtl/spi_master.vhd` | SPI master çekirdeği: standart 32-bit / streaming / okuma / clock-burst |
| `rtl/spi_slave.vhd` | Ring dönüş yolu alıcısı (dönen CLK_OUT/SDO'yu aşırı örnekleyip yakalar) |
| `rtl/admv_bus_ctrl.vhd` | Tek bus kontrolcüsü: init → faz SRAM → NVM cal → beam streaming; içinde 1 master + 1 slave + beam RAM |
| `rtl/admv48281_top.vhd` | Üst modül: AXI-Stream demux, trig sekansı, 7 bus, `load_trig`→LOAD→TRX sıralaması, okuma yönlendirme |
| `sim/admv48281_ring_model.vhd` | ADMV48281 ring zinciri davranış modeli (sadece simülasyon) |
| `sim/admv48281_tb_pkg.vhd` | İki testbench'in paylaştığı yardımcılar (beklenen beam byte'ı, hex, AXIS paket gönderme) |
| `sim/tb_admv48281_top.vhd` | Klasik self-checking testbench (VUnit gerektirmez) |
| `sim/vunit/tb_admv48281_vunit.vhd` + `run.py` | VUnit test paketi (18 test) |
| `docs/admv48281_referans.html` | Görsel referans: akış + timing diyagramları, sinyal ve veri formatı tabloları (tarayıcıda açın) |
| `legacy/` | Önceki DDR tabanlı implementasyon — derleme yoluna eklemeyin (bkz. `legacy/README.md`) |

## SPI frame formatı

Standart ADI frame'i 32 bit, MSB önce, CPOL=0/CPHA=0:

```
bit 31    : R/W#       0 = yazma, 1 = okuma
bit 30:27 : CHIP_ADDR  0000 = broadcast, 1..15 = tek çip
bit 26:22 : CHANNEL    00000 = global / tüm kanallar
bit 21:8  : REG ADDR   A13:A0
bit 7:0   : DATA
```

Streaming: 24 bit header + N × 8 bit veri, adres ascending (`0x000` ← `0xBD`).

## Ring modu — tasarımı belirleyen nokta

UG-2293 Figure 31'de ring bir **repeater zinciri**: her çip CLK_OUT/SDO'dan SCLK/SDIO'yu
yeniden sürer ve "SDO, SDIO verisini eş zamanlı çıkarır". Yani **zincirdeki 8 çip aynı
veriyi görür**; ayrışmayı yalnızca header'daki chip address sağlar.

Bunun iki sonucu var:

1. Her çipe **farklı** gain/phase yazmak → çip başına ayrı transaction.
   Bus başına 8 × (24 + 32×8) = **2240 SCLK**.
2. Broadcast (chip addr `0000`) ile **okuma yapılamaz**. NVM kalibrasyonundaki
   `0x01A` bit[6] beklemesi bu yüzden çip çip yapılır.

Ölçülen beam güncelleme süresi (simülasyon, 100 MHz clk / 25 MHz SCLK):
AXI-Stream transferi 4.2 µs + SPI yazma 89.6 µs ≈ **96 µs**. 7 bus paralel çalıştığı
için çip sayısı değil, **zincir uzunluğu** belirleyici. Daha hızlısı gerekiyorsa:

| SCLK | Gereken `clk` (`G_CLK_DIV_WR`=1) | Beam süresi |
|---|---|---|
| 25 MHz | 100 MHz (div=2) | ~90 µs |
| 50 MHz | 100 MHz (div=1) | ~45 µs |
| 100 MHz | 200 MHz (div=1) | ~22 µs |

`f_sclk = f_clk / (2 × div)`. ADMV48281 maksimumu 133 MHz.

## LOAD pini zamanlaması

UG-2293 Table 2'deki tek LOAD speci **Load Line Toggle Period (t_CLK) = min 7.5 ns,
typ 8 ns**. SCLK'te olduğu gibi periyot = t_HIGH + t_LOW olduğundan darbenin yüksek
ve düşük yarılarının her biri ≥ 3.75 ns olmalı.

Modül bunu `G_CLK_FREQ_HZ`'ten otomatik türetir (`f_load_half_cycles`,
[admv48281_pkg.vhd](rtl/admv48281_pkg.vhd)):

| f_clk | Yarım periyot | LOAD yüksek süresi |
|---|---|---|
| 100 MHz | 1 clock | 10 ns |
| 200 MHz | 1 clock | 5 ns |
| 500 MHz | 2 clock | 4 ns |

`G_LOAD_CYCLES` ile elle genişletebilirsiniz; spec minimumunun altına inerseniz
elaborasyonda `assert ... severity failure` ile durur.

Figure 4 ve Figure 10, LOAD'ın SPI işlemi bittikten (CS yükseldikten) **sonra**
darbelendiğini gösterir — modül de böyle yapar. CS→LOAD ve LOAD→TRX arası gecikme
için UG-2293'te spec yoktur; ikincisi `G_TRX_DELAY_CYCLES` generic'idir
(varsayılan 100 clock = 100 MHz'de 1 µs) ve **ADMV48281 datasheet'ine göre
ayarlanmalıdır**.

## Çip adresleri (`C_CHIP_ADDR`)

`rtl/admv48281_pkg.vhd` içinde sabit tablo — **karttaki hardwired CHIP_ADD0..3
pinleriyle birebir aynı olmalı**:

```vhdl
constant C_CHIP_ADDR : t_chip_addr_map := (
  (1, 2, 3, 4, 5, 6, 7, 8),   -- bus 0
  ...
  (1, 2, 3, 4, 0, 0, 0, 0)    -- bus 6: sadece 4 çip
);
constant C_CHIPS_PER_BUS : t_natural_array := (8, 8, 8, 8, 8, 8, 4);
```

Ring modunda `0000` yalnızca broadcast içindir; zincirdeki hiçbir çipe verilemez.

## Çalışma sırası (`enable` = '1' sonrası)

1. **Init** — `C_INIT_TABLE` broadcast ile yazılır (54 kayıt):
   `0x000←0xBD` (soft reset + 4-wire + ascending, palindrom), `0x07E←0x40/0x54`
   (NVM reset), Table 12 **Band 0 / geniş bant** bias (45 register),
   `0x0C0←0x00` (Band 0), `0x0C1←0x0F` (manuel direct beam seçimi),
   `0x280/0x281/0x288/0x289` (direct ortak kazanç). `0x2FF` ve `0x1021`
   sonrası LOAD toggle edilir.
2. **Faz SRAM** — Table 11'in 64 Q + 64 I katsayısı `0x480`'den ve `0x580`'den
   128'er byte streaming ile yazılır (broadcast).
3. **NVM kalibrasyon** — `0x05F←0x01`, CS pasif iken `G_NVM_BURST` SCLK darbesi
   (≥10354), her çipten `0x01A` bit[6] = 1 olana kadar polling, `0x05F←0x00`.
   Hepsi bitince `init_done='1'`.
4. **Idle** — trig bekler.
5. **Beam** — `trig` → `rx_tx_sel` kilitlenir → `s_axis_tready='1'` → 416 kelimelik
   paket bus RAM'lerine dağıtılır → her bus kendi çiplerine 32'şer byte streaming
   yazar → hepsi bitince `load_pending='1'`.
6. **LOAD** — dışarıdan `load_trig` gelince **tüm LOAD hatları eş zamanlı** toggle
   edilir. Darbe genişliği UG-2293 Table 2'den türetilir (aşağı bkz.) → `load_done`.
   `load_trig` SPI yazma bitmeden gelirse kuyruğa alınır, yazma biter bitmez uygulanır.
7. **TRX** — LOAD'dan `G_TRX_DELAY_CYCLES` sonra `trx_out` = `rx_tx_sel` yapılır
   (`trx_updated`), ardından `beam_done` darbesi.

Sıralama garantisi: **SPI yazma → LOAD → TRX**. TRX pini hiçbir zaman beam verisi
çalışan register'lara yüklenmeden önce yön değiştirmez. Açılışta `trx_out = 0`
(receive), UG-2293'ün TRX_x pin açıklamasına uygun.

Beam turu boyunca (`trig`'den `beam_done`'a kadar) kullanıcı SPI okuması kabul
edilmez: araya giren bir okuma, Figure 10'daki "LOAD öncesi son yazma beam
register'ı olmalı" koşulunu bozabilir.

## AXI4-Stream paket düzeni

TDATA 32 bit, little-endian byte sırası, toplam **384 kelime (1536 byte)**, TLAST sonda.
Paket **yalnızca dinamik busları** (`C_BEAM_ON_TRIG = true`, bus 0..5) taşır; statik
bus 6 pakette yer almaz:

```
kelime   0.. 63 : bus 0, çip 0..7      (çip başına 8 kelime = 32 byte)
kelime  64..127 : bus 1
kelime 128..191 : bus 2
kelime 192..255 : bus 3
kelime 256..319 : bus 4
kelime 320..383 : bus 5
```

Bir çipin 32 byte'ı doğrudan direct beam register bloğunun içeriğidir
(`0x200` veya `0x240`, `rx_tx_sel`'e göre):

```
byte  0 : ch0V gain    byte  1 : ch0V phase
byte  2 : ch0H gain    byte  3 : ch0H phase
...
byte 30 : ch7H gain    byte 31 : ch7H phase
```

Gain indeksi bit[5:0], phase indeksi bit[5:0] (UG-2293 Table 79–82).
C tarafında birebir karşılığı:

```c
uint8_t beam[6][8][32];   // bus (0..5), cip, byte -- statik bus 6 pakette yok
```

**Tampon semantiği (store & forward):** `s_axis_tready` yalnızca trig sonrası
'1' olur; paket tamamen alındıktan sonra SPI yazma başlar.

## Statik bus (bus 6)

`C_BEAM_ON_TRIG` tablosunda `false` işaretli bus statiktir (şu an sadece bus 6):

- **Açılışta bir kez**: NVM kalibrasyonundan sonra `C_STATIC_BEAM_RX` (→ `0x200..0x21F`)
  ve `C_STATIC_BEAM_TX` (→ `0x240..0x25F`) tabloları **çip çip** yazılır — her çip
  kendi chip address'i ile adreslenir ve tablodaki kendi satırını alır — tümü
  yazıldıktan sonra **tek LOAD** ile hepsi aynı anda yüklenir.
  Tablolar [admv48281_pkg.vhd](rtl/admv48281_pkg.vhd) içinde: çip başına 32 byte'lık
  bir satır (`C_STATIC_BEAM_RX(c)(j)` → çip c, register `0x200+j`); hangi j'nin
  hangi kanalın gain/phase'i olduğu tablonun üstündeki eşleme yorumunda yazılıdır.
  Şu an tüm satırlar `C_STATIC_ROW_ZERO` (gain=0, phase=0) ve RX=TX; bir çipe
  farklı beam vermek için o satırı açıkça yazmak yeterli:

  ```vhdl
  2 => (0 => x"3F", 1 => x"20", others => x"00"),  -- cip 2: ch0V ozel
  ```
- **Trig'lerde**: SPI yazması yapılmaz (`beam_start` el sıkışmasına "hazırım"
  diye katılır).
- **Yine de sürülenler**: senkron LOAD darbesi (`load_trig` ile, dizi lockstep —
  aynı değerler yeniden yüklenir, zararsız) ve `trx_out(6)` (diğer buslarla aynı
  anda, aynı sırayla RX/TX geçişi yapar).

## Portlar

```
enable        : ADMV power-up tamamlandıktan sonra '1'
trig          : yükselen kenar, beam turunu başlatır
rx_tx_sel     : '0' = RX direct beam (0x200), '1' = TX direct beam (0x240)
load_trig     : yükselen kenar, LOAD toggle'ını tetikler (erken gelirse kuyruğa alınır)

init_done     : 7 bus da init'i bitirdi
init_err      : NVM polling zaman aşımı
busy          : init veya beam turu sürüyor
load_pending  : seviye, SPI yazma bitti / load_trig bekleniyor
load_done     : LOAD toggle tamamlandı (1 clock)
trx_updated   : TRX pini güncellendi (1 clock)
beam_done     : beam turu tamamen bitti (1 clock)
axis_err      : paket uzunluğu / TLAST uyumsuzluğu

rd_req/rd_bus/rd_chip/rd_addr -> rd_data/rd_valid : tek register SPI okuması
                (0x400 ve üzeri SRAM adreslerinde komut otomatik iki kez gönderilir;
                 istek sadece beam turu dışında kabul edilir)

spi_sclk_out[6:0], spi_mosi[6:0], spi_cs_n[6:0], spi_load[6:0]  (çıkış)
trx_out[6:0]                                     (çıkış, '0'=receive '1'=transmit)
spi_sclk_in[6:0], spi_miso[6:0]                  (giriş, ring dönüşü)
```

`trx_out` bus başına bir pindir; TRX_V ve TRX_H kartta birleştirilir (UG-2293 buna
izin veriyor). Ayrı V/H yönü gerekiyorsa `rx_tx_sel`'in de iki bite çıkması gerekir.

## Saatleme ve ring dönüşü

- Yazma: MOSI, üretilen `spi_sclk_out`'un düşen kenarında sürülür; çip yükselen
  kenarda örnekler.
- Okuma: MISO, **dönen** `spi_sclk_in`'in (zincirin son çipinin CLK_OUT'u) yükselen
  kenarında örneklenir — kontrolcünün kendi SCLK'i ile **değil**. Dönen clock ile
  SDO eş zamanlı geldiği için doğru referans odur.
- `spi_sclk_in` **clock olarak kullanılmaz**: kodda hiçbir yerde
  `rising_edge(spi_sclk_in)` yoktur. Sinyal sistem saatinde aşırı örneklenip kenarı
  tespit edilir, dolayısıyla clock-capable pin, BUFG veya clock kaynağı gerekmez.
  Gereken tek şey yeterli aşırı örnekleme oranıdır: `2 × G_CLK_DIV_RD ≥ 8`
  (elaborasyonda `assert` ile kontrol edilir).

### Okumada bit kayması — üç önlem

Clock-capable olmayan bir pinde, seviye dönüştürücülü ve uzun ring izli bir hatta
okunan baytın kayması tipik bir arızadır. Modül buna karşı:

1. **Giriş filtresi** (`G_RX_FILTER_LEN`, varsayılan 3) — `spi_sclk_in` ve
   `spi_miso` aynı yapıda debounce edilir. Tek bir glitch fazladan kenar sayımına,
   yani tüm baytın kaymasına yol açar. İki sinyal aynı filtreden geçtiği için
   aralarındaki kenar hizası korunur.
2. **Bit sayısına göre yakalama** — veri, okuma penceresi kapandığında değil,
   sayaç beklenen bit sayısına (24 + 8N) ulaştığı anda kilitlenir. Böylece dönen
   son kenar geç gelse bile bayt kaymaz; pencerenin ne zaman kapandığı önemsizleşir.
3. **`rd_short_err[6:0]`** — beklenen bit sayısına hiç ulaşılamazsa ilgili bus için
   kalıcı olarak yanar. Donanımda "veri neden kaydı" sorusunun cevabı budur:
   yanıyorsa dönen kenarlar pencereye sığmıyor demektir.

Pencere koşulu:

```
G_CS_HOLD_CYCLES + G_RX_SETTLE_CYCLES
    > ring_gecikmesi / T_clk + 2 (senkronizasyon) + G_RX_FILTER_LEN
```

Varsayılan `G_RX_SETTLE_CYCLES = 64` (100 MHz'de 640 ns), 8 çiplik gerçek zincirin
~26 ns gecikmesine karşı bol paydır. Okumalar seyrek olduğu için geniş bırakmanın
maliyeti yoktur.

## Donanım bağlantısı — ÖNEMLİ

- **ADMV48281 SPI pinleri 1.8V CMOS'tur.** Zybo Z7 Pmod bankları 3.3V — doğrudan
  bağlamayın, seviye dönüştürücü kullanın veya bankı 1.8V VCCO ile besleyin.
  Her SPI hattına seri 33Ω önerilir (UG-2293).
- Ring pin eşleşmesi: kontrolcü `spi_sclk_out`→1. çip SCLK, `spi_mosi`→1. çip SDIO;
  her çipin CLK_OUT/SDO'su bir sonrakinin SCLK/SDIO'suna; son çipin CLK_OUT/SDO'su
  kontrolcünün `spi_sclk_in`/`spi_miso`'suna. `spi_cs_n` ve `spi_load` tüm çiplere
  dağıtılır.
- **`RING_EN` = 1.8V** olmalı (ring modu), `CHIP_ADD0..3` her çipte `C_CHIP_ADDR`
  ile aynı değere hardwire edilmeli. Bu pinler asla boşta bırakılamaz.
- `TRX_V`/`TRX_H` artık modül tarafından sürülür (`trx_out`, bus başına bir pin;
  V ve H kartta birleştirilir). `RST`, `CHIP_EN`, `RF_PD` pinleri ve power-up
  sırası hâlâ ADMV48281 datasheet'ine göre harici olarak (ör. MicroBlaze/PS GPIO)
  yönetilmelidir. `enable`, power-up tamamlandıktan sonra verilir.
- Pin bütçesi: 7 bus × 7 hat (SCLK, MOSI, CS, LOAD, TRX + dönüş CLK/MISO)
  = **49 pin** (hepsi 1.8V).

## Konfigürasyon

Generic'ler (`admv48281_top`):

| Generic | Varsayılan | Açıklama |
|---|---|---|
| `G_CLK_FREQ_HZ` | 100 000 000 | sistem saati; LOAD darbe genişliği bundan türetilir |
| `G_CLK_DIV_WR` | 2 | yazma SCLK böleni |
| `G_CLK_DIV_RD` | 8 | okuma SCLK böleni (≥4) |
| `G_NVM_BURST` | 10500 | NVM merge clock sayısı (≥10354) |
| `G_LOAD_CYCLES` | 0 | LOAD yarım periyodu; **0 = UG-2293 Table 2'den türet** |
| `G_TRX_DELAY_CYCLES` | 100 | LOAD → TRX bekleme (100 MHz'de 1 µs) |
| `G_RESET_WAIT` | 1000 | soft reset sonrası bekleme (clk) |
| `G_POLL_LIMIT` | 2000 | NVM polling deneme sınırı |
| `G_RX_SETTLE_CYCLES` | 64 | okuma penceresi payı (yukarıdaki koşula bakın) |
| `G_RX_FILTER_LEN` | 3 | dönen CLK_OUT/SDO giriş filtresi; 1 = filtre yok |

Bant seçimi: `C_INIT_TABLE` Band 0 / geniş bant sütunudur. Band 1 ve dar bant
farkları tablonun hemen üstündeki yorumda listelidir.

## Simülasyon

Her iki testbench de 7 bus'a birer `admv48281_ring_model` bağlar (52 çip modellenir).

### VUnit (tercih edilen)

```bash
cd sim/vunit && python run.py
```

Gereksinim: `pip install vunit_hdl` ve desteklenen bir simülatör (NVC, GHDL,
ModelSim/Questa, Riviera-PRO, Xcelium). Faydalı bayraklar: `-l` test listesi,
`-v` ayrıntılı çıktı, `-p 4` paralel koşum, `--gui` dalga formu,
`python run.py "*test_rx_beam*"` tek test.

18 test (her biri ayrı simülasyonda, temiz reset'ten başlar):

| Test | Doğruladığı |
|---|---|
| `test_init_sequence` (`real_nvm`) | Init tablosu 52 çipte, faz SRAM = Table 11, gerçek 10354-clock NVM merge + `0x01A` polling, init LOAD toggle'ları |
| `test_rx_beam` × 5 config | RX direct beam yazımı + beam başına bir LOAD. Config'ler: ring gecikmesi 8/100/400 ns, SCLK böleni 1/2 |
| `test_tx_beam` | TX bloğu doğru, RX bloğu bozulmuyor, TX turu başına bir LOAD |
| `test_back_to_back_beams` | Ard arda iki beam, ikincisi üstüne yazıyor, iki LOAD |
| `test_spi_read` | Ring üzerinden okuma; regular + SRAM (çift okuma) + direct beam geri okuma |
| `test_axis_short_packet` | Erken TLAST → `axis_err`, modül kilitlenmiyor, sonraki tam paket doğru |
| `test_load_trig_gating` | `load_trig` gelmeden LOAD toggle edilmiyor, `beam_done` gelmiyor, TRX değişmiyor; geldiğinde tam olarak bir LOAD |
| `test_load_trig_early` | SPI yazma bitmeden gelen `load_trig` kuyruğa alınıp bir kez uygulanıyor |
| `test_load_pulse_width` × 2 config | LOAD darbe genişliği UG-2293 minimumunu (3.75 ns) sağlıyor ve gereğinden geniş değil. Config'ler: 100 MHz (10 ns) ve 500 MHz (4 ns) |
| `test_trx_sequence` | Açılışta TRX=0; RX beam→0, TX beam→1, tekrar RX→0. TRX geçişi LOAD'dan **sonra** ve `G_TRX_DELAY_CYCLES` gecikmesiyle (ölçülen 1.03 µs) |
| `test_rx_window_too_short` | Okuma penceresi ring gecikmesinden küçükken veri **sessizce kaymıyor**, `rd_short_err` yanıyor |
| `test_static_bus` | Statik bus: açılış beam'i gerçekten yazılıyor (model sentinel'i 0xAA→tablo), init'te +1 LOAD, trig'lerde **hiç CS aktivitesi yok**, bloklar değişmiyor, TRX ve senkron LOAD takip ediyor |
| `test_nvm_timeout` | NVM bit[6] hiç gelmezse `init_err` |

Test paketi **mutasyon testinden geçirildi** — her mutasyon yalnızca beklenen
testlerde hata veriyor, diğerleri geçmeye devam ediyor:

| Sokulan hata | Yakalayan |
|---|---|
| Init tablosunda tek byte bozulması | 2 test |
| Beam byte lane seçiminin kaydırılması | 8 test |
| Beam LOAD darbesinin kaldırılması | 7 test |
| TRX'in LOAD'dan **önce** güncellenmesi | `test_trx_sequence` |
| `load_trig`'in yok sayılıp LOAD'ın otomatik verilmesi | `test_load_trig_gating` |
| Statik açılış beam'inde TX yazımının atlanması | `test_init_sequence` + `test_static_bus` |
| Statik bus'ın trig'de beam yazması | `test_static_bus` |
| Statik yazımların hepsinin tek çipin adresiyle gitmesi | `test_init_sequence` + `test_static_bus` (adreslenmeyen çipler sentinel'de kalır) |
| Okuma verisinin bit sayısı yerine pencere kapanışına göre yakalanması | `test_rx_window_too_short` |

### Klasik testbench (VUnit'siz)

NVC ile:

```bash
nvc --std=93 -a rtl/admv48281_pkg.vhd rtl/spi_master.vhd rtl/spi_slave.vhd rtl/admv_bus_ctrl.vhd rtl/admv48281_top.vhd sim/admv48281_tb_pkg.vhd sim/admv48281_ring_model.vhd sim/tb_admv48281_top.vhd && nvc --std=93 -e tb_admv48281_top && nvc --std=93 -r tb_admv48281_top
```

Vivado xsim ile:

```bash
xvhdl rtl/admv48281_pkg.vhd rtl/spi_master.vhd rtl/spi_slave.vhd rtl/admv_bus_ctrl.vhd rtl/admv48281_top.vhd sim/admv48281_tb_pkg.vhd sim/admv48281_ring_model.vhd sim/tb_admv48281_top.vhd
```

Tek doğrusal sekans halinde init + faz SRAM + NVM + RX/TX beam + SPI okuması
kontrol eder; başarılıysa `TUM TESTLER BASARILI` basar. RTL VHDL-93 ve
VHDL-2008'de derlenir; VUnit akışı 2008 kullanır.

## Bilinen sınırlar

- Ortak kazanç register'ları (`0x280/0x281/0x288/0x289`) init'te bir kez yazılır,
  beam akışına dahil değildir. Beam başına değiştirilmesi gerekirse paket
  formatına eklenmelidir.
- Sıcaklık kompanzasyon tabloları (`0x300-0x37F`) yüklenmez; Table 12 değerleri
  (`0x00C2←0xF7`, `0x02FF←0x80`) temp-comp devresini bypass eder.
- Beam position SRAM (`0x1800-0x37FF`) yüklenmez — beam pointer modu değil,
  direct beam kullanıldığı için gerekmiyor.
- Statik tabloların satırları şu an birbirinin aynı (hepsi 0) olduğu için
  "çip c satır c'yi aldı" eşlemesi veri düzeyinde ancak satırlar farklılaşınca
  gözlemlenebilir; her çipin kendi adresiyle gerçekten yazıldığı ise sentinel
  sayesinde testte kanıtlanıyor.

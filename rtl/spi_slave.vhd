--------------------------------------------------------------------------------
-- spi_slave.vhd
--
-- Ring konfigurasyonunda zincirin sonundan kontrolcuye donen SPI akisini
-- yakalayan alici. Ring bir repeater zinciridir: her cip CLK_OUT/SDO pinlerinden
-- SCLK/SDIO'yu yeniden surer, son cipin CLK_OUT/SDO'su kontrolcunun CLKIN/SDI
-- pinlerine gelir (UG-2293, Figure 31).
--
-- ORNEKLEME KAYNAGI: veri, kontrolcunun kendi SCLK'i ile DEGIL, cipten donen
-- CLK_OUT ile ornekleir. Donen clock 8 ciplik zincirde kayda deger gecikme
-- biriktirdigi ve SDO ile es zamanli geldigi icin dogru referans odur.
--
-- Donen clock CLOCK OLARAK KULLANILMAZ: hicbir yerde rising_edge(sclk_in) yoktur.
-- Sinyal sistem saatinde asiri orneklenip kenari tespit edilir, bu yuzden
-- clock-capable pin, BUFG veya clock kaynagi gerekmez. Gereken tek sey yeterli
-- asiri ornekleme oranidir:  f_clk / f_sclk_donen = 2 * G_CLK_DIV_RD >= 8.
--
-- Kayma (bit shift) hatalarina karsi uc onlem:
--   1) GIRIS FILTRESI - sclk_in ve sdi_in ayni yapida debounce edilir. Clock
--      capable olmayan bir pinde, seviye donusturuculu ve uzun ring izli bir
--      hatta olusan tek bir glitch fazladan kenar sayimina, yani tum bayta
--      kaymaya yol acar. Iki sinyal ayni filtreden gectigi icin aralarindaki
--      kenar hizasi korunur.
--   2) BIT SAYISINA GORE YAKALAMA - veri, pencere kapandiginda degil, sayac
--      beklenen bit sayisina (n_bits) ulastigi anda kilitlenir. Boylece donen
--      son kenar gec gelse bile bayt kaymaz; pencerenin ne zaman kapandigi
--      onemsizlesir.
--   3) short_err - beklenen bit sayisi hic ulasilmazsa isaretlenir. Donanimda
--      "veri neden kaydi" sorusunun cevabi bu bayrak ve bit_cnt cikisidir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_slave is
  generic (
    -- Giris filtresi: bir seviyenin kabul edilmesi icin gereken ardisik ayni
    -- ornek sayisi. 1 = filtre yok. G_CLK_DIV_RD'den kucuk olmalidir, aksi
    -- halde clock'un yuksek/dusuk fazi filtreyi gecemez.
    G_FILTER_LEN  : natural := 3;
    -- true  : donen CLK_OUT'un YUKSELEN kenarinda ornekle (UG-2293 Table 2,
    --         "CLK_OUT will be ready to sample at CLK_OUT rising edge")
    -- false : dusen kenarda ornekle (yalniz sahada dogrulama icin)
    G_SAMPLE_RISE : boolean := true
  );
  port (
    clk   : in std_logic;
    rst_n : in std_logic;

    -- okuma penceresi (spi_master.rd_arm)
    arm : in std_logic;
    -- bu islemde beklenen TOPLAM bit sayisi (24 bit header + 8*N veri)
    n_bits : in std_logic_vector(7 downto 0);

    -- ring donus pinleri
    sclk_in : in std_logic;   -- son cipin CLK_OUT'u
    sdi_in  : in std_logic;   -- son cipin SDO'su

    data      : out std_logic_vector(7 downto 0);
    bit_cnt   : out std_logic_vector(7 downto 0);  -- yakalanan kenar sayisi
    valid     : out std_logic;                     -- 1 clock darbe
    short_err : out std_logic                      -- beklenen bit sayisina ulasilamadi
  );
end entity spi_slave;

architecture rtl of spi_slave is

  -- iki kademeli senkronizasyon (metastabilite)
  signal sclk_s : std_logic_vector(1 downto 0) := (others => '0');
  signal sdi_s  : std_logic_vector(1 downto 0) := (others => '0');

  -- ayni yapida debounce: gecikmeleri esit oldugu icin kenar hizasi bozulmaz
  signal sclk_f   : std_logic := '0';
  signal sdi_f    : std_logic := '0';
  signal sclk_cnt : integer range 0 to G_FILTER_LEN := 0;
  signal sdi_cnt  : integer range 0 to G_FILTER_LEN := 0;
  signal sclk_d   : std_logic := '0';

  signal arm_d : std_logic := '0';
  signal shreg : std_logic_vector(7 downto 0) := (others => '0');
  signal cnt   : unsigned(7 downto 0) := (others => '0');
  signal got   : std_logic := '0';   -- beklenen bit sayisina ulasildi

  signal data_r  : std_logic_vector(7 downto 0) := (others => '0');
  signal cnt_r   : unsigned(7 downto 0) := (others => '0');
  signal vld_r   : std_logic := '0';
  signal serr_r  : std_logic := '0';

  signal sample_edge : std_logic;

begin

  -- filtrelenmis clock uzerinde kenar tespiti
  sample_edge <= (sclk_f and not sclk_d) when G_SAMPLE_RISE
                 else (not sclk_f and sclk_d);

  data      <= data_r;
  bit_cnt   <= std_logic_vector(cnt_r);
  valid     <= vld_r;
  short_err <= serr_r;

  process (clk)
    variable nxt : std_logic_vector(7 downto 0);
  begin
    if rising_edge(clk) then
      vld_r <= '0';

      if rst_n = '0' then
        sclk_s   <= (others => '0');
        sdi_s    <= (others => '0');
        sclk_f   <= '0';
        sdi_f    <= '0';
        sclk_d   <= '0';
        sclk_cnt <= 0;
        sdi_cnt  <= 0;
        arm_d    <= '0';
        shreg    <= (others => '0');
        cnt      <= (others => '0');
        got      <= '0';
        data_r   <= (others => '0');
        cnt_r    <= (others => '0');
        serr_r   <= '0';

      else
        ----------------------------------------------------------------------
        -- senkronizasyon
        sclk_s <= sclk_s(0) & sclk_in;
        sdi_s  <= sdi_s(0)  & sdi_in;
        arm_d  <= arm;

        ----------------------------------------------------------------------
        -- giris filtresi: bir seviye ancak G_FILTER_LEN kez ust uste
        -- goruldugunde kabul edilir (temiz sinyalde sabit gecikme, glitch'te ret)
        if sclk_s(1) /= sclk_f then
          if sclk_cnt >= G_FILTER_LEN - 1 then
            sclk_f   <= sclk_s(1);
            sclk_cnt <= 0;
          else
            sclk_cnt <= sclk_cnt + 1;
          end if;
        else
          sclk_cnt <= 0;
        end if;

        if sdi_s(1) /= sdi_f then
          if sdi_cnt >= G_FILTER_LEN - 1 then
            sdi_f   <= sdi_s(1);
            sdi_cnt <= 0;
          else
            sdi_cnt <= sdi_cnt + 1;
          end if;
        else
          sdi_cnt <= 0;
        end if;

        sclk_d <= sclk_f;

        ----------------------------------------------------------------------
        -- yakalama
        if arm = '1' and arm_d = '0' then
          -- pencere aciliyor
          shreg  <= (others => '0');
          cnt    <= (others => '0');
          got    <= '0';
          serr_r <= '0';

        elsif arm = '1' then
          if sample_edge = '1' then
            nxt   := shreg(6 downto 0) & sdi_f;
            shreg <= nxt;
            cnt   <= cnt + 1;

            -- Veriyi TAM bu anda kilitle. Pencerenin ne zaman kapandigina
            -- bagli olmadigi icin donen son kenar gec gelse de bayt kaymaz.
            if got = '0' and (cnt + 1) = unsigned(n_bits) then
              data_r <= nxt;
              cnt_r  <= cnt + 1;
              vld_r  <= '1';
              got    <= '1';
            end if;
          end if;

        elsif arm = '0' and arm_d = '1' then
          -- pencere kapandi
          cnt_r <= cnt;
          if got = '0' then
            -- beklenen bit sayisina ulasilamadi: veri kaymis olabilir.
            -- Yine de son 8 biti ver, ama hatayi isaretle.
            data_r <= shreg;
            vld_r  <= '1';
            serr_r <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;

end architecture rtl;

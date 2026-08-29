--------------------------------------------------------------------------------
-- spi_slave.vhd
--
-- Ring konfigurasyonunda zincirin sonundan kontrolcuye donen SPI akisini
-- yakalayan alici. Ring bir repeater zinciridir: her cip CLK_OUT/SDO pinlerinden
-- SCLK/SDIO'yu yeniden surer, son cipin CLK_OUT/SDO'su kontrolcunun CLKIN/SDI
-- pinlerine gelir (UG-2293, Figure 31).
--
-- Donen clock 8 ciplik zincirde kayda deger gecikme biriktirdigi icin veri
-- kontrolcunun kendi SCLK'i ile degil, DONEN clock ile ornekleir. Donen clock
-- BUFG/clock-capable pin gerektirmez: sistem saatinde asiri orneklenip kenar
-- tespiti yapilir (f_clk / f_sclk >= 4 olmalidir).
--
-- Okuma penceresi boyunca (arm = '1') tum bitler kaydirilir; pencere kapandiginda
-- 8 bitlik kaydiricida son 8 bit, yani okunan register verisi kalir. Onceki 24
-- bit ring uzerinden geri donen header'dir.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_slave is
  port (
    clk   : in std_logic;
    rst_n : in std_logic;

    -- okuma penceresi (spi_master.rd_arm)
    arm : in std_logic;

    -- ring donus pinleri
    sclk_in : in std_logic;   -- son cipin CLK_OUT'u
    sdi_in  : in std_logic;   -- son cipin SDO'su

    data    : out std_logic_vector(7 downto 0);
    bit_cnt : out std_logic_vector(11 downto 0);  -- teshis: yakalanan bit sayisi
    valid   : out std_logic                       -- 1 clock darbe
  );
end entity spi_slave;

architecture rtl of spi_slave is

  -- iki kademeli senkronizasyon; sclk ve sdi ayni gecikmeyi gordugu icin
  -- aralarindaki kenar hizasi korunur
  signal sclk_s : std_logic_vector(2 downto 0) := (others => '0');
  signal sdi_s  : std_logic_vector(1 downto 0) := (others => '0');

  signal arm_d  : std_logic := '0';
  signal shreg  : std_logic_vector(7 downto 0) := (others => '0');
  signal cnt    : unsigned(11 downto 0) := (others => '0');
  signal data_r : std_logic_vector(7 downto 0) := (others => '0');
  signal cnt_r  : unsigned(11 downto 0) := (others => '0');
  signal vld_r  : std_logic := '0';

  signal sclk_rise : std_logic;

begin

  sclk_rise <= sclk_s(1) and not sclk_s(2);

  data    <= data_r;
  bit_cnt <= std_logic_vector(cnt_r);
  valid   <= vld_r;

  process (clk)
  begin
    if rising_edge(clk) then
      vld_r <= '0';

      if rst_n = '0' then
        sclk_s <= (others => '0');
        sdi_s  <= (others => '0');
        arm_d  <= '0';
        shreg  <= (others => '0');
        cnt    <= (others => '0');
        data_r <= (others => '0');
        cnt_r  <= (others => '0');

      else
        sclk_s <= sclk_s(1 downto 0) & sclk_in;
        sdi_s  <= sdi_s(0) & sdi_in;
        arm_d  <= arm;

        if arm = '1' and arm_d = '0' then
          -- pencere aciliyor
          shreg <= (others => '0');
          cnt   <= (others => '0');

        elsif arm = '1' then
          if sclk_rise = '1' then
            -- donen clock'un yukselen kenarinda ornekle (Table 2: CLK_OUT
            -- yukselen kenarinda SDO ornege hazirdir), MSB once
            shreg <= shreg(6 downto 0) & sdi_s(1);
            cnt   <= cnt + 1;
          end if;

        elsif arm = '0' and arm_d = '1' then
          -- pencere kapandi: son 8 bit okunan veridir
          data_r <= shreg;
          cnt_r  <= cnt;
          vld_r  <= '1';
        end if;
      end if;
    end if;
  end process;

end architecture rtl;

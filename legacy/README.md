# Eski tasarim (DDR tabanli, ADMV4828)

Bu klasor, `rtl/` altindaki yeni ADMV48281 surucusunden onceki AXI4/DDR
tabanli implementasyonu barindirir. Yeni tasarim veriyi DDR yerine
AXI4-Stream'den alir ve 7 SPI bus / 52 cip yonetir.

Bu dosyalar **derleme yoluna dahil edilmemelidir**: `legacy/spi_master.vhd`
ile `rtl/spi_master.vhd` ayni entity adini kullanir ve ayni projede birlikte
bulunursa cakisirlar.

Gerek kalmazsa silinebilir; icerikleri git gecmisinde 22c6e6c commit'inde durur.

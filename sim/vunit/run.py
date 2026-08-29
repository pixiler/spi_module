#!/usr/bin/env python3
"""VUnit kosum betigi - ADMV48281 SPI surucusu.

Kullanim:
    python run.py                       # tum testleri kostur
    python run.py -l                    # test listesini goster
    python run.py -v                    # ayrintili cikti
    python run.py "*test_rx_beam*"       # tek test
    python run.py -p 4                  # 4 paralel is parcacigi
    python run.py --gui                 # dalga formu ile ac

Simulator: NVC (VUNIT_SIMULATOR=nvc ile acikca secilebilir).
"""

from pathlib import Path
from vunit import VUnit

ROOT = Path(__file__).resolve().parent.parent.parent
RTL = ROOT / "rtl"
SIM = ROOT / "sim"

vu = VUnit.from_argv(compile_builtins=False)
vu.add_vhdl_builtins()

lib = vu.add_library("admv_lib")

# RTL - bagimlilik sirasi VUnit tarafindan otomatik cozulur
lib.add_source_files(RTL / "*.vhd")

# Simulasyon destek dosyalari.
# Klasik testbench (sim/tb_admv48281_top.vhd) bilerek dahil edilmez: VUnit
# testbench'i degildir (runner_cfg generic'i yoktur) ve ayri kosturulur.
lib.add_source_files(SIM / "admv48281_tb_pkg.vhd")
lib.add_source_files(SIM / "admv48281_ring_model.vhd")
lib.add_source_files(SIM / "vunit" / "tb_admv48281_vunit.vhd")

tb = lib.test_bench("tb_admv48281_vunit")

# --------------------------------------------------------------------------
# test_init_sequence: varsayilan kisa NVM'e ek olarak gercek 10354 clock'luk
# merge sekansini de kostur.
# --------------------------------------------------------------------------
tb.test("test_init_sequence").add_config(
    name="real_nvm",
    generics=dict(G_NVM_BURST=10500, G_NVM_CLOCKS=10354),
)

# --------------------------------------------------------------------------
# test_rx_beam: ring yayilim gecikmesi taramasi. 8 ciplik gercek zincir
# ~26 ns; ust sinir G_RX_SETTLE_CYCLES penceresiyle belirlenir.
# --------------------------------------------------------------------------
for dly in (8, 100, 400):
    tb.test("test_rx_beam").add_config(
        name=f"ring_dly_{dly}ns",
        generics=dict(G_RING_DLY_NS=dly),
    )

# SCLK bolen taramasi: div=1 -> f_clk/2, div=2 -> f_clk/4
for div_wr in (1, 2):
    tb.test("test_rx_beam").add_config(
        name=f"sclk_div_{div_wr}",
        generics=dict(G_CLK_DIV_WR=div_wr),
    )

# --------------------------------------------------------------------------
# test_load_pulse_width: LOAD darbe genisligi UG-2293 Table 2'den (toggle
# periyodu >= 7.5 ns) turetilir. Farkli sistem saatlerinde de spec'i saglayan
# en kucuk clock sayisinin secildigi dogrulanir.
#   100 MHz -> 1 clock = 10 ns     500 MHz -> 2 clock = 4 ns
# --------------------------------------------------------------------------
for f_mhz in (100, 500):
    tb.test("test_load_pulse_width").add_config(
        name=f"clk_{f_mhz}mhz",
        generics=dict(G_CLK_FREQ_HZ=f_mhz * 1_000_000),
    )

# --------------------------------------------------------------------------
# test_nvm_timeout: model NVM bit6'yi hicbir zaman set etmez, polling siniri
# kucuk tutulur; init_err beklenir.
# --------------------------------------------------------------------------
tb.test("test_nvm_timeout").add_config(
    name="unreachable_nvm",
    generics=dict(G_NVM_CLOCKS=10_000_000, G_POLL_LIMIT=5),
)

vu.main()

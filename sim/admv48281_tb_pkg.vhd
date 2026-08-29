--------------------------------------------------------------------------------
-- admv48281_tb_pkg.vhd  (sadece simulasyon)
--
-- Hem klasik testbench (tb_admv48281_top) hem de VUnit testbench'i
-- (tb_admv48281_vunit) tarafindan paylasilan yardimcilar:
--   * beam paketi icin beklenen byte formulu
--   * hex bicimlendirme
--   * AXI4-Stream beam paketi gonderme proseduru
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.admv48281_pkg.all;

package admv48281_tb_pkg is

  type t_peek_data is array (0 to C_NUM_BUS-1) of std_logic_vector(7 downto 0);
  type t_natvec    is array (0 to C_NUM_BUS-1) of natural;

  -- Beklenen beam byte'i. p (pass) farkli turleri birbirinden ayirmak icin.
  function f_exp(b : natural; c : natural; j : natural; p : natural)
                 return std_logic_vector;

  function f_hex(v : std_logic_vector) return string;
  function f_hex16(n : natural) return string;

  -- Beam paketini AXI4-Stream'den gonder (sadece C_BEAM_ON_TRIG buslari,
  -- f_total_words kelime).
  -- truncate_at = 0  -> tam paket, TLAST son kelimede
  -- truncate_at = N  -> N kelime gonderip TLAST ile erken bitir (hata testi)
  procedure send_beam_packet(
    signal   clk         : in  std_logic;
    signal   tready      : in  std_logic;
    signal   tvalid      : out std_logic;
    signal   tdata       : out std_logic_vector(31 downto 0);
    signal   tlast       : out std_logic;
    constant p           : in  natural;
    constant truncate_at : in  natural);

end package admv48281_tb_pkg;


package body admv48281_tb_pkg is

  function f_exp(b : natural; c : natural; j : natural; p : natural)
                 return std_logic_vector is
  begin
    return std_logic_vector(
             to_unsigned((((b * C_MAX_CHIPS + c) * C_BEAM_BYTES_PER_CHIP + j)
                          + p * 173) mod 256, 8));
  end function;

  function f_hex(v : std_logic_vector) return string is
    constant C_DIG : string(1 to 16) := "0123456789ABCDEF";
    variable u : unsigned(v'length-1 downto 0) := unsigned(v);
    variable r : string(1 to (v'length + 3) / 4);
  begin
    for i in r'range loop
      r(r'length - i + 1) := C_DIG(to_integer(u(3 downto 0)) + 1);
      u := shift_right(u, 4);
    end loop;
    return r;
  end function;

  function f_hex16(n : natural) return string is
  begin
    return f_hex(std_logic_vector(to_unsigned(n, 16)));
  end function;

  procedure send_beam_packet(
    signal   clk         : in  std_logic;
    signal   tready      : in  std_logic;
    signal   tvalid      : out std_logic;
    signal   tdata       : out std_logic_vector(31 downto 0);
    signal   tlast       : out std_logic;
    constant p           : in  natural;
    constant truncate_at : in  natural) is

    variable w     : std_logic_vector(31 downto 0);
    variable last  : std_logic;
    variable n     : natural := 0;
    variable stop  : boolean := false;
  begin
    for b in 0 to C_NUM_BUS-1 loop
      -- statik buslar (C_BEAM_ON_TRIG = false) pakette yer almaz
      next when not C_BEAM_ON_TRIG(b);
      for c in 0 to C_CHIPS_PER_BUS(b)-1 loop
        for k in 0 to C_BEAM_WORDS_PER_CHIP-1 loop

          -- little-endian: byte 4k -> TDATA[7:0]
          w := f_exp(b, c, 4*k + 3, p) & f_exp(b, c, 4*k + 2, p)
               & f_exp(b, c, 4*k + 1, p) & f_exp(b, c, 4*k + 0, p);

          n := n + 1;

          if truncate_at > 0 then
            if n = truncate_at then
              last := '1';
              stop := true;
            else
              last := '0';
            end if;
          elsif n = f_total_words then
            last := '1';
          else
            last := '0';
          end if;

          tdata  <= w;
          tlast  <= last;
          tvalid <= '1';
          wait until rising_edge(clk) and tready = '1';

          exit when stop;
        end loop;
        exit when stop;
      end loop;
      exit when stop;
    end loop;

    tvalid <= '0';
    tlast  <= '0';
  end procedure;

end package body admv48281_tb_pkg;

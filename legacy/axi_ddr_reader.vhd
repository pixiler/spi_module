--------------------------------------------------------------------------------
-- axi_ddr_reader.vhd
-- DDR'dan tek kelime (32 bit) okuyan basit AXI4 read-only master.
-- Block design'da SmartConnect'in slave portuna baglanir; SmartConnect
-- protokol/genislik uyumunu kendisi halleder (MIG / PS DDR fark etmez).
--
-- Kullanim: addr gecerliyken req'e 1 clk'lik pulse ver; done pulse'i ile
-- birlikte rdata gecerlidir. axi_err, SLVERR/DECERR durumunda '1' olur.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi_ddr_reader is
  port (
    clk    : in  std_logic;
    rst_n  : in  std_logic;

    -- kullanici arayuzu
    req     : in  std_logic;
    addr    : in  std_logic_vector(31 downto 0);
    rdata   : out std_logic_vector(31 downto 0);
    done    : out std_logic;
    axi_err : out std_logic;

    -- AXI4 read address kanali
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arcache : out std_logic_vector(3 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;

    -- AXI4 read data kanali
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rlast   : in  std_logic;
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic
  );
end entity axi_ddr_reader;

architecture rtl of axi_ddr_reader is

  type t_state is (ST_IDLE, ST_AR, ST_R);
  signal state : t_state := ST_IDLE;

  signal araddr_r  : std_logic_vector(31 downto 0) := (others => '0');
  signal arvalid_r : std_logic := '0';
  signal rready_r  : std_logic := '0';
  signal rdata_r   : std_logic_vector(31 downto 0) := (others => '0');
  signal done_r    : std_logic := '0';
  signal err_r     : std_logic := '0';

begin

  -- tek beat, 4 byte, INCR burst, normal non-cacheable buffered erisim
  m_axi_arlen   <= x"00";
  m_axi_arsize  <= "010";
  m_axi_arburst <= "01";
  m_axi_arcache <= "0011";
  m_axi_arprot  <= "000";

  m_axi_araddr  <= araddr_r;
  m_axi_arvalid <= arvalid_r;
  m_axi_rready  <= rready_r;

  rdata   <= rdata_r;
  done    <= done_r;
  axi_err <= err_r;

  p_fsm : process(clk)
  begin
    if rising_edge(clk) then
      done_r <= '0';

      if rst_n = '0' then
        state     <= ST_IDLE;
        arvalid_r <= '0';
        rready_r  <= '0';
        err_r     <= '0';
      else
        case state is

          when ST_IDLE =>
            if req = '1' then
              araddr_r  <= addr;
              arvalid_r <= '1';
              state     <= ST_AR;
            end if;

          when ST_AR =>
            if m_axi_arready = '1' then
              arvalid_r <= '0';
              rready_r  <= '1';
              state     <= ST_R;
            end if;

          when ST_R =>
            if m_axi_rvalid = '1' then
              rdata_r <= m_axi_rdata;
              err_r   <= m_axi_rresp(1);
              if m_axi_rlast = '1' then
                rready_r <= '0';
                done_r   <= '1';
                state    <= ST_IDLE;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process p_fsm;

end architecture rtl;

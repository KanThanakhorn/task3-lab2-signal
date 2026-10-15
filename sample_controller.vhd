library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- Sample Controller
-- Generates:
-- 1. sample_tick: 1-clock-cycle pulse at 16,000 Hz (16 kS/s)
-- 2. sample_toggle: square wave toggling every sample (8,000 Hz, period 125 us)
--    Used for oscilloscope hardware timing verification (Lab Question 1 & Procedure 6)
--------------------------------------------------------------------------------

entity sample_controller is
    port (
        clk_50mhz     : in  std_logic;
        reset_n       : in  std_logic;

        sample_tick   : out std_logic;
        sample_toggle : out std_logic
    );
end entity sample_controller;

architecture rtl of sample_controller is

    -- 50,000,000 / 16,000 = 3125 clock cycles
    constant CLOCK_FREQ  : integer := 50_000_000;
    constant SAMPLE_FREQ : integer := 16_000;
    constant SAMPLE_DIV  : integer := CLOCK_FREQ / SAMPLE_FREQ;

    signal counter    : integer range 0 to SAMPLE_DIV - 1 := 0;
    signal toggle_reg : std_logic := '0';

begin

    sample_toggle <= toggle_reg;

    process(clk_50mhz)
    begin
        if rising_edge(clk_50mhz) then
            if reset_n = '0' then
                counter     <= 0;
                sample_tick <= '0';
                toggle_reg  <= '0';
            else
                if counter = SAMPLE_DIV - 1 then
                    counter     <= 0;
                    sample_tick <= '1';
                    toggle_reg  <= not toggle_reg; -- Toggles on each sample event
                else
                    counter     <= counter + 1;
                    sample_tick <= '0';
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
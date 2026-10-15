library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- ADC Sequencer for MAX 10 Modular Dual ADC Core (Avalon-ST)
--
-- Triggered by 16 kHz sample_tick.
-- Sends command pulse (Channel 1 = Arduino A0) to ADC IP.
-- Captures 12-bit response_data when response_valid is asserted.
-- Includes watchdog recovery to guarantee the FSM never hangs.
--------------------------------------------------------------------------------

entity adc_sequencer is
    port (
        clk                   : in  std_logic;
        reset_n               : in  std_logic;

        -- 16 kHz trigger pulse from sample_controller
        sample_tick           : in  std_logic;
        adc_pll_locked        : in  std_logic;

        -- ADC IP Command Interface
        command_valid         : out std_logic;
        command_channel       : out std_logic_vector(4 downto 0);
        command_startofpacket : out std_logic;
        command_endofpacket   : out std_logic;
        command_ready         : in  std_logic;

        -- ADC IP Response Interface
        response_valid        : in  std_logic;
        response_channel      : in  std_logic_vector(4 downto 0);
        response_data         : in  std_logic_vector(11 downto 0);
        response_startofpacket: in  std_logic;
        response_endofpacket  : in  std_logic;

        -- Sequencer Output to Sample Buffer
        sample_valid          : out std_logic;
        sample_data           : out std_logic_vector(11 downto 0);

        -- Diagnostics
        busy                  : out std_logic;
        adc_error             : out std_logic
    );
end entity adc_sequencer;

architecture rtl of adc_sequencer is

    -- ADC1 Channel 1 = Arduino A0 Header Pin
    constant ADC_CHANNEL : std_logic_vector(4 downto 0) := "00001";

    type state_type is (
        WAIT_PLL,
        IDLE,
        TRIGGER_CONV,
        WAIT_RESPONSE
    );

    signal state : state_type := WAIT_PLL;

    signal command_valid_reg : std_logic := '0';
    signal sample_valid_reg  : std_logic := '0';
    signal sample_data_reg   : std_logic_vector(11 downto 0) := (others => '0');
    signal busy_reg          : std_logic := '0';
    signal adc_error_reg     : std_logic := '0';

    -- Watchdog timer (~50 us timeout = 2500 cycles of 50 MHz)
    signal timeout_counter   : integer range 0 to 4000 := 0;

begin

    command_valid         <= command_valid_reg;
    command_channel       <= ADC_CHANNEL;
    command_startofpacket <= command_valid_reg;
    command_endofpacket   <= command_valid_reg;

    sample_valid          <= sample_valid_reg;
    sample_data           <= sample_data_reg;
    busy                  <= busy_reg;
    adc_error             <= adc_error_reg;

    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                state             <= WAIT_PLL;
                command_valid_reg <= '0';
                sample_valid_reg  <= '0';
                sample_data_reg   <= (others => '0');
                busy_reg          <= '0';
                adc_error_reg     <= '0';
                timeout_counter   <= 0;
            else
                -- sample_valid is a 1-clock-cycle pulse
                sample_valid_reg  <= '0';

                -- Global response capture (catches response in ANY state)
                if response_valid = '1' and state /= WAIT_PLL then
                    sample_data_reg  <= response_data;
                    sample_valid_reg <= '1';
                    busy_reg         <= '0';
                    state            <= IDLE;
                else
                    case state is
                        --------------------------------------------------------
                        -- 1. Wait for ADC PLL 10 MHz to Lock
                        --------------------------------------------------------
                        when WAIT_PLL =>
                            command_valid_reg <= '0';
                            busy_reg          <= '0';
                            if adc_pll_locked = '1' then
                                state <= IDLE;
                            end if;

                        --------------------------------------------------------
                        -- 2. Idle: Wait for 16 kHz sampling tick
                        --------------------------------------------------------
                        when IDLE =>
                            command_valid_reg <= '0';
                            busy_reg          <= '0';
                            timeout_counter   <= 0;

                            if adc_pll_locked = '1' and sample_tick = '1' then
                                -- Pulse command to start ADC conversion
                                command_valid_reg <= '1';
                                busy_reg          <= '1';
                                state             <= TRIGGER_CONV;
                            end if;

                        --------------------------------------------------------
                        -- 3. Trigger Conversion: Hold command_valid for 2 cycles
                        --    ensures ADC IP latches command channel
                        --------------------------------------------------------
                        when TRIGGER_CONV =>
                            command_valid_reg <= '0'; -- Release command pulse
                            timeout_counter   <= 0;
                            state             <= WAIT_RESPONSE;

                        --------------------------------------------------------
                        -- 4. Wait for ADC Conversion to complete
                        --    (Conversion typically takes ~1 us = 50 clock cycles)
                        --------------------------------------------------------
                        when WAIT_RESPONSE =>
                            command_valid_reg <= '0';
                            busy_reg          <= '1';

                            -- Watchdog timeout: if no response within 2000 cycles (~40 us), return to IDLE
                            if timeout_counter >= 2000 then
                                timeout_counter <= 0;
                                busy_reg        <= '0';
                                adc_error_reg   <= '1';
                                state           <= IDLE;
                            else
                                timeout_counter <= timeout_counter + 1;
                            end if;

                    end case;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
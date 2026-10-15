library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- Task 3 Top-Level Entity: MAX 10 FPGA Audio Sampling System
--
-- Target Device: Altera MAX 10 (10M50DAF484C7G, DE10-Lite)
--
-- Features:
-- 1. Hardware-controlled 16 kS/s sampling using on-die ADC (Channel 1 = Arduino A0)
-- 2. Hardware timing verification signal on TEST_PIN (Arduino D6)
--    Toggles every sample event (8.0 kHz square wave, period = 125 us)
-- 3. Ping-Pong double buffer (2x 1024 samples) in Block RAM (M9K)
-- 4. Handshake DATA_READY pin (Arduino D8) to notify ESP32
-- 5. Robust SPI Slave in Mode 0 (D10=CS, D11=MOSI, D12=MISO, D13=SCK)
-- 6. On-board diagnostics:
--    - LEDR0: ADC PLL Lock
--    - LEDR1: ADC Command Ready
--    - LEDR2: ADC Sampling Activity
--    - LEDR3: DATA_READY Block Indicator
--    - LEDR[9..4]: Real-time audio amplitude VU meter
--    - HEX[3..0]: 7-segment display of live ADC reading (Hex)
--------------------------------------------------------------------------------

entity task3_top is
    port (
        ----------------------------------------------------------------
        -- 50 MHz System Clock (PIN_P11)
        ----------------------------------------------------------------
        CLOCK_50   : in  std_logic;

        ----------------------------------------------------------------
        -- Active-low Reset Button (KEY0, PIN_B8)
        ----------------------------------------------------------------
        RESET_N    : in  std_logic;

        ----------------------------------------------------------------
        -- SPI Interface to ESP32-S3
        ----------------------------------------------------------------
        SPI_SCLK   : in  std_logic; -- Arduino D13 (PIN_AB20)
        SPI_CS_N   : in  std_logic; -- Arduino D10 (PIN_AB19)
        SPI_MOSI   : in  std_logic; -- Arduino D11 (PIN_AA19)
        SPI_MISO   : out std_logic; -- Arduino D12 (PIN_Y19)

        ----------------------------------------------------------------
        -- Handshake & Timing Output Pins to ESP32 / Oscilloscope
        ----------------------------------------------------------------
        DATA_READY : out std_logic; -- Arduino D8  (PIN_AB17)
        TEST_PIN   : out std_logic; -- Arduino D6  (PIN_AA11)

        ----------------------------------------------------------------
        -- Diagnostic LEDs (LEDR0 - LEDR9)
        ----------------------------------------------------------------
        LEDR       : out std_logic_vector(9 downto 0);

        ----------------------------------------------------------------
        -- 7-Segment Displays (HEX0 - HEX3)
        ----------------------------------------------------------------
        HEX0       : out std_logic_vector(7 downto 0);
        HEX1       : out std_logic_vector(7 downto 0);
        HEX2       : out std_logic_vector(7 downto 0);
        HEX3       : out std_logic_vector(7 downto 0)
    );
end entity task3_top;

architecture rtl of task3_top is

    --------------------------------------------------------------------
    -- Sample Controller Signals
    --------------------------------------------------------------------
    signal sample_tick   : std_logic;
    signal sample_toggle : std_logic;

    --------------------------------------------------------------------
    -- ADC PLL Signals
    --------------------------------------------------------------------
    signal adc_pll_clock  : std_logic;
    signal adc_pll_locked : std_logic;
    signal pll_reset      : std_logic;

    --------------------------------------------------------------------
    -- MAX10 ADC IP Signals
    --------------------------------------------------------------------
    signal adc_command_valid   : std_logic;
    signal adc_command_channel : std_logic_vector(4 downto 0);
    signal adc_command_sop     : std_logic;
    signal adc_command_eop     : std_logic;
    signal adc_command_ready   : std_logic;

    signal adc_response_valid   : std_logic;
    signal adc_response_channel : std_logic_vector(4 downto 0);
    signal adc_response_data    : std_logic_vector(11 downto 0);
    signal adc_response_sop     : std_logic;
    signal adc_response_eop     : std_logic;

    --------------------------------------------------------------------
    -- ADC Sequencer Output
    --------------------------------------------------------------------
    signal sample_valid : std_logic;
    signal sample_data  : std_logic_vector(11 downto 0);
    signal adc_busy     : std_logic;
    signal adc_error    : std_logic;

    --------------------------------------------------------------------
    -- Double Buffer & SPI Handshake
    --------------------------------------------------------------------
    signal block_ready    : std_logic;
    signal transfer_done  : std_logic;
    signal buffer_read    : std_logic;
    signal buffer_address : std_logic_vector(9 downto 0);
    signal buffer_data    : std_logic_vector(11 downto 0);

    --------------------------------------------------------------------
    -- Audio Level & Display Registers
    --------------------------------------------------------------------
    signal latest_sample    : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal display_sample   : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal display_timer    : integer range 0 to 2047 := 0;
    signal activity_counter : integer range 0 to 4095 := 0;
    signal peak_diff        : integer range 0 to 4095 := 0;
    signal vu_decay_counter : integer range 0 to 4095 := 0;
    signal audio_amplitude  : unsigned(11 downto 0) := (others => '0');
    signal vu_meter         : std_logic_vector(5 downto 0) := (others => '0');
    signal sample_activity  : std_logic := '0';

    --------------------------------------------------------------------
    -- Helper function: 7-segment active-low decoder
    --------------------------------------------------------------------
    function hex_to_7seg(digit : std_logic_vector(3 downto 0)) return std_logic_vector is
    begin
        case digit is
            when "0000" => return "11000000"; -- 0
            when "0001" => return "11111001"; -- 1
            when "0010" => return "10100100"; -- 2
            when "0011" => return "10110000"; -- 3
            when "0100" => return "10011001"; -- 4
            when "0101" => return "10010010"; -- 5
            when "0110" => return "10000010"; -- 6
            when "0111" => return "11111000"; -- 7
            when "1000" => return "10000000"; -- 8
            when "1001" => return "10010000"; -- 9
            when "1010" => return "10001000"; -- A
            when "1011" => return "10000011"; -- b
            when "1100" => return "11000110"; -- C
            when "1101" => return "10100001"; -- d
            when "1110" => return "10000110"; -- E
            when "1111" => return "10001110"; -- F
            when others => return "11111111";
        end case;
    end function;

begin

    pll_reset <= not RESET_N;

    --------------------------------------------------------------------
    -- Physical Output Pin Assignments
    --------------------------------------------------------------------
    TEST_PIN   <= sample_toggle;
    DATA_READY <= block_ready;

    --------------------------------------------------------------------
    -- 1. ADC PLL (50 MHz in -> 10 MHz out for MAX 10 ADC)
    --------------------------------------------------------------------
    u_adc_pll : entity work.adc_pll
        port map (
            areset => pll_reset,
            inclk0 => CLOCK_50,
            c0     => adc_pll_clock,
            locked => adc_pll_locked
        );

    --------------------------------------------------------------------
    -- 2. Sample Controller (16 kS/s sampling pulse and toggle)
    --------------------------------------------------------------------
    u_sample_controller : entity work.sample_controller
        port map (
            clk_50mhz     => CLOCK_50,
            reset_n       => RESET_N,
            sample_tick   => sample_tick,
            sample_toggle => sample_toggle
        );

    --------------------------------------------------------------------
    -- 3. MAX 10 On-Chip ADC IP Core (ADC1, Channel 1 = Arduino A0)
    --------------------------------------------------------------------
    u_max10_adc : entity work.max10_adc
        port map (
            clock_clk               => CLOCK_50,
            reset_sink_reset_n      => RESET_N,
            adc_pll_clock_clk       => adc_pll_clock,
            adc_pll_locked_export   => adc_pll_locked,

            -- Command interface
            command_valid           => adc_command_valid,
            command_channel         => adc_command_channel,
            command_startofpacket   => adc_command_sop,
            command_endofpacket     => adc_command_eop,
            command_ready           => adc_command_ready,

            -- Command 2 interface (mirrored with command 1)
            command_2_valid         => adc_command_valid,
            command_2_channel       => adc_command_channel,
            command_2_startofpacket => adc_command_sop,
            command_2_endofpacket   => adc_command_eop,
            command_2_ready         => open,

            -- Response interface
            response_valid          => adc_response_valid,
            response_channel        => adc_response_channel,
            response_data           => adc_response_data,
            response_startofpacket  => adc_response_sop,
            response_endofpacket    => adc_response_eop,

            -- Unused response 2
            response_2_valid         => open,
            response_2_channel       => open,
            response_2_data          => open,
            response_2_startofpacket => open,
            response_2_endofpacket   => open
        );

    --------------------------------------------------------------------
    -- 4. ADC Sequencer (Orchestrates conversion on sample_tick)
    --------------------------------------------------------------------
    u_adc_sequencer : entity work.adc_sequencer
        port map (
            clk                    => CLOCK_50,
            reset_n                => RESET_N,
            sample_tick            => sample_tick,
            adc_pll_locked         => adc_pll_locked,

            command_valid          => adc_command_valid,
            command_channel        => adc_command_channel,
            command_startofpacket  => adc_command_sop,
            command_endofpacket    => adc_command_eop,
            command_ready          => adc_command_ready,

            response_valid         => adc_response_valid,
            response_channel       => adc_response_channel,
            response_data          => adc_response_data,
            response_startofpacket => adc_response_sop,
            response_endofpacket   => adc_response_eop,

            sample_valid           => sample_valid,
            sample_data            => sample_data,
            busy                   => adc_busy,
            adc_error              => adc_error
        );

    --------------------------------------------------------------------
    -- 5. Ping-Pong Double Sample Buffer (2x 1024 samples Block RAM)
    --------------------------------------------------------------------
    u_sample_buffer : entity work.sample_buffer
        port map (
            clk           => CLOCK_50,
            reset_n       => RESET_N,
            sample_valid  => sample_valid,
            adc_data      => sample_data,
            block_ready   => block_ready,
            transfer_done => transfer_done,
            read_enable   => buffer_read,
            read_address  => buffer_address,
            read_data     => buffer_data
        );

    --------------------------------------------------------------------
    -- 6. Synchronous SPI Slave (Connects to ESP32-S3 Master)
    --------------------------------------------------------------------
    u_spi_slave : entity work.spi_slave
        port map (
            clk           => CLOCK_50,
            reset_n       => RESET_N,
            spi_sclk      => SPI_SCLK,
            spi_cs_n      => SPI_CS_N,
            spi_mosi      => SPI_MOSI,
            spi_miso      => SPI_MISO,
            block_ready   => block_ready,
            buffer_data   => buffer_data,
            buffer_addr   => buffer_address,
            buffer_read   => buffer_read,
            transfer_done => transfer_done
        );

    --------------------------------------------------------------------
    -- 7. Real-Time Diagnostics & VU Meter Processing
    --------------------------------------------------------------------
    process(CLOCK_50)
        variable diff : integer;
    begin
        if rising_edge(CLOCK_50) then
            if RESET_N = '0' then
                latest_sample    <= to_unsigned(2048, 12);
                display_sample   <= to_unsigned(2048, 12);
                display_timer    <= 0;
                audio_amplitude  <= (others => '0');
                sample_activity  <= '0';
                activity_counter <= 0;
                peak_diff        <= 0;
                vu_decay_counter <= 0;
                vu_meter         <= (others => '0');
            else
                if sample_valid = '1' then
                    latest_sample <= unsigned(sample_data);

                    -- Activity blinker: toggles every 2048 samples (~4 Hz visible blink)
                    if activity_counter >= 2047 then
                        activity_counter <= 0;
                        sample_activity  <= not sample_activity;
                    else
                        activity_counter <= activity_counter + 1;
                    end if;

                    -- 7-Segment display refresh rate: update every 1024 samples (~15 Hz)
                    if display_timer >= 1023 then
                        display_timer  <= 0;
                        display_sample <= unsigned(sample_data);
                    else
                        display_timer <= display_timer + 1;
                    end if;

                    -- Compute AC magnitude relative to DC offset (~2048)
                    diff := abs(to_integer(unsigned(sample_data)) - 2048);
                    audio_amplitude <= to_unsigned(diff, 12);

                    -- Fast-attack peak detector
                    if diff > peak_diff then
                        peak_diff        <= diff;
                        vu_decay_counter <= 0;
                    else
                        -- Smooth decay (~16 ms per step)
                        if vu_decay_counter >= 256 then
                            vu_decay_counter <= 0;
                            if peak_diff > 20 then
                                peak_diff <= peak_diff - (peak_diff / 8) - 5;
                            else
                                peak_diff <= 0;
                            end if;
                        else
                            vu_decay_counter <= vu_decay_counter + 1;
                        end if;
                    end if;

                    -- 6-LED volume meter thresholding with peak hold
                    if peak_diff > 800 then
                        vu_meter <= "111111";
                    elsif peak_diff > 400 then
                        vu_meter <= "011111";
                    elsif peak_diff > 200 then
                        vu_meter <= "001111";
                    elsif peak_diff > 100 then
                        vu_meter <= "000111";
                    elsif peak_diff > 50 then
                        vu_meter <= "000011";
                    elsif peak_diff > 20 then
                        vu_meter <= "000001";
                    else
                        vu_meter <= "000000";
                    end if;
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------
    -- LED Indicators:
    -- LEDR0: PLL locked
    -- LEDR1: ADC command ready
    -- LEDR2: ADC activity blinker (~4 Hz visible blink)
    -- LEDR3: DATA_READY state
    -- LEDR[9..4]: 6-level audio volume VU meter (Peak hold + decay)
    --------------------------------------------------------------------
    LEDR(0) <= adc_pll_locked;
    LEDR(1) <= adc_command_ready;
    LEDR(2) <= sample_activity;
    LEDR(3) <= block_ready;
    LEDR(9 downto 4) <= vu_meter;

    --------------------------------------------------------------------
    -- 7-Segment Displays:
    -- HEX3: Shows '1' with dot (denotes ADC1 Ch1)
    -- HEX2..HEX0: Displays raw 12-bit ADC reading in Hex (0x000 - 0xFFF)
    --------------------------------------------------------------------
    HEX0 <= hex_to_7seg(std_logic_vector(display_sample(3 downto 0)));
    HEX1 <= hex_to_7seg(std_logic_vector(display_sample(7 downto 4)));
    HEX2 <= hex_to_7seg(std_logic_vector(display_sample(11 downto 8)));
    HEX3 <= "01111001"; -- Displays digit "1." (Ch1)

end architecture rtl;
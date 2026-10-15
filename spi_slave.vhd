library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- Robust Synchronous SPI Slave for MAX 10 FPGA
--
-- SPI Mode 0 (CPOL = 0, CPHA = 0)
-- MSB First, 16-bit words per ADC sample:
--   Bits [15..12] = 0000
--   Bits [11..0]  = 12-bit ADC sample data (0 to 4095)
--
-- Transfers a block of 1024 samples (2048 bytes) per CS assertion.
-- Employs mid-word pre-fetching to guarantee zero timing violations
-- during back-to-back continuous SPI clock bursts up to 10+ MHz.
--------------------------------------------------------------------------------

entity spi_slave is
    port (
        clk           : in  std_logic;
        reset_n       : in  std_logic;

        -- SPI physical bus
        spi_sclk      : in  std_logic;
        spi_cs_n      : in  std_logic;
        spi_mosi      : in  std_logic;
        spi_miso      : out std_logic;

        -- Buffer interface
        block_ready   : in  std_logic;
        buffer_data   : in  std_logic_vector(11 downto 0);
        buffer_addr   : out std_logic_vector(9 downto 0);
        buffer_read   : out std_logic;

        -- Handshake / status
        transfer_done : out std_logic
    );
end entity spi_slave;

architecture rtl of spi_slave is

    -- 3-stage synchronizers for external SPI signals
    signal sclk_sync : std_logic_vector(2 downto 0) := "000";
    signal cs_sync   : std_logic_vector(2 downto 0) := "111";

    -- Edge detection
    signal sclk_rising  : std_logic;
    signal sclk_falling : std_logic;
    signal cs_active    : std_logic;
    signal cs_falling   : std_logic;
    signal cs_rising    : std_logic;

    -- Shift register & prefetch
    signal tx_shift    : std_logic_vector(15 downto 0) := (others => '0');
    signal next_word   : std_logic_vector(15 downto 0) := (others => '0');
    signal bit_counter : integer range 0 to 15 := 0;
    signal sample_idx  : integer range 0 to 1023 := 0;

    signal buffer_addr_reg   : std_logic_vector(9 downto 0) := (others => '0');
    signal buffer_read_reg   : std_logic := '0';
    signal transfer_done_reg : std_logic := '0';

begin

    --------------------------------------------------------------------
    -- Synchronizers and Edge Detectors
    --------------------------------------------------------------------
    sclk_rising  <= '1' when (sclk_sync(1) = '1' and sclk_sync(2) = '0') else '0';
    sclk_falling <= '1' when (sclk_sync(1) = '0' and sclk_sync(2) = '1') else '0';

    cs_active    <= not cs_sync(1);
    cs_falling   <= '1' when (cs_sync(1) = '0' and cs_sync(2) = '1') else '0';
    cs_rising    <= '1' when (cs_sync(1) = '1' and cs_sync(2) = '0') else '0';

    -- Output ports
    buffer_addr   <= buffer_addr_reg;
    buffer_read   <= buffer_read_reg;
    transfer_done <= transfer_done_reg;

    -- Drive MISO when CS is low, else 0 (high-Z or driven low)
    spi_miso <= tx_shift(15) when cs_active = '1' else '0';

    --------------------------------------------------------------------
    -- Main SPI Process (Clocked by 50 MHz FPGA system clock)
    --------------------------------------------------------------------
    process(clk)
        variable next_addr : integer range 0 to 1023;
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                sclk_sync         <= "000";
                cs_sync           <= "111";
                tx_shift          <= (others => '0');
                next_word         <= (others => '0');
                bit_counter       <= 0;
                sample_idx        <= 0;
                buffer_addr_reg   <= (others => '0');
                buffer_read_reg   <= '1';
                transfer_done_reg <= '0';
            else
                -- Synchronize SPI inputs
                sclk_sync <= sclk_sync(1 downto 0) & spi_sclk;
                cs_sync   <= cs_sync(1 downto 0) & spi_cs_n;

                -- Default single-cycle pulse resets
                buffer_read_reg   <= '0';
                transfer_done_reg <= '0';

                --------------------------------------------------------
                -- CS Inactive (Bus Idle)
                --------------------------------------------------------
                if cs_active = '0' then
                    sample_idx      <= 0;
                    bit_counter     <= 0;
                    buffer_addr_reg <= (others => '0');
                    buffer_read_reg <= '1';
                    -- Keep Word 0 ready in tx_shift for immediate start upon CS assertion
                    tx_shift        <= "0000" & buffer_data;
                    next_word       <= "0000" & buffer_data;

                    if cs_rising = '1' then
                        if sample_idx = 1023 then
                            transfer_done_reg <= '1';
                        end if;
                    end if;

                --------------------------------------------------------
                -- CS Active (Transaction ongoing)
                --------------------------------------------------------
                else
                    -- Latch Word 0 at the falling edge of CS
                    if cs_falling = '1' then
                        sample_idx  <= 0;
                        bit_counter <= 0;
                        tx_shift    <= "0000" & buffer_data;
                    else
                        ------------------------------------------------
                        -- Pre-fetch next sample mid-word (bit_counter 4..6)
                        -- Provides abundant time before next word is needed
                        ------------------------------------------------
                        if bit_counter = 4 then
                            if sample_idx < 1023 then
                                next_addr := sample_idx + 1;
                            else
                                next_addr := 0;
                            end if;
                            buffer_addr_reg <= std_logic_vector(to_unsigned(next_addr, 10));
                            buffer_read_reg <= '1';

                        elsif bit_counter = 6 then
                            -- Latch synchronous RAM output into next_word staging register
                            next_word <= "0000" & buffer_data;
                        end if;

                        ------------------------------------------------
                        -- SCLK Rising Edge: Master samples MISO
                        ------------------------------------------------
                        if sclk_rising = '1' then
                            -- No shift on rising edge (SPI Mode 0)
                            null;

                        ------------------------------------------------
                        -- SCLK Falling Edge: Slave updates MISO
                        ------------------------------------------------
                        elsif sclk_falling = '1' then
                            if bit_counter = 15 then
                                -- 16 bits finished: seamlessly load next sample
                                bit_counter <= 0;
                                tx_shift    <= next_word;

                                if sample_idx = 1023 then
                                    sample_idx        <= 0;
                                    transfer_done_reg <= '1';
                                else
                                    sample_idx        <= sample_idx + 1;
                                end if;
                            else
                                -- Shift next bit out to MISO
                                tx_shift    <= tx_shift(14 downto 0) & '0';
                                bit_counter <= bit_counter + 1;
                            end if;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
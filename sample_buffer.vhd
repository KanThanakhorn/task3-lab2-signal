library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- Sample Buffer: True Ping-Pong (Double-Buffer) Block RAM
--
-- Bank 0: samples 0 to 1023
-- Bank 1: samples 1024 to 2047
-- Total size: 2048 x 12-bit = 24,576 bits (stored in MAX 10 M9K memory blocks)
--
-- While the ADC continuously writes into active_bank at 16 kS/s,
-- the SPI slave reads from ready_bank without conflict or tearing.
-- When 1024 samples have been collected, banks switch automatically and
-- block_ready is asserted.
--------------------------------------------------------------------------------

entity sample_buffer is
    port (
        clk           : in  std_logic;
        reset_n       : in  std_logic;

        -- ADC write interface
        sample_valid  : in  std_logic;
        adc_data      : in  std_logic_vector(11 downto 0);

        -- Handshake flags
        block_ready   : out std_logic;
        transfer_done : in  std_logic;

        -- SPI read interface
        read_enable   : in  std_logic;
        read_address  : in  std_logic_vector(9 downto 0);
        read_data     : out std_logic_vector(11 downto 0)
    );
end entity sample_buffer;

architecture rtl of sample_buffer is

    -- 2048 x 12-bit Block RAM (2 banks of 1024 samples)
    type ram_type is array (0 to 2047) of std_logic_vector(11 downto 0);
    signal ram : ram_type;

    -- Bank pointers
    signal active_bank   : std_logic := '0'; -- 0 = Bank 0, 1 = Bank 1
    signal ready_bank    : std_logic := '0';
    signal write_address : integer range 0 to 1023 := 0;

    signal block_ready_reg : std_logic := '0';

begin

    block_ready <= block_ready_reg;

    --------------------------------------------------------------------
    -- Synchronous Write Process (ADC Side)
    --------------------------------------------------------------------
    process(clk)
        variable full_write_addr : integer range 0 to 2047;
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                active_bank     <= '0';
                ready_bank      <= '0';
                write_address   <= 0;
                block_ready_reg <= '0';
            else
                -- Clear ready flag when SPI finishes reading a block
                if transfer_done = '1' then
                    block_ready_reg <= '0';
                end if;

                -- Write ADC sample to active bank
                if sample_valid = '1' then
                    if active_bank = '0' then
                        full_write_addr := write_address;
                    else
                        full_write_addr := 1024 + write_address;
                    end if;

                    ram(full_write_addr) <= adc_data;

                    if write_address = 1023 then
                        -- Active bank is full: switch ping-pong banks
                        write_address   <= 0;
                        ready_bank      <= active_bank;
                        active_bank     <= not active_bank;
                        block_ready_reg <= '1';
                    else
                        write_address   <= write_address + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------
    -- Synchronous Read Process (SPI Slave Side)
    -- Reads from the ready bank
    --------------------------------------------------------------------
    process(clk)
        variable full_read_addr : integer range 0 to 2047;
    begin
        if rising_edge(clk) then
            if ready_bank = '0' then
                full_read_addr := to_integer(unsigned(read_address));
            else
                full_read_addr := 1024 + to_integer(unsigned(read_address));
            end if;

            if read_enable = '1' then
                read_data <= ram(full_read_addr);
            end if;
        end if;
    end process;

end architecture rtl;
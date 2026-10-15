#include <Arduino.h>
#include <SPI.h>

// ==============================================================================
// TASK 3: Real-Time Audio Sampling via MAX 10 FPGA + ESP32-S3
//
// Hardware Connections (DE10-Lite <--> ESP32-S3 UNO):
//   - GND <--> GND (Crucial common ground!)
//   - DE10-Lite D13 (PIN_AB20) <--> ESP32-S3 D13 (GPIO 12) [SPI SCK]
//   - DE10-Lite D12 (PIN_Y19)  <--> ESP32-S3 D12 (GPIO 13) [SPI MISO]
//   - DE10-Lite D11 (PIN_AA19) <--> ESP32-S3 D11 (GPIO 11) [SPI MOSI]
//   - DE10-Lite D10 (PIN_AB19) <--> ESP32-S3 D10 (GPIO 10) [SPI CS]
//   - DE10-Lite D8  (PIN_AB17) <--> ESP32-S3 D8  (GPIO 21) [DATA_READY Handshake]
// ==============================================================================

#define BUFFER_SIZE     1024
#define SERIAL_BAUD     921600   // 921600 baud: reliable on CH340 & fast enough for 16 kS/s
#define SPI_CLOCK_HZ    2000000  // 2 MHz SPI Clock

// ------------------------------------------------------------------------------
// GPIO Pin Mapping for ESP32-S3 UNO Board
// ------------------------------------------------------------------------------
#define PIN_SPI_SCK     12  // Header D13
#define PIN_SPI_MISO    13  // Header D12
#define PIN_SPI_MOSI    11  // Header D11
#define PIN_SPI_CS      10  // Header D10
#define PIN_DATA_READY  21  // Header D8 (fallback: auto-timer if not connected)

// ------------------------------------------------------------------------------
// Double Buffer
// ------------------------------------------------------------------------------
uint16_t sampleBuffer[BUFFER_SIZE];
uint8_t  spiRawBytes[BUFFER_SIZE * 2];

SPIClass spi(FSPI);
unsigned long lastReadTime = 0;
uint32_t packetCount = 0;

void setup() {
    // 1. Initialize Serial
    Serial.begin(SERIAL_BAUD);
    delay(500);

    // 2. Configure CS and Handshake pins
    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH); // CS idle HIGH

    pinMode(PIN_DATA_READY, INPUT_PULLDOWN);

    // 3. Initialize SPI Master
    spi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    lastReadTime = millis();
}

void loop() {
    // Dual trigger: Trigger if DATA_READY is HIGH OR after 64 ms (1 block time @ 16 kS/s)
    bool isDataReady = (digitalRead(PIN_DATA_READY) == HIGH);
    unsigned long now = millis();

    if (isDataReady || (now - lastReadTime >= 64)) {
        lastReadTime = now;

        // ----------------------------------------------------------------------
        // Read 1024 samples from FPGA Block RAM via SPI
        // ----------------------------------------------------------------------
        spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_SPI_CS, LOW); // Assert CS

        delayMicroseconds(5);

        // Read 2048 bytes back-to-back (1024 samples x 16-bit)
        spi.transferBytes(NULL, spiRawBytes, BUFFER_SIZE * 2);

        delayMicroseconds(2);
        digitalWrite(PIN_SPI_CS, HIGH); // Deassert CS
        spi.endTransaction();

        // ----------------------------------------------------------------------
        // Unpack samples (Big-Endian from FPGA to uint16_t)
        // ----------------------------------------------------------------------
        for (int i = 0; i < BUFFER_SIZE; i++) {
            uint16_t highByte = spiRawBytes[2 * i];
            uint16_t lowByte  = spiRawBytes[2 * i + 1];
            sampleBuffer[i]   = ((highByte << 8) | lowByte) & 0x0FFF; // 12-bit ADC value
        }

        // ----------------------------------------------------------------------
        // Send Binary Packet to Host PC (MATLAB / Python)
        // Protocol:
        //   [4B Magic: 0xAA55AA55] + [2B N: 1024] + [2048B: 1024 x uint16]
        // ----------------------------------------------------------------------
        uint32_t magic = 0xAA55AA55;
        uint16_t N = BUFFER_SIZE;

        Serial.write((uint8_t *)&magic, 4);
        Serial.write((uint8_t *)&N, 2);
        Serial.write((uint8_t *)sampleBuffer, BUFFER_SIZE * sizeof(uint16_t));
        Serial.flush();

        packetCount++;
    }
}

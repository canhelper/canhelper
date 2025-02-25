#include <SPI.h>
#include <math.h>

// ----------------------------------------------------------------------------
//  MCP2515 CAN-1 Settings
// ----------------------------------------------------------------------------

#define CAN1_PIN 10
#define CAN1_BAUD_RATE 500000
#define CAN1_MCP_CLOCK MCP_CLOCK_8MHZ

// ----------------------------------------------------------------------------
// MCP2515 CAN-2 Settings
// ----------------------------------------------------------------------------

#define CAN2_PIN 9 // 0: disabled
#define CAN2_BAUD_RATE 125000
#define CAN2_MCP_CLOCK MCP_CLOCK_8MHZ

// ----------------------------------------------------------------------------

#define CAN_MODULE_CAN_ID 0x5f0

#define MCP_CLOCK_8MHZ 8000000
#define MCP_CLOCK_16MHZ 16000000
#define MCP_CLOCK_20MHZ 20000000

#define MCP2515_RXB0CTRL 0x60
#define MCP2515_CANINTF 0x2c
#define MCP2515_RXB0SIDH 0x61
#define MCP2515_WRITE 0x02
#define MCP2515_READ 0x03

typedef struct {
    double baudRate;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} BaudConfig;

BaudConfig calcRegisters(uint32_t clock, uint32_t baudRate, double tolerance)
{
    double a = (double)clock / baudRate / 2.0;

    for (int prescaler = 1; prescaler <= 48; prescaler++) {
        double b = a / prescaler;
        double tq = round(b);

        if (tq < 4 || tq > 32) {
            continue;
        }

        double precision = round(-(b / tq - 1) * 10000) / 10000;

        if (fabs(precision) > tolerance) {
            continue;
        }

        for (double phaseSeg1 = 3; phaseSeg1 < 18; phaseSeg1++) {
            double phaseSeg2 = tq - phaseSeg1;

            if (phaseSeg1 < phaseSeg2 || phaseSeg2 < 2 || phaseSeg2 > 8) {
                continue;
            }

            double propSeg = round(phaseSeg1 / 2.0);
            double baudRateRes = round(baudRate * (1 - precision));
            uint8_t cnf1 = prescaler - 1;
            uint8_t cnf2 = propSeg + 126 + (phaseSeg1 - propSeg - 1) * 8;
            uint8_t cnf3 = phaseSeg2 - 1;

            return (BaudConfig){ baudRateRes, cnf1, cnf2, cnf3 };
        }
    }

    return (BaudConfig){ -1, 0, 0, 0 };
}

uint8_t mcp2515_readRegister(uint8_t pin, uint8_t address)
{
    digitalWrite(pin, LOW);
    SPI.transfer(MCP2515_READ);
    SPI.transfer(address);
    uint8_t data = SPI.transfer(0);
    digitalWrite(pin, HIGH);
    return data;
}

void mcp2515_writeRegister(uint8_t pin, uint8_t address, uint8_t value)
{
    digitalWrite(pin, LOW);
    SPI.transfer(MCP2515_WRITE);
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWrite(pin, HIGH);
}

bool setBaudRate(uint8_t pin, uint32_t canClock, uint32_t baudRate)
{
    BaudConfig config = calcRegisters(canClock, baudRate, 0.01);

    if (config.baudRate > 0) {
        mcp2515_writeRegister(pin, 0x2a, config.cnf1);
        mcp2515_writeRegister(pin, 0x29, config.cnf2);
        mcp2515_writeRegister(pin, 0x28, config.cnf3);

        Serial.print("Set baud rate ");
        Serial.println(baudRate);
        Serial.println(" OK.");
        return true;
    } else {
        Serial.print("Set baud rate ");
        Serial.println(baudRate);
        Serial.println(" failed.");
        return false;
    }
}

void setupMCP2515(uint8_t pin, uint32_t canClock, uint32_t baudRate)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);

    SPI.begin();
    delay(10);

    // Reset MCP2515
    digitalWrite(pin, LOW);
    SPI.transfer(0xC0);
    digitalWrite(pin, HIGH);
    delay(10);

    setBaudRate(pin, canClock, baudRate);
    mcp2515_writeRegister(pin, 0x0F, 0x00); // normal mode
}

void changeBaudRate(uint8_t pin, uint32_t canClock, uint32_t baudRate,
                    uint32_t defaultBaudRate)
{
    mcp2515_writeRegister(pin, 0x0F, 0x80); // config mode

    if (!setBaudRate(pin, canClock, baudRate) && defaultBaudRate > 0) {
        setBaudRate(pin, canClock, defaultBaudRate);
    }

    mcp2515_writeRegister(pin, 0x0F, 0x00); // normal mode
}

inline void clearCanMessage(uint8_t pin)
{
    digitalWrite(pin, LOW);
    SPI.transfer(MCP2515_WRITE);
    SPI.transfer(MCP2515_CANINTF);
    SPI.transfer(0x00);
    digitalWrite(pin, HIGH);
}

void mcp2515_readMessage(uint8_t pin)
{
    if (pin == 0 || !(mcp2515_readRegister(pin, MCP2515_CANINTF) & 0x01)) {
        return;
    }

    digitalWrite(pin, LOW);
    SPI.transfer(MCP2515_READ);
    SPI.transfer(MCP2515_RXB0SIDH);

    uint16_t canId = (SPI.transfer(0) << 3) | (SPI.transfer(0) >> 5);

    // Serial.print("CAN ID 0x");
    // Serial.println(canId, HEX);

    if (canId != CAN_MODULE_CAN_ID) {
        digitalWrite(pin, HIGH);
        clearCanMessage(pin);
        return;
    }

    uint8_t buffer[11];
    for (uint8_t i = 0; i < 11; i++) {
        buffer[i] = SPI.transfer(0);
    }
    digitalWrite(pin, HIGH);
    clearCanMessage(pin);

    uint8_t length = buffer[2] & 0x0f;

    if (length != 8 || buffer[3] != 0xfa || buffer[4] != 0xfa) {
        return;
    }

    uint8_t moduleId = buffer[5];
    uint32_t baudRate = ((uint32_t)buffer[6] << 24) |
                        ((uint32_t)buffer[7] << 16) |
                        ((uint32_t)buffer[8] << 8) | buffer[9];

    uint8_t targetPin = moduleId == 0 ? CAN1_PIN : CAN2_PIN;

    if (targetPin == 0) {
        return;
    }

    uint32_t clock = moduleId == 0 ? CAN1_MCP_CLOCK : CAN2_MCP_CLOCK;
    uint32_t defaultBaudRate = moduleId == 0 ? CAN1_BAUD_RATE : CAN2_BAUD_RATE;

    changeBaudRate(targetPin, clock, baudRate, defaultBaudRate);
}

void setup()
{
    Serial.begin(9600);
    Serial.println("CAN Helper");

    if (CAN1_PIN > 0) {
        setupMCP2515(CAN1_PIN, CAN1_MCP_CLOCK, CAN1_BAUD_RATE);
    }

    if (CAN2_PIN > 0) {
        setupMCP2515(CAN2_PIN, CAN2_MCP_CLOCK, CAN2_BAUD_RATE);
    }
}

void loop()
{
    mcp2515_readMessage(CAN1_PIN);
    mcp2515_readMessage(CAN2_PIN);
}

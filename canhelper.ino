#include <SPI.h>
#include <stdint.h>
#include "Arduino.h"

/*
 * Settings
 */
#define CAN1_PIN 10
#define CAN1_SPEED CAN_500KBPS
#define CAN1_CLOCK MCP_8MHZ

// CAN-2 disabled by default
// #define CAN2_PIN 9 // can2 enabled
#define CAN2_PIN 0  // can2 disabled
#define CAN2_SPEED CAN_125KBPS
#define CAN2_CLOCK MCP_8MHZ

/* --------------------------------------------------------------------- */

enum CAN_CLOCK { MCP_20MHZ,
                 MCP_16MHZ,
                 MCP_8MHZ };

enum CAN_SPEED {
  CAN_5KBPS,
  CAN_10KBPS,
  CAN_20KBPS,
  CAN_31K25BPS,
  CAN_33KBPS,
  CAN_40KBPS,
  CAN_50KBPS,
  CAN_80KBPS,
  CAN_83K3BPS,
  CAN_95KBPS,
  CAN_100KBPS,
  CAN_125KBPS,
  CAN_200KBPS,
  CAN_250KBPS,
  CAN_500KBPS,
  CAN_1000KBPS
};

enum CAN_CLKOUT {
  CLKOUT_DISABLE = -1,
  CLKOUT_DIV1 = 0x0,
  CLKOUT_DIV2 = 0x1,
  CLKOUT_DIV4 = 0x2,
  CLKOUT_DIV8 = 0x3,
};

typedef unsigned char __u8;
typedef unsigned short __u16;
typedef unsigned long __u32;

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000UL /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000UL /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000UL /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFUL /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFUL /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFUL /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28 : CAN identifier (11/29 bit)
 * bit 29   : error message frame flag (0 = data frame, 1 = error message)
 * bit 30   : remote transmission request flag (1 = rtr frame)
 * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef __u32 canid_t;

#define CAN_SFF_ID_BITS 11
#define CAN_EFF_ID_BITS 29

/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

struct can_frame {
  canid_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  __u8 can_dlc;   /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  __u8 data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};

class MCP2515 {
public:
  enum ERROR { ERROR_OK = 0,
               ERROR_FAIL = 1,
               ERROR_ALLTXBUSY = 2,
               ERROR_FAILINIT = 3,
               ERROR_FAILTX = 4,
               ERROR_NOMSG = 5 };

  enum MASK { MASK0,
              MASK1 };

  enum RXF { RXF0 = 0,
             RXF1 = 1,
             RXF2 = 2,
             RXF3 = 3,
             RXF4 = 4,
             RXF5 = 5 };

  enum RXBn { RXB0 = 0,
              RXB1 = 1 };

  enum TXBn { TXB0 = 0,
              TXB1 = 1,
              TXB2 = 2 };

  enum /*class*/ CANINTF : uint8_t {
    CANINTF_RX0IF = 0x01,
    CANINTF_RX1IF = 0x02,
    CANINTF_TX0IF = 0x04,
    CANINTF_TX1IF = 0x08,
    CANINTF_TX2IF = 0x10,
    CANINTF_ERRIF = 0x20,
    CANINTF_WAKIF = 0x40,
    CANINTF_MERRF = 0x80
  };

  enum /*class*/ EFLG : uint8_t {
    EFLG_RX1OVR = (1 << 7),
    EFLG_RX0OVR = (1 << 6),
    EFLG_TXBO = (1 << 5),
    EFLG_TXEP = (1 << 4),
    EFLG_RXEP = (1 << 3),
    EFLG_TXWAR = (1 << 2),
    EFLG_RXWAR = (1 << 1),
    EFLG_EWARN = (1 << 0)
  };

private:
  static const uint8_t CANCTRL_REQOP = 0xE0;
  static const uint8_t CANCTRL_ABAT = 0x10;
  static const uint8_t CANCTRL_OSM = 0x08;
  static const uint8_t CANCTRL_CLKEN = 0x04;
  static const uint8_t CANCTRL_CLKPRE = 0x03;

  enum /*class*/ CANCTRL_REQOP_MODE : uint8_t {
    CANCTRL_REQOP_NORMAL = 0x00,
    CANCTRL_REQOP_SLEEP = 0x20,
    CANCTRL_REQOP_LOOPBACK = 0x40,
    CANCTRL_REQOP_LISTENONLY = 0x60,
    CANCTRL_REQOP_CONFIG = 0x80,
    CANCTRL_REQOP_POWERUP = 0xE0
  };

  static const uint8_t CANSTAT_OPMOD = 0xE0;
  static const uint8_t CANSTAT_ICOD = 0x0E;

  static const uint8_t CNF3_SOF = 0x80;

  static const uint8_t TXB_EXIDE_MASK = 0x08;
  static const uint8_t DLC_MASK = 0x0F;
  static const uint8_t RTR_MASK = 0x40;

  static const uint8_t RXBnCTRL_RXM_STD = 0x20;
  static const uint8_t RXBnCTRL_RXM_EXT = 0x40;
  static const uint8_t RXBnCTRL_RXM_STDEXT = 0x00;
  static const uint8_t RXBnCTRL_RXM_MASK = 0x60;
  static const uint8_t RXBnCTRL_RTR = 0x08;
  static const uint8_t RXB0CTRL_BUKT = 0x04;
  static const uint8_t RXB0CTRL_FILHIT_MASK = 0x03;
  static const uint8_t RXB1CTRL_FILHIT_MASK = 0x07;
  static const uint8_t RXB0CTRL_FILHIT = 0x00;
  static const uint8_t RXB1CTRL_FILHIT = 0x01;

  static const uint8_t MCP_SIDH = 0;
  static const uint8_t MCP_SIDL = 1;
  static const uint8_t MCP_EID8 = 2;
  static const uint8_t MCP_EID0 = 3;
  static const uint8_t MCP_DLC = 4;
  static const uint8_t MCP_DATA = 5;

  enum /*class*/ STAT : uint8_t { STAT_RX0IF = (1 << 0),
                                  STAT_RX1IF = (1 << 1) };

  static const uint8_t STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF;

  enum /*class*/ TXBnCTRL : uint8_t {
    TXB_ABTF = 0x40,
    TXB_MLOA = 0x20,
    TXB_TXERR = 0x10,
    TXB_TXREQ = 0x08,
    TXB_TXIE = 0x04,
    TXB_TXP = 0x03
  };

  static const uint8_t EFLG_ERRORMASK = EFLG_RX1OVR | EFLG_RX0OVR | EFLG_TXBO | EFLG_TXEP | EFLG_RXEP;

  enum /*class*/ INSTRUCTION : uint8_t {
    INSTRUCTION_WRITE = 0x02,
    INSTRUCTION_READ = 0x03,
    INSTRUCTION_BITMOD = 0x05,
    INSTRUCTION_LOAD_TX0 = 0x40,
    INSTRUCTION_LOAD_TX1 = 0x42,
    INSTRUCTION_LOAD_TX2 = 0x44,
    INSTRUCTION_RTS_TX0 = 0x81,
    INSTRUCTION_RTS_TX1 = 0x82,
    INSTRUCTION_RTS_TX2 = 0x84,
    INSTRUCTION_RTS_ALL = 0x87,
    INSTRUCTION_READ_RX0 = 0x90,
    INSTRUCTION_READ_RX1 = 0x94,
    INSTRUCTION_READ_STATUS = 0xA0,
    INSTRUCTION_RX_STATUS = 0xB0,
    INSTRUCTION_RESET = 0xC0
  };

  enum /*class*/ REGISTER : uint8_t {
    MCP_RXF0SIDH = 0x00,
    MCP_RXF0SIDL = 0x01,
    MCP_RXF0EID8 = 0x02,
    MCP_RXF0EID0 = 0x03,
    MCP_RXF1SIDH = 0x04,
    MCP_RXF1SIDL = 0x05,
    MCP_RXF1EID8 = 0x06,
    MCP_RXF1EID0 = 0x07,
    MCP_RXF2SIDH = 0x08,
    MCP_RXF2SIDL = 0x09,
    MCP_RXF2EID8 = 0x0A,
    MCP_RXF2EID0 = 0x0B,
    MCP_CANSTAT = 0x0E,
    MCP_CANCTRL = 0x0F,
    MCP_RXF3SIDH = 0x10,
    MCP_RXF3SIDL = 0x11,
    MCP_RXF3EID8 = 0x12,
    MCP_RXF3EID0 = 0x13,
    MCP_RXF4SIDH = 0x14,
    MCP_RXF4SIDL = 0x15,
    MCP_RXF4EID8 = 0x16,
    MCP_RXF4EID0 = 0x17,
    MCP_RXF5SIDH = 0x18,
    MCP_RXF5SIDL = 0x19,
    MCP_RXF5EID8 = 0x1A,
    MCP_RXF5EID0 = 0x1B,
    MCP_TEC = 0x1C,
    MCP_REC = 0x1D,
    MCP_RXM0SIDH = 0x20,
    MCP_RXM0SIDL = 0x21,
    MCP_RXM0EID8 = 0x22,
    MCP_RXM0EID0 = 0x23,
    MCP_RXM1SIDH = 0x24,
    MCP_RXM1SIDL = 0x25,
    MCP_RXM1EID8 = 0x26,
    MCP_RXM1EID0 = 0x27,
    MCP_CNF3 = 0x28,
    MCP_CNF2 = 0x29,
    MCP_CNF1 = 0x2A,
    MCP_CANINTE = 0x2B,
    MCP_CANINTF = 0x2C,
    MCP_EFLG = 0x2D,
    MCP_TXB0CTRL = 0x30,
    MCP_TXB0SIDH = 0x31,
    MCP_TXB0SIDL = 0x32,
    MCP_TXB0EID8 = 0x33,
    MCP_TXB0EID0 = 0x34,
    MCP_TXB0DLC = 0x35,
    MCP_TXB0DATA = 0x36,
    MCP_TXB1CTRL = 0x40,
    MCP_TXB1SIDH = 0x41,
    MCP_TXB1SIDL = 0x42,
    MCP_TXB1EID8 = 0x43,
    MCP_TXB1EID0 = 0x44,
    MCP_TXB1DLC = 0x45,
    MCP_TXB1DATA = 0x46,
    MCP_TXB2CTRL = 0x50,
    MCP_TXB2SIDH = 0x51,
    MCP_TXB2SIDL = 0x52,
    MCP_TXB2EID8 = 0x53,
    MCP_TXB2EID0 = 0x54,
    MCP_TXB2DLC = 0x55,
    MCP_TXB2DATA = 0x56,
    MCP_RXB0CTRL = 0x60,
    MCP_RXB0SIDH = 0x61,
    MCP_RXB0SIDL = 0x62,
    MCP_RXB0EID8 = 0x63,
    MCP_RXB0EID0 = 0x64,
    MCP_RXB0DLC = 0x65,
    MCP_RXB0DATA = 0x66,
    MCP_RXB1CTRL = 0x70,
    MCP_RXB1SIDH = 0x71,
    MCP_RXB1SIDL = 0x72,
    MCP_RXB1EID8 = 0x73,
    MCP_RXB1EID0 = 0x74,
    MCP_RXB1DLC = 0x75,
    MCP_RXB1DATA = 0x76
  };

  static const uint32_t DEFAULT_SPI_CLOCK = 10000000;  // 10MHz

  static const int N_TXBUFFERS = 3;
  static const int N_RXBUFFERS = 2;

  static const struct TXBn_REGS {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
  } TXB[N_TXBUFFERS];

  static const struct RXBn_REGS {
    REGISTER CTRL;
    REGISTER SIDH;
    REGISTER DATA;
    CANINTF CANINTF_RXnIF;
  } RXB[N_RXBUFFERS];

  uint8_t SPICS;
  uint32_t SPI_CLOCK;
  SPIClass *SPIn;

private:
  void startSPI();
  void endSPI();

  ERROR setMode(const CANCTRL_REQOP_MODE mode);

  uint8_t readRegister(const REGISTER reg);
  void readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n);
  void setRegister(const REGISTER reg, const uint8_t value);
  void setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n);
  void modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data);

  void prepareId(uint8_t *buffer, const bool ext, const uint32_t id);

public:
  MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK = DEFAULT_SPI_CLOCK, SPIClass *_SPI = nullptr);
  ERROR reset(void);
  ERROR setConfigMode();
  ERROR setListenOnlyMode();
  ERROR setSleepMode();
  ERROR setLoopbackMode();
  ERROR setNormalMode();
  ERROR setClkOut(const CAN_CLKOUT divisor);
  ERROR setBitrate(const CAN_SPEED canSpeed);
  ERROR setBitrate(const CAN_SPEED canSpeed, const CAN_CLOCK canClock);
  ERROR setFilterMask(const MASK num, const bool ext, const uint32_t ulData);
  ERROR setFilter(const RXF num, const bool ext, const uint32_t ulData);
  ERROR sendMessage(const TXBn txbn, const struct can_frame *frame);
  ERROR sendMessage(const struct can_frame *frame);
  ERROR readMessage(const RXBn rxbn, struct can_frame *frame);
  ERROR readMessage(struct can_frame *frame);
  bool checkReceive(void);
  bool checkError(void);
  uint8_t getErrorFlags(void);
  void clearRXnOVRFlags(void);
  uint8_t getInterrupts(void);
  uint8_t getInterruptMask(void);
  void clearInterrupts(void);
  void clearTXInterrupts(void);
  uint8_t getStatus(void);
  void clearRXnOVR(void);
  void clearMERR();
  void clearERRIF();
  uint8_t errorCountRX(void);
  uint8_t errorCountTX(void);
};

MCP2515::MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK, SPIClass *_SPI) {
  if (_SPI != nullptr) {
    SPIn = _SPI;
  } else {
    SPIn = &SPI;
    SPIn->begin();
  }

  SPICS = _CS;
  SPI_CLOCK = _SPI_CLOCK;
  pinMode(SPICS, OUTPUT);
  digitalWrite(SPICS, HIGH);
}

void MCP2515::startSPI() {
  SPIn->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(SPICS, LOW);
}

void MCP2515::endSPI() {
  digitalWrite(SPICS, HIGH);
  SPIn->endTransaction();
}

MCP2515::ERROR MCP2515::reset(void) {
  startSPI();
  SPIn->transfer(INSTRUCTION_RESET);
  endSPI();

  delay(10);

  uint8_t zeros[14];
  memset(zeros, 0, sizeof(zeros));
  setRegisters(MCP_TXB0CTRL, zeros, 14);
  setRegisters(MCP_TXB1CTRL, zeros, 14);
  setRegisters(MCP_TXB2CTRL, zeros, 14);

  setRegister(MCP_RXB0CTRL, 0);
  setRegister(MCP_RXB1CTRL, 0);

  setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

  // receives all valid messages using either Standard or Extended Identifiers that
  // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
  modifyRegister(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                 RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
  modifyRegister(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK, RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

  // clear filters and masks
  // do not filter any standard frames for RXF0 used by RXB0
  // do not filter any extended frames for RXF1 used by RXB1
  RXF filters[] = { RXF0, RXF1, RXF2, RXF3, RXF4, RXF5 };
  for (int i = 0; i < 6; i++) {
    bool ext = (i == 1);
    ERROR result = setFilter(filters[i], ext, 0);
    if (result != ERROR_OK) {
      return result;
    }
  }

  MASK masks[] = { MASK0, MASK1 };
  for (int i = 0; i < 2; i++) {
    ERROR result = setFilterMask(masks[i], true, 0);
    if (result != ERROR_OK) {
      return result;
    }
  }

  return ERROR_OK;
}

uint8_t MCP2515::readRegister(const REGISTER reg) {
  startSPI();
  SPIn->transfer(INSTRUCTION_READ);
  SPIn->transfer(reg);
  uint8_t ret = SPIn->transfer(0x00);
  endSPI();

  return ret;
}

void MCP2515::readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n) {
  startSPI();
  SPIn->transfer(INSTRUCTION_READ);
  SPIn->transfer(reg);
  // mcp2515 has auto-increment of address-pointer
  for (uint8_t i = 0; i < n; i++) {
    values[i] = SPIn->transfer(0x00);
  }
  endSPI();
}

void MCP2515::setRegister(const REGISTER reg, const uint8_t value) {
  startSPI();
  SPIn->transfer(INSTRUCTION_WRITE);
  SPIn->transfer(reg);
  SPIn->transfer(value);
  endSPI();
}

void MCP2515::setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n) {
  startSPI();
  SPIn->transfer(INSTRUCTION_WRITE);
  SPIn->transfer(reg);
  for (uint8_t i = 0; i < n; i++) {
    SPIn->transfer(values[i]);
  }
  endSPI();
}

void MCP2515::modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data) {
  startSPI();
  SPIn->transfer(INSTRUCTION_BITMOD);
  SPIn->transfer(reg);
  SPIn->transfer(mask);
  SPIn->transfer(data);
  endSPI();
}

uint8_t MCP2515::getStatus(void) {
  startSPI();
  SPIn->transfer(INSTRUCTION_READ_STATUS);
  uint8_t i = SPIn->transfer(0x00);
  endSPI();

  return i;
}

MCP2515::ERROR MCP2515::setConfigMode() {
  return setMode(CANCTRL_REQOP_CONFIG);
}

MCP2515::ERROR MCP2515::setListenOnlyMode() {
  return setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP2515::ERROR MCP2515::setSleepMode() {
  return setMode(CANCTRL_REQOP_SLEEP);
}

MCP2515::ERROR MCP2515::setLoopbackMode() {
  return setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP2515::ERROR MCP2515::setNormalMode() {
  return setMode(CANCTRL_REQOP_NORMAL);
}

MCP2515::ERROR MCP2515::setMode(const CANCTRL_REQOP_MODE mode) {
  modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

  unsigned long endTime = millis() + 10;
  bool modeMatch = false;
  while (millis() < endTime) {
    uint8_t newmode = readRegister(MCP_CANSTAT);
    newmode &= CANSTAT_OPMOD;

    modeMatch = newmode == mode;

    if (modeMatch) {
      break;
    }
  }

  return modeMatch ? ERROR_OK : ERROR_FAIL;
}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed) {
  return setBitrate(canSpeed, MCP_16MHZ);
}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed, CAN_CLOCK canClock) {
  ERROR error = setConfigMode();
  if (error != ERROR_OK) {
    return error;
  }

  uint8_t set, cfg1, cfg2, cfg3;
  set = 1;
  switch (canClock) {
    case (MCP_8MHZ):
      switch (canSpeed) {
        case (CAN_5KBPS):  //   5KBPS
          cfg1 = 0X1F;
          cfg2 = 0XBF;
          cfg3 = 0X87;
          break;

        case (CAN_10KBPS):  //  10KBPS
          cfg1 = 0X0F;
          cfg2 = 0XBF;
          cfg3 = 0X87;
          break;

        case (CAN_20KBPS):  //  20KBPS
          cfg1 = 0X07;
          cfg2 = 0XBF;
          cfg3 = 0X87;
          break;

        case (CAN_31K25BPS):  //  31.25KBPS
          cfg1 = 0X07;
          cfg2 = 0XA4;
          cfg3 = 0X84;
          break;

        case (CAN_33KBPS):  //  33.333KBPS
          cfg1 = 0X47;
          cfg2 = 0XE2;
          cfg3 = 0X85;
          break;

        case (CAN_40KBPS):  //  40Kbps
          cfg1 = 0X03;
          cfg2 = 0XBF;
          cfg3 = 0X87;
          break;

        case (CAN_50KBPS):  //  50Kbps
          cfg1 = 0X03;
          cfg2 = 0XB4;
          cfg3 = 0X86;
          break;

        case (CAN_80KBPS):  //  80Kbps
          cfg1 = 0X01;
          cfg2 = 0XBF;
          cfg3 = 0X87;
          break;

        case (CAN_100KBPS):  // 100Kbps
          cfg1 = 0X01;
          cfg2 = 0XB4;
          cfg3 = 0X86;
          break;

        case (CAN_125KBPS):  // 125Kbps
          cfg1 = 0X01;
          cfg2 = 0XB1;
          cfg3 = 0X85;
          break;

        case (CAN_200KBPS):  // 200Kbps
          cfg1 = 0X00;
          cfg2 = 0XB4;
          cfg3 = 0X86;
          break;

        case (CAN_250KBPS):  // 250Kbps
          cfg1 = 0X00;
          cfg2 = 0XB1;
          cfg3 = 0X85;
          break;

        case (CAN_500KBPS):  // 500Kbps
          cfg1 = 0X00;
          cfg2 = 0X90;
          cfg3 = 0X82;
          break;

        case (CAN_1000KBPS):  //   1Mbps
          cfg1 = 0X00;
          cfg2 = 0X80;
          cfg3 = 0X80;
          break;

        default:
          set = 0;
          break;
      }
      break;

    case (MCP_16MHZ):
      switch (canSpeed) {
        case (CAN_5KBPS):  //   5Kbps
          cfg1 = 0X3F;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_10KBPS):  //  10Kbps
          cfg1 = 0X1F;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_20KBPS):  //  20Kbps
          cfg1 = 0X0F;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_33KBPS):  //  33.333Kbps
          cfg1 = 0X4E;
          cfg2 = 0XF1;
          cfg3 = 0X85;
          break;

        case (CAN_40KBPS):  //  40Kbps
          cfg1 = 0X07;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_50KBPS):  //  50Kbps
          cfg1 = 0X07;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_80KBPS):  //  80Kbps
          cfg1 = 0X03;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_83K3BPS):  //  83.333Kbps
          cfg1 = 0X03;
          cfg2 = 0XBE;
          cfg3 = 0X07;
          break;

        case (CAN_95KBPS):  //  95Kbps
          cfg1 = 0X03;
          cfg2 = 0XAD;
          cfg3 = 0X07;
          break;

        case (CAN_100KBPS):  // 100Kbps
          cfg1 = 0X03;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_125KBPS):  // 125Kbps
          cfg1 = 0X03;
          cfg2 = 0XF0;
          cfg3 = 0X86;
          break;

        case (CAN_200KBPS):  // 200Kbps
          cfg1 = 0X01;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_250KBPS):  // 250Kbps
          cfg1 = 0X41;
          cfg2 = 0XF1;
          cfg3 = 0X85;
          break;

        case (CAN_500KBPS):  // 500Kbps
          cfg1 = 0X00;
          cfg2 = 0XF0;
          cfg3 = 0X86;
          break;

        case (CAN_1000KBPS):  //   1Mbps
          cfg1 = 0X00;
          cfg2 = 0XD0;
          cfg3 = 0X82;
          break;

        default:
          set = 0;
          break;
      }
      break;

    case (MCP_20MHZ):
      switch (canSpeed) {
        case (CAN_33KBPS):  //  33.333Kbps
          cfg1 = 0X0B;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_40KBPS):  //  40Kbps
          cfg1 = 0X09;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_50KBPS):  //  50Kbps
          cfg1 = 0X09;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_80KBPS):  //  80Kbps
          cfg1 = 0X04;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_83K3BPS):  //  83.333Kbps
          cfg1 = 0X04;
          cfg2 = 0XFE;
          cfg3 = 0X87;
          break;

        case (CAN_100KBPS):  // 100Kbps
          cfg1 = 0X04;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_125KBPS):  // 125Kbps
          cfg1 = 0X03;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_200KBPS):  // 200Kbps
          cfg1 = 0X01;
          cfg2 = 0XFF;
          cfg3 = 0X87;
          break;

        case (CAN_250KBPS):  // 250Kbps
          cfg1 = 0X41;
          cfg2 = 0XFB;
          cfg3 = 0X86;
          break;

        case (CAN_500KBPS):  // 500Kbps
          cfg1 = 0X00;
          cfg2 = 0XFA;
          cfg3 = 0X87;
          break;

        case (CAN_1000KBPS):  //   1Mbps
          cfg1 = 0X00;
          cfg2 = 0XD9;
          cfg3 = 0X82;
          break;

        default:
          set = 0;
          break;
      }
      break;

    default:
      set = 0;
      break;
  }

  if (set) {
    setRegister(MCP_CNF1, cfg1);
    setRegister(MCP_CNF2, cfg2);
    setRegister(MCP_CNF3, cfg3);
    return ERROR_OK;
  } else {
    return ERROR_FAIL;
  }
}

MCP2515::ERROR MCP2515::setClkOut(const CAN_CLKOUT divisor) {
  if (divisor == CLKOUT_DISABLE) {
    /* Turn off CLKEN */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

    /* Turn on CLKOUT for SOF */
    modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
    return ERROR_OK;
  }

  /* Set the prescaler (CLKPRE) */
  modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

  /* Turn on CLKEN */
  modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

  /* Turn off CLKOUT for SOF */
  modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
  return ERROR_OK;
}

void MCP2515::prepareId(uint8_t *buffer, const bool ext, const uint32_t id) {
  uint16_t canid = (uint16_t)(id & 0x0FFFF);

  if (ext) {
    buffer[MCP_EID0] = (uint8_t)(canid & 0xFF);
    buffer[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    buffer[MCP_SIDL] = (uint8_t)(canid & 0x03);
    buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
    buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
    buffer[MCP_SIDH] = (uint8_t)(canid >> 5);
  } else {
    buffer[MCP_SIDH] = (uint8_t)(canid >> 3);
    buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
    buffer[MCP_EID0] = 0;
    buffer[MCP_EID8] = 0;
  }
}

MCP2515::ERROR MCP2515::setFilterMask(const MASK mask, const bool ext, const uint32_t ulData) {
  ERROR res = setConfigMode();
  if (res != ERROR_OK) {
    return res;
  }

  uint8_t tbufdata[4];
  prepareId(tbufdata, ext, ulData);

  REGISTER reg;
  switch (mask) {
    case MASK0:
      reg = MCP_RXM0SIDH;
      break;
    case MASK1:
      reg = MCP_RXM1SIDH;
      break;
    default:
      return ERROR_FAIL;
  }

  setRegisters(reg, tbufdata, 4);

  return ERROR_OK;
}

MCP2515::ERROR MCP2515::setFilter(const RXF num, const bool ext, const uint32_t ulData) {
  ERROR res = setConfigMode();
  if (res != ERROR_OK) {
    return res;
  }

  REGISTER reg;

  switch (num) {
    case RXF0:
      reg = MCP_RXF0SIDH;
      break;
    case RXF1:
      reg = MCP_RXF1SIDH;
      break;
    case RXF2:
      reg = MCP_RXF2SIDH;
      break;
    case RXF3:
      reg = MCP_RXF3SIDH;
      break;
    case RXF4:
      reg = MCP_RXF4SIDH;
      break;
    case RXF5:
      reg = MCP_RXF5SIDH;
      break;
    default:
      return ERROR_FAIL;
  }

  uint8_t tbufdata[4];
  prepareId(tbufdata, ext, ulData);
  setRegisters(reg, tbufdata, 4);

  return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const TXBn txbn, const struct can_frame *frame) {
  if (frame->can_dlc > CAN_MAX_DLEN) {
    return ERROR_FAILTX;
  }

  const struct TXBn_REGS *txbuf = &TXB[txbn];

  uint8_t data[13];

  bool ext = (frame->can_id & CAN_EFF_FLAG);
  bool rtr = (frame->can_id & CAN_RTR_FLAG);
  uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

  prepareId(data, ext, id);

  data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

  memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

  setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

  modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

  uint8_t ctrl = readRegister(txbuf->CTRL);
  if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
    return ERROR_FAILTX;
  }
  return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const struct can_frame *frame) {
  if (frame->can_dlc > CAN_MAX_DLEN) {
    return ERROR_FAILTX;
  }

  TXBn txBuffers[N_TXBUFFERS] = { TXB0, TXB1, TXB2 };

  for (int i = 0; i < N_TXBUFFERS; i++) {
    const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
    uint8_t ctrlval = readRegister(txbuf->CTRL);
    if ((ctrlval & TXB_TXREQ) == 0) {
      return sendMessage(txBuffers[i], frame);
    }
  }

  return ERROR_ALLTXBUSY;
}

MCP2515::ERROR MCP2515::readMessage(const RXBn rxbn, struct can_frame *frame) {
  const struct RXBn_REGS *rxb = &RXB[rxbn];

  uint8_t tbufdata[5];

  readRegisters(rxb->SIDH, tbufdata, 5);

  uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

  if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK) {
    id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
    id = (id << 8) + tbufdata[MCP_EID8];
    id = (id << 8) + tbufdata[MCP_EID0];
    id |= CAN_EFF_FLAG;
  }

  uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
  if (dlc > CAN_MAX_DLEN) {
    return ERROR_FAIL;
  }

  uint8_t ctrl = readRegister(rxb->CTRL);
  if (ctrl & RXBnCTRL_RTR) {
    id |= CAN_RTR_FLAG;
  }

  frame->can_id = id;
  frame->can_dlc = dlc;

  readRegisters(rxb->DATA, frame->data, dlc);

  modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

  return ERROR_OK;
}

MCP2515::ERROR MCP2515::readMessage(struct can_frame *frame) {
  ERROR rc;
  uint8_t stat = getStatus();

  if (stat & STAT_RX0IF) {
    rc = readMessage(RXB0, frame);
  } else if (stat & STAT_RX1IF) {
    rc = readMessage(RXB1, frame);
  } else {
    rc = ERROR_NOMSG;
  }

  return rc;
}

bool MCP2515::checkReceive(void) {
  uint8_t res = getStatus();
  if (res & STAT_RXIF_MASK) {
    return true;
  } else {
    return false;
  }
}

bool MCP2515::checkError(void) {
  uint8_t eflg = getErrorFlags();

  if (eflg & EFLG_ERRORMASK) {
    return true;
  } else {
    return false;
  }
}

uint8_t MCP2515::getErrorFlags(void) {
  return readRegister(MCP_EFLG);
}

void MCP2515::clearRXnOVRFlags(void) {
  modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP2515::getInterrupts(void) {
  return readRegister(MCP_CANINTF);
}

void MCP2515::clearInterrupts(void) {
  setRegister(MCP_CANINTF, 0);
}

uint8_t MCP2515::getInterruptMask(void) {
  return readRegister(MCP_CANINTE);
}

void MCP2515::clearTXInterrupts(void) {
  modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP2515::clearRXnOVR(void) {
  uint8_t eflg = getErrorFlags();
  if (eflg != 0) {
    clearRXnOVRFlags();
    clearInterrupts();
    //modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
  }
}

void MCP2515::clearMERR() {
  //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
  //clearInterrupts();
  modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP2515::clearERRIF() {
  //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
  //clearInterrupts();
  modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

uint8_t MCP2515::errorCountRX(void) {
  return readRegister(MCP_REC);
}

uint8_t MCP2515::errorCountTX(void) {
  return readRegister(MCP_TEC);
}

const struct MCP2515::TXBn_REGS MCP2515::TXB[MCP2515::N_TXBUFFERS] = { { MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA },
                                                                       { MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA },
                                                                       { MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA } };

const struct MCP2515::RXBn_REGS MCP2515::RXB[N_RXBUFFERS] = { { MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF },
                                                              { MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF } };

MCP2515 *can1;
MCP2515 *can2;

void setup() {
  while (!Serial)
    ;

  Serial.begin(9600);
  SPI.begin();

  can1 = new MCP2515(CAN1_PIN);
  can1->reset();
  can1->setBitrate(CAN1_SPEED, CAN1_CLOCK);
  can1->setNormalMode();

  if (CAN2_PIN > 0) {
    can2 = new MCP2515(CAN2_PIN);
    can2->reset();
    can2->setBitrate(CAN2_SPEED, CAN2_CLOCK);
    can2->setNormalMode();
  }
}

void loop() {
}

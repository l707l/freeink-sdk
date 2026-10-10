#include "Uc8253CDriver.h"

#include <BoardConfig.h>

namespace freeink {
namespace {

// UC8253 / UC8253C Command Set
constexpr uint8_t CMD_PANEL_SETTING = 0x00;
constexpr uint8_t CMD_POWER_SETTING = 0x01;
constexpr uint8_t CMD_POWER_OFF = 0x02;
constexpr uint8_t CMD_POWER_ON = 0x04;
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x06;
constexpr uint8_t CMD_DEEP_SLEEP = 0x07;
constexpr uint8_t CMD_DTM1 = 0x10;
constexpr uint8_t CMD_DISPLAY_REFRESH = 0x12;
constexpr uint8_t CMD_DTM2 = 0x13;
constexpr uint8_t CMD_PLL_CONTROL = 0x30;
constexpr uint8_t CMD_VCOM_DATA_INTERVAL = 0x50;
constexpr uint8_t CMD_RESOLUTION = 0x61;
constexpr uint8_t CMD_VCOM_DC = 0x82;
constexpr uint8_t CMD_CASCADE_SETTING = 0xE0;
constexpr uint8_t CMD_FORCE_TEMPERATURE = 0xE5;

// Controller-native geometry: the UC8253 sees a 240x416 portrait panel.
// The facade's framebuffer is landscape 416x240 (52 bytes per row, 240 rows),
// matching the rest of the FreeInk/CrossPoint landscape-native renderer.
// writePlane() rotates each plane into the controller's 240x416 RAM (30 bytes per row, 416 rows).
constexpr uint16_t CTRL_W = 240;
constexpr uint16_t CTRL_H = 416;
constexpr uint16_t CTRL_WB = CTRL_W / 8;  // 30

}  // namespace

Uc8253CDriver::Uc8253CDriver()
    : _fbW(BoardConfig::ACTIVE.displayWidth ? BoardConfig::ACTIVE.displayWidth : 240),
      _fbH(BoardConfig::ACTIVE.displayHeight ? BoardConfig::ACTIVE.displayHeight : 416),
      _fbWb(_fbW / 8) {
  const uint32_t totalBytes = static_cast<uint32_t>(_fbWb) * _fbH;
  _prevFb = static_cast<uint8_t*>(malloc(totalBytes));
  if (_prevFb != nullptr) {
    memset(_prevFb, 0xFF, totalBytes);
  }
}

uint32_t Uc8253CDriver::spiHz() const {
  // Clamp display SPI transaction frequency to <= 10 MHz (UC8253 limit)
  const uint32_t hz = BoardConfig::ACTIVE.displaySpiHz;
  return (hz != 0 && hz <= 10000000) ? hz : 10000000;
}

int8_t Uc8253CDriver::spiMiso() const {
  // Shared SPI bus: MISO is connected to SD card module
  return BoardConfig::ACTIVE.sd.miso;
}

int8_t Uc8253CDriver::coCs() const {
  // SD card CS pin: must be held HIGH during display SPI transactions
  return BoardConfig::ACTIVE.sd.cs;
}

PanelGeometry Uc8253CDriver::geometry() const {
  return {_fbW, _fbH, _fbWb, static_cast<uint32_t>(_fbWb) * _fbH};
}

void Uc8253CDriver::powerOn(EpdBus& bus) {
  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" UC8253C_PON");
    _isScreenOn = true;
  }
}

void Uc8253CDriver::powerOff(EpdBus& bus) {
  if (_isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" UC8253C_POF");
    _isScreenOn = false;
  }
}

void Uc8253CDriver::initController(EpdBus& bus) {
  // Soft reset to ensure timing controller is in a clean known state
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(0x1E);
  bus.data(0x0D);
  delay(1);

  // Panel setting:
  // Base is 0x1F: KW monochrome mode, UD=1, SHL=1, SHD_N=1, RST_N=1
  // Second byte 0x0D: TS_AUTO=1, TIEG=1, VC_LUTZ=1 (VCOM floats automatically after refresh)
  uint8_t psr0 = 0x1F;
  if (BoardConfig::ACTIVE.orientation.mirrorX) {
    psr0 ^= 0x04;  // Toggle SHL (horizontal mirror -> 0x1B)
  }
  if (BoardConfig::ACTIVE.orientation.mirrorY) {
    psr0 ^= 0x08;  // Toggle UD (vertical mirror)
  }

  bus.cmd(CMD_PANEL_SETTING);
  bus.data(psr0);
  bus.data(0x0D);
}

// Write framebuffer plane directly to controller RAM (240x416 portrait).
void Uc8253CDriver::writePlane(EpdBus& bus, uint8_t command, const uint8_t* fb) {
  bus.cmd(command);
  bus.beginTxn();
  const uint32_t totalBytes = static_cast<uint32_t>(_fbWb) * _fbH;
  if (fb != nullptr) {
    bus.rawWriteBytes(fb, totalBytes);
  } else {
    uint8_t blankLine[_fbWb];
    memset(blankLine, 0xFF, sizeof(blankLine));
    for (uint16_t r = 0; r < _fbH; ++r) {
      bus.rawWriteBytes(blankLine, _fbWb);
    }
  }
  bus.endTxn();
}

void Uc8253CDriver::triggerRefresh(EpdBus& bus, bool fullRefresh, bool turnOff) {
  if (fullRefresh) {
    // Full refresh using OTP calibrated waveform (0x5A / 0x53)
    bus.cmd(CMD_CASCADE_SETTING);
    bus.data(0x02);  // TSFIX
    bus.cmd(CMD_FORCE_TEMPERATURE);
    bus.data(0x5A);  // 0x5A clean full refresh (~1000 ms) without gray tint
    bus.cmd(CMD_VCOM_DATA_INTERVAL);
    bus.data(0x97);

    powerOn(bus);
    bus.cmd(CMD_DISPLAY_REFRESH);
    bus.waitBusy(" UC8253C_REF_FULL");
  } else {
    // Fast partial refresh waveform (0x6E)
    bus.cmd(CMD_CASCADE_SETTING);
    bus.data(0x02);  // TSFIX
    bus.cmd(CMD_FORCE_TEMPERATURE);
    bus.data(0x6E);  // Fast update waveform (~300 ms)
    bus.cmd(CMD_VCOM_DATA_INTERVAL);
    bus.data(0xD7);

    powerOn(bus);
    bus.cmd(CMD_DISPLAY_REFRESH);
    bus.waitBusy(" UC8253C_REF_FAST");
  }

  // Always power off booster after refresh to discharge gates and prevent vertical lines / ghosting
  powerOff(bus);

  // Undo TSFIX and return controller to default floating VCOM state
  initController(bus);
}

void Uc8253CDriver::begin(EpdBus& bus) {
  _fbW = BoardConfig::ACTIVE.displayWidth ? BoardConfig::ACTIVE.displayWidth : 240;
  _fbH = BoardConfig::ACTIVE.displayHeight ? BoardConfig::ACTIVE.displayHeight : 416;
  _fbWb = _fbW / 8;
  const uint32_t totalBytes = static_cast<uint32_t>(_fbWb) * _fbH;
  if (_prevFb == nullptr) {
    _prevFb = static_cast<uint8_t*>(malloc(totalBytes));
  }
  if (_prevFb != nullptr) {
    memset(_prevFb, 0xFF, totalBytes);
  }
  bus.reset(10);
  _isScreenOn = false;
  _fastRefreshCount = 0;
  _hasPrev = false;
  initController(bus);
}

void Uc8253CDriver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  const uint32_t totalBytes = static_cast<uint32_t>(_fbWb) * _fbH;
  if (_prevFb == nullptr) {
    _prevFb = static_cast<uint8_t*>(malloc(totalBytes));
    if (_prevFb != nullptr) {
      memset(_prevFb, 0xFF, totalBytes);
    }
  }
  const uint8_t* actualPrev = prev ? prev : (_hasPrev ? _prevFb : nullptr);

  bool useFast = (mode == RefreshMode::Fast && actualPrev != nullptr);
  if (useFast) {
    if (_ghostClearInterval != 0 && _fastRefreshCount >= _ghostClearInterval) {
      useFast = false;
      _fastRefreshCount = 0;
    } else {
      _fastRefreshCount++;
    }
  } else {
    _fastRefreshCount = 0;
  }

  if (useFast) {
    // Fast differential refresh: no hardware reset! DTM1 holds previous state, DTM2 receives new image.
    // The UC8253C controller calculates differential transitions instantly (~300ms) without flashing.
    writePlane(bus, CMD_DTM1, actualPrev);
    writePlane(bus, CMD_DTM2, fb);
    triggerRefresh(bus, false, turnOff);
  } else {
    // Full clean refresh: hardware reset and full GC waveform to eliminate ghosting
    bus.reset(10);
    _isScreenOn = false;
    initController(bus);
    writePlane(bus, CMD_DTM1, fb);
    writePlane(bus, CMD_DTM2, fb);
    triggerRefresh(bus, true, turnOff);
  }

  if (_prevFb != nullptr && fb != nullptr) {
    memcpy(_prevFb, fb, totalBytes);
    _hasPrev = true;
  }
}

void Uc8253CDriver::deepSleep(EpdBus& bus) {
  powerOff(bus);
  bus.cmd(CMD_DEEP_SLEEP);
  bus.data(0xA5);
}

PanelDriver& uc8253CDriver() {
  static Uc8253CDriver instance;
  return instance;
}

}  // namespace freeink

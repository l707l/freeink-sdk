#pragma once

// UC8253C panel driver — WeAct Studio 3.7" E-Paper Module (240x416 B/W, ESP32-S3 SuperMini).
// Hardware resolution: 240x416 pixels (portrait native).
// Compatible with UC8253 / UC8253C / GDEY037T03 panels.

#include "PanelDriver.h"

namespace freeink {

class Uc8253CDriver : public PanelDriver {
 public:
  Uc8253CDriver();

  uint32_t spiHz() const override;
  BusyPolarity busyPolarity() const override { return BusyPolarity::ActiveLow; }  // UC8253 BUSY is LOW when busy
  PanelGeometry geometry() const override;
  int8_t spiMiso() const override;
  int8_t coCs() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;

 private:
  void initController(EpdBus& bus);
  void powerOn(EpdBus& bus);
  void powerOff(EpdBus& bus);
  void writePlane(EpdBus& bus, uint8_t command, const uint8_t* fb);
  void triggerRefresh(EpdBus& bus, bool fullRefresh, bool turnOff);

  uint16_t _fbW;   // 240
  uint16_t _fbH;   // 416
  uint16_t _fbWb;  // 30 bytes per row

  bool _isScreenOn = false;
  uint8_t _fastRefreshCount = 0;
  uint8_t _ghostClearInterval = 10;
  uint8_t* _prevFb = nullptr;
  bool _hasPrev = false;
};

PanelDriver& uc8253CDriver();

}  // namespace freeink

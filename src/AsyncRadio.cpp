#include "AsyncRadio.hpp"

namespace __radio_details {
SX1262* sx1262 = nullptr;

volatile bool radioPacketSent = false;
volatile bool radioPreambleReceived = false;
volatile bool radioPacketReceived = false;

ICACHE_RAM_ATTR
void handleRadioPacketSent(void) {
  radioPacketSent = true;
}

ICACHE_RAM_ATTR
void handleRadioPacketReceived(void) {
  radioPacketReceived = true;
}

ICACHE_RAM_ATTR
void handleRadioPacketReceivedSX1262(void) {
  uint16_t irqStatus = sx1262->getIrqStatus();
  sx1262->clearIrqStatus();

  switch (irqStatus) {
  case RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED:
    radioPreambleReceived = true;
    break;
  case RADIOLIB_SX126X_IRQ_RX_DONE:
    radioPacketReceived = true;
    break;
  }
}
} // namespace __radio_details

#pragma once

#include <RadioLib.h>
#include <string>

namespace __radio_details {
extern SX1262* sx1262;

extern volatile bool radioPacketSent;
extern volatile bool radioPreambleReceived;
extern volatile bool radioPacketReceived;

ICACHE_RAM_ATTR
extern void handleRadioPacketSent(void);

ICACHE_RAM_ATTR
extern void handleRadioPacketReceived(void);

ICACHE_RAM_ATTR
extern void handleRadioPacketReceivedSX1262(void);
} // namespace __radio_details

template <typename Driver>
class AsyncRadio {
private:
  Driver _driver;

  float _freq = 0;

  void (*_onTransmit)() = []() {
  };
  void (*_onPreamble)() = []() {
  };
  void (*_onPacket)() = []() {
  };

  bool _transmitting = false;
  bool _receiving = false;

  int (*_textEncoder)(const char* in, int in_len, char* out) = nullptr;
  int (*_textDecoder)(const char* in, int in_len, char* out) = nullptr;

  int32_t _rssi = 0;
  float _snr = 0;
  int32_t _ferr = 0;

  void reduceFrequencyDrift() {
    int32_t ferr = getFrequencyError();
    if (std::abs(ferr) < 500) {
      return;
    }

    _driver.setFrequency(_freq += (ferr * -0.2e-6));

    // setFrequency puts _driver into standby
    listen();
  }

public:
  AsyncRadio(Module* module) : _driver{module} {
    if constexpr (std::is_same_v<Driver, SX1262>) {
      __radio_details::sx1262 = &_driver;
    }
  }

  void tick() {
    if (__radio_details::radioPacketSent) {
      __radio_details::radioPacketSent = false;

      _transmitting = false;

      listen();

      _onTransmit();
    }

    if (__radio_details::radioPreambleReceived) {
      __radio_details::radioPreambleReceived = false;

      _receiving = true;

      _onPreamble();
    }

    if (__radio_details::radioPacketReceived) {
      __radio_details::radioPacketReceived = false;

      _receiving = false;

      _rssi = 0;
      _snr = 0;
      _ferr = 0;

      reduceFrequencyDrift();

      _onPacket();
    }
  }

  void onTransmit(void (*onTransmit)()) {
    _onTransmit = onTransmit;
  }

  void onPreamble(void (*onPreamble)()) {
    _onPreamble = onPreamble;
  }

  void onPacket(void (*onPacket)()) {
    _onPacket = onPacket;
  }

  void setTextEncoder(int (*textEncoder)(const char* in, int in_len, char* out)) {
    _textEncoder = textEncoder;
  }

  void setTextDecoder(int (*textDecoder)(const char* in, int in_len, char* out)) {
    _textDecoder = textDecoder;
  }

  int16_t begin(float freq = (434.0F), float bw = (125.0F), uint8_t sf = (uint8_t)9U, uint8_t cr = (uint8_t)7U,
      uint8_t syncWord = (uint8_t)18U, int8_t power = (int8_t)10, uint16_t preambleLength = (uint16_t)8U) {
    _freq = freq;

    int16_t rc = _driver.begin(freq, bw, sf, cr, syncWord, power, preambleLength);
    return rc;
  }

  int16_t transmit(std::string_view str) {
    _transmitting = true;

    _driver.setPacketSentAction(__radio_details::handleRadioPacketSent);

    const char* data = str.data();
    size_t data_len = str.length();

    char encoded[256] = {0};
    if (_textEncoder) {
      int encoded_len = _textEncoder(str.data(), str.length(), encoded);

      data = encoded;
      data_len = encoded_len;
    }

    int16_t rc = _driver.startTransmit((uint8_t*)data, data_len, 0);
    return rc;
  }

  int16_t transmit(const String& str) {
    int16_t rc = transmit(std::string_view{str.c_str(), str.length()});
    return rc;
  }

  int16_t transmit(const char* format, ...) {
    char line[256];

    va_list args;
    va_start(args, format);
    size_t length = vsprintf(line, format, args);
    va_end(args);

    int16_t rc = transmit(std::string_view{line, length});
    return rc;
  }

  int16_t listen() {
    if constexpr (std::is_same_v<Driver, SX1262>) {
      constexpr uint32_t RX_TIMEOUT = RADIOLIB_SX126X_RX_TIMEOUT_INF;
      constexpr uint16_t IRQ_FLAGS = RADIOLIB_SX126X_IRQ_RX_DEFAULT | RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;
      constexpr uint16_t IRQ_MASK = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;

      _driver.setPacketReceivedAction(__radio_details::handleRadioPacketReceivedSX1262);

      int16_t rc = _driver.startReceive(RX_TIMEOUT, IRQ_FLAGS, IRQ_MASK, 0);
      return rc;
    } else {
      _driver.setPacketReceivedAction(__radio_details::handleRadioPacketReceived);

      int16_t rc = _driver.startReceive();
      return rc;
    }
  }

  int16_t readData(std::string& str, size_t len = 0) {
    (void)len;

    std::string buf;
    buf.resize(_driver.getPacketLength());
    int16_t rc = _driver.readData((uint8_t*)buf.data(), buf.length());

    if (_textDecoder) {
      str = std::string{};
      str.resize(256);
      int str_len = _textDecoder(buf.data(), buf.length(), str.data());
      str.resize(str_len);
    } else {
      str = std::move(buf);
    }

    return rc;
  }

  int16_t readData(String& str, size_t len = 0) {
    (void)len;

    std::string buf;
    int16_t rc = readData(buf);

    str = String{buf.data()};

    return rc;
  }

  int16_t readData(uint8_t* data, size_t len) {
    int16_t rc = _driver.readData(data, len);
    return rc;
  }

  std::string readString() {
    std::string result;
    readData(result);
    return result;
  }

  auto setCurrentLimit(float currentLimit) {
    return _driver.setCurrentLimit(currentLimit);
  }

  auto setRxBoostedGainMode(bool rxbgm) {
    return _driver.setRxBoostedGainMode(rxbgm);
  }

  auto getTimeOnAir(size_t len) {
    return _driver.getTimeOnAir(len);
  }

  auto getDataRate() {
    return _driver.getDataRate();
  }

  auto getRSSI() {
    if (_rssi == 0) {
      _rssi = _driver.getRSSI();
    }

    return _rssi;
  }

  auto getSNR() {
    if (_snr == 0) {
      _snr = _driver.getSNR();
    }

    return _snr;
  }

  auto getFrequencyError() {
    if (_ferr == 0) {
      _ferr = _driver.getFrequencyError();
    }

    return _ferr;
  }

  auto getFrequency() {
    return _freq;
  }

  auto transmitting() {
    return _transmitting;
  }

  auto receiving() {
    return _receiving;
  }
};

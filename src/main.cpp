#include "credentials.hpp"
#include "heltec.hpp"
#include "pin_config.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <OneButtonTiny.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>
#include <WiFi.h>
#include "gazatu-rf.hpp"

static SSD1306Wire gfx{0x3c, PIN_DISPLAY_SDA, PIN_DISPLAY_SCL, GEOMETRY_64_32};

OneButtonTiny button0{PIN_BUTTON_USER};

AsyncRadio<SX1262> radio{new Module(PIN_RADIO_CS, PIN_RADIO_D1, PIN_RADIO_RST, PIN_RADIO_D2)};

int16_t setupRadio(void) {
  constexpr float FREQ = 869.47; // frequency in MHz
  constexpr float BW = 20.8;     // bandwidth in kHz (10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125)
  constexpr uint8_t SF = 10;     // spreading factor (6 to 12)
  constexpr uint8_t CR = 6;      // coding rate (5 to 8, error correction)
  constexpr uint8_t SYNC = 27;   // sync word
  constexpr uint8_t POWER = 22;  // output power in dBm
  constexpr uint8_t PRELEN = 8;  // preamble length

  int16_t rc = radio.begin(FREQ, BW, SF, CR, SYNC, POWER, PRELEN);

  if (rc == RADIOLIB_ERR_NONE) {
    radio.setCurrentLimit(125);
    radio.setRxBoostedGainMode(true);
  }

  return rc;
}

AsyncWebServer server{80};
AsyncWebSocket socket{"/ws"};

// void enterDeepSleep() {
//   WiFi.disconnect(true);
//   gfx.displayOff();
//   radio.sleep(false);
//   enableDisplay(false);
//   setBoardLED(0);

//   // Set all pins to input to save power
//   pinMode(PIN_VBAT_CTRL, INPUT);
//   pinMode(PIN_VBAT_ADC, INPUT);
//   pinMode(PIN_RADIO_D1, INPUT);
//   pinMode(PIN_RADIO_RST, INPUT);
//   pinMode(PIN_RADIO_D0, INPUT);
//   pinMode(PIN_SPI_CS, INPUT);
//   pinMode(PIN_SPI_MISO, INPUT);
//   pinMode(PIN_SPI_MOSI, INPUT);
//   pinMode(PIN_SPI_SCK, INPUT);
//   pinMode(PIN_DISPLAY_SDA, INPUT);
//   pinMode(PIN_DISPLAY_SCL, INPUT);
//   pinMode(PIN_DISPLAY_RST, INPUT);

//   esp_deep_sleep_start();
// }

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.printf("booted\n");

  enableDisplay();

  gfx.init();
  gfx.setContrast(255);
  gfx.flipScreenVertically();

  gfx.setFont(ArialMT_Plain_10);
  gfx.setTextAlignment(TEXT_ALIGN_LEFT);
  gfx.printf("booted\n");
  gfx.display();

  // gfx->begin();
  // gfx->setRotation(1);
  // gfx->fillScreen(BLACK);
  // gfx->setCursor(0, 0);

  // gfx->setTextSize(1);
  // gfx->setTextColor(WHITE, BLACK);
  // gfx->printf("booted\n");

  // gfx->printf("connecting to wifi...\n");
  WiFi.setHostname("heltec");
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // gfx->println("failed to connect to wifi");
  }

  button0.attachClick([]() {
    rf::transmit(radio, rf::SignalStrengthRequest{});
    Serial.printf("[SX1262] Sending...\n");
  });

  // gfx->printf("local ipv4 = %s\n", WiFi.localIP().toString().c_str());

  int16_t radioBeginState = setupRadio();
  Serial.println(F("[SX1262] Started!"));

  int16_t radioListenState = radio.listen();
  gfx.printf("RF %d\n", radioListenState);
  Serial.println(F("[SX1262] Listening!"));

  radio.onTransmit([]() {
    Serial.printf("[SX1262] transmit done\n");

    JsonDocument json;
    json["type"] = "sent";

    String jsonStr;
    serializeJson(json, jsonStr);
    socket.textAll(jsonStr);
  });

  radio.onPreamble([]() {
    Serial.printf("[SX1262] preamble detected\n");

    JsonDocument json;
    json["type"] = "preamble";

    String jsonStr;
    serializeJson(json, jsonStr);
    socket.textAll(jsonStr);
  });

  radio.onPacket([]() {
    int32_t rssi = radio.getRSSI();

    gfx.printf("%d dBM\n", rssi);
    Serial.printf("[SX1262] %d dBM\n", rssi);

    rf::handle(radio,
        rf::handlers{
            [](std::monostate&) {
            },
            [](rf::SignalStrengthRequest&) {
            },
            [](rf::SignalStrengthResponse& txSignal) {
              int32_t rssi = radio.getRSSI();
              float snr = radio.getSNR();
              int32_t ferr = radio.getFrequencyError();

              JsonDocument json;
              json["type"] = "message";
              json["rssi"] = rssi;
              json["snr"] = snr;
              json["ferr"] = ferr;
              auto tx = json["tx"].to<JsonObject>();
              tx["rssi"] = txSignal.rssi * -1;
              tx["snr"] = txSignal.snr;
              tx["ferr"] = txSignal.ferr * 100;

              String jsonStr;
              serializeJson(json, jsonStr);
              Serial.printf("%s\n", jsonStr.c_str());
              socket.textAll(jsonStr);
            },
        });
  });

  socket.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        std::string_view body{(char*)data, len};

        rf::transmit(radio, rf::SignalStrengthRequest{});
        Serial.printf("[SX1262] Sent\n");
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    }
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "TEST");
  });
  server.addHandler(&socket);
  server.begin();
}

void loop() {
  button0.tick();

  radio.tick();

  socket.cleanupClients();
}

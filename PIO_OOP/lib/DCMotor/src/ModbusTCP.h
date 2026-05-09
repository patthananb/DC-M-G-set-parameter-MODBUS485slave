#pragma once

#include "Config.h"

#if DCMOTOR_ENABLE_MODBUS_TCP

#include <Arduino.h>
#include <Network.h>
#include <WiFi.h>
#include "Measurements.h"

namespace config { namespace modbus {

using dcmotormeasurement::Measurements;

struct ModbusTCP {
public:
  explicit ModbusTCP(uint16_t port = settings::MODBUS_TCP_PORT);

  bool begin(const char* ssid = settings::WIFI_SSID,
             const char* password = settings::WIFI_PASSWORD,
             uint8_t unitId = settings::MODBUS_ADDR,
             uint8_t regCount = settings::REG_COUNT,
             uint32_t connectTimeoutMs = settings::WIFI_CONNECT_TIMEOUT_MS);

  void update(const Measurements& m);
  void poll();

  bool isConnected() const;
  IPAddress localIP() const;

private:
  struct MbapHeader {
    uint16_t transactionId = 0;
    uint16_t protocolId    = 0;
    uint16_t length        = 0;
    uint8_t  unitId        = 0;
  };

  bool readFrame(WiFiClient& client, MbapHeader& header, uint8_t* pdu, size_t& pduLen);
  void handleRequest(WiFiClient& client, const MbapHeader& header, const uint8_t* pdu, size_t pduLen);
  void sendReadHoldingRegisters(WiFiClient& client,
                                const MbapHeader& header,
                                uint16_t start,
                                uint16_t count);
  void sendException(WiFiClient& client,
                     const MbapHeader& header,
                     uint8_t function,
                     uint8_t code);
  void writeMbap(WiFiClient& client,
                 const MbapHeader& request,
                 uint16_t pduLen);

  WiFiServer server_;
  WiFiClient client_;
  uint16_t   regs_[settings::REG_COUNT] = {};
  uint16_t   port_                      = settings::MODBUS_TCP_PORT;
  uint8_t    unitId_                    = settings::MODBUS_ADDR;
  uint8_t    regCount_                  = settings::REG_COUNT;
};

using TCP = ModbusTCP;

}  // namespace modbus
}  // namespace config

#endif  // DCMOTOR_ENABLE_MODBUS_TCP

#include "Config.h"

#if DCMOTOR_ENABLE_MODBUS_TCP

#include "ModbusTCP.h"
#include "RegisterMap.h"

namespace config { namespace modbus {

namespace {
uint16_t readU16(const uint8_t* data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

void writeU16(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value >> 8);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}
}  // namespace

ModbusTCP::ModbusTCP(uint16_t port)
    : server_(port), port_(port) {}

bool ModbusTCP::begin(const char* ssid,
                      const char* password,
                      uint8_t unitId,
                      uint8_t regCount,
                      uint32_t connectTimeoutMs) {
  unitId_ = unitId;
  regCount_ = (regCount > settings::REG_COUNT) ? settings::REG_COUNT : regCount;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - started) < connectTimeoutMs) {
    delay(settings::WIFI_CONNECT_POLL_MS);
  }

  if (WiFi.status() != WL_CONNECTED) return false;

  server_.begin(port_);
  return true;
}

void ModbusTCP::update(const Measurements& m) {
  writeMeasurementsToHoldingRegisters(regs_, regCount_, m);
}

void ModbusTCP::poll() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!client_ || !client_.connected()) {
    WiFiClient next = server_.accept();
    if (next) {
      client_ = next;
      client_.setTimeout(settings::MODBUS_TCP_CLIENT_TIMEOUT_MS);
      client_.setNoDelay(true);
    }
  }

  if (!client_ || !client_.connected() ||
      client_.available() < settings::MODBUS_TCP_MBAP_HEADER_BYTES) {
    return;
  }

  MbapHeader header;
  uint8_t pdu[settings::MODBUS_TCP_MAX_PDU_BYTES] = {};
  size_t pduLen = 0;
  if (!readFrame(client_, header, pdu, pduLen)) {
    client_.stop();
    return;
  }

  handleRequest(client_, header, pdu, pduLen);
}

bool ModbusTCP::isConnected() const {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress ModbusTCP::localIP() const {
  return WiFi.localIP();
}

bool ModbusTCP::readFrame(WiFiClient& client,
                          MbapHeader& header,
                          uint8_t* pdu,
                          size_t& pduLen) {
  uint8_t mbap[settings::MODBUS_TCP_MBAP_HEADER_BYTES] = {};
  if (client.readBytes(mbap, sizeof(mbap)) != sizeof(mbap)) return false;

  header.transactionId = readU16(&mbap[0]);
  header.protocolId    = readU16(&mbap[2]);
  header.length        = readU16(&mbap[4]);
  header.unitId        = mbap[6];

  if (header.length == 0) return false;

  pduLen = header.length - 1;
  if (pduLen > settings::MODBUS_TCP_MAX_PDU_BYTES) return false;
  if (pduLen == 0) return true;

  return client.readBytes(pdu, pduLen) == pduLen;
}

void ModbusTCP::handleRequest(WiFiClient& client,
                              const MbapHeader& header,
                              const uint8_t* pdu,
                              size_t pduLen) {
  if (header.protocolId != 0 || header.unitId != unitId_) return;
  if (pduLen == 0) return;

  const uint8_t function = pdu[0];
  if (function != settings::MODBUS_TCP_READ_HOLDING_REGISTERS) {
    sendException(client, header, function, settings::MODBUS_TCP_ILLEGAL_FUNCTION);
    return;
  }

  if (pduLen != 5) {
    sendException(client, header, function, settings::MODBUS_TCP_ILLEGAL_DATA_VALUE);
    return;
  }

  const uint16_t start = readU16(&pdu[1]);
  const uint16_t count = readU16(&pdu[3]);
  if (count == 0 || count > settings::MODBUS_TCP_MAX_READ_REGISTERS) {
    sendException(client, header, function, settings::MODBUS_TCP_ILLEGAL_DATA_VALUE);
    return;
  }

  if (start >= regCount_ || (start + count) > regCount_) {
    sendException(client, header, function, settings::MODBUS_TCP_ILLEGAL_DATA_ADDRESS);
    return;
  }

  sendReadHoldingRegisters(client, header, start, count);
}

void ModbusTCP::sendReadHoldingRegisters(WiFiClient& client,
                                         const MbapHeader& header,
                                         uint16_t start,
                                         uint16_t count) {
  const uint16_t pduLen = 2 + count * 2;
  writeMbap(client, header, pduLen);

  client.write(settings::MODBUS_TCP_READ_HOLDING_REGISTERS);
  client.write(static_cast<uint8_t>(count * 2));

  uint8_t word[2] = {};
  for (uint16_t i = 0; i < count; ++i) {
    writeU16(word, regs_[start + i]);
    client.write(word, sizeof(word));
  }
}

void ModbusTCP::sendException(WiFiClient& client,
                              const MbapHeader& header,
                              uint8_t function,
                              uint8_t code) {
  writeMbap(client, header, 2);
  client.write(static_cast<uint8_t>(function | settings::MODBUS_TCP_EXCEPTION_FLAG));
  client.write(code);
}

void ModbusTCP::writeMbap(WiFiClient& client,
                          const MbapHeader& request,
                          uint16_t pduLen) {
  uint8_t mbap[settings::MODBUS_TCP_MBAP_HEADER_BYTES] = {};
  writeU16(&mbap[0], request.transactionId);
  writeU16(&mbap[2], 0);
  writeU16(&mbap[4], pduLen + 1);
  mbap[6] = request.unitId;
  client.write(mbap, sizeof(mbap));
}

}  // namespace modbus
}  // namespace config

#endif  // DCMOTOR_ENABLE_MODBUS_TCP

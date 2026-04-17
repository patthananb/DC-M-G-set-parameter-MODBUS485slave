#include <ModbusRTUSlave.h>

HardwareSerial RS485Serial(1);
ModbusRTUSlave modbus(RS485Serial);

void initSampling100Hz();
bool updateMeasurementsFromTimer();
void getLatestMeasurements(int16_t &speedOut, float &vmotOut, float &vgenOut);

// Array to hold 5 registers (1 for int + 2 for float + 2 for float)
uint16_t holdingRegisters[5];

void setup() {
  Serial.begin(115200);

  // ESP32-C6 UART1 pins: TXD=GPIO6, RXD=GPIO7
  RS485Serial.begin(9600, SERIAL_8N1, 7, 6);

  // Begin Modbus slave on address 1, baudrate 9600 (8-N-1)
  modbus.begin(1, 9600);
  
  // Configure the library to use our 5-register array
  modbus.configureHoldingRegisters(holdingRegisters, 5);

  initSampling100Hz();
}

void loop() {
  // Update samples when the 100 Hz timer marks the next sample period.
  const bool sampleUpdated = updateMeasurementsFromTimer();

  int16_t speed = 0;
  float vmot = 0.0f;
  float v_gen = 0.0f;
  getLatestMeasurements(speed, vmot, v_gen);

  // Assign the 16-bit integer directly to register 0
  holdingRegisters[0] = speed;

  // Safely copy the 32-bit floats into the remaining 16-bit Modbus registers
  // vmot -> registers 1 and 2
  memcpy(&holdingRegisters[1], &vmot, sizeof(vmot));
  // v_gen -> registers 3 and 4
  memcpy(&holdingRegisters[3], &v_gen, sizeof(v_gen));

  // Listen for and respond to incoming Modbus requests
  modbus.poll();

  if (sampleUpdated) {
    Serial.print("speed=");
    Serial.print(speed);
    Serial.print(", vmot=");
    Serial.print(vmot, 3);
    Serial.print(" V, vgen=");
    Serial.print(v_gen, 3);
    Serial.println(" V");
  }
}
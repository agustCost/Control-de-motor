#include <Arduino.h>
#include <ModbusRTUSlave.h>

#define MODBUS_SERIAL Serial2   //pines 16(Rx) y 17(Tx)

//Estos valores se modifican en funcion de la configuracion del master
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1

ModbusRTUSlave modbus(MODBUS_SERIAL);

//se crean las listas de coils, discrete inputs, holding registers e input registers
const uint8_t numCoils = 2;
const uint8_t numDiscreteInputs = 2;
const uint8_t numHoldingRegisters = 2;
const uint8_t numInputRegisters = 2;

bool coils[numCoils];
bool discreteInputs[numDiscreteInputs];
uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

void setup() {
  //se inicializan los registros y la comunicacion por modbus
  modbus.configureCoils(coils, numCoils);
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);

  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
}

void loop() {
  modbus.poll(); //cuando se llama a esta funcion, se fija si hay alguna solicitud del master
}

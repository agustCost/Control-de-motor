//Codigo placa slave Id 1 (Medicion)
#include <Arduino.h>
#include <ModbusRTUSlave.h>

#define MODBUS_SERIAL Serial2   //pines 16(Rx) y 17(Tx)
#define potPin 25
#define switchPin1 14
#define switchPin2 13

//Estos valores se modifican en funcion de la configuracion del master
#define MODBUS_BAUD 9600
#define MODBUS_CONFIG SERIAL_8N1
int MODBUS_UNIT_ID;

ModbusRTUSlave modbus(MODBUS_SERIAL, 18);

static unsigned long lastUpdate = 0;

//se crean las listas de coils, discrete inputs, holding registers e input registers
const uint8_t numCoils = 0;
const uint8_t numDiscreteInputs = 0;
const uint8_t numHoldingRegisters = 0;
const uint8_t numInputRegisters = 1;

bool coils[numCoils];
bool discreteInputs[numDiscreteInputs];
uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

int vibracion (){
  return (int)(analogRead(potPin) / 40.95);
}

void setup() {
  //se inicializan los registros y la comunicacion por modbus
  pinMode(potPin, INPUT);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW); 

  bool sw1 = digitalRead(switchPin1);
  bool sw2 = digitalRead(switchPin2);

  modbus.configureCoils(coils, numCoils);
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);
  
  if(sw1 && sw2){
    MODBUS_UNIT_ID = 0;
  }else if (sw2){
    MODBUS_UNIT_ID = 2;
  }else if (sw1){
    MODBUS_UNIT_ID = 1;
  }else{
    MODBUS_UNIT_ID = 3;
  }

  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG, 16, 17);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
  Serial.begin(9600);
}

void loop() {
  if (millis() - lastUpdate > 500) {
    inputRegisters[0] = vibracion();
    lastUpdate = millis();
  }
  modbus.poll();
}

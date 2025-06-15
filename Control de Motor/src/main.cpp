//Codigo placa slave Id 1 (Medicion)
#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>
#include <ModbusSlaveLogic.h>

//Definicion de pines y variables globales
#define MODBUS_SERIAL Serial2   //pines 16(Rx) y 17(Tx)
#define potPin 25
#define switchPin1 14
#define switchPin2 13
#define adcTempPin 34

int vibrationSum = 0;
int sampleCount = 0;
static unsigned long lastPotRead = 0;
static unsigned long lastUpdate = 0;

//Parametros de comunicacion serial
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1

//Creacion del objeto modbus
ModbusRTUSlave modbus(MODBUS_SERIAL, 18);

//Creacion de las listas de discrete inputs, holding registers e input registers
const uint8_t numDiscreteInputs = 2;
const uint8_t numHoldingRegisters = 2;
const uint8_t numInputRegisters = 2;

uint16_t holdingRegisters[numHoldingRegisters];
bool discreteInputs[numDiscreteInputs];
uint16_t inputRegisters[numInputRegisters];

//Medida de vibracion del motor
int vibration (){
  float avgVar = vibrationSum/sampleCount;
  vibrationSum = 0;
  sampleCount = 0;
  return (int)(avgVar*100/2048);
}

float gain = 10.15;     //Ganancia del sensor de temperatura  

//Medida de temperatura
float readTemperature(){
  int adcValue = analogRead(adcTempPin);
  float voltaje = ((adcValue / 4095.0)) * 3.3;
  float sensorVoltaje = voltaje / gain;
  float temperature = sensorVoltaje * 100; // 10mV / C
  return temperature;
}

void setup() {
  //Inicializacion de los pines
  pinMode(potPin, INPUT);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW); 

  //Inicializacion de los registros MODBUS
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);

  //Lectura de los dipswitches
  int sw1 = digitalRead(switchPin1);
  int sw2 = digitalRead(switchPin2);

  //Seleccion de Slave ID en funcion del estado de los dipswitches
  int MODBUS_UNIT_ID;

  if(sw1 == 1 && sw2 == 1){
    MODBUS_UNIT_ID = 0;
  }else if (sw2 == 1){
    MODBUS_UNIT_ID = 2;
  }else if (sw1 == 1){
    MODBUS_UNIT_ID = 1;
  }else{
    MODBUS_UNIT_ID = 3;
  }

  //Inicializacion de la comunicacion con MODBUS
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG, 16, 17);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
}

void loop() {
  if (micros() - lastPotRead > 1000) {
    vibrationSum += abs(analogRead(potPin)-2048);
    sampleCount++;
    lastPotRead = micros();
  }

  if (millis() - lastUpdate > 500) {

    //Actualizacion de valores medidos
    inputRegisters[0] = vibration();
    inputRegisters[1] = readTemperature(); 

    //Generacion de alarma de vibracion
    if(inputRegisters[0] >= holdingRegisters[0]){
      discreteInputs[0] = true;
    }else{
      discreteInputs[0] = false;
    }

    //Generacion de alarma de temperatura
    if(inputRegisters[1] >= holdingRegisters[1]){
      discreteInputs[1] = true;
    }else{
      discreteInputs[1] = false;
    }

    lastUpdate = millis();
  }

  modbus.poll();
}

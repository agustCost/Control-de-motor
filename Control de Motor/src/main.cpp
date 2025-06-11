//Codigo placa slave Id 2 (Motor)
#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>
#include <ModbusSlaveLogic.h>

#define MODBUS_SERIAL Serial2   //pines 16(Rx) y 17(Tx)
#define potPin 4
#define pwmPin 25
#define switchPin1 22
#define switchPin2 23
#define encoderPin 14

//Estos valores se modifican en funcion de la configuracion del master
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1
int MODBUS_UNIT_ID;

ModbusRTUSlave modbus(MODBUS_SERIAL, 18);

static unsigned long lastUpdate = 0;

int velPot;
int velPotPrev;
int pulseCount;
int pulsePrev = 1;
int pulseRead;
float samplingPeriod = 0.1;
float speed;

const float ppr = 823.1;  //considering gearbox


//se crean las listas de coils, discrete inputs, holding registers e input registers
const uint8_t numHoldingRegisters = 1;
const uint8_t numInputRegisters = 1;

uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

int velocidad (){
  return (int)(analogRead(potPin)*127/4095);
}

void setup() {
  //se inicializan los registros y la comunicacion por modbus
  pinMode(potPin, INPUT);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW); 

  ledcSetup(0, 1000, 8);
  ledcAttachPin(25, 0);

  int sw1 = digitalRead(switchPin1);
  int sw2 = digitalRead(switchPin2);

  int velPot = velocidad();
  int velPotPrev = velocidad();

  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);
  
  if(sw1 == 1 && sw2 == 1){
    MODBUS_UNIT_ID = 0;
  }else if (sw2 == 1){
    MODBUS_UNIT_ID = 2;
  }else if (sw1 == 1){
    MODBUS_UNIT_ID = 1;
  }else{
    MODBUS_UNIT_ID = 3;
  }

  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG, 16, 17);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
  Serial.begin(9600);
}

void loop() {
  ///////////////////////////////
  // LECTURA ENCODER
  pulseRead = digitalRead(encoderPin);

  if((pulseRead != pulsePrev) && (pulsePrev == 1)){
    pulseCount++;
  }

  pulsePrev = pulseRead;

  ///////////////////////////////

  if (millis() - lastUpdate > 1000) {
    
    inputRegisters[0] = (pulseCount/ppr)*60;
    pulseCount = 0;

    velPot = velocidad();
    if (velPot != velPotPrev){
      holdingRegisters[0] = velPot;
      velPotPrev = velPot;
    }

    //analogWrite(pwmPin, holdingRegisters[0]);
    ledcWrite(0, holdingRegisters[0]);

    lastUpdate = millis();
    
  }

  modbus.poll();
}
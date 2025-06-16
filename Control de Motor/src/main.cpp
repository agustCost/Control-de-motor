//Codigo placa slave Id 2 (Motor)
#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>
#include <ModbusSlaveLogic.h>

//Definicion de pines y variables globales
#define MODBUS_SERIAL Serial2   //pines 16(Rx) y 17(Tx)
#define potPin 4
#define pwmPin 25
#define switchPin1 22
#define switchPin2 23
#define encoderPin 14

static unsigned long lastUpdate = 0;
int potPrev = 0;
int pulseCount = 0;
int pulsePrev = 1;
int pulseRead;
float samplingPeriod = 0.5;
int pwm = 0;
const float ppr = 823.1;  //Pulsos del encoder por revolucion

//Parametros de comunicacion serial
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1

//Creacion del objeto modbus
ModbusRTUSlave modbus(MODBUS_SERIAL, 18);

//Creacion de las listas de holding registers e input registers
const uint8_t numHoldingRegisters = 1;
const uint8_t numInputRegisters = 1;

uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

//Medicion de la velocidad objetivo mediante la posicion del potenciometro
void velocidadRef (){
 int pot = analogRead(potPin);
  if (pot < potPrev-10 || pot > potPrev+10){
    holdingRegisters[0] = pot*100/4095;
    potPrev = pot;
  }
}

void setup() {
  //Inicializacion de los pines
  pinMode(potPin, INPUT);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW); 

  //Inicializacion del PWM
  ledcSetup(0, 1000, 8);
  ledcAttachPin(pwmPin, 0);

  //Inicializacion de los registros MODBUS
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
  // Lectura del encoder
  pulseRead = digitalRead(encoderPin);

  if((pulseRead != pulsePrev) && (pulsePrev == 1)){
    pulseCount++;
  }

  pulsePrev = pulseRead;

  if (millis() - lastUpdate > 500) {
    
    // Calculo de la velocidad medida
    inputRegisters[0] = (pulseCount/ppr)*60/samplingPeriod;
    pulseCount = 0;
    
    // Actualizacion de la velocidad objetivo
    velocidadRef();

    // PID y actualizacion del ciclo de trabajo del PWM
    int diff = holdingRegisters[0] - inputRegisters[0];
    pwm += diff*255/100;

    if(pwm > 255){
      pwm = 255;
    }else if(pwm < 0){
      pwm = 0;
    }
    if(holdingRegisters[0] == 0){
      pwm = 0;
    }
    // Generacion del PWM
    ledcWrite(0, pwm);
    lastUpdate = millis();
  }

  modbus.poll();
}

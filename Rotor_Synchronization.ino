/* Code for Syncronization of Motors spinning opposite to each other
 * Set Initial Zero Positions of Top motor and Bot motor as 3150 and 150 respectively to compensate for the physical placement offset between the two encoders
 * Bottom Motor Clockwise, Top motor Anti Clockwise rotation
 * Using SerialUSB instead of Serial prevents the Arduino from Auto Resetting when opening the Serial Monitor (apparently)*/
  
#include <SPI.h>
#include <Wire.h>
extern TwoWire Wire1;
#include <Servo.h>
#include <PID_v1.h>
#include <DirectIO.h>

/* Anti clockwise rotation Bot Motor pins */
#define botMotorPin 11
#define encoderPinIBot 35

/* Clockwise rotation Top Motor pins */
#define topMotorPin 12
#define encoderPinITop 45

/* SPI pins */
#define MOSI PROGMEM MOSI
#define MISO PROGMEM MISO
#define SCK PROGMEM SCK

#define CSNBot PROGMEM 10
#define CSNTop PROGMEM 4

/* Encoder's non-volatile register addresses */
#define ANGLECOM PROGMEM 0x3FFF
#define SETTINGS1 PROGMEM 0x0018
#define SETTINGS2 PROGMEM 0x0019
#define ZPOSM PROGMEM 0x0016
#define ZPOSL PROGMEM 0x0017

/* Bot motor RPM PID variables */
double botMotorPIDInput;
double botMotorPIDOutput;
double botMotorPIDSetpoint;

/* Top motor RPM PID variables */
double topMotorPIDInput;
double topMotorPIDOutput;
double topMotorPIDSetpoint;

/* Bot Motor Yaw PID variables */
double yawPIDInputBot;
double yawPIDOutputBot;
double yawPIDSetpointBot;

/* Top Motor Yaw PID variables */
double yawPIDInputTop;
double yawPIDOutputTop;
double yawPIDSetpointTop;

uint16_t botMotorSpeed;
uint16_t topMotorSpeed;
uint16_t inputSpeed;
uint16_t botMotorCount;
uint16_t topMotorCount;

volatile bool Aold1 = false;
volatile bool Bnew1 = false;
volatile bool Aold2 = false;
volatile bool Bnew2 = false;

volatile uint32_t dTimeBot;
volatile uint32_t dTimeTop;
volatile uint32_t lTimeBot;
volatile uint32_t lTimeTop;

volatile bool encoderBoolBot = true;
volatile bool encoderBoolTop = true;
volatile uint16_t encoderCountBot = 0;
volatile uint16_t encoderCountTop = 0;

/* User Input variables */
uint16_t phaseOffset = 0;

bool triggerBool = false;

uint8_t upperNum;
uint8_t lowerNum;

/* Direct IO pin access declarations. Faster than digitalRead(). Tied to attachInterrupts()  */
Input<31> encoderPinABot;
Input<33> encoderPinBBot;
Input<43> encoderPinATop;
Input<41> encoderPinBTop;
Output<27> triggerPin;

/* Library instantiations */
PID botMotorPID(&botMotorPIDInput,&botMotorPIDOutput,&botMotorPIDSetpoint,0.1,0.2,0,DIRECT);
PID topMotorPID(&topMotorPIDInput,&topMotorPIDOutput,&topMotorPIDSetpoint,0.1,0.2,0,DIRECT);
PID yawPIDBot(&yawPIDInputBot,&yawPIDOutputBot,&yawPIDSetpointBot,0.1,0.3,0,DIRECT);
PID yawPIDTop(&yawPIDInputTop,&yawPIDOutputTop,&yawPIDSetpointTop,0.1,0.3,0,DIRECT);
Servo botMotor;
Servo topMotor;

void setup(){
  
  /* ESC calibration */    
  pinMode(botMotorPin,OUTPUT);
  digitalWrite(botMotorPin,LOW);
  botMotor.attach(botMotorPin);
  botMotor.writeMicroseconds(0);
  pinMode(topMotorPin,OUTPUT);
  digitalWrite(topMotorPin,LOW);
  topMotor.attach(topMotorPin);
  topMotor.writeMicroseconds(0);
  delay(7000);
  
  /* Serial Communication Initialization. Uninitialize if not needed */
  SerialUSB.begin(115200);  
  
  SPI.begin();

  /* IIC Communication Initialisation */
  Wire.begin(8);
  Wire1.begin(9);
  Wire.onRequest(RequestEventBot);  
  Wire1.onRequest(RequestEventTop);    
  
  pinMode(CSNBot,OUTPUT);
  pinMode(CSNTop,OUTPUT);

  pinMode(triggerPin,OUTPUT);
  digitalWrite(triggerPin,LOW);
  
  /* Bottom motor interrupt 'A','B','I' pins */
  pinMode(encoderPinABot,INPUT);  
  digitalWrite(encoderPinABot,LOW);
  pinMode(encoderPinBBot,INPUT);
  digitalWrite(encoderPinBBot,LOW);
  pinMode(encoderPinIBot,INPUT);
  digitalWrite(encoderPinIBot,LOW);
  
  /* Top motor Interrupt 'A','B','I' pins */
  pinMode(encoderPinATop,INPUT);
  digitalWrite(encoderPinATop,LOW);
  pinMode(encoderPinBTop,INPUT);
  digitalWrite(encoderPinBTop,LOW);
  pinMode(encoderPinITop,INPUT);
  digitalWrite(encoderPinITop,LOW);
  
  /* Bottom motor interrupts */
  attachInterrupt(digitalPinToInterrupt(31),DoEncoderABot,CHANGE);
  attachInterrupt(digitalPinToInterrupt(33),DoEncoderBBot,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinIBot),DoEncoderIBot,FALLING);

  /* Top motor Interrupts */
  attachInterrupt(digitalPinToInterrupt(43),DoEncoderATop,CHANGE);
  attachInterrupt(digitalPinToInterrupt(41),DoEncoderBTop,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinITop),DoEncoderITop,FALLING);

  botMotorPID.SetMode(AUTOMATIC);
  botMotorPID.SetOutputLimits(1120,1750);  
  botMotorPID.SetSampleTime(8);
    
  topMotorPID.SetMode(AUTOMATIC);
  topMotorPID.SetOutputLimits(1120,1750);
  topMotorPID.SetSampleTime(8);
  
  yawPIDBot.SetMode(AUTOMATIC);
  yawPIDBot.SetOutputLimits(-10,10);
  yawPIDBot.SetSampleTime(4);
  
  yawPIDTop.SetMode(AUTOMATIC);
  yawPIDTop.SetOutputLimits(-10,10);
  yawPIDTop.SetSampleTime(4);
  
  /* Set the desired Zero Position as a permanent write to the Bot Encoder's registers */
  ZeroPosFunction(850+phaseOffset);
  AS5147PWrite(CSNBot,SETTINGS1,0x0004);
  AS5147PWrite(CSNBot,SETTINGS2,0x0020);
  AS5147PWrite(CSNBot,ZPOSM,upperNum);
  AS5147PWrite(CSNBot,ZPOSL,lowerNum);
  
  /* Set the desired Zero Position as a permanent write to the Top Encoder's registers */
  ZeroPosFunction(3150+phaseOffset);  
  AS5147PWrite(CSNTop,SETTINGS1,0x0000);
  AS5147PWrite(CSNTop,SETTINGS2,0x0020);
  AS5147PWrite(CSNTop,ZPOSM,upperNum);
  AS5147PWrite(CSNTop,ZPOSL,lowerNum);
}

/* Interrupt on 'A' changing state for bot motor */
void DoEncoderABot(){
  Bnew1^Aold1 ? encoderCountBot++ : encoderCountBot--;
  Aold1 = encoderPinABot.read();
}

/* Interrupt on 'B' changing state for bot motor */
void DoEncoderBBot(){
  Bnew1 = encoderPinBBot.read();
  Bnew1^Aold1 ? encoderCountBot++ : encoderCountBot--;
}

/* Interrupt on 'I' changing state for bot motor */
void DoEncoderIBot(){
  volatile uint32_t cTimeBot = micros();
  dTimeBot = cTimeBot - lTimeBot;
  lTimeBot = cTimeBot; 
  encoderCountBot = 0;
  encoderBoolBot = !encoderBoolBot;
  triggerBool = true;
}

/* Interrupt on 'A' changing state for top motor */
void DoEncoderATop(){
  Bnew2^Aold2 ? encoderCountTop++ : encoderCountTop--;
  Aold2 = encoderPinATop.read();
}

/* Interrupt on 'B' changing state for top motor */
void DoEncoderBTop(){
  Bnew2 = encoderPinBTop.read();
  Bnew2^Aold2 ? encoderCountTop++ : encoderCountTop--; 
}

/* Interrupt on 'I' changing state for top motor */
void DoEncoderITop(){  
  volatile uint32_t cTimeTop = micros();
  dTimeTop = cTimeTop - lTimeTop;
  lTimeTop = cTimeTop;
  encoderCountTop = 0;
  encoderBoolTop = !encoderBoolTop;
}

/* IIC message to send speeds to IMC studio through the Arduino Unos */
void RequestEventBot(){
  byte myArray[2];
  myArray[0] = (botMotorSpeed >> 8) & 0xFF;
  myArray[1] = botMotorSpeed & 0xFF;
  Wire.write(myArray,2); 
}

void RequestEventTop(){
  byte myArray[2];
  myArray[0] = (topMotorSpeed >> 8) & 0xFF;
  myArray[1] = topMotorSpeed & 0xFF;
  Wire1.write(myArray,2); 
}

/* Function to SPI write to the Encoder's registers */
uint16_t AS5147PWrite(int SSPin, uint16_t registerAddress, uint16_t data) {
  uint16_t command = 0x0000;
  command |= registerAddress;
  command |= static_cast<uint16_t>(spiCalcEvenParity(command)<<0xF);
  
  SPI.beginTransaction(SPISettings(3000000,MSBFIRST,SPI_MODE1));
  
  digitalWrite(SSPin,LOW);
  SPI.transfer16(command);
  digitalWrite(SSPin,HIGH);

  uint16_t dataToSend = 0x0000;
  dataToSend |= data;
  dataToSend |= static_cast<uint16_t>(spiCalcEvenParity(dataToSend)<<0xF);

  digitalWrite(SSPin,LOW);
  SPI.transfer16(dataToSend);
  digitalWrite(SSPin,HIGH);

  digitalWrite(SSPin,LOW);
  uint16_t response = SPI.transfer16(0x0000);
  digitalWrite(SSPin,HIGH);

  SPI.endTransaction();

  return response & ~0xC000;
}

/* Function to SPI read from the Encoder's registers */
uint16_t AS5147PRead(int SSPin, uint16_t registerAddress){
  uint16_t command = 0x4000;
  command = command | registerAddress;
  command |= static_cast<uint16_t>(spiCalcEvenParity(command)<<0xF);

  SPI.beginTransaction(SPISettings(3000000,MSBFIRST,SPI_MODE1));

  digitalWrite(SSPin,LOW);
  SPI.transfer16(command);
  digitalWrite(SSPin,HIGH);

  digitalWrite(SSPin,LOW);
  uint16_t response = SPI.transfer16(0x00);
  digitalWrite(SSPin,HIGH);

  SPI.endTransaction();

  return response & ~0xC000;
}

/* Parity check for SPI write/read confirmation */
uint8_t spiCalcEvenParity(uint16_t value){
  uint8_t count = 0;
  for(uint8_t i = 0; i < 16; i++)
  {
    if(value & 0x1)
    {
      count++;
    }
    value >>= 1;
  }
  return count & 0x1;
}

/* Zero Position function to split the desired Zero Poistion into upper 8 bits and lower 6 bits */
void ZeroPosFunction(float intValue)
{
  float value = intValue/64;
  upperNum = (int)value;
  float temp = value - (long)(value); 
  long p = 1;
  for(int i=0; i< 3; i++) p*=10;
  long decimalPart = p * temp;
  lowerNum = decimalPart/16; 
}

/* Read out Encoder's register values for debugging */
void DumpRegisterValues(){ 
  SerialUSB.print("ANGLECOMBot: "); 
  SerialUSB.println(AS5147PRead(CSNBot,ANGLECOM)); 
  SerialUSB.print("ZPOSMBot: "); 
  SerialUSB.println(AS5147PRead(CSNBot,ZPOSM));
  SerialUSB.print("ZPOSLBot: "); 
  SerialUSB.println(AS5147PRead(CSNBot,ZPOSL));
  SerialUSB.print("ANGLECOMTop: "); 
  SerialUSB.println(AS5147PRead(CSNTop,ANGLECOM)); 
  SerialUSB.print("ZPOSMTop: "); 
  SerialUSB.println(AS5147PRead(CSNTop,ZPOSM));
  SerialUSB.print("ZPOSLTop: "); 
  SerialUSB.println(AS5147PRead(CSNTop,ZPOSL));
  SerialUSB.println();
}

/* Debugging prints */
void DebuggingPrints(){
  SerialUSB.print("Input Speed: ");
  SerialUSB.println(inputSpeed);
  SerialUSB.print("Top Speed: ");
  SerialUSB.print(topMotorSpeed);
  SerialUSB.print(" | Bot Speed: ");
  SerialUSB.print(botMotorSpeed);
  SerialUSB.print(" | Top Count: ");
  SerialUSB.print(topMotorCount);
  SerialUSB.print(" | Bot Count: ");
  SerialUSB.print(botMotorCount);   
  SerialUSB.print(" | Top YAW PID: ");
  SerialUSB.print(yawPIDOutputTop); 
  SerialUSB.print(" | Bot YAW PID: ");
  SerialUSB.println(yawPIDOutputBot);
  SerialUSB.println();
}

/* RPM and count calculations for Bot Motor */
void RPMandCountFunctionBot(){  
  float SpeedInRPMBot;
  float Multiplier = ((60 * 1000000)/1024);
  if(micros() < (2*dTimeBot + lTimeBot)){
    SpeedInRPMBot = Multiplier/dTimeBot;
    botMotorSpeed = SpeedInRPMBot*1000;
    botMotorCount = encoderCountBot;
    if(encoderBoolBot ^ encoderBoolTop){
      botMotorCount = map(botMotorCount,0,2048,512,0);
    }else{
      botMotorCount = map(botMotorCount,0,2048,0,512); 
    }
  }else{
    SpeedInRPMBot = 0;    
    encoderCountBot = 0;
    botMotorCount = 0;
    botMotorSpeed = 0;
  }  
}

/* RPM and count calculations for Top Motor */
void RPMandCountFunctionTop(){  
  float SpeedInRPMTop;
  float Multiplier = ((60 * 1000000)/1024);
  if(micros() < (2*dTimeTop + lTimeTop)){
    SpeedInRPMTop = Multiplier/dTimeTop;    
    topMotorSpeed = SpeedInRPMTop*1000;
    topMotorCount = encoderCountTop;  
    if(encoderBoolTop ^ encoderBoolBot){
      topMotorCount = map(topMotorCount,0,2048,512,0);
    }else{
      topMotorCount = map(topMotorCount,0,2048,0,512);
    }  
  }else{
    SpeedInRPMTop = 0;
    encoderCountTop = 0;
    topMotorCount = 0;
    topMotorSpeed = 0;
  }   
}

/* Bot Motor RPM and YAW PIDs */
void BotMotorPIDs(){
  botMotorPIDInput = botMotorSpeed;
  botMotorPIDSetpoint = inputSpeed;
  botMotorPID.Compute();
  
  yawPIDInputBot = botMotorCount;
  yawPIDSetpointBot = topMotorCount;
  yawPIDBot.Compute();
}

/* Top Motor RPM and YAW PIDs */
void TopMotorPIDs(){
  topMotorPIDInput = topMotorSpeed;
  topMotorPIDSetpoint = inputSpeed;
  topMotorPID.Compute();

  yawPIDInputTop = topMotorCount;
  yawPIDSetpointTop = botMotorCount;
  yawPIDTop.Compute();
}

/* Trigger function for camera shutter  */
void TriggerFunction(){
  uint16_t timeNow;
  uint16_t timeThen;
  triggerPin.write(HIGH);
  timeNow = micros();
  if(timeNow - timeThen >= 5){
    triggerPin.write(LOW);
    timeThen = timeNow;
  }  
}

void loop(){  
  
  uint16_t totalPIDBot;
  uint16_t totalPIDTop;
  
  /* User Input for speed */
  if(SerialUSB.available()){
    inputSpeed = SerialUSB.parseInt();
  }      

  RPMandCountFunctionBot();
  RPMandCountFunctionTop();
  
  BotMotorPIDs();
  totalPIDBot = botMotorPIDOutput + yawPIDOutputBot;
  
  /* Bottom motor anti-clockwise */
  botMotor.writeMicroseconds(totalPIDBot);

  TopMotorPIDs();
  totalPIDTop = topMotorPIDOutput + yawPIDOutputTop;
  
  /* Top motor clockwise */
  topMotor.writeMicroseconds(totalPIDTop);
    
  /* Trigger code, boolean variable controlled by the top motor */
  if(triggerBool){    
    TriggerFunction();
    triggerBool = false;    
  }  
  
  //DumpRegisterValues(); 
  DebuggingPrints();
}



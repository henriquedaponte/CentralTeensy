#include <Arduino.h>
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MLX90640.h>
#include <math.h>
#include <TeensyThreads.h>

#define INC_ADDRESS 0x68
#define ACC_CONF  0x20  //Page 91
#define GYR_CONF  0x21  //Page 93
#define CMD       0x7E  //Page 65

//  =================== CAN Variables =====================================================
const int ledPin = 13;
const int hallEffectPin = 41;
const double TIMER_MICROSECONDS = 150000;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;

// =================== BMI323 Variables ===================================================
uint16_t gyr_x, gyr_y, gyr_z;
int16_t signed_gyr_x, signed_gyr_y, signed_gyr_z;

// =================== Hall Effect Variables ==============================================
IntervalTimer halleffectTimer;
volatile  double last_rpm = 0;
volatile int numOfRotations = 0;

// =================== BMI323 Helper Function Definitions ==============================================
// Write data in 16 bits
void writeRegister16(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  //Low 
  Wire.write((uint16_t)value & 0xff);
  //High
  Wire.write((uint16_t)value >> 8);
  Wire.endTransmission();
}

// Read data in 16 bits
uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  int n = Wire.requestFrom(INC_ADDRESS, 4);  
  uint16_t data[20];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }  
  return (data[3]|data[2] << 8);
}

// Read all axis
void readAllAccel() {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 20);
  uint16_t data[20];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }

  // Offset = 2 because the 2 first bytes are dummy (useless)
  int offset = 2;
  gyr_x = (data[offset + 6]   | (uint16_t)data[offset + 7] << 8);  //0x06
  gyr_y = (data[offset + 8]   | (uint16_t)data[offset + 9] << 8);  //0x07
  gyr_z = (data[offset + 10]  | (uint16_t)data[offset + 11] << 8); //0x08
}

void softReset(){  
  writeRegister16(CMD, 0xDEAF);
  delay(50);    
}

int16_t twosComplementToNormal(uint16_t raw) {
    if (raw & (1 << 15)) {
        return -((~raw + 1) & 0xFFFF);
    } else {
        return raw;
    }
}

void thread_func() {
  while(true) {
    uint8_t halleffectOuput = digitalRead(hallEffectPin);
    if (halleffectOuput == 0) {
      numOfRotations ++;
    }
  }
}


// =================== Setup Functions ==================================================
uint16_t readRegister16(uint8_t reg);
void readAllAccel(); 
int16_t twosComplementToNormal(uint16_t raw); 

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(hallEffectPin, INPUT);
  threads.addThread(thread_func, 1);
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  Wire1.begin();
  Wire1.setClock(400000); //Increase I2C clock speed to 1MHz
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  softReset();
  writeRegister16(ACC_CONF, 0x708B); 
  writeRegister16(GYR_CONF, 0x708B);
}

void loop() {
// =================== CAN Message Setup =================================================
  //msg.id = 0x100; // CAN message ID
  //msg.len = 8;    // Message length (up to 8 bytes)
  Serial.println("\n Loop Running... \n");

// =================== BMI323 Logic ======================================================
  readRegister16(0x02);
  if(readRegister16(0x02) == 0x00) {
    //Read ChipID
    Serial.print("ChipID:");
    Serial.print(readRegister16(0x00));    
    readAllAccel();             
    signed_gyr_x = twosComplementToNormal(gyr_x)/262.1;
    signed_gyr_y = twosComplementToNormal(gyr_y)/262.1;
    signed_gyr_z = twosComplementToNormal(gyr_z)/262.1;
    Serial.print(" \tgyr_x:");
    Serial.print(signed_gyr_x);
    Serial.print(" \tgyr_y:");
    Serial.print(signed_gyr_y);
    Serial.print(" \tgyr_z:");
    Serial.println(signed_gyr_z);   
  }

  // delay(500);
  
// Attempt to read a message and print if successful
  bool readResult = CANbus.read(msg);
  Serial.print("Reading Message: ");
  Serial.println(readResult);

  if (readResult) {
    if(msg.id == 0x100) {
      digitalWrite(ledPin, HIGH);
      Serial.print("Received message with ID: ");
      Serial.println(msg.id, HEX);
      Serial.print("Message contents: ");
      Serial.println();
      float temp_object = ((msg.buf[0] << 8) | msg.buf[1]) / 100.;
      Serial.print("Temp: ");
      Serial.println(temp_object);

      float temp_ambient = ((msg.buf[2] << 8) | msg.buf[3]) / 100.;
      Serial.print("Ambient: ");
      Serial.println(temp_ambient);
      digitalWrite(ledPin, LOW);
    }
    if(msg.id == 0x101) {
      digitalWrite(ledPin, HIGH);
      Serial.print("Received message with ID: ");
      Serial.println(msg.id, HEX);
      Serial.print("Message contents: ");
      Serial.println();
      float temp_object = ((msg.buf[0] << 8) | msg.buf[1]) / 100.;
      Serial.print("Temp: ");
      Serial.println(temp_object);

      float temp_ambient = ((msg.buf[2] << 8) | msg.buf[3]) / 100.;
      Serial.print("Ambient: ");
      Serial.println(temp_ambient);
      digitalWrite(ledPin, LOW);
    }
    if(msg.id == 0x102) {
      digitalWrite(ledPin, HIGH);
      Serial.print("Received message with ID: ");
      Serial.println(msg.id, HEX);
      Serial.print("Message contents: ");
      Serial.println();
      float temp_object = ((msg.buf[0] << 8) | msg.buf[1]) / 100.;
      Serial.print("Temp: ");
      Serial.println(temp_object);

      float temp_ambient = ((msg.buf[2] << 8) | msg.buf[3]) / 100.;
      Serial.print("Ambient: ");
      Serial.println(temp_ambient);
      digitalWrite(ledPin, LOW);
    }
    if(msg.id == 0x103) {
      digitalWrite(ledPin, HIGH);
      Serial.print("Received message with ID: ");
      Serial.println(msg.id, HEX);
      Serial.print("Message contents: ");
      Serial.println();
      float temp_object = ((msg.buf[0] << 8) | msg.buf[1]) / 100.;
      Serial.print("Temp: ");
      Serial.println(temp_object);

      float temp_ambient = ((msg.buf[2] << 8) | msg.buf[3]) / 100.;
      Serial.print("Ambient: ");
      Serial.println(temp_ambient);
      digitalWrite(ledPin, LOW);
    }
  }
}
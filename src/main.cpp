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

const int ledPin = 13;
const int hallEffectPin = 41;
const double TIMER_MICROSECONDS = 150000;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MLX90640 mlx2;
uint16_t gyr_x, gyr_y, gyr_z;
int16_t signed_gyr_x, signed_gyr_y, signed_gyr_z;
IntervalTimer halleffectTimer;
volatile  double last_rpm = 0;
volatile int numOfRotations = 0;

 // =================== Function Declarations ======================================================

uint16_t readRegister16(uint8_t reg);
void readAllAccel(); // Read all axis
int16_t twosComplementToNormal(uint16_t raw); // Convert Raw Data in 2's Complement to Normal


void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:

 // =================== CAN Message Setup =================================================

  msg.id = 0x124; // CAN message ID
  msg.len = 8;    // Message length (up to 8 bytes)
  Serial.println("\n Loop Running... \n");

  // =================== BMI323 Logic ======================================================
  readRegister16(0x02);
  if(readRegister16(0x02) == 0x00) {
    //Read ChipID
    Serial.print("ChipID:");
    Serial.print(readRegister16(0x00));    
    readAllAccel();             // read all accelerometer/gyroscope/temperature data  
    printf("gyr_x: %d\n", gyr_x);
    printf("gyr_y: %d\n", gyr_y);
    printf("gyr_z: %d\n", gyr_z);
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

  // =======================================================================================

  //====================== MLX90640 Logic ==================================================
  float temp640[32*24];
  mlx2.getFrame(temp640);
  float sum = 0;
  for (uint8_t h=0; h<24; h++) {
    for (uint8_t w=0; w<32; w++) {
      float t = temp640[h*32 + w] - 273.0;
      Serial.print(t); Serial.print(" ");
      sum += t;
    }
    printf("\n");
  }
  
  float average = sum / (32*24);
  Serial.print("Average: "); 
  Serial.println(average);

  // delay(500);
  //=========================================================================================

  float tempO = mlx.readObjectTempC();
  float tempA = mlx.readAmbientTempC();
  
  int sendObjVal = tempO * 100;
  msg.buf[0] = sendObjVal >> 8;
  msg.buf[1] = sendObjVal & 0xFF;

  int sendAmbVal = tempA * 100;
  msg.buf[2] = sendAmbVal >> 8;
  msg.buf[3] = sendAmbVal & 0xFF;

  bool writeResult = CANbus.write(msg);  // Send the message
  Serial.print("Writing Message: ");
  Serial.println(writeResult);  // Print the result of the write operation

  delay(500);  // Wait for a while before trying to read

  // Attempt to read a message and print if successful
  bool readResult = CANbus.read(msg);
  Serial.print("Reading Message: ");
  Serial.println(readResult);

  if (readResult) {
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




 // =================== Function Definitions ======================================================

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
}

int16_t twosComplementToNormal(uint16_t raw) {
    if (raw & (1 << 15)) {
        return -((~raw + 1) & 0xFFFF);
    } else {
        return raw;
    }
}

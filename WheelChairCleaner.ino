/* Arduino code for wheel chair cleaner
 * hardware & software design : willysung
 * hardware element : GY521 , bluetooth , BLDC motor ,
 *                    SD card , DSM501A dust sensor
 * since 2016.04.01                   
 */

/*------------SD卡模組-------------*/
//** MOSI : pin51
//** MISO : pin50
//** CLK : pin52
//** CS : pin53
/*------------SD卡模組-------------*/

/*-------------灰塵模組---------------*/
// Connect the Pin_3 of DSM501A to Arduino 5V
// Connect the Pin_5 of DSM501A to Arduino GND
// Connect the Pin_2 of DSM501A to Arduino D8、D9
/*-------------灰塵模組---------------*/

#include <SPI.h>
#include <SD.h>
#include <MPU6050.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <String.h>
#include "Timer.h"

/*--------------GY521-----------------*/
MPU6050 mpu;
MPU6050 mpu2(0x69);
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;
/*--------------GY521-----------------*/

/*--------------DSM501A-----------------*/
int dust_pin1 = 8;
int dust_pin2 = 9;
unsigned long duration1;
unsigned long duration2;
unsigned long starttime;
unsigned long sampletime_ms = 3000;   //sample time 3s ;
unsigned long lowpulseoccupancy1 = 0;
unsigned long lowpulseoccupancy2 = 0;
float ratio1 = 0;
float ratio2 = 0;
float concentration1 = 0;
float concentration2 = 0;
int gLed = 32;
int yLed = 31;
/*--------------DSM501A-----------------*/

File myFile;
Timer t;
char incomingByte ;  //bluetooth received byte
const int WheelDrive_1 = 11; //motor driver pin M
const int WheelDrive_2 = 10; 
int rLed = 30;
boolean acc_detect = false;    //detect if the data of GY521 has over the threshold
boolean ifready = false;       
boolean ifclean = false;
volatile int GrabValue = 1;    //a state to see if wheel chair is ready

void GY521_Init(){
    Serial.println("Initialize MPU");
    mpu.initialize();
    mpu2.initialize();
    Serial.println(mpu.testConnection() ? "mpu1 Connected":"mpu1 Connection failed");
    Serial.println(mpu2.testConnection() ? "mpu2 Connected":"mpu2 Connection failed");
    mpu.setXAccelOffset(2830);     //calibrate the GY521's offset
    mpu.setYAccelOffset(-3605);
    mpu.setZAccelOffset(1602);
    mpu.setXGyroOffset(76);
    mpu.setYGyroOffset(19);
    mpu.setZGyroOffset(58);
    
    mpu2.setXAccelOffset(400);
    mpu2.setYAccelOffset(-5633);
    mpu2.setZAccelOffset(1800);
    mpu2.setXGyroOffset(91);
    mpu2.setYGyroOffset(-31);
    mpu2.setZGyroOffset(9);

    pinMode(rLed,OUTPUT);
}

void SD_Init(){
    pinMode(53,OUTPUT);
    if (!SD.begin(53)) {
      Serial.println("SDcard initialization failed!");
      return;
    }
    Serial.println("SDcard initialization done.");
}

void Motor_Init(){
    pinMode(WheelDrive_1, OUTPUT); 
    analogWrite(WheelDrive_1, 0);
    pinMode(WheelDrive_2, OUTPUT); 
    analogWrite(WheelDrive_2, 0);
}
void Dust_Init(){
    pinMode(dust_pin1,INPUT);
    pinMode(dust_pin2,INPUT);
    pinMode(gLed,OUTPUT);
    digitalWrite(gLed,HIGH);  //turn green LED on in the beginning
    pinMode(yLed,OUTPUT);
    starttime = millis();    //get the current time;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    SD_Init();
    GY521_Init();
    Motor_Init();
    Dust_Init();
}

void loop(){
  t.update();
  GetPosition();
  //dust_detect();
  delay(100);
}

void serialEvent() {
    incomingByte = Serial.read();
    if(incomingByte == 'a'){
      if(ifready){
        Serial.write("yyyyyy");      //send many words to help APP fetch the value
      }
      else {
        Serial.write("nnnnnn");
      }
    }
    if(incomingByte == 'c'){
      if(ifclean){
        Serial.write('y');
      }else Serial.write('n');
    }
    if(incomingByte == '1'){
        Serial.println("Start");
        analogWrite(WheelDrive_1, 255);
        analogWrite(WheelDrive_2, 255);
        t.after(5000,motor_stop);
        if(incomingByte == '0'){
            analogWrite(WheelDrive_1, 0);
            analogWrite(WheelDrive_2, 0);
        }
     }    

    if(incomingByte == '0'){
        analogWrite(WheelDrive_1, 0);
        analogWrite(WheelDrive_2, 0);
    }
}

void motor_stop(){
  analogWrite(WheelDrive_1, 0);
  analogWrite(WheelDrive_2, 0);
}
void GetPosition(){   
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    
    /*Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");*/
    //Serial.print(gz); Serial.print("\t");
    /*Serial.print(ax2); Serial.print("\t");
    Serial.print(ay2); Serial.print("\t");
    Serial.print(az2); Serial.print("\t");
    Serial.print(gx2); Serial.print("\t");
    Serial.print(gy2); Serial.print("\t");*/
    //Serial.print(gz2); Serial.println("\t");

    //writeFile("gz1.txt", gz);
    //writeFile("gz2.txt", gz2);
    delay(1000);

    if(gz2 < -500){
       digitalWrite(rLed,HIGH);
       t.pulse(rLed,5000,LOW);
       if(GrabValue == 1){
          ifready = true;
          GrabValue = 2;
       }
    }
    else{
       digitalWrite(rLed,LOW);
    }
}

void dust_detect(){
    duration1 = pulseIn(dust_pin1, LOW);
    duration2 = pulseIn(dust_pin2, LOW);
    lowpulseoccupancy1 = lowpulseoccupancy1 + duration1;
    lowpulseoccupancy2 = lowpulseoccupancy2 + duration2;

    if ((millis()-starttime) > sampletime_ms){//if the sampel time == 30s
       ratio1 = lowpulseoccupancy1/(sampletime_ms*10.0);                          // Integer percentage 0=>100
       ratio2 = lowpulseoccupancy2/(sampletime_ms*10.0);
       concentration1 = 1.1*pow(ratio1,3)-3.8*pow(ratio1,2)+520*ratio1+0.62;      // using spec sheet curve
       concentration2 = 1.1*pow(ratio2,3)-3.8*pow(ratio2,2)+520*ratio2+0.62;
       /*Serial.print("concentration = ");
       Serial.print(concentration1);
       Serial.print("   ");
       Serial.print(concentration2);
       Serial.print(" pcs/0.01cf  -  ");*/
    if (concentration1 && concentration2 < 25000) {
       digitalWrite(gLed, HIGH);
       digitalWrite(yLed, LOW);
       ifclean = true;
    }
    else {
       digitalWrite(gLed, LOW);
       digitalWrite(yLed, HIGH);
       ifclean = false;
    }   
    //writeFile("dust1.txt",concentration1 );
    //writeFile("dust2.txt",concentration2 );
    lowpulseoccupancy1 = 0;
    lowpulseoccupancy2 = 0;
    starttime = millis();
  }
}

void writeFile( char* filename, float content) {
    myFile = SD.open(filename, FILE_WRITE);
    if (myFile) {
      myFile.println(content);
      myFile.close();
    } 
}




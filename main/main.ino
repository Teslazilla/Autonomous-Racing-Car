/**********************
*                    Company: Teslazilla                      *
* Members: Axel Castrejon, Momchil Zhechev, Yoana Ivanova,    *
*          Adelina Muchanga and Dimitar Tarakchive            *
* University: Saxion University of Applied Sciences           * 
* Subject: Project Systen                                     *
* Project: Autonomous RC car                                  *
*********************/


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                 //
//  Credits:                                                                                                       //
//  Libraries and test codes:                                                                                      //
//                                                                                                                 //
//  "Adafruit SSD1306" : By Adafruit; For OLED display; not implemented into the main code                         // 
//  "Bifrost library for HC-SR04" : By Jeremy Lindsay; For HC-SR04 ultrasonic distance sensor                      // 
//  "ESP32Servo" : By Kevin Harrington; Allows ESP32 boards to control servo; not implemented into the main code   //
//  "MPU6050_light" : By rfetick; For light and fast comunication with the MPU6050 (gyroscope)                     //
//  "NewPing" : By Tim Eckel; A library that makes working with ultrasonic sensors easy                            //
//                                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <Servo.h>   // Library for Servo motor
#include <NewPing.h> //a library which would help in having a code avoiding delays
#include "Wire.h"
#include <MPU6050_light.h>

//Motor states
#define BRAKE 0
#define ROTATE_FORWARD    3 //1
#define ROTATE_BACKWARD   2 //2
#define CS_THRESHOLD 15   

//Infrared sensor pins      
const int ir_sensor_pin1 = 11;
const int ir_sensor_pin3 = 12;
const int ir_sensor_pin5 = 13;

//Angles for the wheels (servo motor)
const int angle0 = 70;
const int angle1 = 90;
const int angle2 = 135;

//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

#define PWM_MOTOR_1 5

#define CURRENT_SEN_1 A2

#define EN_PIN_1 A0
#define EN_PIN_2 A1

#define MOTOR_1 0

short defaultSpeed = 27;
short fastSpeed = 90;
unsigned short motor_Status = BRAKE;

#include <Arduino.h>

Servo angleWheels;

bool wentUp = false;

MPU6050 mpu(Wire);
unsigned long timerMPU = 0;

void setup()                         
{
  Serial.begin(9600); // Serial monitor at baud rate 9600
  
  angleWheels.attach(6);
  
  pinMode(ir_sensor_pin1, INPUT); // Pin set as input
  pinMode(ir_sensor_pin3, INPUT);
  pinMode(ir_sensor_pin5, INPUT);
  
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);

  pinMode(CURRENT_SEN_1, OUTPUT);
 
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

//  //Ultrasonic Sensor
//  botPingTimer = millis() + pingSpeed; // Sensor 1 fires after 1 second (pingSpeed)
//  topPingTimer = botPingTimer + 35; // Sensor 2 fires 35ms later  
  
  //Gyro setup
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() 
{
    digitalWrite(EN_PIN_1, HIGH);
    digitalWrite(EN_PIN_2, HIGH);
     
     
    Forward();
    Servo_Ir();
    //Ultrasonic_Sensor();
    Gyro();
}

void Stop()
{
  Serial.println("Stop");
  motor_Status = BRAKE;
  defaultSpeed= 0;
  motorGo(MOTOR_1, motor_Status, defaultSpeed);
}

void Reverse()
{
  Serial.println("Reverse");
  motor_Status = ROTATE_BACKWARD;
  motorGo(MOTOR_1, motor_Status, defaultSpeed);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if(motor == MOTOR_1)
  {
    if(direct == ROTATE_BACKWARD)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == ROTATE_FORWARD)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm); 
  }

}

void Forward()
{
  Serial.println("Forward");
  motor_Status = ROTATE_FORWARD;
  motorGo(MOTOR_1, motor_Status, defaultSpeed);
}

void Servo_Ir() {
  if(digitalRead(ir_sensor_pin3)&&digitalRead(ir_sensor_pin5)&&digitalRead(ir_sensor_pin1)){
    Serial.println("Stopping");
    Stop();
    while(true); //This will make the program run this infinitly when the car reaches the end, so it won't try to move again, until it is restarted.
    return;
  }
  
 if (digitalRead(ir_sensor_pin3)) { // if Pin logic is HIGH  
    angleWheels.write(angle1);
  } else if (digitalRead(ir_sensor_pin1)) { // if Pin logic is HIGH  
    angleWheels.write(angle0);
  } else if (digitalRead(ir_sensor_pin5)) {
   angleWheels.write(angle2);
  }

}

void Gyro() {
  mpu.update();
  
  if((millis()-timerMPU)>10){ // run every 10ms

    if (mpu.getAngleY() < -7) { // Detect the car going up
        defaultSpeed = fastSpeed;
        wentUp = true;
        Serial.println("Going faster");  
    } else if (mpu.getAngleY() > -2) {// Detect the car going back to flat
        if(wentUp) {
          Serial.println("Slowing down");
          defaultSpeed = 15;
          Reverse();
          delay(50);  
        } else {
          defaultSpeed = 27;
          Serial.println("Going back to normal speed");
        }  
    }
    timerMPU = millis();  
  }
}

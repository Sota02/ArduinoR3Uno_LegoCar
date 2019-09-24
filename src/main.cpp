#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h> 

//bluetooth
const byte btRxPin = 10;
const byte btTxPin = 11;
SoftwareSerial btport(btRxPin, btTxPin);
char receivedChar; 
//servo motor
Servo servo1;
const uint8_t servoPin = 9;
int initialAngle;
const int limitAngle = 25;
const int steerAngle = 8; 
//dc motor
const uint8_t L293D_ENable1Pin = 3;
const uint8_t L293D_1APin = 5;
const uint8_t L293D_2APin = 6;
uint8_t ifGoingForward = 0; //0 is stop, 1 is go forward
uint8_t ifGoingBackward = 0; //0 is stop, 1 is go backward
int drivingDirection = 0; //0=stop,1=forward,-1=backward
int motorSpeed; //current motorspeed. any value between 0-255
const uint8_t initialMotorSpeed = 120; //initial speed when accel button is pressed
//LED
const uint8_t LEDPin = 13;

void setup() {  
    //serial communication to config HC-05
    Serial.begin(9600);
    Serial.println("You can enter AT commands:");
    //bluetooth over SoftwareSerial
    btport.begin(38400); //38400 = default bluetooth communication rate 
    //servo motor
    pinMode(servoPin,OUTPUT); 
    servo1.attach(servoPin);
    Serial.print("Servo angle inital value:");
    Serial.println(servo1.read());  
    initialAngle = servo1.read();
    //dc motor
    pinMode(L293D_ENable1Pin,OUTPUT);
    pinMode(L293D_1APin,OUTPUT);
    pinMode(L293D_2APin,OUTPUT);
    digitalWrite(L293D_ENable1Pin,LOW);
    motorSpeed = initialMotorSpeed;
    //LED
    pinMode(LEDPin,OUTPUT); 
    digitalWrite(LEDPin,HIGH);
    
}

void steerWheel(char x){
  if(x=='l'){
    if(servo1.read()-steerAngle >= initialAngle-limitAngle){
      servo1.write(servo1.read()-steerAngle);
    } 
  }else if(x=='r'){
    if(servo1.read()+steerAngle <= initialAngle+limitAngle){
      servo1.write(servo1.read()+steerAngle);
    }
  }
  Serial.print("angleServo:");    
  Serial.println(servo1.read());
}

void stepOnGasPedal(char x){
  if(x == 'f'){
    if(drivingDirection!=1){
      drivingDirection = 1;
      motorSpeed = initialMotorSpeed;
    }else{
      if(motorSpeed<= 255){
        motorSpeed+=20;
      }
    }
  }else if(x == 'b'){
    if(drivingDirection != -1){
      drivingDirection = -1;
      motorSpeed = initialMotorSpeed;
    }else{
      if(motorSpeed<= 255){
        motorSpeed+=20;
      }
    }
  }
}

void loop()
{
    //receive char via bluetooth app. Then execute a action.
    if (btport.available()){
        receivedChar = btport.read();
        Serial.println(receivedChar);

        if(receivedChar == 'l'){
          steerWheel('l');
        }else if(receivedChar == 'r'){
          steerWheel('r');
        }else if(receivedChar == 'f'){
          stepOnGasPedal('f');
        }else if(receivedChar == 'b'){
          stepOnGasPedal('b');
        }else if(receivedChar == 's'){
            digitalWrite(L293D_ENable1Pin,LOW);
            motorSpeed = 0;
            drivingDirection = 0;
        }else if(receivedChar == 'p'){
          digitalWrite(LEDPin,HIGH);
        }else if(receivedChar == 'q'){
          digitalWrite(LEDPin,LOW);
        }
    }

    //dc motor
    if(drivingDirection==1){
      analogWrite(L293D_ENable1Pin,motorSpeed);
      digitalWrite(L293D_1APin, HIGH);
      digitalWrite(L293D_2APin, LOW);
    }else if(drivingDirection==-1){
      analogWrite(L293D_ENable1Pin,motorSpeed);
      digitalWrite(L293D_1APin, LOW);
      digitalWrite(L293D_2APin, HIGH);
    }
        
    //read input from Serial -> output to bluetooth
    if (Serial.available()){
       btport.write(Serial.read());
    }
    
}
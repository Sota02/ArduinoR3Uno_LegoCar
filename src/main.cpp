#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h> 

//bluetooth
const byte btRxPin = 10;
const byte btTxPin = 11;
SoftwareSerial btport(btRxPin, btTxPin);
char receivedChar; 
//servo motor
const uint8_t servoPin = 9;
Servo servo1;
//dc motor
const uint8_t L293D_ENable1 = 3;
const uint8_t L293D_1A = 5;
const uint8_t L293D_2A = 6;
uint8_t ifGoingForward;
uint8_t ifGoingBackward;
int motorSpeed; //any value between 0-255
const uint8_t initialMotorSpeed = 100;
//LED
const uint8_t LEDPin = 13;

void setup() {
    //Serial
    Serial.begin(9600);
    Serial.println("Enter AT commands:");
    //bluetooth over SoftwareSerial
    btport.begin(38400); //38400 = default bluetooth communication rate 
    //servo motor
    pinMode(servoPin,OUTPUT); 
    servo1.attach(servoPin);
    //dc motor
    pinMode(L293D_ENable1,OUTPUT);
    pinMode(L293D_1A,OUTPUT);
    pinMode(L293D_2A,OUTPUT);
    digitalWrite(L293D_ENable1,LOW);
    ifGoingForward = 0;
    ifGoingBackward = 0;
    motorSpeed = initialMotorSpeed;
    //LED
    pinMode(LEDPin,OUTPUT); 
    digitalWrite(LEDPin,HIGH);
    
}
 
void loop()
{
    //read char from bluetooth
    if (btport.available()){
        receivedChar = btport.read();
        Serial.write(receivedChar);

        if(receivedChar == 'q'){
          digitalWrite(LEDPin,LOW);
        }else if(receivedChar == 'p'){
          digitalWrite(LEDPin,HIGH);
        }else if(receivedChar == 'l'){
          servo1.write(servo1.read()-10);   //steer to left    
        }else if(receivedChar == 'r'){
          servo1.write(servo1.read()+10);   //steer to right 
        }else if(receivedChar == 'n'){
          servo1.write(90);    //steer neutral... temporary not used
        }else if(receivedChar == 'f'){
          if(ifGoingForward==0){
            ifGoingForward=1;
            ifGoingBackward=0;
          }
        }else if(receivedChar == 'b'){
          if(ifGoingBackward==0){
            ifGoingBackward=1;
            ifGoingForward=0;
          }
        }else if(receivedChar == 's'){
            digitalWrite(L293D_ENable1,LOW);
            ifGoingForward=0;
            ifGoingBackward=0;
            motorSpeed = initialMotorSpeed;
        }else if(receivedChar == 'u'){
          if(motorSpeed<= 255){
            motorSpeed+=10;
          }
        }else if(receivedChar == 'd'){
          if(motorSpeed>= 0){
            motorSpeed-=10;
          }
        }
    }

    //dc motor
    if(ifGoingForward){
      analogWrite(L293D_ENable1,motorSpeed);
      digitalWrite(L293D_1A, HIGH);
      digitalWrite(L293D_2A, LOW);
      
    }else if(ifGoingBackward){
      analogWrite(L293D_ENable1,motorSpeed);
      digitalWrite(L293D_1A, LOW);
      digitalWrite(L293D_2A, HIGH);
      
    }
        
    //read input from Serial -> output to bluetooth
    if (Serial.available()){
       btport.write(Serial.read());
    }
    
}
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h> 

#define echoPin 2 // Echo Pin
#define trigPin 4 // Trigger Pin
double Duration = 0; //受信した間隔
double Distance = 0; //距離
int distanceCounter = 0;

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
uint8_t ifGoingBackward = 0; //0=stop, 1=go backward
int drivingDirection = 0; //0=stop,1=forward,-1=backward
int motorSpeed; //current motorspeed. value is between 0-255
const uint8_t initialMotorSpeed = 120; //initial speed when accel button is pressed
//LED
const uint8_t LEDPin = 13;
uint8_t LEDstatus = 0; //0=off,1=on

//Methods
void steerWheel(char c);
void stepOnGasPedal(char c);
void stepOnGasPedal(char c);
void stopMotor();
void toggleLED();
void rotateDCMotor();
void analyzeDistance();

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
    //distance censor
    pinMode( echoPin, INPUT );
    pinMode( trigPin, OUTPUT );
    //LED
    pinMode(LEDPin,OUTPUT); 
    digitalWrite(LEDPin,HIGH);
    LEDstatus = 1;
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
          stopMotor();
        }else if(receivedChar == 'p'){
          toggleLED();  
        }
    }

    //dc motor
    rotateDCMotor();

    //distance sensor
    analyzeDistance();
}

void steerWheel(char c){
  if(c=='l'){
    if(servo1.read()-steerAngle >= initialAngle-limitAngle){
      servo1.write(servo1.read()-steerAngle);
    } 
  }else if(c=='r'){
    if(servo1.read()+steerAngle <= initialAngle+limitAngle){
      servo1.write(servo1.read()+steerAngle);
    }
  }
  Serial.print("angleServo:");    
  Serial.println(servo1.read());
}

void stepOnGasPedal(char c){
  if(c == 'f'){
    if(drivingDirection!=1){
      drivingDirection = 1;
      motorSpeed = initialMotorSpeed;
    }else{
      if(motorSpeed<= 255){
        motorSpeed+=20;
      }
    }
  }else if(c == 'b'){
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

void stopMotor(){
  digitalWrite(L293D_ENable1Pin,LOW);
  motorSpeed = 0;
  drivingDirection = 0;
}

void rotateDCMotor(){
  if(drivingDirection==1){
      analogWrite(L293D_ENable1Pin,motorSpeed);
      digitalWrite(L293D_1APin, HIGH);
      digitalWrite(L293D_2APin, LOW);
  }else if(drivingDirection==-1){
      analogWrite(L293D_ENable1Pin,motorSpeed);
      digitalWrite(L293D_1APin, LOW);
      digitalWrite(L293D_2APin, HIGH);
  }
}

void toggleLED(){
  if(LEDstatus==0){
    digitalWrite(LEDPin,HIGH);
    LEDstatus=1;
  }else{
    digitalWrite(LEDPin,LOW);
    LEDstatus=0;
  }
}

void analyzeDistance(){
  if(distanceCounter%80000==0) {
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite( trigPin, HIGH ); //超音波を出力
    delayMicroseconds( 10 ); //
    digitalWrite( trigPin, LOW );
    Duration = pulseIn( echoPin, HIGH ); //センサからの入力
    if (Duration > 0) {
      Duration = Duration/2; //往復距離を半分にする
      Distance = Duration*340*100/1000000; // 音速を340m/sに設定
      Serial.print("Distance:");
      Serial.print(Distance);
      Serial.println(" cm");
    }
  }
  distanceCounter+=1;
}
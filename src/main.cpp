#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h> 

//bluetooth HC-05
#define btRxPin 10
#define btTxPin 11
SoftwareSerial btport(btRxPin, btTxPin);
char receivedChar;
//servo motor
#define servoPin 9
Servo servo1;
int initialAngle;
const int limitAngle = 40;
const int steerAngle = 10; 
//dc motor
#define L293D_ENable1Pin 3
#define L293D_1APin 5
#define L293D_2APin 6
int drivingDirection = 0; //0=stop,1=forward,-1=backward
int motorSpeed; //current motorspeed. value is between 0-255
const uint8_t initialMotorSpeed = 195; //initial speed when accel button is pressed
const uint8_t accelerationTempo = 20;
//ultra sonic distance sensor HC-SR04
#define echoPin 2
#define trigPin 4
double Duration = 0; 
double Distance = 0;
int distanceCounter = 0;
//LED
#define LEDPin 13
uint8_t LEDstatus = 0; //0=off,1=on

//Methods
void showReceivedChar();
void steerWheel(char c);
void stepOnGasPedal(char c);
void stopMotor();
void toggleLED();
void rotateDCMotor();
void measureDistance();


void setup() {  
    //serial communication to configure HC-05
    Serial.begin(9600);
    Serial.println("Hello! Robot car booted.");
    //bluetooth over SoftwareSerial
    btport.begin(38400); //38400 = default bluetooth communication rate 
    //servo motor
    pinMode(servoPin,OUTPUT); 
    servo1.attach(servoPin);
    Serial.print("Inital servo angle: ");
    Serial.println(servo1.read());  
    initialAngle = servo1.read();
    //dc motor
    pinMode(L293D_ENable1Pin,OUTPUT);
    pinMode(L293D_1APin,OUTPUT);
    pinMode(L293D_2APin,OUTPUT);
    digitalWrite(L293D_ENable1Pin,LOW);
    motorSpeed = initialMotorSpeed;
    //ultrasonic distance censor
    pinMode( echoPin, INPUT );
    pinMode( trigPin, OUTPUT );
    //LED
    pinMode(LEDPin,OUTPUT); 
    digitalWrite(LEDPin,HIGH);
    LEDstatus = 1;
}

void loop()
{
    if (btport.available()){
        //show a char in serial console which received from bluetooth app
        showReceivedChar();
        
        //execute a action
        if(receivedChar == 'l'){
          Serial.println("steer left");
          steerWheel('l');
        }else if(receivedChar == 'r'){
          Serial.println("steer right");
          steerWheel('r');
        }else if(receivedChar == 'f'){
          Serial.println("go forward");
          stepOnGasPedal('f');
        }else if(receivedChar == 'b'){
          Serial.println("go backward");
          stepOnGasPedal('b');
        }else if(receivedChar == 's'){
          Serial.println("stop the car");
          stopMotor();
        }else if(receivedChar == 'p'){
          Serial.println("put LED ON/OFF");
          toggleLED();  
        }
    }

    //dc motor
    rotateDCMotor();
    //distance sensor
    measureDistance();
}

void showReceivedChar(){
  receivedChar = btport.read();
  if(receivedChar!='0' && receivedChar!='o') {
    Serial.print("Received char via Bluetooth: ");
    Serial.println(receivedChar);
  }
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
      if(motorSpeed + accelerationTempo <= 255){
        motorSpeed+=accelerationTempo;
      }
    }
  }else if(c == 'b'){
    if(drivingDirection != -1){
      drivingDirection = -1;
      motorSpeed = initialMotorSpeed;
    }else{
      if(motorSpeed + accelerationTempo <= 255){
        motorSpeed+=accelerationTempo;
      }
    }
  }
}

void stopMotor(){
  digitalWrite(L293D_ENable1Pin,LOW);
  motorSpeed = 0;
  drivingDirection = 0;
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

void measureDistance(){
  if(distanceCounter%20000==0) {
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite( trigPin, HIGH ); //output ultrasonic sensor
    delayMicroseconds(10);
    digitalWrite( trigPin, LOW );
    Duration = pulseIn( echoPin, HIGH ); //input from sensor = round trip time
    if (Duration > 0) {
      //show distance in serial
      Duration = Duration/2;
      Distance = Duration*340*100/1000000; // define the speed of sonic as 340m/s
      Serial.print("Distance from a object: ");
      Serial.print(Distance);
      Serial.println(" cm");

      //stop the car if the distance is short
      if(Distance < 20 && drivingDirection == 1){
        stopMotor();
        Serial.println("Sensor detected a object too near. stop.");
      }
    }
  }
  distanceCounter+=1;
}
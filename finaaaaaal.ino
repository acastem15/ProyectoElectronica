// Arduino Obstacle Avoidance + Voice Control Robot
// Created by DIY Builder
//Contact me here https://www.instagram.com/diy.builder/
//You need to install the AFMotor and NewPing Libraries before uplodaing the sketch

#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#define TRIGGER_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 300
#define IR A5
#define TEMP A2
#define LUZ 13

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);



//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

Servo myservo;

String voice;

void setup() {
Serial.begin(9600);
myservo.attach(10);
myservo.write(90);
pinMode(IR, INPUT);
int v = 100;
motor1.setSpeed(v);
motor2.setSpeed(v);
motor3.setSpeed(v);
motor4.setSpeed(v);
pinMode(TRIGGER_PIN, OUTPUT); 
pinMode(ECHO_PIN, INPUT); 

}

void loop() {
  //int distance = sonar.ping_cm();
  //Serial.println(distance);
  int IR1 = digitalRead(IR); 

  //Serial.println(IR);
  long duracion; 
  long distancia; 

int luminosidad = digitalRead(LUZ);
Serial.print(luminosidad);

int entradilla =analogRead(TEMP);
float V = (entradilla*5)/1023.0;
float C = V*100;
Serial.print(C);
delay (1000);
 
if(Serial.available()>0) {

  voice="";
  voice = String(Serial.readString());
  voice.trim();

  

  

if (voice == "turn left") {
  left();
}else if (voice == "left") {
  left();
}else if(voice == "turn right") {
  right();
}else if(voice == "right") {
  right();
}
else if(voice == "stop") {
  Stop();
}
}
while(voice == "move forward") {
  //Serial.println("En el while de forward");
  forward();
}
while(voice == "move backward") {
  backward();
}

}



void forward() {

  long duracion; 
  long distancia; 

  digitalWrite(TRIGGER_PIN, LOW); 
  delayMicroseconds(4); 
  digitalWrite(TRIGGER_PIN, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIGGER_PIN, LOW); 

  duracion = pulseIn(ECHO_PIN, HIGH); 
  duracion = duracion/2; 
  distancia = duracion/29; 

 if(distancia <= 20){
  Stop();
  voice="";
 }else {
  //motor1.setSpeed(255); 
  motor1.run(FORWARD); 
  //motor2.setSpeed(255);
  motor2.run(FORWARD);
  //motor3.setSpeed(255); 
  motor3.run(FORWARD); 
  //motor4.setSpeed(255); 
  motor4.run(FORWARD); 
}
}
void backward() {
  int IR_Sensor = digitalRead(IR);

  if(IR_Sensor == 0) {
    Stop();
    voice="";
  }else {
  //motor1.setSpeed(255); 
  motor1.run(BACKWARD); 
  //motor2.setSpeed(255); 
  motor2.run(BACKWARD); 
  //motor3.setSpeed(255); 
  motor3.run(BACKWARD);
  //motor4.setSpeed(255); 
  motor4.run(BACKWARD); 
 
}
}
void left() {
  myservo.write(180);
  delay(500);
  myservo.write(90);
  delay(500);
  motor1.run(BACKWARD);
  //motor1.setSpeed(255);
  motor2.run(BACKWARD);
  //motor2.setSpeed(255);
  motor3.run(FORWARD);
  //motor3.setSpeed(255);
  motor4.run(FORWARD);
  //motor4.setSpeed(255);
  delay(700);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void right() {
  myservo.write(0);
  delay(500);
  myservo.write(90);
  delay(500);
  motor1.run(FORWARD);
  //motor1.setSpeed(255);
  motor2.run(FORWARD);
  //motor2.setSpeed(255);
  motor3.run(BACKWARD);
  //motor3.setSpeed(255);
  motor4.run(BACKWARD);
  //motor4.setSpeed(255);
  delay(700);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


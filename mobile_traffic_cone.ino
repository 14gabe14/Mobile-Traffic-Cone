//Librerias para el uso de la aplicacion blynk
#define BLYNK_USE_DIRECT_CONNECT
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(2, 3);

#define BLYNK_PRINT DebugSerial
#include <BlynkSimpleSerialBLE.h>

char auth[] = "77618d2367ca490e9ea6341dac7eca69";

BlynkTimer timer; //blynk timer

//Left motor
int IN1 = 2;
int IN2 = 3;
//Right motor
int IN3 = 4;
int IN4 = 5;

//pins are assigned to digital and analogical infrared sensors
//Sensor 1
int Sensor1D = 11;
int Sensor2D = 12;
//Sensor 2
int Sensor1A = A2;
int Sensor2A = A1;

//variables which will contain input values from infrared sensors
int S1D; //IR sensor digital input
int S2D;

int S1A; //IR sensor analogical input
int S2A;

int value; //value of Blynk button to activate the line following mechanism

//values of the buttons to manually control the movement of the car through the Blynk Apps
int a, t, i, d;

void myTimerEvent(){ //timer event
  //sensor reading
  S1D = digitalRead(Sensor1D);
  S2D = digitalRead(Sensor2D);
  //value determines if the line following mechanism will be executed
  if(value == 1){
    followLine(S1D, S2D);//Line following function
  }else if(value == 0){
    stop_movement();
  }
  if(a == 1){
    forward();
  }
  if(t==1){
    backward();
  }
  if(i==1){
    left();
  }
  if(d==1){
    right();
  }
}
BLYNK_WRITE(V1){// widget WRITEs to Virtual Pin V1
  value = param.asInt(); //'value' cambiara en funcion de la activacion del boton
}

void setup() {
  //Debug console
  DebugSerial.begin(9600);

  DebugSerial.println("Waiting for connections...");
  
  Serial.begin(9600);
  Blynk.begin(Serial, auth);

  timer.setInterval(100L, myTimerEvent);//Blynk timer, information is updated every 100 miliseconds within myTimerEvent()

  //motors and sensor are initialized
  pinMode(Sensor1D, INPUT);
  pinMode(Sensor2D, INPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {

  Blynk.run();
  timer.run();
}

void SItest(int S1A, int S1D, int S2A, int S2D){
  // extract data from the digital and analog input of the infrared sensor
  // was used to verify that the sensors give different inputs given a light and dark surface
  // so that the vehicle can distinguish a line

  Serial.print("S1: ");
  Serial.print(S1A);
  Serial.print(", ");
  Serial.println(S1D);
  Serial.print("S2: ");
  Serial.print(S2A);
  Serial.print(", ");
  Serial.println(S2D);  
}

void forward(){
  //left motor goes forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  //right motor goes forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void backward(){
  //right motor goes backwards
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //left motor goes backwards
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void left(){
  //right motor goes backwards
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  //left motor goes forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right(){
  //right motor goes forwards
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //left motor goes backwards
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop_movement(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void followLine(int S1d, int S2d){
  //funicion seguidor de linea a base de la activacion de los sensores infrarojos digitales
  if(S1d == LOW && S2d == HIGH){
    left();
  }else if(S1d == HIGH && S2d == LOW){
    right();
  }else if(S1d == HIGH && S2d == HIGH){
    forward();
  }else{
    stop_movement();
  }
}

BLYNK_WRITE(V4){ // widget WRITEs to Virtual Pin V4 (boton avanzar)
  a = param.asInt();
}
BLYNK_WRITE(V5){ // widget WRITEs to Virtual Pin V5 (boton retroceder)
  t = param.asInt();
}
BLYNK_WRITE(V6){ // widget WRITEs to Virtual Pin V6 (boton izquierda)
  i = param.asInt();
}
BLYNK_WRITE(V7){ // widget WRITEs to Virtual Pin V7 (boton derecha)
  d = param.asInt();
}
BLYNK_WRITE(V8){ boton modo entrenamiento
  ar = param.asInt();
}

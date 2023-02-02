#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0.05; 
float Ki = 0; 
float Kd = 0.3; 

int P;
int I;
int D;

int lastError = 0;
boolean onoff = false;

uint8_t vitesseCalibrage = 40;

const double rapport = 11/10;

const uint8_t maxspeeda = 150*rapport;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100*rapport;
const uint8_t basespeedb = 100;

#define aphase 13 // partie gauche
#define aenbl 11
#define freinMD 8

#define bphase 12 // partie droite
#define benbl 3
#define freinMG 9

const int ledMauvais = 44;
const int ledBon = 42;
const int ledTresBon = 45;
const int ledExellant = 43;
const int ledcalibrage = 10;

const int capteurGauche = 40;
const int capteurDroit = 41;

const int buttoncalibrate = 33;
const int buttonstart = 32;

int sens;

void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){46, 47, 48, 49, 50, 51, 52, 53}, SensorCount);
  qtr.setEmitterPin(7);//LEDON PIN

  pinMode(capteurGauche, INPUT);
  pinMode(capteurDroit, INPUT);

  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  
  pinMode(ledMauvais, OUTPUT);
  pinMode(ledBon, OUTPUT);
  pinMode(ledTresBon, OUTPUT);
  pinMode(ledExellant, OUTPUT);
  
  pinMode(ledcalibrage, OUTPUT);
  
  pinMode(buttoncalibrate, INPUT_PULLUP);
  pinMode(buttonstart, INPUT_PULLUP);
  
  delay(500);
  
  pinMode(5, OUTPUT);

  boolean Ok = false;
  while (Ok == false) {
    if(digitalRead(buttoncalibrate) == LOW) {
      digitalWrite(ledcalibrage, HIGH);
      calibration();
      Ok = true;
    }
  }
  digitalWrite(ledcalibrage, LOW);
  forward_brake(0, 0);
}

void loop() {
  if(digitalRead(buttonstart) == LOW) {
    onoff =! onoff;
    if(onoff = true) {
      delay(1000);
    }
    else {
      delay(50);
    }
  }
  if (onoff == true) {
    sens = 3500;
    PID_control(sens);
  }
  else {
    forward_brake(0,0);
  }
}

void forward_brake(int posa, int posb) {
  digitalWrite(aphase, LOW);
  digitalWrite(bphase, LOW);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

void calibration() {
  demiCercle(vitesseCalibrage, 1);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    if (digitalRead(capteurGauche) == true){
      demiCercle(vitesseCalibrage, 1);
      digitalWrite(ledExellant, HIGH);
    }
    else if (digitalRead(capteurDroit) == true){
      demiCercle(vitesseCalibrage, 0);
      digitalWrite(ledMauvais, HIGH);
    }
    else{
      digitalWrite(ledExellant, LOW);
      digitalWrite(ledMauvais, LOW);
    }
  }
}
void demiCercle(int vitesse, bool sens){
  if (sens == 0){
    digitalWrite(aphase, HIGH);
    digitalWrite(bphase, LOW);
    analogWrite(aenbl, vitesse*rapport);
    analogWrite(benbl, vitesse);
  }
  if (sens == 1){
    digitalWrite(aphase, LOW);
    digitalWrite(bphase, HIGH);
    analogWrite(aenbl, vitesse*rapport);
    analogWrite(benbl, vitesse);
  }
}

void PID_control(int consigne) {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = consigne - position;
  testPID(position);

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; 
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  forward_brake(motorspeeda, motorspeedb);
}

void testPID(uint16_t pos) {
  if (pos == 3500){
    digitalWrite(ledExellant, HIGH);
  }
  else {
    digitalWrite(ledExellant, LOW);
  }
  
  if (pos >= 3000 && pos <= 4000){
    digitalWrite(ledTresBon, HIGH);
  }
  else {
    digitalWrite(ledTresBon, LOW);
  }
  
  if (pos >= 2500 && pos <= 4500){
    digitalWrite(ledBon, HIGH);
  }
  else {
    digitalWrite(ledBon, LOW);
  }
  
  if (pos >= 2000 && pos <= 5000){
    digitalWrite(ledMauvais, HIGH);
  }
  else {
    digitalWrite(ledMauvais, LOW);
  }
}

#include "QTRSensorsMy.h"

QTRSensors qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

uint8_t vitesseCalibrage = 40;

const double rapport = 11/10;

#define aphase 13 // partie gauche
#define aenbl 11
#define freinMD 8

#define bphase 12 // partie droite
#define benbl 3
#define freinMG 9

const int ledMauvais = 44;
const int ledExellant = 43;
const int ledcalibrage = 10;

const int capteurGauche = 40;
const int capteurDroit = 41;

const int buttoncalibrate = 33;

void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){46, 47, 48, 49, 50, 51, 52, 53}, sensorCount);
  qtr.setEmitterPin(7);//LEDON PIN

  pinMode(capteurGauche, INPUT);
  pinMode(capteurDroit, INPUT);

  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  
  pinMode(ledMauvais, OUTPUT);
  pinMode(ledExellant, OUTPUT);
  
  pinMode(ledcalibrage, OUTPUT);
  
  pinMode(buttoncalibrate, INPUT_PULLUP);
  
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

  for(int i; i < sensorCount; i++){
    Serial.println(qtr.readCalibrateMinMy(i) + ", " + qtr.readCalibrateMaxMy(i));
  }
}

void loop(){
  
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

void forward_brake(int posa, int posb) {
  digitalWrite(aphase, LOW);
  digitalWrite(bphase, LOW);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

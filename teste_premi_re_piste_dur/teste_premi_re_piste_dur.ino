#include <QTRSensors.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0.06;
float Ki = 0.0008;
float Kd = 0.8;

int P;
int I;
int D;

int lastError = 0;
boolean onoff = false;

uint8_t vitesseCalibrage = 40;
uint8_t vitesseTurn = 80;

const double rapport = 11 / 10;

const uint8_t maxspeeda = 90 * rapport;
const uint8_t maxspeedb = 90;
const uint8_t basespeeda = 60 * rapport;
const uint8_t basespeedb = 60;

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
const int buttonresetcompteur = 30;

const int nombreInter = 1;
int variable = 0;
int variableI = 0;
int compteur = 0;

int virageGauche = 100;
int virageDroite = 4;

/*
int virageGaucheE = 0;
int virageDroiteE = 1;
*/
int sens;


void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    46, 47, 48, 49, 50, 51, 52, 53
  }, SensorCount);
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

  lcd.init();
  lcd.begin(16, 2);

  delay(500);

  pinMode(5, OUTPUT);

  boolean Ok = false;
  while (Ok == false) {
    if (digitalRead(buttoncalibrate) == LOW) {
      digitalWrite(ledcalibrage, HIGH);
      calibration();
      Ok = true;
    }
  }
  digitalWrite(ledcalibrage, LOW);
  digitalWrite(ledMauvais, LOW);
  digitalWrite(ledExellant, LOW);
  forward_brake(0, 0);
}

//bonne chance pour capter :)
void loop() {
  if (digitalRead(buttonstart) == LOW) {
    onoff = ! onoff;
    if (onoff = true) {
      delay(1000);
    }
    else {
      delay(50);
    }
  }
  if (digitalRead(buttonresetcompteur) == LOW) {
    compteur = 0;
    variableGauche = 0;
    variableDroite = 0;
    digitalWrite(ledMauvais, LOW);
    digitalWrite(ledExellant, LOW);
  }
  if (onoff == true) {
    else if (digitalRead(capteurDroit) == true || digitalRead(capteurGauche) == true) {
      if (variableDroite == virageDroite){
        
      } 
      else {
        sens = 3500;
      }
      if (variableDroiteI == 0) {
        variable++;
        variableI++;
        compteur++;
      }
    }
    else {
      sens = 3500;
    }
    PID_control(sens);
    if (digitalRead(capteurGauche) == false && digitalRead(capteurDroit) == false) {
      variableI = 0;
      variableI = 0;
      compteurEcran(variableDroite, variableGauche);
    }
  }
  else {
    forward_brake(0, 0);
  }
}

//Fonction pour avancer
void forward_brake(int posa, int posb) {
  digitalWrite(aphase, LOW);
  digitalWrite(bphase, LOW);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

//Calibration automatique qui part du milieu
void calibration() {
  demiCercle(vitesseCalibrage, 1);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    if (digitalRead(capteurGauche) == true) {
      demiCercle(vitesseCalibrage, 1);
      digitalWrite(ledExellant, HIGH);
    }
    else if (digitalRead(capteurDroit) == true) {
      demiCercle(vitesseCalibrage, 0);
      digitalWrite(ledMauvais, HIGH);
    }
    else {
      digitalWrite(ledExellant, LOW);
      digitalWrite(ledMauvais, LOW);
    }
  }
}

//fonction de rotation simple sur l'axe Y(vertical).
//les arguments sont la vitesse de déplacement et le sens (0 pour gauche et 1 pour droite).
//Il est possible que les sens soient inversés.
void demiCercle(int vitesse, bool sens) {
  if (sens == 0) {
    digitalWrite(aphase, HIGH);
    digitalWrite(bphase, LOW);
    analogWrite(aenbl, vitesse * rapport);
    analogWrite(benbl, vitesse);
  }
  if (sens == 1) {
    digitalWrite(aphase, LOW);
    digitalWrite(bphase, HIGH);
    analogWrite(aenbl, vitesse * rapport);
    analogWrite(benbl, vitesse);
  }
}

void compteurEcran(int x, int y) {
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.setCursor(4, 0);
  lcd.print(x);
  lcd.setCursor(0, 1);
  lcd.print("G:");
  lcd.setCursor(4, 1);
  lcd.print(y);
}

void PID_control(int consigne) {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = consigne - position;
  //testPID(position);

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;

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

void tournerDroiteD(){
  if(digitalRead(capteurDroit)){
    while(digitalRead(capteurDroit)){
      avancerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(vitesseTurn);
    }
  }
}

void tournerDroiteJ(){
  while(digitalRead(capteurGauche)){
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(0);
  }
  if(!digitalRead(capteurGauche)){
    while(!digitalRead(capteurGauche)){
      avancerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(0);
    }
  }
  while(digitalRead(capteurGauche){
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(0);
  }
}

void tournerDroiteE(){
  digitalWrite(ledTresBon, HIGH);
  forward_brake(0,0);
  if(digitalRead(capteurDroit) == false){
    while(digitalRead(capteurDroit) == false){
      reculerMoteurDroit(vitesseTurn);
      avancerMoteurGauche(0);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurGauche) == false){
    avancerMoteurDroit(0);
    avancerMoteurGauche(vitesseTurn);
  }
  forward_brake(0,0);
  if(digitalRead(capteurDroit) == false){
    while(digitalRead(capteurDroit) == false){
      reculerMoteurDroit(vitesseTurn);
      avancerMoteurGauche(0);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit) == true){
    reculerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(0);
  }
  forward_brake(0,0);
  if(digitalRead(capteurGauche) == true){
    while(digitalRead(capteurGauche) == true){
      avancerMoteurDroit(0);
      avancerMoteurGauche(vitesseTurn);
    }
  }
  digitalWrite(ledTresBon, LOW);
}
  
void avancerMoteurGauche(int vitesse) {
  digitalWrite(bphase, LOW);
  analogWrite(benbl, vitesse);
}

void avancerMoteurDroit(int vitesse) {
  digitalWrite(aphase, LOW);
  analogWrite(aenbl, vitesse);
}

void reculerMoteurGauche(int vitesse) {
  digitalWrite(bphase, HIGH);
  analogWrite(benbl, vitesse);
}

void reculerMoteurDroit(int vitesse) {
  digitalWrite(aphase, HIGH);
  analogWrite(aenbl, vitesse);
}

void tourner(int vitesse, bool sens) {
  if (sens == 0) {
    digitalWrite(aphase, HIGH);
    digitalWrite(bphase, LOW);
    analogWrite(aenbl, vitesse / 2);
    analogWrite(benbl, vitesse);
  }
  if (sens == 1) {
    digitalWrite(aphase, LOW);
    digitalWrite(bphase, HIGH);
    analogWrite(aenbl, vitesse * rapport);
    analogWrite(benbl, vitesse * rapport / 2);
  }
} 
/*void testPID(uint16_t pos) {
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
  }*/

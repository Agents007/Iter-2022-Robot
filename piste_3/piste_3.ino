#include <QTRSensors.h>
#include <LiquidCrystal_I2C.h>
#include <Vector.h>

using namespace std;

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
uint8_t vitesseTurn = 55;
uint8_t vitesseTurn2 = 40;

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

const int nbrVirage = 11;

int depart = 1;
int arrivee = 1;

switch (depart){
  case 1:
    const int nbrMagasin = 2;
    int Magasin[nbrMagasin] = {7, 2};
  break;
  case 2:
    const int nbrMagasin = 3;
    int Magasin[nbrMagasin] = {7, 4, 2};
  break;
  case 3:
    const int nbrMagasin = 4;
    int Magasin[nbrMagasin] = {7, 4, 4, 2};
  break;
  case 4:
    const int nbrMagasin = 5;
    Magasin[nbrMagasin] = {7, 4, 4, 4, 2};
  break;
  case 5:
    const int nbrMagasin = 4;
    int Magasin[nbrMagasin] = {8, 3, 3, 1};
  break;
  case 6:
    const int nbrMagasin = 3;
    int Magasin[nbrMagasin] = {8, 3, 1};
  break;
  case 7:
    const int nbrMagasin = 2;
    int Magasin[nbrMagasin] = {8, 1};
  break;
  default:
    danseLED();
  break;
}

switch (arrivee){
  case 1:
    const int nbrParking = 5;
    int Parking[nbrParking] = {2, 3, 3, 3, 3};
  break;
  case 2:
    const int nbrParking = 4;
    int Parking[nbrParking] = {2, 3, 3, 1};
  break;
  case 3:
    const int nbrParking = 3;
    int Parking[nbrParking] = {2, 3, 1};
  break;
  case 4:
    const int nbrParking = 2;
    int Parking[nbrParking] = {2, 1};
  break;
  case 5:
    const int nbrParking = 2;
    int Parking[nbrParking] = {3, 2};
  break;
  case 6:
    const int nbrParking = 5;
    int Parking[nbrParking] = {3, 3, 3, 3, 2};
  break;
  case 7:
    const int nbrParking = 6;
    int Parking[nbrParking] = {3, 3, 3, 3, 3, 2};
  break;
  case 8:
    const int nbrParking = 7;
    int Parking[nbrParking] = {3, 3, 3, 3, 3, 3, 2};
  break;
  case 9:
    const int nbrParking = 8;
    int Parking[nbrParking] = {3, 3, 3, 3, 3, 3, 3, 3};
  break;
  default:
    danseLED();
  break;
}

Vector<int> gps = fusionTab(Magasin, nbrMagasin, Parking, nbrParking);

int virage;

/*
1 = tourner Gauche E
2 = tourner Droite E

3 = tourner Gauche D
4 = tourner Droite D

5 = tourner Gauche J
6 = tourner Droite J

7 = tourner Gauche T
8 = tourner Droite T

9 = tourner Gauche L
10 = tourner Droite L
*/


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
  /*if (digitalRead(buttonresetcompteur) == LOW) {
    compteur = 0;
    variable = 0;
    variableI = 0;
    digitalWrite(ledMauvais, LOW);
    digitalWrite(ledExellant, LOW);
  }*/
  if (onoff == true) {
    if (digitalRead(capteurDroit) == true || digitalRead(capteurGauche) == true) {
      virage = gps[variable];
      if (virage <= 8 && virage >= 0){
        switch (virage) {
          case 1:
            tournerGaucheE();
          break;
          case 2:
            tournerDroiteE();
          break;
          case 3:
            sens = 3500;
          break;
          case 4:
            forward_brake(0, 0);
            delay(100);
          break;
          case 5:
            tournerGaucheJ();
          break;
          case 6:
            tournerDroiteJ();
          break;
          case 7:
            tournerGaucheT();
          break;
          case 8:
            tournerDroiteT();
          break;
          case 9:
            tournerGaucheL();
          break;
          case 10:
            tournerDroiteL();
          break;
          default:
            sens = 3500;
          break;
        }
      }
      else{
        sens = 3500;
      }
      if (variableI == 0) {
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
      compteurEcran(variable, variable);
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

void tournerDroiteJ(){
  while(digitalRead(capteurGauche)){
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(vitesseTurn);
  }
  forward_brake(0,0);
  if(!digitalRead(capteurGauche)){
    while(!digitalRead(capteurGauche)){
      avancerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(0);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurGauche)){
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(0);
  }
  forward_brake(0,0);
}

void tournerGaucheJ(){
  while(digitalRead(capteurGauche)){
    avancerMoteurGauche(vitesseTurn);
    avancerMoteurDroit(vitesseTurn);
  }
  forward_brake(0,0);
  if(!digitalRead(capteurGauche)){
    while(!digitalRead(capteurGauche)){
      avancerMoteurDroit(vitesseTurn);
      avancerMoteurGauche(0);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit)){
    avancerMoteurGauche(vitesseTurn);
    avancerMoteurDroit(0);
  }
  forward_brake(0,0);
}

void tournerDroiteL(){
  if(digitalRead(capteurDroit) == true){
    while(digitalRead(capteurDroit) == true){
      avancerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(vitesseTurn);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit) == false){
    avancerMoteurGauche(vitesseTurn);
    reculerMoteurDroit(255);
    digitalWrite(ledBon, HIGH);
  }
  forward_brake(0,0);
  if(digitalRead(capteurDroit) == true){
    while(digitalRead(capteurDroit) == true){
      avancerMoteurGauche(vitesseTurn);
      reculerMoteurDroit(255);
    }
  }
  forward_brake(0,0);
}

void tournerGaucheL(){
  if(digitalRead(capteurGauche) == true){
    while(digitalRead(capteurGauche) == true){
      avancerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(vitesseTurn);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurGauche) == false){
    avancerMoteurDroit(vitesseTurn);
    reculerMoteurGauche(vitesseTurn);
  }
  forward_brake(0,0);
  if(digitalRead(capteurGauche) == true){
    while(digitalRead(capteurGauche) == true){
      avancerMoteurDroit(vitesseTurn);
      reculerMoteurGauche(vitesseTurn);
    }
  }
  forward_brake(0,0);
}

void tournerGaucheT() {
  digitalWrite(ledBon, HIGH);
  while(digitalRead(capteurGauche) == true || digitalRead(capteurDroit) == true) {
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(vitesseTurn);
  }
  forward_brake(0,0);
  while(digitalRead(capteurGauche) == false) {
    avancerMoteurDroit(vitesseTurn);
    reculerMoteurGauche(vitesseTurn);
  }
  while(digitalRead(capteurGauche) == true) {
    avancerMoteurDroit(vitesseTurn);
    reculerMoteurGauche(vitesseTurn);
  }
  while(digitalRead(capteurDroit) == false) {
    avancerMoteurDroit(vitesseTurn2);
    reculerMoteurGauche(vitesseTurn2);
  }
  avancerMoteurDroit(0);
  reculerMoteurGauche(0);
  forward_brake(0,0);
  if (digitalRead(capteurDroit) == false){
    while(digitalRead(capteurDroit) == false) {
      avancerMoteurDroit(0);
      avancerMoteurGauche(vitesseTurn);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit) == true) {
    avancerMoteurDroit(0);
    avancerMoteurGauche(vitesseTurn);
  }
  forward_brake(0,0);
  digitalWrite(ledBon, LOW);
}

void tournerDroiteT() {
  digitalWrite(ledBon, HIGH);
  while(digitalRead(capteurGauche) == true || digitalRead(capteurDroit) == true) {
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(vitesseTurn);
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit) == false) {
    avancerMoteurGauche(vitesseTurn);
    reculerMoteurDroit(vitesseTurn);
  }
  while(digitalRead(capteurDroit) == true) {
    avancerMoteurGauche(vitesseTurn);
    reculerMoteurDroit(vitesseTurn);
  }
  while(digitalRead(capteurGauche) == false) {
    avancerMoteurGauche(vitesseTurn);
    reculerMoteurDroit(vitesseTurn);
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit) == true) {
    avancerMoteurGauche(0);
    avancerMoteurDroit(vitesseTurn);
  }
  forward_brake(0,0);
  digitalWrite(ledBon, LOW);
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

void tournerGaucheE(){
  digitalWrite(ledTresBon, HIGH);
  forward_brake(0,0);
  if(digitalRead(capteurGauche) == false){
    while(digitalRead(capteurGauche) == false){
      reculerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(0);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurDroit) == false){
    avancerMoteurDroit(vitesseTurn);
    avancerMoteurGauche(0);
  }
  forward_brake(0,0);
  if(digitalRead(capteurGauche) == false){
    while(digitalRead(capteurGauche) == false){
      reculerMoteurGauche(vitesseTurn);
      avancerMoteurDroit(0);
    }
  }
  forward_brake(0,0);
  while(digitalRead(capteurGauche) == true){
    reculerMoteurGauche(vitesseTurn);
    avancerMoteurDroit(0);
  }
  forward_brake(0,0);
  if(digitalRead(capteurDroit) == true){
    while(digitalRead(capteurDroit) == true){
      avancerMoteurDroit(vitesseTurn);
      avancerMoteurGauche(0);
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

Vector<int> fusionTab(int tabA[], const int tailleTabA, int tabB[], const int tailleTabB) {
  Vector<int> tab;
  int taille = tailleTabA + tailleTabB;

  for (int i = 0; i < taille; i++)
  {
    if (i <= tailleTabA - 1) {
      tab.push_back(tabA[i]);
    }
    else {
      tab.push_back(tabB[i - tailleTabA]);
    }
  }
  return tab;
}

void danseLED(){
  for (int i = 0, i < 7, i++){
    digitalWrite(ledMauvais, HIGH);
    delay(250);
    digitalWrite(ledMauvais, LOW);
    digitalWrite(ledBon, HIGH);
    delay(250);
    digitalWrite(ledBon, LOW);
    digitalWrite(ledTresBon, HIGH);
    delay(250);
    digitalWrite(ledTresBon, LOW);
    digitalWrite(ledExellant, HIGH);
    delay(250);
    digitalWrite(ledExellant, LOW);
  }
}

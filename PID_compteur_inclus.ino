/*
 * File name: PID_compteur_inclus
 *
 * Hardware list : - Arduino Mega
 *                 - QTR-8RC
 *
 * Description: Système de contrôle PID qui permet au robot de suivre une ligne noire sur un fond blanc.
 *
 * Author: Noukypop01 / Not007btw
 */

#include <QTRSensors.h> //Assurez-vous d'installer la library

/*************************************************************************
 * Initialisation de l'objet Sensor Array
 *************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*************************************************************************
 * Variables de contrôle PID
 *************************************************************************/
float Kp = 0; // lié à la proportionelle;
              // changé la valeur en essais-erreur (ex: 0.07).
float Ki = 0; // lié à l'intégral;
              // changé la valeur en essais-erreur (ex: 0.0008).
float Kd = 0; // lié au dérivé;
              // changé la valeur en essais-erreur (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
 * Variables global
 *************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
 * Variables vitesse moteur (entre 0 - arrêt ; et 255 - vitesse maximum)
 *************************************************************************/
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

// /*************************************************************************
//  * DRV8835 GPIO pins declaration
//  *************************************************************************/
// int mode = 8;
// int aphase = 9;
// int aenbl = 6;
// int bphase = 5;
// int benbl = 3;

// int ledMauvais = 53;
// int ledBon = 55;
// int ledTresBon = 54;
// int ledExellant = 81;

/*************************************************************************
 * Buttons pins declaration
 *************************************************************************/
int buttoncalibrate = 17; // ou pin A3
int buttonstart = 2;
int buttonResetCompteur = 26;

const int nbrVirage = 0;
bool tableauTrajectoire[nbrVirage] = {};
int compteur = 0;
int compteurInf = 0;
double sens;

/*************************************************************************
 * Function Name: setup
 **************************************************************************
 * Summary :
 * C'est une fonction au démarrage  de la carte Arduino. Elle active les
 * pins pour les capteurs et les moteurs. Ensuite l'utilisateur doit bouger
 * le robot au dessus de la ligne pendant 10s pour le calibrer
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 *************************************************************************/
void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  qtr.setEmitterPin(7); // LEDON PIN

  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);

  pinMode(ledMauvais, OUTPUT);
  pinMode(ledBon, OUTPUT);
  pinMode(ledTresBon, OUTPUT);
  pinMode(ledExellant, OUTPUT);

  digitalWrite(mode, HIGH); // one of the two control interfaces
                            //(simplified drive/brake operation)
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  boolean Ok = false;
  while (Ok == false)
  { // the main function won't start until the robot is calibrated
    if (digitalRead(buttoncalibrate) == HIGH)
    {
      calibration(); // calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  forward_brake(0, 0); // stop the motors
}

/*************************************************************************
 * Function Name: calibration
 **************************************************************************
 * Summary:
 * C'est la fonction de calibration pour le capteur QTR-8RC. Le nom d'appel
 * de la fonction est "qtr.calibrate()" implanter via la library. Pendant
 * approximativement 10s chacun des 8 capteurs va se calibrer en 'lisant' la
 * piste
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 *************************************************************************/
void calibration()
{
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

/*************************************************************************
 * Function Name: loop
 **************************************************************************
 * Summary:
 * C'est la fonction principal. Quand le boutton start est pressé, le robot
 * va basculer entre suivre la ligne et s'arrêter. Quand il suit la ligne
 * la fonction appel le contôle PID
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 *************************************************************************/
void loop()
{
  if (digitalRead(buttonstart) == HIGH)
  {
    onoff = !onoff;
    if (onoff = true)
    {
      delay(1000); // un delay avant que le robot démarre
    }
    else
    {
      delay(50);
    }
  }
  if (digitalRead(buttonResetCompteur) == HIGH)
  {
    compteur = 0;
  }
  if (onoff == true)
  {
    sens = 3500;
    if (sensorValues[2] == true && sensorValues[5] == true)
    {
      if (tableauTrajectoire[compteur] == 0)
      {
        sens = 500;
      }
      if (tableauTrajectoire[compteur] == 1)
      {
        sens = 2000;
      }
      if (compteurInf == 0)
      {
        compteur++;
      }
      compteurInf++;
      // PID_control(sens);
      if (sensorValues[2] == false && sensorValues[5] == false)
      {
        compteurInf = 0;
      }
    }

    PID_control(sens);
  }
  else
  {
    forward_brake(0, 0); // stop the motors
  }
}

// /*************************************************************************
//  * Function Name: forward_brake
//  **************************************************************************
//  * Summary:
//  * This is the control interface function of the motor driver. As shown in
//  * the Pololu's documentation of the DRV8835 motor driver, when the MODE is
//  * equal to 1 (the pin is set to output HIGH), the robot will go forward at
//  * the given speed specified by the parameters. The phase pins control the
//  * direction of the spin, and the enbl pins control the speed of the motor.
//  *
//  * A warning though, depending on the wiring, you might need to change the
//  * aphase and bphase from LOW to HIGH, in order for the robot to spin forward.
//  *
//  * Parameters:
//  *  int posa: int value from 0 to 255; controls the speed of the motor A.
//  *  int posb: int value from 0 to 255; controls the speed of the motor B.
//  *
//  * Returns:
//  *  none
//  *************************************************************************/
// void forward_brake(int posa, int posb)
// {
//   set the appropriate values for aphase and bphase so that the robot goes straight
//   digitalWrite(aphase, LOW);
//   digitalWrite(bphase, LOW);
//   analogWrite(aenbl, posa);
//   analogWrite(benbl, posb);
// }

/*************************************************************************
 * Function Name: PID_control
 **************************************************************************
 * Summary:
 * C'est la fonction de contrôle PID. Le PID utilise la proportionnel, le dérivé et l'intégral
 * pour un contrôle optimale. La correction est appliqué sur la vitesse des moteurs.
 *
 * Parameters:
 * none
 *
 * Returns:
 *  none
 *************************************************************************/
void PID_control(double consigne)
{
  uint16_t position = qtr.readLineBlack(sensorValues); // lit la position actuel
  int error = consigne - position;                     // 3500 est la position idéal (le centre du robot)
  testPID(position);

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd; // calcul la correction nécessaire à appliquer sur
                                             // la vitesse des moteurs

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda)
  {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb)
  {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0)
  {
    motorspeeda = 0;
  }
  if (motorspeedb < 0)
  {
    motorspeedb = 0;
  }
  forward_brake(motorspeeda, motorspeedb);
}

void testPID(uint16_t pos)
{
  if (pos == 3500)
  {
    digitalWrite(ledExellant, HIGH);
  }
  else
  {
    digitalWrite(ledExellant, LOW);
  }

  if (pos >= 3000 && pos <= 4000)
  {
    digitalWrite(ledTresBon, HIGH);
  }
  else
  {
    digitalWrite(ledTresBon, LOW);
  }

  if (pos >= 2500 && pos <= 4500)
  {
    digitalWrite(ledBon, HIGH);
  }
  else
  {
    digitalWrite(ledBon, LOW);
  }

  if (pos >= 2000 && pos <= 5000)
  {
    digitalWrite(ledMauvais, HIGH);
  }
  else
  {
    digitalWrite(ledMauvais, LOW);
  }
}

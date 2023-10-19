/**************************************************************************************************
Nom du fichier : PID.cpp
Auteur : Samuel Hamelin et Guillaume Béland
Date de création : 2023/10/16

Description : Fichier du PID
              
Notes : Utilise un struct pour retourner plusieurs valeurs

Modifications : 

***************************************************************************************************/

// *************************************************************************************************
//  INCLUDES
// *************************************************************************************************	

#include <Arduino.h>
#include <LibRobus.h>
#include "Move.h"

//namespace MOVE {
  // *************************************************************************************************
  //  CONSTANTES
  // *************************************************************************************************
  /* VIDE */

  // dimensions de la piste: 16x10 pieds, donc une case en x = 5.333 pouces et une case en y = 10.909 pouces
  char parcours[11][36] = {
    {"     _____________________________ "},
    {"    / __________________________  |"},
    {"   / / _____________________ \\  | |"},
    {"  / / / ___________________ \\ \\ | |"},
    {" / / / / ___________       \\ \\ \\| |"},
    {"|        |          \\    /| | | | |"},
    {"|        |___________\\  /_| | | | |"},
    { "\\ \\ \\ \\_____________ _____/ / /| |"},
    {"  \\ \\ \\______________|______/ / | |"},
    {"   \\ \\_______________|_______/__| |"},
    {"    \\________________|____________|"}
  };

  //position de départ du robot (x: 4.5 po ; y : 7.5 po)
  float startPos[] = {int(8.24175), int(10.12601)};
    
  // *************************************************************************************************
  //  FONCTIONS LOCALES
  // *************************************************************************************************
  /* VIDE */
  
  // *************************************************************************************************
  //  STRUCTURES ET UNIONS
  // *************************************************************************************************
  /* VIDE */

  // *************************************************************************************************
  // VARIABLES GLOBALES
  // *************************************************************************************************
  /* VIDE */
  
  /**
   * @brief Calcul la valeur du PID
   *
   * @param incomingValues PID values
   */
  void calculPID(valeursPID *incomingValues)
  {
      float error = incomingValues->Sp - incomingValues->Pv;
      float dt = millis() - incomingValues->Ti;
      incomingValues->p = incomingValues->Kp * error;
      incomingValues->i += incomingValues->Ki * (error*dt);
      incomingValues->d = incomingValues->Kd * (error - incomingValues->Out) / dt;
      incomingValues->Out = incomingValues->p + incomingValues->i + incomingValues->d;
      incomingValues->Ti = millis();
      
  }
 /**
   * @brief Retourne les distances parcourues en pouces par les moteurs
   *
   * @param incomingValues PID values
   */
  /*void valeursDistance getDistance()
  {
    Distance.G = pulseToDist*float(ENCODER_Read(0));
    Distance.D = pulseToDist*float(ENCODER_Read(1));
  }*/

  float distanceMoyenne()
  {
    return (Distance.D+Distance.G)/2;
  }

  float updatePos()
  {

    static float oldPulseG = 0.0;
    static float oldPulseD = 0.0;
    float pulseG = ENCODER_Read(0) - oldPulseG;
    float pulseD = ENCODER_Read(1) - oldPulseD;
    float posRatio = pulseG/pulseD;
    
    float radius = ((wheelBaseDiameter * posRatio)/(1-posRatio));
    float radiusRobot = radius + wheelBaseDiameter/2;
    position.orientation = pulseG/radius + M_PI/2;
    position.x = radiusRobot - (radiusRobot*cos(position.orientation));
    position.y = radiusRobot * sin(position.orientation);

    oldPulseG = pulseG;
    oldPulseD = pulseD;
    
  }

  float speedG()
  {
    
    static float past = 0.0;
    static float speedMotor = 0.0;
    static float oldPulse = 0.0;
    float present = millis();
    float pulse = ENCODER_Read(0);
    speedMotor = 1000.0 * pulseToDist*float(pulse-oldPulse)/float(present - past);
  
    past = present;
    oldPulse = pulse;
    
    return speedMotor;
  }

float speedD()
  {
    
    static float past = 0.0;
    static float speedMotor = 0.0;
    static float oldPulse = 0.0;
    float present = millis();
    float pulse = ENCODER_Read(1);
    speedMotor = 1000.0*pulseToDist*float(pulse-oldPulse)/float(present - past);
    
    past = present;
    oldPulse = pulse;
    
    return speedMotor;
  }
 
//}

float vitesse = 0.3;
bool target = 0.0;
struct valeursPID pid = {};


void setup(){
  BoardInit();
  Serial.begin(9600);
  pid.Kp = 0.1;
}

void loop(){

  /*if (ROBUS_IsBumper(0)){
    target = !target;
  }
  pid.Sp = target;
  calculPID(&pid);
  MOTOR_SetSpeed(0, vitesse + pid.Out);
  MOTOR_SetSpeed(1, vitesse + pid.Out);*/
  MOTOR_SetSpeed(0, vitesse);
  MOTOR_SetSpeed(1, vitesse);
  Serial.print("Moteur gauche : ");
  Serial.print(speedG());
  Serial.print("        Moteur droite : ");
  Serial.println(speedD());
}
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

namespace MOVE {
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
  float startPos[] = {{int(8.24175)}, {int(10.12601)}};
    
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
  void valeursDistance getDistance()
  {
    Distance.G = pulseToDist*float(ENCODER_Read(0));
    Distance.D = pulseToDist*float(ENCODER_Read(1));
      }

  float distanceMoyenne()
  {
    return (Distance.D+Distance.G)/2;
  }

  float updatePos(){

    static float oldPulseG = 0;
    static float oldPulseD = 0;
    float pulseG = ENCODER_Read(0) - oldPulseG;
    float pulseD = ENCODER_Read(1) - oldPulseD;
    float posRatio = pulseG/pulseD;
    
    float radius = ((diametreRobot * posRatio)/(1-posRatio));
    float radiusRobot = radius + diametreRobot/2;
    float orientation = 360 * pulseG/(2*M_PI*radius) + 90;
    float posX = radiusRobot - (radiusRobot*cos(orientation));
    float posY = radiusRobot * sin(orientation);

    oldPulseG = pulseG;
    oldPulseD = pulseD;
    
  }

  float speed(bool motor){
    
    static float delai = 0.0;
    static float speedMotor = 0.0;
    static float oldPulse = 0.0;
    if (millis() - delai > 10){
      
      speedMotor = pulseToDist*float(ENCODER_Read(motor)-oldPulse)/float(millis() - delai);
      delai = millis();
      oldPulse = ENCODER_Read(motor);
    }
    return speedMotor;
  }

 
}

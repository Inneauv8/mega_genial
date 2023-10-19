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
#include "PID.h"

namespace PID {
  // *************************************************************************************************
  //  CONSTANTES
  // *************************************************************************************************
  /* VIDE */
  unsigned long delai = 0;
  float speeeeed = 0;
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
   * @brief Initialisation du programme.
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

  valeursDistance getDistance(){
  //retourne la distance parcourue en pouces par le moteur gauche
  PID::valeursDistance Distance;
  Distance.G = pulseToDist*float(ENCODER_Read(0));
  Distance.D = pulseToDist*float(ENCODER_Read(1));
  return Distance;
  }

  float distanceMoyenne(){

    PID::valeursDistance Distance = getDistance();
    return (Distance.D+Distance.G)/2;
  }

  float vitesse(bool moteur){

    if (millis() - delai > 10){
      
      speeeeed = pulseToDist*float(ENCODER_ReadReset(moteur))/(millis() - delai);
      delai = millis();
    }
    return speeeeed;
  }
}

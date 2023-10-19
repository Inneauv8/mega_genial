/****************************************************************************************
Nom du fichier : MOVE.h
Auteur :  Guillaume Béland et Samuel Hamelin                 
Date de création : 17/10/2023
  
****************************************************************************************/
#ifndef MOVE_H
#define MOVE_H
#include <math.h>

namespace MOVE {
  // *************************************************************************************************
  //  CONSTANTES
  // *************************************************************************************************
  /* VIDE */
  #define diametreRobot 8.0
  #define diametreRoue 3.0

  float pulseToDist = M_PI*diametreRoue/3200.0;

  // *************************************************************************************************
  //  STRUCTURES ET UNIONS
  // *************************************************************************************************
  struct valeursPID 
  {
      float Kp; // Constante proportionnelle
      float Ki; // Constante intégrale
      float Kd; // Constante dérivée
      float Ti; // Temps initial
      float Sp; // Set Point (Valeur voulue)
      float Pv; // Point Value (Valeur réelle)
      float p; // Valeur proportionnelle
      float i; // Valeur intégrale
      float d; // Valeur dérivée
      float Out; // Valeur de sortie
  };

  struct valeursDistance {
    float G;
    float D;
  };
  
  
  // *************************************************************************************************
  //  PROTOTYPE DE FONCTIONS
  // *************************************************************************************************
  
  void calculPID(valeursPID *incomingValues);
  
  valeursDistance getDistance();

  float vitesse(bool moteur);

  float distanceMoyenne();
  
  // *************************************************************************************************
  // VARIABLES GLOBALES
  // *************************************************************************************************
  /* VIDE */
  
  // *************************************************************************************************
  // VARIABLES LOCALES
  // *************************************************************************************************
  /* VIDE */
}

#endif

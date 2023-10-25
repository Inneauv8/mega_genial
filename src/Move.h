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
  #define WHEEL_BASE_DIAMETER 7.480315
  #define WHEEL_DIAMETER 2.992126


  extern float pulseToDist;

  // *************************************************************************************************
  //  STRUCTURES ET UNIONS
  // *************************************************************************************************
  struct valeursPID 
  {
      valeursPID() : Kp(0.0), Ki(0.0), Kd(0.0), Ti(0.0), Sp(0.0), Pv(0.0) {}
      float Kp; // Constante proportionnelle
      float Ki; // Constante intégrale
      float Kd; // Constante dérivée
      float Ti; // Temps initial
      float dt; // Intervalle de temps pour le calcul d'intégrales et de dérivées
      float Sp; // Set Point (Valeur voulue)
      float Pv; // Process Value (Valeur réelle)
      float p; // Valeur proportionnelle
      float i; // Valeur intégrale
      float d; // Valeur dérivée
      float Out; // Valeur de sortie
  };

  struct valeursDistance {
    float G;
    float D;
  };
  
  struct posRobot {
      float x;
      float y;
      float orientation;

  };

  extern valeursDistance Distance;
  extern posRobot position;
  
  // *************************************************************************************************
  //  PROTOTYPE DE FONCTIONS
  // *************************************************************************************************
  
  void calculPID(valeursPID *incomingValues);
  
  valeursDistance getDistance();

  float speed(bool moteur);

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

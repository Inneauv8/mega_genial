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
#include <PID.h>

// *************************************************************************************************
//  CONSTANTES
// *************************************************************************************************
/* VIDE */
float pid = 0 ;

// *************************************************************************************************
//  FONCTIONS LOCALES
// *************************************************************************************************
float PID(float sp, float pv, float kp, float ki, float kd, float dt)
{
    
    float p = sp-pv;
    float i =+ ki*(p*dt);
    float d = (p-pid)/dt;
    pid = kp*p+i+kd*d;
    return pid;
    
}

// *************************************************************************************************
//  STRUCTURES ET UNIONS
// *************************************************************************************************
/* VIDE */

// *************************************************************************************************
// VARIABLES GLOBALES
// *************************************************************************************************
/* VIDE */

/**
 * @brief Fonction de contrôle d'un PID. La fonction prends toutes les valeurs nécessaires pour 
 * faire fonctionner le PID, tout en restant réutiliseable 
 * @author 
 */

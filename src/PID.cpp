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
struct structPID valeursPID;

// *************************************************************************************************
//  FONCTIONS LOCALES
// *************************************************************************************************
struct calculPID(valeursPID)
{
    
    float valeursPID.p = valeursPID.Sp-valeursPID.Pv;
    float valeursPID.i =+ valeursPID.ki*(valeursPID.p*(valeursPID.Tr-valeursPID.Ti));
    float valeursPID.d = (valeursPID.p-valeursPID.Out)/(valeursPID.Tr-valeursPID.Ti);
    valeursPID.Out = valeursPID.kp*valeursPID.p+valeursPID.i+valeursPID.kd*valeursPID.d;
    return 0;
    
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
 * @brief Initialisation du programme.
 * @author 
 */

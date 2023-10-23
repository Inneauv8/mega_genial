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
  float startPos[] = {int(8.24175), int(10.12601)};

  float vitesse = 25.0;
    
  float x = 0, y = 0, theta = 0;
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
      incomingValues->dt = (millis() - incomingValues->Ti)/1000.0;
      float error = (incomingValues->Sp - incomingValues->Pv)*incomingValues->dt;
      incomingValues->p = incomingValues->Kp * error;
      incomingValues->i += incomingValues->Ki * (error*incomingValues->dt);
      if (error == 0.0){incomingValues->i = 0.0;}
      incomingValues->d = incomingValues->Kd * (error - incomingValues->Out) / incomingValues->dt;
      incomingValues->Out += incomingValues->p + incomingValues->i + incomingValues->d;
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

  void updatePos()
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
  
  float averageSpeedD()
  {
    #define averageSize  200
    static float average[averageSize] = {};

    for (int i = 0; i < averageSize; i++)
    {
      average[i] = vitesse;
    }

    float sum = 0.0;
    for (int i = 0; i < averageSize; i++)
    {
      sum += average[i];
    }
    sum /= averageSize;

    for (int i = 1; i < averageSize; i++)
      {
        average[i - 1] = average[i];
      }

      average[averageSize - 1] = speedD();
    return sum;
  }

  float averageSpeedG()
  {
    #define averageSize  200
    static float average[averageSize] = {};
    for (int i = 0; i < averageSize; i++)
    {
      average[i] = vitesse;
    }

    float sum = 0.0;
    for (int i = 0; i < averageSize; i++)
    {
      sum += average[i];
    }
    sum /= averageSize;

    for (int i = 1; i < averageSize; i++)
      {
        average[i - 1] = average[i];
      }

      average[averageSize - 1] = speedG();
    return sum;
  }

  float speedToVoltage(bool motor, float speed)
  {
    float slope = 0.0;
    float b = 0.0;
    float voltage = 0.0;
    if (!motor)
    {
      slope = 33.0150041911;
      b = 0.6033235541;
      voltage = (speed-b)/slope;
    }
    else if (motor)
    {
      slope = 35.8882648785;
      b = 1.1290486169;
      voltage = (speed-b)/slope;
    }
    
    if (voltage < -1.0)
    {
      voltage = -1.0;
    }

    if (voltage > 1.0)
    {
      voltage = 1.0;
    }

    if ( voltage < 0.02 && voltage > -0.02)
    {
      voltage = 0.0;
    }

    return voltage;
  }

  float moveRadius(float xFinal, float yFinal, float finalOrientation)
  {
    float x1 = position.x;
    float y1 = position.y;
    
    float slope1 = (yFinal-y1)/(xFinal-x1);
    float slope2 = tan(finalOrientation);
    float b1 = y1 + (yFinal - y1)/2 + (x1 + (xFinal - x1)/2) * (1/slope1);
    float b2 = yFinal + xFinal/slope2;
    float xRadius = (b2 - b1)/(1/slope2 - 1/slope1);
    float yRadius = b1 - xRadius/slope1;
    float radius = sqrt(sq(xRadius - x1) + sq(yRadius - y1));
    return radius;
  }

  void showDataPID(valeursPID *incomingValues)
  {
    Serial.print("Kp : ");
    Serial.print(incomingValues->Kp);
    Serial.print("\t Ki : ");
    Serial.print(incomingValues->Ki);
    Serial.print("\t Kd : ");
    Serial.print(incomingValues->Kd);
    Serial.print("\t Ti : ");
    Serial.print(incomingValues->Ti);
    Serial.print("\t dt : ");
    Serial.print(incomingValues->dt);
    Serial.print("\t Sp : ");
    Serial.print(incomingValues->Sp);
    Serial.print("\t Pv : ");
    Serial.print(incomingValues->Pv);
    Serial.print("\t p : ");
    Serial.print(incomingValues->p);
    Serial.print("\t i : ");
    Serial.print(incomingValues->i);
    Serial.print("\t d : ");
    Serial.print(incomingValues->d);
    Serial.print("\t Out : ");
    Serial.println(incomingValues->Out);
    Serial.println();
  }

  void updatePIDG(float Sp)
  {
    static struct valeursPID pidG = {};
    pidG.Kp = 0.5;
    pidG.Ki = 0.0;
    pidG.Kd = 0.0;

    pidG.Sp = Sp;
    pidG.Pv = speedG();
    calculPID(&pidG);
    MOTOR_SetSpeed(0, (speedToVoltage(0, pidG.Out)));
    showDataPID(&pidG);
  }

  void updatePIDD(float Sp)
  {
    static struct valeursPID pidD = {};
    pidD.Kp = 2.5;
    pidD.Ki = 0.0;
    pidD.Kd = 0.0;

    pidD.Sp = Sp;
    pidD.Pv = speedD();
    calculPID(&pidD);
    MOTOR_SetSpeed(1, (speedToVoltage(1, pidD.Out)));
    showDataPID(&pidD);
  }


  void updatePIDMain(float speed, float ratio)
  {
    static struct valeursPID pidDist = {};
    pidDist.Kp = 2.5;
    pidDist.Ki = 0.0;
    pidDist.Kd = 0.0;
    float speedL = speedG();
    float speedR = speedD();

    //Ancien code
    //pidDist.Sp = (speedG() - speedD())*pidDist.dt;
    //pidDist.Pv = (ENCODER_Read(0) - ENCODER_Read(1));
    //calculPID(&pidDist);
    //float correctSpeed = pidDist.Out / (2*pidDist.dt);

    //Test SH
    pidDist.Sp = ratio;
    


    if(speedR == 0)
    {
      speedR = 0.0001;
    }
    if(speedL == 0)
    {
      speedL = 0.0001;
    }

    pidDist.Pv = (speedL / speedR);

    calculPID(&pidDist);
    updatePIDG(speed * pidDist.Out);
    updatePIDD(speed / pidDist.Out);
    showDataPID(&pidDist);
  }
  
  float radiusToSpeedG(float moveRadiusRobot, float finalOrientation)
  {
    float initialOrientation = position.orientation;
    float arcCercleAngle = finalOrientation - initialOrientation;
    float distRobot = 2*moveRadiusRobot*M_PI*abs(arcCercleAngle)/(2*M_PI);
    float time = distRobot/vitesse;
    float speedBig = 2*(moveRadiusRobot + wheelBaseDiameter/2)*M_PI*abs(arcCercleAngle)/(2*M_PI*time);
    float speedSmall= 2*(moveRadiusRobot - wheelBaseDiameter/2)*M_PI*abs(arcCercleAngle)/(2*M_PI*time);
    if (arcCercleAngle < 0)
    {
      return speedSmall;
    }
    else //(arcCercleAngle > 0)
    {
      return speedBig;
    }
  }

  float radiusToSpeedD(float moveRadiusRobot, float finalOrientation)
  {
    float initialOrientation = position.orientation;
    float arcCercleAngle = finalOrientation - initialOrientation;
    float distRobot = 2*moveRadiusRobot*M_PI*abs(arcCercleAngle)/(2*M_PI);
    float time = distRobot/vitesse;
    float speedBig = 2*(moveRadiusRobot + wheelBaseDiameter/2)*M_PI*abs(arcCercleAngle)/(2*M_PI*time);
    float speedSmall= 2*(moveRadiusRobot - wheelBaseDiameter/2)*M_PI*abs(arcCercleAngle)/(2*M_PI*time);
    if (arcCercleAngle > 0)
    {
      return speedSmall;
    }
    else //(arcCercleAngle < 0)
    {
      return speedBig;
    }
  }

  void move(float xFinal, float yFinal, float finalOrientation)
  {
    float radius = moveRadius(xFinal, yFinal, finalOrientation);
    float ratio = radiusToSpeedG(radius, finalOrientation)/radiusToSpeedD(radius, finalOrientation);
    updatePIDMain(vitesse, ratio);
    updatePos();
  }

}

using namespace MOVE;

float ratio = 1.0;

void setup(){
  BoardInit();
  Serial.begin(9600);

  ENCODER_Reset(0);
  ENCODER_Reset(1);
  updatePos();
}

void loop(){

  
  if (!ROBUS_IsBumper(0))
  {
    ratio = 1.0;
    x = 0, y = 0, theta = 0;
  }
  else
  {
    ratio = 0.0;
  }

  move(x, y, theta);

}

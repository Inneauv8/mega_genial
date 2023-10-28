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

  valeursDistance Distance = {};
  posRobot position = {};

  float pulseToDist = M_PI*WHEEL_DIAMETER/3200.0;
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

  float vitesse = 5.0;
    
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
   * @brief Print an array of 10 floating-point numbers.
   *
   * This function prints the given array of 10 floating-point numbers to the serial port.
   *
   * @param[in] one The first number.
   * @param[in] two The second number.
   * @param[in] three The third number.
   * @param[in] four The fourth number.
   * @param[in] five The fifth number.
   * @param[in] six The sixth number.
   * @param[in] seven The seventh number.
   * @param[in] eight The eighth number.
   * @param[in] nine The ninth number.
   * @param[in] ten The tenth number.
   */
  void printData(float one, float two, float three, float four, float five, float six, float seven, float eight, float nine, float ten)
  {
    float data[10] = {one, two, three, four, five, six, seven, eight, nine, ten};
    for(int i = 0; i < 10; i++)
    {
      Serial.print(data[i], 10);
      Serial.print("\t");
    }
    Serial.println();
  }

  /**
   * @brief Function to calculate the PID values and control the motors.
   *
   * This function takes PID values and controls the motors based on the error
   * between the set point (Sp) and the process variable (Pv).
   *
   * @param[in] incomingValues PID values.
   * @param[in] resetIOnZeroError Flag to reset the integral term when error is zero.
   * @return The calculated control output.
   */
  float calculPID(valeursPID *incomingValues, bool resetIOnZeroError)
  {
      const float epsilon = 0.0001f;
      // Calculate delta time
      long startTime = micros();
      float dt = (micros() - incomingValues->initialTime) * 0.000001f;
      incomingValues->initialTime = startTime;

      if (fabs(dt) > epsilon && !isnan(incomingValues->Sp) && !isnan(incomingValues->Pv)) {
        float error = (incomingValues->Sp - incomingValues->Pv) * dt;

        float p = incomingValues->Kp * error;
        incomingValues->integral += incomingValues->Ki * (error * dt);
        float d = incomingValues->Kd * (error - incomingValues->Out) / dt;

        if (error == 0.0 && resetIOnZeroError) {incomingValues->integral = 0.0;}

        incomingValues->Out += p + incomingValues->integral + d;
      } else {
        Serial.println("DEBUG - Invalid value fed to PID or delta time is too small");
      }

      return incomingValues->Out;
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

  float noNan(float value)
  {
    return isnan(value) ? FLT_MAX : value;
  }

  float noZero(float value)
  {
    return value == 0 ? FLT_MIN : value;
  }

  float averageDistance()
  {
    return (Distance.Right + Distance.Left) / 2;
  }

  float correctAngle(float angle)
  {
    int initialAngle = angle;
    if(angle < -M_PI)
    {
      angle -= initialAngle;
      Serial.print("here");
    } else if(angle > M_PI)
    {
      angle -= initialAngle;
      Serial.print("or here");
    };

    return angle;
  }

   /**
   * @brief Function to update the robot's position based on encoder readings.
   */
  void updatePos()
  {

    static float oldPulseG = 0.0;
    static float oldPulseD = 0.0;
    static float oldPosX = 0.0;
    static float oldPosY = 0.0;
    static float positionX = 0.0;
    static float positionY = 0.0;
    static float ka = 1.137;
    static float kx = 1.0505;
    float pulseG = ENCODER_Read(0) - oldPulseG;
    float pulseD = ENCODER_Read(1) - oldPulseD;
    float posRatio = pulseG/noZero(pulseD);
    float radius = (WHEEL_BASE_DIAMETER * posRatio)/noZero(1-posRatio);
    float radiusRobot;
    float angle = 0.0;

    if (posRatio > 1 || posRatio < -1)
    {
      angle = pulseToDist * pulseG / noZero(radius);
      radiusRobot = radius - WHEEL_BASE_DIAMETER/2;
    }
    else
    {
      angle = pulseToDist * pulseD / noZero(radius + WHEEL_BASE_DIAMETER);
      radiusRobot = radius + WHEEL_BASE_DIAMETER/2;
    }

    position.orientation += ka * angle;
    position.orientation = wrap(position.orientation, -M_PI, M_PI);
    
    position.x += kx * ((radiusRobot * (cos(position.orientation) - cos(position.orientation-angle))));
    position.y += radiusRobot * (sin(position.orientation) - sin(position.orientation-angle));

    oldPulseG = ENCODER_Read(0);
    oldPulseD = ENCODER_Read(1);
    
  }

  float computeRightMotorSpeed()
  {
    
    static float past = 0.0;
    static float speedMotor = 0.0;
    static float oldPulse = 0.0;
    float present = micros();
    float pulse = ENCODER_Read(0);
    speedMotor = 1000000.0 * pulseToDist*float(pulse-oldPulse)/float(present - past);
  
    past = present;
    oldPulse = pulse;
    
    return speedMotor;
  }

  float computeLeftMotorSpeed()
    {
      
      static float past = 0.0;
      static float speedMotor = 0.0;
      static float oldPulse = 0.0;
      float present = micros();
      float pulse = ENCODER_Read(1);
      speedMotor = 1000000.0*pulseToDist*float(pulse-oldPulse)/float(present - past);
      
      past = present;
      oldPulse = pulse;
      
      return speedMotor;
    }
  
  float averageSpeedD()
  {
    #define averageSize  5
    static float average[averageSize] = {};
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

      average[averageSize - 1] = computeLeftMotorSpeed();
    return sum;
  }

  float averageSpeedG()
  {
    #define averageSize  5
    static float average[averageSize] = {};
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

      average[averageSize - 1] = computeRightMotorSpeed();
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

  long double moveRadius(float xFinal, float yFinal, float finalOrientation)
  {
    float x1 = position.x;
    float y1 = position.y;
    
    float slope1 = noZero(noNan((yFinal-y1)/(xFinal-x1)));
    float slope2 = noZero(noNan(tan(finalOrientation)));
    float deltaInverseSlope = noNan(1/slope2 - 1/slope1);
    long double b1 = y1 + (yFinal - y1)/2 + (x1 + (xFinal - x1)/2) * (1/slope1);
    long double b2 = yFinal + xFinal/slope2;
    long double xRadius = (b2 - b1)/(deltaInverseSlope);
    long double yRadius = b1 - xRadius/slope1;
    long double radius = sqrt(sq(xRadius - x1) + sq(yRadius - y1));
    float r = float(radius);
    if (isinf(radius) == 1)
    {
      radius = 1000000000;
    }
    if (isinf(radius) == -1)
    {
      radius = -1000000000;
    }
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
    Serial.print("\t Initial Time : ");
    Serial.print(incomingValues->initialTime);
    Serial.print("\t Set Point : ");
    Serial.print(incomingValues->Sp);
    Serial.print("\t Process Value : ");
    Serial.print(incomingValues->Pv);
    Serial.print("\t Integral : ");
    Serial.print(incomingValues->integral);
    Serial.print("\t Out : ");
    Serial.println(incomingValues->Out);
    Serial.println();
  }

  void updatePIDG(float Sp)
  {
    static struct valeursPID pidG = {};
    pidG.Kp = 2.5;
    pidG.Ki = 0.0;
    pidG.Kd = 0.0;

    pidG.Sp = Sp;
    pidG.Pv = computeRightMotorSpeed();
    calculPID(&pidG);
    //printData(13, speedToVoltage(0, pidG.Out), pidG.Out, 0, 0, 0, 0, 0, 0, 0);
    MOTOR_SetSpeed(0, (speedToVoltage(0, pidG.Out)));
    //showDataPID(&pidG);
  }

  void updatePIDD(float Sp)
  {
    static struct valeursPID pidD = {};
    pidD.Kp = 2.5;
    pidD.Ki = 0.0;
    pidD.Kd = 0.0;

    pidD.Sp = Sp;
    pidD.Pv = computeLeftMotorSpeed();
    calculPID(&pidD);
    MOTOR_SetSpeed(1, (speedToVoltage(1, pidD.Out)));
    //showDataPID(&pidD);
  }

  /**
   * @brief Update the robot's movement using a PID control system.
   *
   * This function updates the robot's movement by using a PID (Proportional-Integral-Derivative) control system.
   * It adjusts the left and right motor speeds to achieve the desired overall speed (`speed`) and the rate of change in speed (`dV`).
   *
   * @param[in] speed The desired overall speed for the robot's movement.
   * @param[in] dV The desired rate of change in speed (acceleration or deceleration).
   */
  void updatePIDMain(float speed, float dV)
  {
    static struct valeursPID pidSpeed = {};
    pidSpeed.Kp = 3.0;
    pidSpeed.Ki = 1.0;
    pidSpeed.Kd = 0.0005;

    static float speedL;
    static float speedR;
    speedL = computeRightMotorSpeed();
    speedR = computeLeftMotorSpeed();

    //Test SH
    pidSpeed.Sp = dV;
    pidSpeed.Pv = speedL - speedR;

    calculPID(&pidSpeed);

    if(speed + abs(pidSpeed.Out / 2) > 30)
    {
      speed = 30 - abs(pidSpeed.Out / 2);
    } 
    else if(speed - abs(pidSpeed.Out / 2) > 30)
    {
      speed = -30 - abs(pidSpeed.Out / 2);
    }
    
    updatePIDG(speed + (pidSpeed.Out / 2));
    updatePIDD(speed - (pidSpeed.Out / 2));
  }
  
  /*float radiusToSpeedG(double moveRadiusRobot, float finalOrientation)
  {
    float initialOrientation = position.orientation;
    float arcCercleAngle = finalOrientation - initialOrientation;
    double distRobot = moveRadiusRobot*abs(arcCercleAngle);
    float time = noNan(distRobot/vitesse);
    double speedBig = (moveRadiusRobot + WHEEL_BASE_DIAMETER/2)*noNan(abs(arcCercleAngle)/time);
    double speedSmall= (moveRadiusRobot - WHEEL_BASE_DIAMETER/2)*noNan(abs(arcCercleAngle)/time);
    if (arcCercleAngle < 1 && arcCercleAngle > -1)
    {
      return vitesse;
    }
    if (arcCercleAngle < 0.1)
    {
      return speedSmall;
    }
    else //(arcCercleAngle > 0.1)
    {
      return speedBig;
    }
  }

  float radiusToSpeedD(double moveRadiusRobot, float finalOrientation)
  {
    float initialOrientation = position.orientation;
    float arcCercleAngle = finalOrientation - initialOrientation;
    double distRobot = moveRadiusRobot*abs(arcCercleAngle);
    float time = noNan(distRobot/vitesse);
    double speedBig = (moveRadiusRobot + WHEEL_BASE_DIAMETER/2)*noNan(abs(arcCercleAngle)/time);
    double speedSmall= (moveRadiusRobot - WHEEL_BASE_DIAMETER/2)*noNan(abs(arcCercleAngle)/time);
    
    if (arcCercleAngle < 0.1 && arcCercleAngle > -0.1)
    {
      return vitesse;
    }
    if (arcCercleAngle > 0.1)
    {
      return speedSmall;
    }
    else //(arcCercleAngle < 0.1)
    {
      return speedBig;
    }
  }*/

  /**
   * @brief Calculate the rate of change in speed (dV) required for a given move radius and final orientation.
   *
   * This function calculates the rate of change in speed (dV) needed to achieve a specific move radius and final orientation for the robot's movement.
   * It considers the robot's initial orientation, the angle to the final orientation, and the move radius to determine the required dV.
   *
   * @param[in] moveRadiusRobot The move radius for the robot's movement.
   * @param[in] finalOrientation The desired final orientation for the robot's movement.
   * 
   * @return The calculated rate of change in speed (dV) to achieve the specified move radius and orientation.
   */
  float radiusToDV(double moveRadiusRobot, float finalOrientation)
  {
    float initialOrientation = position.orientation;
    float arcCercleAngle = finalOrientation - initialOrientation;
    float dV = 0.0;
    double distRobot = moveRadiusRobot*abs(arcCercleAngle);
    float time = noNan(distRobot/vitesse);
    double speedBig = (moveRadiusRobot + WHEEL_BASE_DIAMETER/2)*noNan(abs(arcCercleAngle)/time);
    double speedSmall= (moveRadiusRobot - WHEEL_BASE_DIAMETER/2)*noNan(abs(arcCercleAngle)/time);
    
    if (arcCercleAngle < 0.1 && arcCercleAngle > -0.1)
    {
      return 0.0;
    }
    if (arcCercleAngle > 0.1)
    {
      dV = speedSmall - speedBig;
      return dV;
    }
    else //(arcCercleAngle < 0.1)
    {
      dV = speedBig - speedSmall;
      return dV; 
    }
    
  }

  /**
   * @brief Move the robot to a specified position and orientation.
   *
   * This function controls the robot's movement to reach the desired final position and orientation.
   *
   * @param[in] xFinal Final X position.
   * @param[in] yFinal Final Y position.
   * @param[in] finalOrientation Final orientation in radians.
   */
  void move(float xFinal, float yFinal, float finalOrientation)
  {

    static float prevX = 0.0;
    static float prevY = 0.0;
    static float prevOrientation = 0.0;
    static float distTotal = 0.0;
    static float distRobot = 0.0;
    float arcCercleAngle = finalOrientation - position.orientation;
    float dV = 0.0;
    float radius = 0.0;

    if (arcCercleAngle < 0.1 && arcCercleAngle > -0.1)
    {
      dV = 0.0;
    }
    else
    {
      if((xFinal - position.x == 0) && !((abs(finalOrientation - position.orientation)) == M_PI/2))
      {
        float middle = (yFinal - position.y)/2;
        float slope2 = tan(finalOrientation);
        float b2 = yFinal + xFinal/slope2;
        float centerX = -slope2 * (middle - b2);
        radius = dist(xFinal, yFinal, centerX, middle);
      }
      else
      {
        if((yFinal - position.y == 0) && !(abs(finalOrientation - position.orientation) == M_PI/2))
        {
          float middle = (xFinal - position.x)/2;
          float slope2 = tan(finalOrientation);
          float b2 = yFinal + xFinal/slope2;
          float centerY = -slope2 * (middle - b2);

          radius = dist(xFinal, yFinal, middle, centerY);;
        }
        else
        {
          if((abs(finalOrientation - position.orientation) == M_PI/2) && !(yFinal - position.y == 0) && !(xFinal - position.x == 0))
          {
            float slope1 = (yFinal-position.y)/(xFinal-position.x);
            double b1 = position.y + (yFinal - position.y)/2 + (position.x + (xFinal - position.x)/2) * (1/slope1);
            float centerY = slope1 * xFinal + b1;
            radius = dist(xFinal, yFinal, xFinal, centerY);;
          }
          else
          {
            radius = moveRadius(xFinal, yFinal, finalOrientation);
          }
        }
      }
      dV = radiusToDV(radius, finalOrientation);
    }
    
    if (arcCercleAngle < 0.1 && arcCercleAngle > -0.1)
    {
      distRobot = dist(position.x, position.y, xFinal, yFinal);
    }
    else
    {
      distRobot = radius * abs(arcCercleAngle);
    }
    
    if(xFinal != prevX || yFinal != prevY || finalOrientation != prevOrientation)
    {
      distTotal = distRobot;
    }
    float speed = 0.0;
    if(distTotal == 0)
    {
      speed = 0.0;
    }
    else
    {
      speed = vitesse*distRobot/distTotal;
    }
    updatePIDMain(speed, dV);
    updatePos();
    //printData(arcCercleAngle, radius, distRobot, distTotal, speed, dV, position.x, position.y, position.orientation, 0);
    prevX = xFinal;
    prevY = yFinal;
    prevOrientation = finalOrientation;

    /*float oldTime = 0.0;
    float dt = millis() - oldTime;
    float initialOrientation = position.orientation;
    float arcCercleAngle = finalOrientation - initialOrientation;
    static float radius = 0.0;
    static double distRobot = 0.0;
    if (arcCercleAngle == 0)
    {
      distRobot = sqrt(sq(xFinal - position.x) + sq(yFinal - position.y));
      radius = 1000000000;
    }
    else
    {
      radius = moveRadius(xFinal, yFinal, finalOrientation);
      distRobot = radius*abs(arcCercleAngle);
    }
    
    static struct valeursPID pidDist = {};
    pidDist.Kp = 2.0;
    pidDist.Ki = 0.0;
    pidDist.Kd = 0.0;
    pidDist.Sp = 0.0;
    pidDist.Pv = distRobot;
    calculPID(&pidDist);
    
    float speedG = radiusToSpeedG(radius, finalOrientation);
    float speedD = radiusToSpeedD(radius, finalOrientation);
    float dV = speedG- speedD;
    static float averageSpeed = 0.0;
    
    static float oldPID = 0.0;
    averageSpeed = averageSpeed + ((pidDist.Out - oldPID)/dt);
    updatePos();
    printData(arcCercleAngle, radius, distRobot, speedG, speedD, dV, averageSpeed, pidDist.Out, position.y, position.orientation);
    updatePIDMain(averageSpeed, dV);
    oldPID = pidDist.Out; */
    
    /*if(distRobot < 0.3 && distRobot > -0.3)
    {
      averageSpeed = 0.0;
    }*/

    /*float time1 = 0.0;
    float time2 = millis();
    float dt = time2 - time1;
    float radius = moveRadius(xFinal, yFinal, finalOrientation);
    float speedG = radiusToSpeedG(radius, finalOrientation);
    float speedD = radiusToSpeedD(radius, finalOrientation);
    float dV = speedG- speedD;
    float averageSpeed = (speedG + speedD) / 2;
    float distance = sqrt(sq(xFinal - position.x) + sq(yFinal - position.y));

    static struct valeursPID pidDist = {};
    pidDist.Kp = 2.0;
    pidDist.Ki = 0.01;
    pidDist.Kd = 0.001;
    pidDist.Sp = 0.0;
    pidDist.Pv = distance;
    calculPID(&pidDist);
    float  correctSpeed = pidDist.Out/dt;
    averageSpeed = averageSpeed - correctSpeed;

    updatePIDMain(averageSpeed, dV);
    updatePos();
    printData(radius, speedG, speedD , dV , averageSpeed , distance ,0 , 0, 0, 0);
    */

    //*************************************************************************
    //! @param updatePIDMain() : prend une vitesse et une différence de vitesse
    //  corrige pour que les deux moteurs aies une différence de vitesse(courbure)
    //  le robot se déplace à la vitesse donnée
    //
    //! @param moveRadius() : prend les positions en x et et l'orientation
    //  retourne le rayon de courbure que le robot doit prendre pour atteindre un point
    //  avec la bonne orientation
    // 
    //! @param radiusToSpeed() (gauche et droit) : prends le rayon et l'orientation finale
    //!  retourne la vitesse que chaque roue doit prendre, se basant sur @param vitesse.
    //**********************************************************************************
    /*J'ai besoins que move fasse:
    - prenne une position finale
    - se déplace vers cette position
    - s'arrête à cette position


    */
  }

  void printPosition(bool type)
  {
    if(type)
    {
      Serial.print(position.x);
      Serial.print("\t");
      Serial.print(position.y);
      Serial.print("\t");
      Serial.print(position.orientation);
      Serial.print("\t");
    }
    if(!type)
    {
      Serial.print(x);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(theta);
      Serial.print("\t");
    }
    Serial.println();
  }
}

using namespace MOVE;

float dV = 0.0;


void setup(){
  BoardInit();
  Serial.begin(9600);

  ENCODER_Reset(0);
  ENCODER_Reset(1);
  updatePos();
  
  
}

void loop()
{
 
 if (ROBUS_IsBumper(0))
  {
    //dV = 0.0;
    
    x = 0, y = 3, theta = 0;
  }
  else
  {
    //dV = 20.0;
  
  
  }
  delay(5);
  //move(x, y, theta);
  updatePIDMain(vitesse, radiusToDV(18, M_PI / 2));
  printPosition(0);
  
  
  /*if(position.x != y || position.y != y)
  {
    move(x, y, theta);
  }*/

  /*if(position.x > (x+0.3) && position.x < (x-0.3))
  {
    move(x, y, theta);
  }
  if(position.y > (y+0.3) && position.y < (y-0.3))
  {
    move(x, y, theta);
  }
  if(position.orientation > (theta+0.3) && position.orientation < (theta-0.3))
  {
    move(x, y, theta);
  }*/
  /*else
  {
    updatePIDMain(0,0);
  }*/
  
  //delay(5);
  

  
  /*
  prendre les deux vitesses, comparer entre elles pour trouver l'erreur, ajuster la vitesse pour atteindre SP. 

  convertir vitesse en position ou lire encodeurs, 

  fonction PID pour les deux moteurs: utilise les fonctions pids de chaque moteur pour se déplacer à un point précis 
  avec la bonne orientation. 

  valeurs d'Entrée: position en x initiale, position en x finale, position en y initiale, position en y finale, orientation initiale,
  orientation finale.

  valeurs de sortie : rien

  utilise la vitesse générale comme facteur de conversion pour trouver un dt à utiliser pour convertir les déplacements 
  calculés par les maths d'arcs de cercle en vitesse à donner aux moteurs. le feed back se fait par l'update de la position et 
  de l'orientation.

  PV = (ecart entre pulse des deux moteurs)

  Out : changement de l'écart d'encodeur entre les deux moteurs
  à convertir en correction de vitesse pour chaque moteur
  */

}

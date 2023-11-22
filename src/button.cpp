/**************************************************************************************************
Nom du fichier : button.cpp
Auteur : Maxime Boucher
Date de création : 2023/11/07

Description : Fonction pour lire bouton avec debounce ajustable
              
Notes : 

Modifications : 

***************************************************************************************************/


#include <Arduino.h>
#include <LibRobus.h>
#include  "button.h"

bool buttonPressed = false;
const int buttonPins[] = {38};
const float buttonDebounceTimes[] = {20};

void initButton(const int buttonPins[], int nbrButton)
{
  for (int i = 0; i < nbrButton; i++) pinMode(buttonPins[i], INPUT_PULLUP);
}

/*
bool isButtonPressedFast(int buttonPin)
{
  return !digitalRead(buttonPin);
}

bool isButtonPressed(int buttonPin, uint32_t debounceTime, int timeOut)
{
  static uint32_t lastDebounceTime;
  static bool buttonState = 0;
  static bool lastButtonState = 0;
  uint32_t timer = millis();

  static bool lastDebounceButtonState = 0;
  while (timer + timeOut > millis())
  {
    buttonState = isButtonPressedFast(buttonPin);
    if (buttonState != lastButtonState) lastDebounceTime = millis();
    if (millis() - lastDebounceTime > debounceTime)
    {
      if (buttonState != lastDebounceButtonState)
      {
        lastDebounceButtonState = buttonState;
        return buttonState;
      }
    }
    lastButtonState = buttonState;
  }
  return lastDebounceButtonState;
}


//Test debounce no while.
void updateButtonState(const int buttonPins[], const float buttonDebounceTimes[], int nbrButton)
{
  static long timeSinceChecked = 0;
  for(int i = 0; i < nbrButton; i++)
  {
    if (buttonDebounceTimes[i] + timeSinceChecked <= millis())
    {
      bool buttonState = !digitalRead(buttonPins[i]);

      if (buttonState != buttonPressed)
      {
        buttonPressed = buttonState;
        if (buttonPressed == true)
        {
          Serial.println("Pressed");
        }
      }
      timeSinceChecked = millis();
    }
  }
}

void setup()
{
  BoardInit();
  initButton(buttonPins, 1);
}

void loop()
{
  updateButtonState(buttonPins, buttonDebounceTimes, 1);
}*/
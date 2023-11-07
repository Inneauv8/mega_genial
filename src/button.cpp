/**************************************************************************************************
Nom du fichier : button.cpp
Auteur : Maxime Boucher
Date de création : 2023/11/07

Description : Fonction pour lire bouton avec debounce ajustable
              
Notes : 

Modifications : 

***************************************************************************************************/


#include <Arduino.h>
#include  "button.h"

void initButton(const int buttonPin[], int nbrButton){
  for (int i = 0; i < nbrButton; i++) pinMode(buttonPin[i], INPUT_PULLUP);
}

bool isButtonPressedFast(int buttonPin){
  return !digitalRead(buttonPin);
}

bool isButtonPressed(int buttonPin, uint32_t debounceTime, int timeOut){
  static uint32_t lastDebounceTime;
  static bool buttonState = 0;
  static bool lastButtonState = 0;
  uint32_t timer = millis();

  static bool lastDebounceButtonState = 0;
  while (timer + timeOut > millis()){
    buttonState = isButtonPressedFast(buttonPin);
    if (buttonState != lastButtonState) lastDebounceTime = millis();
    if (millis() - lastDebounceTime > debounceTime){
      if (buttonState != lastDebounceButtonState) {
        lastDebounceButtonState = buttonState;
        return buttonState;
      }
    }
    lastButtonState = buttonState;
  }
  return lastDebounceButtonState;
}

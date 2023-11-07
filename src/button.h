#include <Arduino.h>

void initButton(const int buttonPin[], int nbrButton);
bool isButtonPressedFast(int buttonPin);
bool isButtonPressed(int buttonPin, uint32_t debounceTime, int timeOut);
#include <Arduino.h>
#include <LibRobus.h>

void updateButtonState();
bool isButtonReleased(int button);

bool lastState[4] = {0};
bool state[4] = {0};
int starterState = 0;
#define LABYRINTHE 1
#define DEFAULTDRAWING 2
#define SDDRAWING 3

void setup()
{
    BoardInit();
    Serial.begin(9600);
}

void loop()
{

    updateButtonState();
    delay(2);

    switch (starterState)
    {
    case LABYRINTHE:

        if (isButtonReleased(LEFT))
        {
            Serial.println("Facile");
            starterState = 0;
        }

        if (isButtonReleased(FRONT))
        {
            Serial.println("Moyen");
            starterState = 0;
        }

        if (isButtonReleased(RIGHT))
        {
            Serial.println("Difficile");
            starterState = 0;
        }
        break;

    case DEFAULTDRAWING:
        if (isButtonReleased(LEFT))
        {
            Serial.println("Dessin 1");
            starterState = 0;
        }

        if (isButtonReleased(FRONT))
        {
            Serial.println("Dessin 2");
            starterState = 0;
        }

        if (isButtonReleased(RIGHT))
        {
            Serial.println("Dessin 3");
            starterState = 0;
        }
        break;
    case SDDRAWING:
        if (isButtonReleased(LEFT))
        {
            Serial.println("Dessin special 1");
            starterState = 0;
        }

        if (isButtonReleased(FRONT))
        {
            Serial.println("Dessin special 2");
            starterState = 0;
        }

        if (isButtonReleased(RIGHT))
        {
            Serial.println("Dessin special 3");
            starterState = 0;
        }
        break;
    default:
        if (isButtonReleased(LEFT))
        {
            Serial.println("Labyrinthe");
            starterState = LABYRINTHE;
        }

        if (isButtonReleased(FRONT))
        {
            Serial.println("Default Drawing");
            starterState = DEFAULTDRAWING;
        }

        if (isButtonReleased(RIGHT))
        {
            Serial.println("SD Drawing");
            starterState = SDDRAWING;
        }

        if (isButtonReleased(REAR))
        {
            Serial.println("Reset");
        }
        break;
    }
}

void updateButtonState()
{
    for (int i = 0; i < 4; i++)
    {
        lastState[i] = state[i];
        state[i] = ROBUS_IsBumper(i);
    }
}

bool isButtonReleased(int button)
{
    return !state[button] && lastState[button];
}
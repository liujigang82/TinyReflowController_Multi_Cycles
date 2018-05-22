//
// Created by JIGANG on 22/5/2018.
//

//
// Created by JIGANG on 22/5/2018.
//
#include <Adafruit_MAX31856.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

int ssrPin = 3;
int lcdRsPin = 10;
int lcdEPin = 9;
int lcdD4Pin = 8;
int lcdD5Pin = 7;
int lcdD6Pin = 6;
int lcdD7Pin = 5;
// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
        140, 146, 146, 140, 128, 128, 128, 128
};

int thermocoupleCSPin = 2;
int switchPin = A1;
long lastDebounceTime;

typedef enum DEBOUNCE_STATE
{
    DEBOUNCE_STATE_IDLE,
    DEBOUNCE_STATE_CHECK,
    DEBOUNCE_STATE_RELEASE
} debounceState_t;

LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(thermocoupleCSPin);
unsigned long nextCheck;
double input;
debounceState_t debounceState;



typedef enum SWITCH
{
    SWITCH_NONE,
    SWITCH_1,
    SWITCH_2
}	switch_t;


switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100


void setup()
{
    Serial.begin(115200);
    // Initialize thermocouple interface
    thermocouple.begin();
    thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

    nextCheck = millis();
    digitalWrite(ssrPin, LOW);

    pinMode(ssrPin, OUTPUT);          // sets the digital pin 13 as output

    switchStatus = SWITCH_NONE;

    lcd.begin(8, 2);
    lcd.createChar(0, degree);
    lcd.clear();
    lcd.print(" SSR  ");
    lcd.setCursor(0, 1);
    lcd.print(" Test ");
    delay(2000);
    lcd.clear();
    lcd.print(" v1.00  ");
    lcd.setCursor(0, 1);
    lcd.print("22-05-18");
    delay(2000);
    lcd.clear();

}

void loop()
{
    input = thermocouple.readThermocoupleTemperature();
    Serial.print("Switch:");
    Serial.print(switchMask);
    Serial.print("    Temperature:");
    Serial.println(input);

    lcd.clear();
    lcd.print("Switch:");
    lcd.print(switchMask);
    lcd.setCursor(0, 1);
    lcd.print(input);
    lcd.print("C ");
    if (switchMask == SWITCH_1)
    {
        // If currently reflow process is on going
        digitalWrite(ssrPin, HIGH);       // sets the digital pin 13 on
    }
        // Switch 2 is pressed
    else if (switchMask == SWITCH_2)
    {
        // Only can switch reflow profile during idle
        digitalWrite(ssrPin, LOW);
    }

    // Simple switch debounce state machine (analog switch)

    switch (debounceState)
    {
        case DEBOUNCE_STATE_IDLE:

            // No valid switch press
            switchStatus = SWITCH_NONE;

            switchValue = readSwitch();

            // If either switch is pressed
            if (switchValue != SWITCH_NONE)
            {
                // Keep track of the pressed switch
                switchMask = switchValue;
                // Intialize debounce counter
                lastDebounceTime = millis();
                // Proceed to check validity of button press
                debounceState = DEBOUNCE_STATE_CHECK;
            }
            break;

        case DEBOUNCE_STATE_CHECK:
            switchValue = readSwitch();
            if (switchValue == switchMask)
            {
                // If minimum debounce period is completed
                if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
                {
                    // Valid switch press
                    switchStatus = switchMask;
                    // Proceed to wait for button release
                    debounceState = DEBOUNCE_STATE_RELEASE;
                }
            }
                // False trigger
            else
            {
                // Reinitialize button debounce state machine
                debounceState = DEBOUNCE_STATE_IDLE;
            }
            break;

        case DEBOUNCE_STATE_RELEASE:
            switchValue = readSwitch();
            if (switchValue == SWITCH_NONE)
            {
                // Reinitialize button debounce state machine
                debounceState = DEBOUNCE_STATE_IDLE;
            }
            break;
    }
}

switch_t readSwitch(void)
{
    int switchAdcValue = 0;

    switchAdcValue = analogRead(switchPin);

    // Add some allowance (+10 ADC step) as ADC reading might be off a little
    // due to 3V3 deviation and also resistor value tolerance
    if (switchAdcValue >= 1000) return SWITCH_NONE;
    if (switchAdcValue <= 10) return SWITCH_1;
    if (switchAdcValue <= 522) return SWITCH_2;

    return SWITCH_NONE;
}
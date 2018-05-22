/*******************************************************************************
  Title: Arduino Uno Controller
  Version: 1.00
  Date: 21-05-2018
  Company: RR@NTU
  Author: Liu Jigang

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  Lead-Free Reflow Curve
  ======================

  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ==================================

  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of Arduino MAX31856 library. Adafruit has been the source of tonnes of
  tutorials, examples, and libraries for everyone to learn.

  ==========================================
  Spence Konde (www.drazzy.com/e/)
  ==========================================
  Maintainer of the ATtiny core for Arduino:
  https://github.com/SpenceKonde/ATTinyCore

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this Tiny Reflow Controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - MAX31856 Library:
    >> https://github.com/adafruit/Adafruit_MAX31856

  Revision  Description
  ========  ===========
  1.00      Initial public release.

*******************************************************************************/

// ***** INCLUDES *****
#include <EEPROM.h>
#include <Adafruit_MAX31855.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
    REFLOW_STATE_IDLE,
    REFLOW_STATE_PREHEAT,
    REFLOW_STATE_SOAK,
    REFLOW_STATE_REFLOW,
    REFLOW_STATE_COOL,
    REFLOW_STATE_COMPLETE,
    REFLOW_STATE_TOO_HOT,
    REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
    REFLOW_STATUS_OFF,
    REFLOW_STATUS_ON
} reflowStatus_t;


typedef enum DEBOUNCE_STATE
{
    DEBOUNCE_STATE_IDLE,
    DEBOUNCE_STATE_CHECK,
    DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE
{
    REFLOW_PROFILE_LEADFREE,
    REFLOW_PROFILE_LEADED
} reflowProfile_t;

// ***** CONSTANTS *****

#define CYCLE_TIME 2
#define REFLOW_HOLD_TIME 2*60*60*1000
#define COOLING_HOLD_TIME 2*60*60*1000

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 30       //50
#define TEMPERATURE_SOAK_MIN 35  //150
#define TEMPERATURE_COOL_MIN 30 //100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 40     //200
#define TEMPERATURE_REFLOW_MAX_LF 100 //250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 80    //180
#define TEMPERATURE_REFLOW_MAX_PB 100 //224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000


// ***** PIN ASSIGNMENT *****
int ssrPin = 3;

int thermoCLK = 13;
int thermoDo = 12;
int thermoCS = 10;

int buzzerPin = 14;
//**********cycle counter ***************
int cycleCounter = 0;


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long startReflow = 0;
unsigned long startCooling = 0;

unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Reflow profile type
reflowProfile_t reflowProfile;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status

// Seconds timer
int timerSeconds;
// Thermocouple fault status
unsigned char fault;

// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
// MAX31856 thermocouple interface
Adafruit_MAX31855 thermocouple = Adafruit_MAX31855(thermoCS);

void setup()
{
    // Check current selected reflow profile
    unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);
    if ((value == 0) || (value == 1))
    {
        // Valid reflow profile value
        reflowProfile = value;
    }
    else
    {
        // Default to lead-free profile
        EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
        reflowProfile = REFLOW_PROFILE_LEADFREE;
    }

    // SSR pin initialization to ensure reflow oven is off
    digitalWrite(ssrPin, LOW);
    pinMode(ssrPin, OUTPUT);

    // Buzzer pin initialization to ensure annoying buzzer is off
    digitalWrite(buzzerPin, LOW);
    pinMode(buzzerPin, OUTPUT);


    // Initialize thermocouple interface
    thermocouple.begin();
    //thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

    // Start-up splash
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(buzzerPin, LOW);
    delay(2000);


    // Serial communication at 115200 bps
    Serial.begin(115200);

    // Turn off LED (active high)
    digitalWrite(LED_BUILTIN, LOW);
    // Set window size
    windowSize = 2000;
    // Initialize time keeping variable
    nextCheck = millis();
    // Initialize thermocouple reading variable
    nextRead = millis();
    // Initialize LCD update timer
    updateLcd = millis();

    timerSeconds = 0;
    // Initialize PID control window starting time
    windowStartTime = millis();
    // Ramp up to minimum soaking temperature
    setpoint = TEMPERATURE_SOAK_MIN;

    soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
    reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
    soakMicroPeriod = SOAK_MICRO_PERIOD_LF;

    // Tell the PID to range between 0 and the full window size
    reflowOvenPID.SetOutputLimits(0, windowSize);
    reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
    // Turn the PID on
    reflowOvenPID.SetMode(AUTOMATIC);

    reflowStatus = REFLOW_STATUS_ON;
    reflowState = REFLOW_STATE_PREHEAT;
}

void loop()
{
    // Current time
    unsigned long now;

    // Time to read thermocouple?
    if (millis() > nextRead)
    {
        // Read thermocouple next sampling period
        nextRead += SENSOR_SAMPLING_TIME;
        // Read current temperature
        input = thermocouple.readCelsius();
        // Check for thermocouple fault
        fault = thermocouple.readError();

        // If any thermocouple fault is detected
        if (fault)
        {
            // Illegal operation
            Serial.print("error");
            reflowState = REFLOW_STATE_ERROR;
            reflowStatus = REFLOW_STATUS_OFF;
        }
    }

    if (millis() > nextCheck)
    {
        // Check input in the next seconds
        nextCheck += 1000;
        // If reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON)
        {
            // Toggle red LED as system heart beat
            digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));
            // Increase seconds timer for reflow curve analysis
            timerSeconds++;
            // Send temperature and time stamp to serial
            Serial.print(timerSeconds);
            Serial.print(",");
            Serial.print("temprature:");
            Serial.print(input);
            Serial.print(",");
            Serial.print("setpoint:");
            Serial.print(setpoint);
            Serial.print(",");
            Serial.println(output);
            Serial.print(",");
            Serial.print("reflowState:");
            Serial.println(reflowState);
        }
        else
        {
            // Turn off red LED
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

    // Reflow oven controller state machine
    switch (reflowState)
    {
        case REFLOW_STATE_IDLE:
            //Serial.print("idle");
            // If oven temperature is still above room temperature
            if (input >= TEMPERATURE_ROOM)
            {
                reflowState = REFLOW_STATE_TOO_HOT;
            }
            else
            {
                // If switch is pressed to start reflow process
            }
            break;

        case REFLOW_STATE_PREHEAT:
            reflowStatus = REFLOW_STATUS_ON;
            // If minimum soak temperature is achieve
            if (input >= TEMPERATURE_SOAK_MIN)
            {
                // Chop soaking period into smaller sub-period
                timerSoak = millis() + soakMicroPeriod;
                // Set less agressive PID parameters for soaking ramp
                reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
                // Ramp up to first section of soaking temperature
                setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
                // Proceed to soaking state
                reflowState = REFLOW_STATE_SOAK;
            }
            break;

        case REFLOW_STATE_SOAK:
            //Serial.print("soak");
            // If micro soak temperature is achieved
            if (millis() > timerSoak)
            {
                timerSoak = millis() + soakMicroPeriod;
                // Increment micro setpoint
                setpoint += SOAK_TEMPERATURE_STEP;
                if (setpoint > soakTemperatureMax)
                {
                    // Set agressive PID parameters for reflow ramp
                    reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
                    // Ramp up to first section of soaking temperature
                    setpoint = reflowTemperatureMax;
                    // Proceed to reflowing state
                    reflowState = REFLOW_STATE_REFLOW;
                }
            }
            break;

        case REFLOW_STATE_REFLOW:
            //Serial.print("reflow");
            // We need to avoid hovering at peak temperature for too long
            // Crude method that works like a charm and safe for the components
            if (startReflow == 0)
                startReflow = millis();
            if (input >= (reflowTemperatureMax - 5) && millis() - startReflow >= REFLOW_HOLD_TIME)
            {
                // Set PID parameters for cooling ramp
                reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
                // Ramp down to minimum cooling temperature
                setpoint = TEMPERATURE_COOL_MIN;
                // Proceed to cooling state
                reflowState = REFLOW_STATE_COOL;
                startReflow = 0;
            }
            break;

        case REFLOW_STATE_COOL:
            //Serial.print("cool");
            // If minimum cool temperature is achieve
            if (startCooling == 0)
                startCooling = millis();
            if (input <= TEMPERATURE_COOL_MIN && millis() - startCooling >= COOLING_HOLD_TIME)
            {
                // Retrieve current time for buzzer usage
                buzzerPeriod = millis() + 1000;
                // Turn on buzzer to indicate completion
                digitalWrite(buzzerPin, HIGH);
                // Turn off reflow process

                startCooling = 0;
                cycleCounter++;

                if(cycleCounter <= CYCLE_TIME)
                    reflowState = REFLOW_STATE_REFLOW;
                else // Proceed to reflow Completion state
                    reflowState = REFLOW_STATE_COMPLETE;
            }
            break;

        case REFLOW_STATE_COMPLETE:
            //Serial.print("complete");
            if (millis() > buzzerPeriod)
            {
                // Turn off buzzer
                digitalWrite(buzzerPin, LOW);
                // Reflow process ended
                reflowState = REFLOW_STATE_IDLE;
                reflowStatus = REFLOW_STATUS_OFF;
            }
            break;

        case REFLOW_STATE_TOO_HOT:
            //Serial.print("hot");
            // If oven temperature drops below room temperature
            if (input < TEMPERATURE_ROOM)
            {
                // Ready to reflow
                reflowState = REFLOW_STATE_IDLE;
            }
            break;

        case REFLOW_STATE_ERROR:
            //Serial.print("state error");
            // Check for thermocouple fault
            fault = thermocouple.readError();

            // If thermocouple problem is still present
            if (fault)
            {
                // Wait until thermocouple wire is connected
                reflowState = REFLOW_STATE_ERROR;
            }
            else
            {
                // Clear to perform reflow process
                reflowState = REFLOW_STATE_IDLE;
            }
            break;
    }


    // PID computation and SSR control
    if (reflowStatus == REFLOW_STATUS_ON)
    {
        now = millis();

        reflowOvenPID.Compute();

        if ((now - windowStartTime) > windowSize)
        {
            // Time to shift the Relay Window
            windowStartTime += windowSize;
        }
        if (output > (now - windowStartTime))
        {
            digitalWrite(ssrPin, HIGH);
        }
        else digitalWrite(ssrPin, LOW);
    }
        // Reflow oven process is off, ensure oven is off
    else
    {
        digitalWrite(ssrPin, LOW);
    }
}

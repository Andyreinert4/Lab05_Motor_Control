// ****************************************************************************
// Title        : Lab05_Motor_Control
// File Name    : 'main.cpp'
// Target MCU   : Espressif ESP32 (Doit DevKit Version 1)
//
// EGRT 390     : Example of basic DC motor control 
//
// Revision History:
// When         Who         Description of change
// -----------  ----------- -----------------------
// 4-April-2025  A. Reinert  Initial Commit
// 7-April-2025  A. Reinert  Changed motor pins to board config
// 7-April-2025  A. Reinert  Added monitor speed and encoder ini
// 7-April-2025  A. Reinert  Added all serial mointor commands
// 7-April-2025  A. Reinert  Added update time for serial monitor
// 7-April-2025  A. Reinert  Modified RPM calculation for serial monitor
// ****************************************************************************

// Include Files
// *************************************************************************
#include <Arduino.h>
#include <debounce.h>

// Globals
// *************************************************************************

// Heartbeat LED
const uint8_t LED = LED_BUILTIN;      // Pin number connected to LED
const uint16_t BLINK_INTERVAL = 1000; // Blink on/off time in milliseconds
bool ledState = true;                 // Default state
unsigned long ledBlinkTime = 0;       // Time of last LED blink

// PWM configurations
const uint8_t PWM_CHANNEL = 0;    // PWM channel for motor control
const uint16_t PWM_FREQ = 5000;   // PWM frequency (Hz)
const uint8_t PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

// Motor control pins
const uint8_t IN1 = 27;           // L298N input 1
const uint8_t IN2 = 26;           // L298N input 2
const uint8_t ENA = 14;           // L298N enable A (PWM)

// Encoder configuration
const uint8_t ENCODER_A = 36;      // Encoder A channel (VP)
const uint8_t ENCODER_B = 39;      // Encoder B channel (VN)
volatile long encoderPosition = 0; // Current encoder position
volatile uint8_t lastEncoded = 0;  // Last encoded state

// Speed calculation variables
long lastPosition = 0;                    // Last encoder position
unsigned long lastSpeedCalc = 0;          // Time of last speed calculation
const unsigned long SPEED_INTERVAL = 250; // Speed calculation interval (ms)
float motorRPM = 0.0;                     // Motor speed in RPM
const float PULSES_PER_REV = 31.0;        // Pulses per revolution (quadrature)

// Speed averaging
const int RPM_ARRAY_SIZE = 10;  // Number of samples for averaging
float rpmArray[RPM_ARRAY_SIZE]; // Array to store RPM values
int rpmIndex = 0;               // Current index in the array
bool rpmArrayFull = false;      // Flag to indicate if array is full

// Button control
const uint8_t BUTTON = 17;           // Pin for push button
const uint8_t DEBOUNCE_INTERVAL = 1; // Update button state every 1ms
bool motorRunning = false;           // Motor state flag
unsigned long lastDebounceTime = 0;  // Time of last debounce update

// Setup button debouncing with active HIGH logic
Debounce motorButton(BUTTON, HIGH);

// Command processing variables
String command = "";          // To store the complete command
bool commandComplete = false; // Flag to indicate command is complete

// Status update timing
static unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 2000; // 2 seconds

// Function Prototypes
// *************************************************************************
void IRAM_ATTR handleEncoder();
void calculateSpeed();
void setMotorState(bool running);
float convertToDegrees(long position);
float calculateAverageRPM(float newRPM);
void processCommand();
void displayStatus();

// Setup Code
// *************************************************************************
void setup()
{
    Serial.begin(115200); // Start serial communication
    pinMode(LED, OUTPUT); // Set LED pin as output

    // Configure encoder pins as inputs
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    // Initialize the lastEncoded variable
    lastEncoded = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);

    // Set up interrupts for encoder pins
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), handleEncoder, CHANGE);

    // Configure motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Configure button pin as input
    pinMode(BUTTON, INPUT);

    // Configure PWM for motor speed control
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENA, PWM_CHANNEL);

    // Initialize RPM averaging array
    for (int i = 0; i < RPM_ARRAY_SIZE; i++)
    {
        rpmArray[i] = 0.0;
    }

    // Initial motor state is off
    setMotorState(false);

    Serial.println("DC Motor Control with L298N and Encoder Feedback");
    Serial.println("------------------------------------------------");
    Serial.println("Commands:");
    Serial.println("F<speed> - Forward (0-100%)");
    Serial.println("R<speed> - Reverse (0-100%)");
    Serial.println("S - Stop");
    Serial.println("AUTO - Automated control mode");
    Serial.println("MANUAL - Manual control mode");
}

// Main program
// *************************************************************************
void loop()
{
    // Heartbeat LED code
    if (millis() - ledBlinkTime > BLINK_INTERVAL)
    {
        ledBlinkTime = millis();     // Update time of last LED blink
        ledState = !ledState;        // Toggle LED state
        digitalWrite(LED, ledState); // Set LED state
    }

    // Process serial input character by character
    while (Serial.available() > 0)
    {
        char inChar = (char)Serial.read(); // Read the incoming character
        Serial.write(inChar);              // Echo it back immediately

        if (inChar == '\n' || inChar == '\r')
        {
            // End of command, process it
            commandComplete = true;
        }
        else
        {
            // Add character to command string
            command += inChar;
        }
    }

    // Process completed command
    if (commandComplete)
    {
        processCommand();
        command = "";          // Reset command string
        commandComplete = false; // Reset command complete flag
    }

    // Check if button is pressed and toggle motor state
    if (motorButton.isPressed())
    {
        motorRunning = !motorRunning; // Toggle motor state
        setMotorState(motorRunning);  // Apply the motor state

        if (motorRunning)
        {
            // Reset position when starting motor
            encoderPosition = 0;
            lastPosition = 0;

            // Reset RPM array
            for (int i = 0; i < RPM_ARRAY_SIZE; i++)
            {
                rpmArray[i] = 0.0;
            }
            rpmIndex = 0;
            rpmArrayFull = false;

            Serial.println("Motor turned ON");
        }
        else
        {
            Serial.println("Motor turned OFF");
        }
    }

    // Calculate and display motor speed at regular intervals only when motor is running
    if (motorRunning)
    {
        calculateSpeed();
    }

    // Display system status
    displayStatus();
}

// Functions
// ****************************************************************************

// Process serial commands
void processCommand()
{
    if (command.length() > 0)
    {
        char cmdChar = toupper(command.charAt(0));

        switch (cmdChar)
        {
        case 'F': // Forward
        {
            uint8_t speedPercent = command.substring(1).toInt();
            speedPercent = constrain(speedPercent, 0, 100);
            uint8_t pwmValue = map(speedPercent, 0, 100, 0, 255);

            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            ledcWrite(PWM_CHANNEL, pwmValue);

            Serial.print("Forward: ");
            Serial.print(speedPercent);
            Serial.println("%");
        }
        break;

        case 'R': // Reverse
        {
            uint8_t speedPercent = command.substring(1).toInt();
            speedPercent = constrain(speedPercent, 0, 100);
            uint8_t pwmValue = map(speedPercent, 0, 100, 0, 255);

            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            ledcWrite(PWM_CHANNEL, pwmValue);

            Serial.print("Reverse: ");
            Serial.print(speedPercent);
            Serial.println("%");
        }
        break;

        case 'S': // Stop
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            ledcWrite(PWM_CHANNEL, 0);

            Serial.println("Stopped");
            break;

        case 'A': // Automated control mode
            Serial.println("Automated control mode activated");
            // Add automated control logic here
            break;

        case 'M': // Manual control mode
            Serial.println("Manual control mode activated");
            // Add manual control logic here
            break;

        case 'I': // System information display
            Serial.println("System Information:");
            Serial.print("Motor State: ");
            Serial.println(motorRunning ? "Running" : "Stopped");
            Serial.print("Motor RPM: ");
            Serial.println(motorRPM, 2);
            Serial.print("Encoder Position: ");
            Serial.println(encoderPosition);
            break;

        case 'H': // Help command
            Serial.println("Available Commands:");
            Serial.println("F<speed> - Set forward speed (0-100%)");
            Serial.println("R<speed> - Set reverse speed (0-100%)");
            Serial.println("S - Stop motor");
            Serial.println("I - System information display");
            Serial.println("H - Help command (displays available commands)");
            break;

        default:
            Serial.println("Unknown command. Use F<speed>, R<speed>, S, AUTO, or MANUAL.");
            break;
        }
    }
}

// Display system status
void displayStatus()
{
    if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
        lastStatusUpdate = millis();

        Serial.print("Motor State: ");
        Serial.println(motorRunning ? "Running" : "Stopped");
        Serial.print("Motor RPM: ");
        Serial.println(motorRPM, 2);
        Serial.print("Encoder Position: ");
        Serial.println(encoderPosition);
    }
}

// Interrupt handler for encoder state changes
void IRAM_ATTR handleEncoder()
{
    uint8_t encoded = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
    uint8_t sum = (lastEncoded << 2) | encoded;

    switch (sum)
    {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        encoderPosition--;
        break;

    case 0b0010:
    case 0b0100:
    case 0b1011:
    case 0b1101:
        encoderPosition++;
        break;
    }

    lastEncoded = encoded;
}

// Calculate motor speed in RPM
void calculateSpeed() {
    if (millis() - lastSpeedCalc >= SPEED_INTERVAL) {
        float timeElapsed = (millis() - lastSpeedCalc) / 1000.0; // Convert to seconds
        long positionChange = encoderPosition - lastPosition;
        float instantRPM = (positionChange / PULSES_PER_REV) * (60.0 / timeElapsed); // Calculate RPM

        motorRPM = calculateAverageRPM(instantRPM); // Smooth the RPM value

        lastSpeedCalc = millis();
        lastPosition = encoderPosition;
    }
}

// Calculate rolling average of RPM values
float calculateAverageRPM(float newRPM)
{
    rpmArray[rpmIndex] = newRPM;
    rpmIndex = (rpmIndex + 1) % RPM_ARRAY_SIZE;

    if (rpmIndex == 0)
    {
        rpmArrayFull = true;
    }

    float sum = 0.0;
    int count = rpmArrayFull ? RPM_ARRAY_SIZE : rpmIndex;

    for (int i = 0; i < count; i++)
    {
        sum += rpmArray[i];
    }

    return sum / count;
}

// Set motor state (on/off)
void setMotorState(bool running)
{
    if (running)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CHANNEL, 255);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CHANNEL, 0);
    }
}

// Convert encoder position to degrees
float convertToDegrees(long position)
{
    return (float)position * (360.0 / PULSES_PER_REV);
}
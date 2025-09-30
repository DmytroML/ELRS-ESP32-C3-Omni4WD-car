
/*
 * ------------------------------------
 * ESP32-C3 RC Car Controller
 * 
 * This firmware implements a 4-wheel drive RC car controller using ESP32-C3
 * with ExpressLRS (ELRS) radio control protocol.
 * 
 * Hardware:
 * - Board: ESP32C3-Mini / Adafruit QT Py ESP32-C3
 * - Radio: ELRS 2.4GHz receiver
 * - Motors: 4x DC motors with H-bridge drivers
 * 
 * Features:
 * - Independent 4-wheel control
 * - Proportional steering
 * - Emergency stop
 * - Real-time serial monitoring
 * 
 * Author: Dmytro
 * Created: 2025
 * ------------------------------------
 */

// Required libraries
#include <Arduino.h>          // Core Arduino functionality
#include <WiFi.h>            // WiFi capabilities (if needed)
#include <esp_wifi.h>        // ESP32 WiFi specific functions
#include <AlfredoCRSF.h>     // ELRS protocol handling
#include <HardwareSerial.h>  // Serial communication
#include <math.h>            // Mathematical functions for steering calculations

// PWM Configuration
#define LEDC_RESOLUTION_BITS 10    // 10-bit resolution (0-1023)
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)  // Calculate max PWM value (1023)
#define LEDC_FREQUENCY 50                                    // PWM frequency in Hz
#define MOTOR_MIN_PWM 300                                   // Minimum PWM value for motor movement
#define MOTOR_MAX_PWM 2048                                  // Maximum PWM value (reduced from 4095 for better control)

// ELRS (ExpressLRS) Radio Configuration
HardwareSerial crsfSerial(1);                              // UART1 for ELRS communication
AlfredoCRSF crsf;                                          // ELRS protocol handler
#define PIN_RX 20                                          // ELRS receiver pin
#define PIN_TX 11                                          // ELRS transmitter pin

// Motor Pin Definitions
// Each motor requires 3 pins:
// - enable: PWM signal for speed control
// - in1/in2: Direction control pins (H-bridge configuration)

// Left Front Motor (Motor 1)
const int enablePin1 = 4;    // PWM speed control
const int motor1In1 = 5;     // Direction control - Forward when HIGH
const int motor1In2 = 6;     // Direction control - Reverse when HIGH

// Right Front Motor (Motor 2)
const int enablePin2 = 3;    // PWM speed control
const int motor2In1 = 7;     // Direction control - Forward when HIGH
const int motor2In2 = 8;     // Direction control - Reverse when HIGH

// Left Back Motor (Motor 3) 
const int enablePin3 = 2;    // PWM speed control
const int motor3In1 = 9;     // Direction control - Forward when HIGH
const int motor3In2 = 10;    // Direction control - Reverse when HIGH

// Right Back Motor (Motor 4)
const int enablePin4 = 1;    // PWM speed control
const int motor4In1 = 0;     // Direction control - Forward when HIGH
const int motor4In2 = 21;    // Direction control - Reverse when HIGH

// Motor Control Variables
int desiredLeftPWM = 0;      // Desired PWM value for left front motor
int desiredRightPWM = 0;     // Desired PWM value for right front motor
int desiredMotor3PWM = 0;    // Desired PWM value for left back motor
int desiredMotor4PWM = 0;    // Desired PWM value for right back motor

// Timing Control
volatile unsigned long serverPrevTime = 0;   // Previous time stamp for control loop
const unsigned long serverInterval = 50;     // Control loop interval (20Hz)

// Movement Control Variables
int speed_percent = 0;       // Speed percentage (0-100)
int direction = 0;          // Movement direction: 1 (Forward), -1 (Backward), 0 (Stop)
int x_Val = 0;             // X-axis joystick value (-500 to 500)
int y_Val = 0;             // Y-axis joystick value (-500 to 500)
float rotate_Val = 0;      // Calculated rotation angle in radians


void handleEmergencyStop();
void updateMotorSignals();

void ELRSdecode();

/**
 * Setup function - initializes all hardware components
 * 
 * Initialization sequence:
 * 1. Emergency stop (safe state)
 * 2. Serial communication
 * 3. ELRS radio
 * 4. Motor control pins
 * 5. PWM channels
 */
void setup() {
    // Start in safe state
    handleEmergencyStop();
    
    // Initialize debug serial communication
    Serial.begin(115200);
    
    // Initialize ELRS radio communication
    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
    if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

    crsf.begin(crsfSerial);



    pinMode(motor1In1, OUTPUT);
    pinMode(motor1In2, OUTPUT);
    pinMode(motor2In1, OUTPUT);
    pinMode(motor2In2, OUTPUT);
    pinMode(motor3In1, OUTPUT);
    pinMode(motor3In2, OUTPUT);
    pinMode(motor4In1, OUTPUT);
    pinMode(motor4In2, OUTPUT);
    //ledcAttach(enablePin1, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    //ledcAttach(enablePin2, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    //ledcAttach(enablePin3, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    //ledcAttach(enablePin4, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);

    pinMode(enablePin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    pinMode(enablePin3, OUTPUT);
    pinMode(enablePin4, OUTPUT);


    
    ledcAttach(enablePin1, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    ledcAttach(enablePin2, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    ledcAttach(enablePin3, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    ledcAttach(enablePin4, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
    digitalWrite(motor1In1, LOW);
    digitalWrite(motor1In2, LOW);
    digitalWrite(motor2In1, LOW);
    digitalWrite(motor2In2, LOW);
    digitalWrite(motor3In1, LOW);
    digitalWrite(motor3In2, LOW);
    digitalWrite(motor4In1, LOW);
    digitalWrite(motor4In2, LOW);
    ledcWrite(enablePin1, 0);
    ledcWrite(enablePin2, 0);
    ledcWrite(enablePin3, 0);
    ledcWrite(enablePin4, 0);

}

/**
 * Main program loop
 * 
 * Executes control cycle every serverInterval (50ms = 20Hz):
 * 1. Updates ELRS radio data
 * 2. Processes control inputs
 * 3. Updates motor outputs
 * 4. Provides debug information via Serial
 */
void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - serverPrevTime >= serverInterval) {
        // Update ELRS protocol handler
        crsf.update();
        
        // Process radio control inputs
        ELRSdecode();

        Serial.print(map(constrain(crsf.getChannel(5),1000,2000),1000,2000,0,1)); //канал 5 - включение/выключение 
        Serial.print(", ");
        Serial.print(speed_percent); //канал 5 - включение/выключение 
        Serial.print(", ");
        Serial.print(direction); //канал 5 - включение/выключение 
        Serial.print(", ");
        Serial.print(x_Val); //канал 5 - включение/выключение 
        Serial.print(", ");
        Serial.print(y_Val); //канал 5 - включение/выключение 
        Serial.print(",");  
        Serial.print(rotate_Val*180/M_PI); //канал 5 - включение/выключение  
        Serial.print(", ");
        Serial.println(speed_percent*cosf(rotate_Val)*cosf(rotate_Val)); //канал 5 - включение/выключение 



        serverPrevTime = currentMillis;

        updateMotorSignals();        

    }




}


/**
 * Decodes ELRS radio control signals and updates control variables
 * 
 * Channel mapping:
 * - Channel 1: Left/Right steering (-500 to 500)
 * - Channel 2: Forward/Backward throttle (-500 to 500)
 * - Channel 5: Emergency stop switch (0 or 1)
 */
void ELRSdecode()
{
    // Check emergency stop switch (Channel 5)
    if(map(constrain(crsf.getChannel(5),1000,2000),1000,2000,0,1)==0) {
        handleEmergencyStop();
    } 
    else {
        // Process throttle (Channel 2)
        speed_percent = map(constrain(crsf.getChannel(2),1000,2000),1000,2000,-100,100); 
        if (speed_percent <0) {speed_percent = speed_percent*-1;}; // Absolute value for speed

        // Process steering controls
        x_Val = map(constrain(crsf.getChannel(1),1000,2000),1000,2000,-500,500);  // Left/Right
        y_Val = map(constrain(crsf.getChannel(2),1000,2000),1000,2000,-500,500);  // Forward/Backward

        if (y_Val >-50 && y_Val <50  ) {direction=0;}//канал 2 - вперед/назад
        else if (y_Val >50) {direction=1;}
        else if (y_Val <-50) { direction=-1;}

        if (x_Val != 0 && y_Val!=0) 
        {
            rotate_Val= atan2f((float) x_Val, (float) y_Val);
        }
        else if(x_Val ==0 && y_Val >0) {rotate_Val=0.0;}
        else if(x_Val ==0 && y_Val <0) {rotate_Val=M_PI;}
        else if(y_Val ==0 && x_Val >0) {rotate_Val=M_PI/2;}
        else if(y_Val ==0 && x_Val <0) {rotate_Val=-M_PI/2;}
        else {rotate_Val=0;}
    }
}




/**
 * Emergency Stop Handler
 * 
 * Immediately stops all motors by:
 * 1. Setting all PWM values to 0
 * 2. Setting all direction control pins to LOW
 * 3. Disabling all motor outputs
 */
void handleEmergencyStop() {
    // Reset desired PWM values
    desiredLeftPWM = 0;
    desiredRightPWM = 0;
    desiredMotor3PWM = 0;
    desiredMotor4PWM = 0;
    
    // Disable all direction control pins
    digitalWrite(motor1In1, LOW);
    digitalWrite(motor1In2, LOW);
    digitalWrite(motor2In1, LOW);
    digitalWrite(motor2In2, LOW);
    digitalWrite(motor3In1, LOW);
    digitalWrite(motor3In2, LOW);
    digitalWrite(motor4In1, LOW);
    digitalWrite(motor4In2, LOW);
    
    // Set all PWM outputs to 0
    ledcWrite(enablePin1, 0);
    ledcWrite(enablePin2, 0);
    ledcWrite(enablePin3, 0);
    ledcWrite(enablePin4, 0);
 
}

void updateMotorSignals() {



    int leftOut = (int)map(speed_percent, 0,100,0, MOTOR_MAX_PWM);
    int rightOut = (int)map(speed_percent, 0,100,0, MOTOR_MAX_PWM);
    int motor3Out = (int)map(speed_percent, 0,100,0, MOTOR_MAX_PWM);
    int motor4Out = (int)map(speed_percent, 0,100,0, MOTOR_MAX_PWM);
    

    if (direction == 1) // Forward
    {
        digitalWrite(motor1In1, HIGH);
        digitalWrite(motor1In2, LOW);
        digitalWrite(motor2In1, HIGH);
        digitalWrite(motor2In2, LOW);
        digitalWrite(motor3In1, HIGH);
        digitalWrite(motor3In2, LOW);
        digitalWrite(motor4In1, HIGH);
        digitalWrite(motor4In2, LOW);

        if (rotate_Val == 0)
        {
                ledcWrite(enablePin1, leftOut);
                ledcWrite(enablePin2, leftOut);
                ledcWrite(enablePin3, leftOut);
                ledcWrite(enablePin4, leftOut);
        }
        else if (rotate_Val > 0)
        {
                ledcWrite(enablePin1, (int)leftOut*cosf(rotate_Val)*cosf(rotate_Val));
                ledcWrite(enablePin2, rightOut);
                ledcWrite(enablePin3, (int)motor3Out*cosf(rotate_Val)*cosf(rotate_Val));
                ledcWrite(enablePin4, motor4Out);
        }
        else if (rotate_Val < 0)
        {
                ledcWrite(enablePin1, leftOut);
                ledcWrite(enablePin2, (int)rightOut*cosf(rotate_Val)*cosf(rotate_Val));
                ledcWrite(enablePin3, motor3Out);
                ledcWrite(enablePin4, (int)motor4Out*cosf(rotate_Val)*cosf(rotate_Val));
        }
    }
    
    else if (direction == -1) // Backward
    {
        digitalWrite(motor1In1, LOW);
        digitalWrite(motor1In2, HIGH);
        digitalWrite(motor2In1, LOW);
        digitalWrite(motor2In2, HIGH);
        digitalWrite(motor3In1, LOW);
        digitalWrite(motor3In2, HIGH);
        digitalWrite(motor4In1, LOW);
        digitalWrite(motor4In2, HIGH);
        if (rotate_Val == (float)M_PI)
        {
                ledcWrite(enablePin1, (int)MOTOR_MAX_PWM/8);
                ledcWrite(enablePin2, (int)MOTOR_MAX_PWM/8);
                ledcWrite(enablePin3, (int)MOTOR_MAX_PWM/8);
                ledcWrite(enablePin4, (int)MOTOR_MAX_PWM/8);
        }
        else if (rotate_Val > 0)
        {
                ledcWrite(enablePin1, (int)MOTOR_MAX_PWM*cosf(rotate_Val)/8);
                ledcWrite(enablePin2, MOTOR_MAX_PWM/8);
                ledcWrite(enablePin3, (int)MOTOR_MAX_PWM/2*cosf(rotate_Val)/8);
                ledcWrite(enablePin4, MOTOR_MAX_PWM/8);
        }
        else if (rotate_Val < 0)
        {
                ledcWrite(enablePin1, MOTOR_MAX_PWM/8);
                ledcWrite(enablePin2, (int)MOTOR_MAX_PWM*cosf(rotate_Val)/8);
                ledcWrite(enablePin3, MOTOR_MAX_PWM/8);
                ledcWrite(enablePin4, (int)MOTOR_MAX_PWM*cosf(rotate_Val)/8
            );
        }
    }
    else // Stop
    {
        digitalWrite(motor1In1, LOW);
        digitalWrite(motor1In2, LOW);
        digitalWrite(motor2In1, LOW);
        digitalWrite(motor2In2, LOW);
        digitalWrite(motor3In1, LOW);
        digitalWrite(motor3In2, LOW);
        digitalWrite(motor4In1, LOW);
        digitalWrite(motor4In2, LOW);
        ledcWrite(enablePin1, 0);
        ledcWrite(enablePin2, 0);
        ledcWrite(enablePin3, 0);
        ledcWrite(enablePin4, 0);
    }
}
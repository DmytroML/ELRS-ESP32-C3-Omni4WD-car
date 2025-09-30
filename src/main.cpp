
/*
 * ------------------------------------
 * Board Module: ESP32C3-Mini
 * elrs 2.4
 * 4095

*/
#include <Arduino.h>
#include <WiFi.h>

#include <esp_wifi.h> 


#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <math.h>




#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50
#define MOTOR_MIN_PWM 300 
#define MOTOR_MAX_PWM 2048//4095


// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
#define PIN_RX 20
#define PIN_TX 11

// Left Front Motor (Motor 1)
const int enablePin1 = 4;   // PWM pin
const int motor1In1 = 5;    // Direction control 1
const int motor1In2 = 6;    // Direction control 2

// Right Front Motor (Motor 2)
const int enablePin2 = 3;  // PWM pin
const int motor2In1 = 7;   // Direction control 1
const int motor2In2 = 8;    // Direction control 2

// Left Back Motor (Motor 3) 
const int enablePin3 = 2;   // PWM pin
const int motor3In1 = 9;    // Direction control 1
const int motor3In2 = 10;   // Direction control 2

// Right Back Motor (Motor 4)
const int enablePin4 = 1;   // PWM pin
const int motor4In1 = 0;    // Direction control 1
const int motor4In2 = 21;    // Direction control 2

int desiredLeftPWM = 0;
int desiredRightPWM = 0;
int desiredMotor3PWM = 0;
int desiredMotor4PWM = 0;



volatile unsigned long serverPrevTime = 0;
const unsigned long serverInterval = 50;


int speed_percent = 0; // Speed percentage (0-100)
int direction = 0; // 1 for Forward, 0 for Backward
int x_Val = 0;
int y_Val = 0;
float rotate_Val = 0;


void handleEmergencyStop();
void updateMotorSignals();

void ELRSdecode();

void setup() {
    
    handleEmergencyStop();
    // Initialize Serial for debugging
    Serial.begin(115200);
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

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - serverPrevTime >= serverInterval) {

        // Must call crsf.update() in loop() to process data
        crsf.update();
        // Now you can read channel values
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


void ELRSdecode()
{
    if(map(constrain(crsf.getChannel(5),1000,2000),1000,2000,0,1)==0) {handleEmergencyStop();} 
    else {
        speed_percent = map(constrain(crsf.getChannel(2),1000,2000),1000,2000,-100,100); //канал 2 - скорость
        if (speed_percent <0) {speed_percent = speed_percent*-1;};

        x_Val = map(constrain(crsf.getChannel(1),1000,2000),1000,2000,-500,500); //канал 1 - право/лево
        y_Val = map(constrain(crsf.getChannel(2),1000,2000),1000,2000,-500,500); //канал 2 - перед/назад

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




void handleEmergencyStop() {
    desiredLeftPWM = 0;
    desiredRightPWM = 0;
    desiredMotor3PWM = 0;
    desiredMotor4PWM = 0;
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
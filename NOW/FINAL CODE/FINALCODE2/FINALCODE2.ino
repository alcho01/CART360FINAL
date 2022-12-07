//Include the Adafruit PWM Servo library for the utilization of the PCA9685 Servo Shield
#include <Adafruit_PWMServoDriver.h>
//Include Pitches to play my melodies
#include "pitches.h"

/* SERVO */
//Create 3 variables to be called for the servo motors
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x40);

/* Determine the Minimum and Maximum pulse length count. I played around with these numbers till I was
satisfied with the outcome */
int servoMinStateRelax = 100;
int servoMaxStateRelax = 250;

int servoMinStatePanic = 350;
int servoMaxStatePanic = 650;

//Set the Servo Frequency
int servoFreq = 50;

//Determine the pin output for each servo on the shield
int servo1 = 0;
int servo2 = 8;
int servo3 = 15;

//Servo Timers
//Timer Variables for the panic state
int timeAfterStatePanicServo1 = 1500;
int timeAfterStatePanicServo2 = 2000;
int timeAfterStatePanicServo3 = 2500;
int timeFinisher = 3000;

long unsigned int previousTime = 0;
long unsigned int timeTurnedLowServo1;
long unsigned int timeTurnedLowServo2;
long unsigned int timeTurnedLowServo3;
long unsigned int timeOverFinisher;

//Bools to check ServoTimes
bool checkTimerActivationServo1 = false;
bool checkTimerActivationServo2 = false;
bool checkTimerActivationServo3 = false;
bool timeOver = false;

//Delayer

int delayer = 3;


/* PIR SENSOR */

//Set pin for PIR Sensor
int PIR_SENSOR_PIN = 3;

//No motion at the start of the program
int currentMovementState = true;

//Sensor Status 
int value = 0;

// When the sensor is low, it stays high for this duration
int timeAfterPIRLow = 5000;

long unsigned int timeTurnedLow;

//read when the sensor turns off
boolean readLowTime; 

//I researched that it takes roughly 10 to 60 seconds to calibrate the sensor.
int calibratePIR = 30;


/* Surface Transducer */

int SPEAKER_PIN = 12; 


/* Vibration Sensors */

//Set pins
int diskPin1 = A0;
int diskPin2 = A3;
int diskPin3 = A7;

//Set a threshold to see if the sensor goes over it
int threshold = 100;

//Set a variable to read the vibration
int sensorVibReading1 = 0;
int sensorVibReading2 = 0;
int sensorVibReading3 = 0;


/* Sounds */

//Melodies
int melody [ ] = {
   NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4
};

int melody2 [ ] = {
   NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, 0, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_A4,
};

int melody3 [ ] = {
  NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4,
};

int melody4 [ ] = {
  NOTE_D4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4, 
};

//Durations of notes - Higher the number faster the note is and vice versa

int noteDurations[] = {
  4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2
};

int noteDurations2[] = {
  4, 4, 4, 4, 3, 4, 3, 4, 4, 4, 4, 3, 4, 4, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3
};

int noteDurations3[] = {
  4, 4, 4, 4, 4, 4, 3, 4, 4, 4, 4, 4, 4, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
};

int noteDurations4[] = {
  4, 8, 4, 8, 4
};

void setup() {
  //Debug purposes
  Serial.begin(57600);

  /* Servos */
  
  //Start each board
  pwm1.begin();
  pwm2.begin();
  pwm3.begin();

  //Set the oscillator frequency 
  pwm1.setOscillatorFrequency(27000000);
  pwm2.setOscillatorFrequency(27000000);
  pwm3.setOscillatorFrequency(27000000);

  //Set the servo frequency
  pwm1.setPWMFreq(servoFreq);
  pwm2.setPWMFreq(servoFreq);
  pwm3.setPWMFreq(servoFreq);

  /* PIR SENSOR */

  //Read if there is movement via digitalRead
  value = digitalRead(PIR_SENSOR_PIN);

  //PIR as input - Whats going in
  pinMode(PIR_SENSOR_PIN, INPUT);

  //Sensor is off
  readLowTime = LOW;

  //For testing to see when calibration is complete
  Serial.println("Calibrating Sensor");
  //Convert time to milliseconds
  delay(calibratePIR * 1000);
  //For testing to see when calibration is complete
  Serial.println("Calibration Complete");

  /* SPEAKER */

  pinMode(SPEAKER_PIN, OUTPUT);
  
}

void loop() {
  relaxed();
  panic();
  panicPart2();
  panicPart3();
  panicPart4();
  checkForMovement();
  checkForVibration();
 //areaAAlarm();
}

/* ALARMS */

/* 
   AREA A, B, C are concerned with the vibration sensors scattered along the environment.
   When the requirement of a certain area is met, a melody will produce based on the pitches.h file.
   I made areas A, B, C, have a melody that fits together. Therefore, if all alarms are triggered in the correct sequence it will be a full flowing melody.
   The Panic Alarm just notifies the user when movement via the PIR sensor is detected.
*/

//Area A
void areaAAlarm() {

    for (int thisNote = 0; thisNote < 15; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    
    tone(SPEAKER_PIN, melody[thisNote], noteDuration);

      int pauseBetweenNotes = noteDuration * 1.30;

    delay(pauseBetweenNotes);

    noTone(SPEAKER_PIN);
  }
}

//Area B
void areaBAlarm() {

    for (int thisNote = 0; thisNote < 24; thisNote++) {
    int noteDuration = 1000 / noteDurations2[thisNote];

    tone(SPEAKER_PIN, melody2[thisNote], noteDuration);

      int pauseBetweenNotes = noteDuration * 1.10;

    delay(pauseBetweenNotes);

    noTone(SPEAKER_PIN);
  }
}

//Area C
void areaCAlarm(){

  for (int thisNote = 0; thisNote < 28; thisNote++) {
    int noteDuration = 1000 / noteDurations3[thisNote];

    tone(SPEAKER_PIN, melody3[thisNote], noteDuration);

      int pauseBetweenNotes = noteDuration * 1.10;

    delay(pauseBetweenNotes);

    noTone(SPEAKER_PIN);
  }
}

//Panic Alarm
void panicAlarm() {

   for (int thisNote = 0; thisNote < 5; thisNote++) {
    int noteDuration = 1000 / noteDurations4[thisNote];

    tone(SPEAKER_PIN, melody4[thisNote], noteDuration);

      int pauseBetweenNotes = noteDuration * 1.10;

    delay(pauseBetweenNotes);

    noTone(SPEAKER_PIN);
  } 
}

/* 
  This function checks for movement from the PIR sensor.
  When it detects movement the panic alarm activates, the timer for servo1 commences, and the bool of servo1 turns true.
  If no movement is detected it does the opposite.

  This function will transition from relaxed state to panic state & panic state to relaxed state.

*/
void checkForMovement() {
  //Read if there is movement via digital
  value = digitalRead(PIR_SENSOR_PIN);
  //Check to see if the input is high
  if (value == HIGH) {
    panicAlarm();
    //panic();
    delayer = 1;
   //Count the millis when the panic state activates
   timeTurnedLowServo1 = millis();
   checkTimerActivationServo1 = true;
    //Change the currentmovement state to high if it is low
  if (currentMovementState == LOW) {
    //Read the given line for testing purposes to see if it works
    Serial.println("DETECTED");
    currentMovementState = HIGH;
    }
  }

    else {
    //turning it off 
    if (currentMovementState == HIGH) {
      Serial.println("STOPPED");
      currentMovementState = LOW;
      //If no movement keep the delayer at 5 seconds
      delayer = 5;
      //Count the millis when the PIR goes low
      timeTurnedLow = millis();
      relaxed();
      //Check to see if enough time has passed 
   if (!currentMovementState && (millis() - timeTurnedLow) > timeAfterPIRLow) {
     Serial.println("Time High Ended");
      noTone(SPEAKER_PIN); //Turn off the buzzer if it is playing
   }
  }
 }
}

/*
  This function checks for vibration from any of the following vibration sensors.
  Each pin is connected to an analog in pin that translates an int to the serial monitor.
  If the value of the int is greater than 100, the sensor will do it's specific tasks.
*/

void checkForVibration() {
  
  int sensorVibReading1 = analogRead(A0);
  Serial.print("Threshold1: ");
  Serial.println(sensorVibReading1);

    if (sensorVibReading1 >= threshold) {
     Serial.println("Area A detected");
     areaAAlarm();
     presenceFeltA();     
  }

  delay(2);

  int sensorVibReading2 = analogRead(A3);
  Serial.print("Threshold2: ");
  Serial.println(sensorVibReading2);

    if (sensorVibReading2 >= threshold) {
     Serial.println("Area B detected");
     areaBAlarm();
     presenceFeltB();
  }

  delay(2);

  int sensorVibReading3 = analogRead(A7);
  Serial.print("Threshold3: ");
  Serial.println(sensorVibReading3);

  if (sensorVibReading3 >= threshold) {
     Serial.println("Area C detected");
     areaCAlarm();
     presenceFeltC();
  } 

  delay(2);
  
}

/*
  This function loops throughout the program.
  It is the relaxed state meaning the servos will rotate back and forth at the same rotation until interaction occurs.
*/

void relaxed() {

     for (int i = servoMinStateRelax; i <= servoMaxStateRelax; i++) {
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = servoMaxStateRelax; i >= servoMinStateRelax; i--) {  
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
}

/*
 This function happens when human detection occurs.
 What happens is that each servo gets isolated to a movement, and timer.
 After the designated time passes for one servo the timer begins for the second servo and then the third until they all are rotating.
 The speed is also set a bit faster by changing the delayer from 1 to 5.
 At the end of the function everything gets reset back to the relaxed state.
*/

void panic() {
  
  if (checkTimerActivationServo1 == true && timeTurnedLowServo1 > timeAfterStatePanicServo1) {
    Serial.println("Activate Panic Servo1");

    timeTurnedLowServo2 = millis();
    checkTimerActivationServo2 = true;
    delayer = 1; 

   for (int i = servoMinStatePanic; i <= servoMaxStatePanic ; i++) {
     pwm1.setPWM(servo1, 0, i);
    // pwm2.setPWM(servo2, 0, i);
    // pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = servoMaxStatePanic ; i >= servoMinStatePanic; i--) {  
     pwm1.setPWM(servo1, 0, i);
    // pwm2.setPWM(servo2, 0, i);
    // pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
  }
 }

void panicPart2() {
    if (checkTimerActivationServo2 == true &&  timeTurnedLowServo1 > timeAfterStatePanicServo2) {
    Serial.println("Activate Panic Servo2");

    timeTurnedLowServo3 = millis();
    checkTimerActivationServo3 = true;
    delayer = 1; 

   for (int i = 200; i <= 300 ; i++) {
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
    // pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = 300 ; i >= 200; i--) {  
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
   //  pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
  }
}

void panicPart3() {
    if (checkTimerActivationServo3 == true && timeTurnedLowServo1 > timeAfterStatePanicServo3) {
    Serial.println("Activate Panic Servo3");

    timeOverFinisher = millis();
    timeOver = true;
    delayer = 5; 

   for (int i = 40; i <= 180 ; i++) {
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = 180 ; i >= 40; i--) {  
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
  }
}

  void panicPart4() {
  
  if (timeOver == true && timeTurnedLowServo1 > timeFinisher) {
    Serial.println("stop");
   delayer = 3; 

  checkTimerActivationServo1 = false;
  checkTimerActivationServo2 = false;
  checkTimerActivationServo3 = false;
  timeOver = false;

  timeTurnedLowServo1 = millis();
  }
}

/*
  This function is a child of sensorVibReading.
  The servos will react based on the vibration it intakes.
  Also I organized them by areas.
  Therefore, Area A will occupy the center area of the environment, while Area B will take the left and Area C will take the right.
*/

//Area A (CENTER)
void presenceFeltA() {

   for (int i = servoMinStateRelax; i <= servoMaxStateRelax; i++) {
     //pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     //pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = servoMaxStateRelax; i >= servoMinStateRelax; i--) {  
    // pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
    // pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
}

//Area B (LEFT)
void presenceFeltB() {

   for (int i = servoMinStateRelax; i <= servoMaxStateRelax; i++) {
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     //pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = servoMaxStateRelax; i >= servoMinStateRelax; i--) {  
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     //pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
}

//Area C (RIGHT)*
void presenceFeltC() {

   for (int i = servoMinStateRelax; i <= servoMaxStateRelax; i++) {
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     pwm3.setPWM(servo3, 0, i);
     delay(delayer);   
   }

   for (int i = servoMaxStateRelax; i >= servoMinStateRelax; i--) {  
     pwm1.setPWM(servo1, 0, i);
     pwm2.setPWM(servo2, 0, i);
     pwm3.setPWM(servo3, 0, i);
     delay(delayer);
   }
}

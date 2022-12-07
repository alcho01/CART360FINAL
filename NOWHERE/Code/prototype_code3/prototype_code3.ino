/*The code is pretty straight forward and simple. It focuses on the communication between user movement enabling the buzzer to sound off, notifying
the user that movement was detected. I added a buzzerSFX function to generate 3 tones creating a "panic" type sound. When the buzzer sounds off the
servo motors will increase in speed based on the delayer value decreasing. The motors will then return to the original "relaxed" state.

*/

//Include servo library
#include <Servo.h> 

//PIR SENSOR 

//Set pin for PIR sensor
int PIR_SENSOR_PIN = 12;

//Set a variable for if motion is detected or not
int currentMovementState = LOW;

//Pin Status
int value = 0; 

//PIEZO

//set pin for buzzer
int BUZZER = 11;
//set initial delay
int delayer = 15;


//SERVO MOTORS

//Set pins for servos
int SERVO_PIN1 = 3;
int SERVO_PIN2 = 6;
int SERVO_PIN3 = 8;

//Initialize the servos

Servo servo1;
Servo servo2;
Servo servo3;

//Buzzer function to produce "Panic Sound"
void buzzerSFX() {
  
//Play the buzzer sound
  tone(BUZZER, 493);
  //Delay 
  delay(300);
  //Stop the buzzer 
  noTone(BUZZER);
  
//Play the buzzer sound 
  tone(BUZZER, 293);
  //Delay 
  delay(300);
  //Stop the buzzer 
  noTone(BUZZER);

//Play the buzzer sound
  tone(BUZZER, 329);
  //Delay 
  delay(300);
  //Stop the buzzer 
  noTone(BUZZER);
}


void setup() {
  //Testing purposes
  Serial.begin(57600);

  //Read if there is movement via digital
  value = digitalRead(PIR_SENSOR_PIN);

  //Assign the pin to the servos
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
  servo3.attach(SERVO_PIN3);

  //Set inputs and outputs

  //PIR as input - Whats going in
  pinMode(PIR_SENSOR_PIN, INPUT);

  //Buzzer as output - Whats going out
  pinMode(BUZZER, OUTPUT);
}





void loop() {
  //Read if there is movement via digital
  value = digitalRead(PIR_SENSOR_PIN);
  if (value == HIGH) {
    //Play the buzzer sound
    buzzerSFX();
    //If the motion is detected set the delay speed to 5, to make the rotation speed faster
    delayer = 5;
  if (currentMovementState == LOW) {
    Serial.println("DETECTED");
    currentMovementState == HIGH;
    }
  }
  else {

    Serial.println("STOPPED");
    currentMovementState == LOW;
    //If no movement keep the delayer at 15 seconds
    delayer = 15;
   }
   
  //Servo rotates from 0 degrees to 180 degrees at a constant speed throughout the entire program
  for (int i = 0; i < 90; i++) {
    servo1.write(i);
    servo2.write(i);
    servo3.write(i);
    //Add a delay based on if there is detection or not 
    delay(delayer);
  }
  //Servo rotates from 180 degrees to 0 degrees at a constant speed throughout the entire program
   for (int i = 90; i < 0; i--) {
    servo1.write(i);
    servo2.write(i);
    servo3.write(i);
    //Add a delay based on if there is detection or not 
    delay(delayer);
  }
  
}

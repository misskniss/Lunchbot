#include <NewPing.h> 
#include <Servo.h>
#
# Author: Robert Ivans
# Date: March 2016
# Purpose; Control the scooper from the arduino mounted to Turtlebot
#
#
/* Install the above library using the following instructions:
 *    http://playground.arduino.cc/Code/NewPing
 */

/* PIN & Distance Configs
 * Change these pin values to the pins that you end up actually
 * end up using for the sensor.
 * 
 * MAX_DISTANCE units: CM
*/

// servo constants
Servo servo1, servo2;

#define SERVO1_PIN 5
#define SERVO1_FIRST_POSITION 0
#define SERVO1_SECOND_POSITION 160
#define SERVO2_PIN 4
#define SERVO2_FIRST_POSITION 0
#define SERVO2_SECOND_POSITION 90

// stepper constants
#define STEPS_PER_REVOLUTION 200
#define STEP_PIN             9
#define DIR_PIN              8
#define STEP_DELAY           10
#define OPEN_STEPS           150 // number of steps to open the scoop
#define TOO_MANY_STEPS       300 // closing steps w/o endstop

// stepper variables
int steps_taken = 0; // 0 means fully closed


// endstop constants -- endstop is high open, low closed
#define ENDSTOP_PIN          2

// endstop variables
int endstop_state = 1; // 0 is closed, 1 is open

// rangefinder constants
#define TRIGGER_PIN          7
#define ECHO_PIN             6
#define MAX_DISTANCE         60 // Max distance for an egg ~ 2ft
#define EGG_IN_POSITION      10 // approx 4 inches from sensor
#define HITS_FOR_CLOSE       5
#define RANGE_DELAY          100//250

// rangefinder variables
unsigned int hits = 0; // used for anti-glitching. when hits == 5 then activate scoop.

//state variable
unsigned int state = 4; // the state of the machine

/*
 * Set up the instance of the NewPing, called it 'sonar'
 * Pass in the pins being used and the Max distance we want
 * to pick up on an object. 
 * 
 * If the distance from and object
 * is > the max distance, sonar.ping() will return 0.
 */
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 


/*
 * Use the Arduino IDE's serial console set to 115200 baud
 * to watch these values come in from the sensor. 
 */
void setup() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(SERVO1_FIRST_POSITION);
  servo2.write(SERVO2_FIRST_POSITION);
  Serial.begin(9600);
  pinMode(DIR_PIN, OUTPUT);     
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENDSTOP_PIN, INPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW); 
  Serial.println(4);
  //Serial.println("waiting...");
}

/*
 * In the loop, we ping, calculate the distance to the the thing,
 * then close the scoop if we are within the EGG_IN_POSITION 
 * range. 
 * 
 * We will need logic here that once we get to a range the kinect
 * can no longer see the egg (~1 ft) we keep driving toward the 
 * egg until we get the egg inside the EGG_IN_POSITION distance.
 * 
 */
void loop() {
  delay(RANGE_DELAY);
  endstop_state = digitalRead(ENDSTOP_PIN);
  unsigned int echo_time = sonar.ping();
  unsigned int distance = echo_time / US_ROUNDTRIP_CM;
  
  if(state == 1 or state == 4){
     if ( (distance > 0) and (distance <= EGG_IN_POSITION) ){
       if(hits<HITS_FOR_CLOSE) hits = hits + 1;      
       if(hits >= HITS_FOR_CLOSE){
        // "I've found an egg! Closing scoop."
        //Serial.println("Egg");
        close_scoop();
       }
     }else hits = 0;  // distance > EGG_IN_POSITION
    
  }else if(state == 3){
    if(Serial.available() > 0) open_scoop();
 
  }else{ //state == 4
    if(Serial.available() > 0) im_clear_of_the_egg();
  }
    
}

byte close_scoop() {
  //Serial.print("Scooping!\n");
  state = 2;
  Serial.println(2);
  //Serial.println("Closing...");
  /*
   * Scoop closing code.
   *
   */
  hits = 0;
  digitalWrite(DIR_PIN, HIGH);
  steps_taken = 0;
  while (endstop_state == HIGH) { 
      digitalWrite(STEP_PIN, HIGH);
      delay(STEP_DELAY);          
      digitalWrite(STEP_PIN, LOW); 
      delay(STEP_DELAY);
      endstop_state = digitalRead(ENDSTOP_PIN);
      steps_taken = steps_taken + 1;
  }
  //Serial.println("Closed!");
  state = 3;
  Serial.println(3);
  
  servo1.write(SERVO1_SECOND_POSITION);
  servo2.write(SERVO2_SECOND_POSITION);
  // "The scoop is closed. Rangefinder deactivated. Waiting for signal to open scoop."
  //do nothing just wait for response
  //Serial.println("waiting...");
  Serial.read();
  hits = 0;
  return 0;
}

byte open_scoop() {// need to suppress the scooping when an egg is set down
  /* Scoop opening code.
   *
   */
  hits = 0;
  steps_taken = 0;
  digitalWrite(DIR_PIN, LOW);
  //Serial.println("Openning...");
  while (steps_taken < OPEN_STEPS) {
      digitalWrite(STEP_PIN, HIGH);
      delay(STEP_DELAY);          
      digitalWrite(STEP_PIN, LOW); 
      delay(STEP_DELAY);
      steps_taken = steps_taken + 1;
  }
  state = 4;
  // "Scoop is open. Waiting for signal to activate rangefinder."
  Serial.println(4);
  //do nothing just wait for response
  //Serial.println("waiting...");
  Serial.read();
  //state = 1;
  //Serial.println(1);
  return 0;
}

byte im_clear_of_the_egg() { // state that prevents picking up an egg that was just set down
  // "Rangefinder activated. Hunting eggs."
  //do nothing just wait for response
  Serial.read();
  state = 1;
  Serial.println(1);
  //Serial.println("Open & Detecting...");
  servo1.write(SERVO1_FIRST_POSITION);
  servo2.write(SERVO2_FIRST_POSITION);
}


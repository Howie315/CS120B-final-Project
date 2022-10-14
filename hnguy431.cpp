
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIG_PIN A4
#define ECHO_PIN A5
#define DISTANCE_MAX 200

#define RIGHT A2
#define LEFT A3



NewPing sonar(TRIG_PIN, ECHO_PIN, DISTANCE_MAX);
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;



typedef struct task {
  int state;
  unsigned long period;
  unsigned long elapsedTime;
  int (*TickFct)(int);

} task;

int delay_gcd;
const unsigned short tasksNum = 2;
task tasks[tasksNum];
enum States {INIT, FORWARDS, TURN_RIGHT, TURN_LEFT, STOP} SM1_state;


int Right_Value = 0;
int Left_Value = 0;
int pos = 0;
int distance = 0;





void stateTransitions(int state) {

  distance = sonar.ping_cm();
  Right_Value = digitalRead(RIGHT);             // read the value from Right IR sensor:
  Left_Value = digitalRead(LEFT);               // read the value from Left IR sensor:


  if ((distance > 1) && (distance < 15) ) {
    state = FORWARDS;
    moveForward();
    //myservo.write(90);
  }

  else if ((Right_Value == 0) && (Left_Value == 1)) {
    state = TURN_LEFT;
    turnLeft();
    myservo.write(160);



  }

  else if ((Right_Value == 1) && (Left_Value == 0)) {
    state = TURN_RIGHT;
    turnRight();
    myservo.write(70);

  }



  else if (distance > 15) {
    state = STOP;
    moveStop();

  }

}

int SM1_Tick(int state) {
  // STATE TRANSITIONS
  switch (state) {
    case INIT:
      stateTransitions(state);

      break;
    case FORWARDS:
      stateTransitions(state);
      break;

    case TURN_RIGHT:
      stateTransitions(state);
      break;
    case TURN_LEFT:
      stateTransitions(state);
      break;
    case STOP:
      stateTransitions(state);
      break;
  }
  // STATE ACTIONS
  switch (state) {
    case INIT:
      break;
    case FORWARDS:
      moveForward();

      break;

    case TURN_RIGHT:
      turnRight();

      break;
    case TURN_LEFT:
      turnLeft();
      break;
    case STOP:
      moveStop();
      break;
  }
  return state;
}

enum SM2_States { SM2_INIT, SM2_S0, SM2_S1};
int SM2_Tick(int state) {
  //Read thing
  switch (state) { // State transitions
    case SM2_INIT:
      //State Transition
      state = SM2_S0;
      break;
    case SM2_S0:
      state = SM2_S1;
      //State Transition
      break;
    case SM2_S1:
      state = SM2_S0;
      //State Transition
      break;
  }
  switch (state) { // State Action
    case SM2_INIT:
      //State Action
      break;
    case SM2_S0:
      //State Action


      //State Action
      break;
    case SM2_S1:
      //State Action


      //State Action
      break;
  }

  return state;
}

void moveStop() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor3.setSpeed(0);
  motor4.run(RELEASE);
}

void moveForward() {

  motor1.setSpeed(90);  //define motor1 speed:
  motor1.run(FORWARD);
  motor2.setSpeed(90);  //define motor2 speed:
  motor2.run(BACKWARD);
  motor3.setSpeed(90);  //define motor3 speed:
  motor3.run(FORWARD);
  motor4.setSpeed(90);  //define motor4 speed:
  motor4.run(BACKWARD);
}


void moveBackward() {
  motor1.setSpeed(80);  //define motor1 speed:
  motor1.run(BACKWARD);
  motor2.setSpeed(80);  //define motor2 speed:
  motor2.run(FORWARD);
  motor3.setSpeed(80);  //define motor3 speed:
  motor3.run(BACKWARD);
  motor4.setSpeed(80);  //define motor4 speed:
  motor4.run(FORWARD);


}

void turnRight() {
  motor1.setSpeed(110);  //define motor1speed:
  motor1.run(FORWARD);
  motor2.setSpeed(110);  //define motor2 speed:
  motor2.run(BACKWARD);
  motor3.setSpeed(110);  //define motor3 speed:
  motor3.run(BACKWARD);
  motor4.setSpeed(110);  //define motor4 speed:
  motor4.run(FORWARD);


}

void turnLeft() {
  motor1.setSpeed(110);  //define motor1 speed:
  motor1.run(BACKWARD);
  motor2.setSpeed(110);  //define motor2 speed:
  motor2.run(FORWARD);
  motor3.setSpeed(110);  //define motor3 speed:
  motor3.run(FORWARD);
  motor4.setSpeed(110);  //define motor4 speed:
  motor4.run(BACKWARD);



}

void setup() {


  Serial.begin(9600); //initailize serial communication at 9600 bits per second:
  myservo.attach(10); // servo attached to pin 10 of Arduino UNO



  pinMode(RIGHT, INPUT); //set analog pin RIGHT as an input:
  pinMode(LEFT, INPUT);  //set analog pin RIGHT as an input:

  unsigned char i = 0;
  tasks[i].state = INIT;
  tasks[i].period = 150;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &SM1_Tick;
  i++;
  tasks[i].state = SM2_INIT;
  tasks[i].period = 150;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &SM2_Tick;
  delay_gcd = 500; // GCD
}
void loop() {


  Serial.print("distance");
  Serial.println(distance);                         // print the distance in serial monitor

  Serial.print("RIGHT");
  Serial.println(Right_Value);                      // print the right IR sensor value in serial monitor:
  Serial.print("LEFT");
  Serial.println(Left_Value);                       //print the left IR sensor value in serial monitor:
  unsigned char i;
  for (i = 0; i < tasksNum; ++i) {
    if ( (millis() - tasks[i].elapsedTime) >= tasks[i].period) {
      tasks[i].state = tasks[i].TickFct(tasks[i].state);
      tasks[i].elapsedTime = millis(); // Last time this task was ran

    }
  }

}

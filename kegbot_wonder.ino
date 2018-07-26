#include <NewPing.h>
#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

/* KEGBOT MODES */
const int USER_DRIVE = 1;
const int ROBOT_DRIVE = 2;

RoboClaw roboclaw(15,14);

#define SONAR_NUM     5 // Number or sensors.
#define ITERATION_NUM 4 // Number or iterations.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define DEBUG 1

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x) 
#endif

#define MOTOR_STILL 1500

//Servo myservo1; // create servo object to control a RoboClaw channel
//Servo myservo2; // create servo object to control a RoboClaw channel

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
unsigned int history[SONAR_NUM][ITERATION_NUM]; // Store ping distances.
unsigned int average[SONAR_NUM];
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
uint8_t currentIteration = 0;

int speed_left = 0, speed_right = 0;
char commands[256];
int command_index = 0;
int have_command = 0;
int mode = USER_DRIVE;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(11, 10, MAX_DISTANCE),
  NewPing(9, 8, MAX_DISTANCE),
  NewPing(13, 12, MAX_DISTANCE),
  NewPing(3, 2, MAX_DISTANCE),
  NewPing(7, 6, MAX_DISTANCE),
//  NewPing(25, 26, MAX_DISTANCE),
//  NewPing(27, 28, MAX_DISTANCE),
//  NewPing(29, 30, MAX_DISTANCE),
//  NewPing(31, 32, MAX_DISTANCE),
//  NewPing(34, 33, MAX_DISTANCE),
//  NewPing(35, 36, MAX_DISTANCE),
//  NewPing(37, 38, MAX_DISTANCE),
//  NewPing(39, 40, MAX_DISTANCE),
//  NewPing(50, 51, MAX_DISTANCE),
//  NewPing(52, 53, MAX_DISTANCE)
};

void setup() {

  Serial.begin(115200);
  Serial2.begin(9600);
  roboclaw.begin(9600);
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
//  myservo1.attach(4);
//  myservo2.attach(5);
  
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {
        oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      }
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      if(cm[currentSensor] == 0) {
        history[currentSensor][currentIteration] = MAX_DISTANCE;
      }
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of your code would go here.
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()) {
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    history[currentSensor][currentIteration] = cm[currentSensor];
  }
}

void computeAverage() {
  
  unsigned int sum;
  unsigned int valid_readings;
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    
//    DEBUG_PRINT("Average ");
//    DEBUG_PRINT(i);
//    DEBUG_PRINT(": ");
    
    sum = valid_readings = 0;
    
    for (uint8_t j = 0; j < ITERATION_NUM; j++) {

//      DEBUG_PRINT(history[i][j]);
//      if(j < ITERATION_NUM - 1) {
//        DEBUG_PRINT(" + ");
//      }
      
      if(history[i][j] > 0) {
        sum += history[i][j];
        valid_readings++;
      } else {
        sum += 200;
        valid_readings++;
      }
    }
    
    if(valid_readings) {
      average[i] = sum / valid_readings;
    } else {
      average[i] = 0;
    }
    
//    DEBUG_PRINT("= ");
//    DEBUG_PRINT(sum);
//    DEBUG_PRINT(" / ");
//    DEBUG_PRINT(valid_readings);
//    DEBUG_PRINT(" = ");
//    DEBUG_PRINTLN(average[i]);
    
  }
  
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  
  computeAverage();
  
  currentIteration = (currentIteration + 1) % ITERATION_NUM;
  Serial.println('test');
  readCommand();
//  moveRobot();
//  wander();
  
//  for (uint8_t i = 0; i < SONAR_NUM; i++) {
//    Serial.print(average[i]);
//    Serial.print("\t");
//  }

//  for (uint8_t i = 0; i < SONAR_NUM; i++) {
//    Serial.print(i);
//    Serial.print("=");
//    Serial.print(cm[i]);
//    Serial.print(" (");
//    Serial.print(average[i]);
//    Serial.print(") ");
//    Serial.print("cm ");
//  }
  
  Serial.println();
  
}

void readCommand() {
  
  int result;
  char character;
    Serial.println('read command');
  while (Serial2.available()) {
    Serial.println('test');
    // read the incoming byte:
    character = Serial2.read();
    
    if(character > 0) {
      commands[command_index] = character;
      command_index++;
    }
    
    if(character == '\n') {
      have_command = 1;
    }
  }
  
  if(have_command) {
    
    //if the command is to change a mode
    if(strstr(commands, "mode") != NULL) {
      result = sscanf(commands,"{mode:%d}", &mode);
    } else {
      result = sscanf(commands,"{left:%d,right:%d}", &speed_left, &speed_right);
    }
    
    have_command = 0;
    command_index = 0;
    
    Serial.println(commands);
  }
  
}

void moveRobot() {
    
    int left = 1500 - (int)(speed_left * 1.25);
    int right = 1500 - (int)(speed_right * 1.25);
    
    Serial.println(left);
    //Serial.println(right);
    
    roboclaw.BackwardM1(address, left); //Cmd 0
    roboclaw.BackwardM2(address, right); //Cmd 5
    
}

//void sendReadings() {
//  
//  Serial.print("{\"readings\":[");
//  
//  for(int i=0; i<num_ir; i++) {
//    Serial.print(readings[i]);
//    if(i < num_ir-1) {
//      Serial.print(",");
//    }
//  }
//  
//  Serial.println("]}");
//  
//}

void wander() {
  
  float minDistance = 1000;
  int minSensor = -1;
  
  for(int i=0; i<SONAR_NUM; i++) {
    if(average[i] > 0 && average[i] < minDistance) {
      minDistance = average[i];
      minSensor = i;
    }
  }
  
//  Serial.print(minSensor);
//  Serial.print(" ");
//  Serial.print(minDistance);
  
  
//  roboclaw.ForwardMixed(address, 0);
//  return;
  
  if(minDistance >= 40) {

    Serial.println(" forward");
    
    //forward
    roboclaw.BackwardM1(address,25); //Cmd 0
    roboclaw.BackwardM2(address,25); //Cmd 5
//    myservo1.writeMicroseconds(MOTOR_STILL - 50);
//    myservo2.writeMicroseconds(MOTOR_STILL - 50);
    
  } else {
    
    if(true) {
      Serial.println(" stop");
      roboclaw.ForwardM1(address,0); //Cmd 0
      roboclaw.BackwardM2(address,0); //Cmd 5
    } else if(minSensor < 2) {
      
      Serial.println(" left");
      
      //turn left
      roboclaw.ForwardM1(address,25); //Cmd 0
      roboclaw.BackwardM2(address,25); //Cmd 5
//      myservo1.writeMicroseconds(MOTOR_STILL + 50);
//      myservo2.writeMicroseconds(MOTOR_STILL - 50);
      
    } else {
      
      Serial.println(" right");
      
      //turn right
      roboclaw.ForwardM1(address,25); //Cmd 0
      roboclaw.BackwardM2(address,25); //Cmd 5
//      myservo1.writeMicroseconds(MOTOR_STILL - 50);
//      myservo2.writeMicroseconds(MOTOR_STILL + 50);
      
    }
  }
  
}


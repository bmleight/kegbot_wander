#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

/* KEGBOT MODES */
const int USER_DRIVE = 1;
const int ROBOT_DRIVE = 2;

RoboClaw roboclaw(15,14);

int speed_left = 0, speed_right = 0;
char commands[256];
int command_index = 0;
int have_command = 0;
int mode = USER_DRIVE;

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial2.begin(9600);
  roboclaw.begin(9600);
}

void loop() {
  readCommand();
  moveRobot();
}

void readCommand() {
  
  int result;
  char character;

  while (Serial2.available()) {
    
//    Serial.println("Serial2 is available");
    
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
    
//    Serial.println(commands);
  }
  
}

void moveRobot() {
    
    int left = 64 - (int)(speed_left * 64 / 100);
    int right = 64 - (int)(speed_right * 64 / 100);
    
//    Serial.print("L: ");
//    Serial.print(left);
//    Serial.print(" R: ");
//    Serial.print(right);
//    Serial.print(" LS: ");
//    Serial.print(speed_left);
//    Serial.print(" RS: ");
//    Serial.println(speed_right);
//    Serial.println(left);
//    Serial.println(right);
    
//    left = right = 0;
    
    roboclaw.ForwardBackwardM1(address, left); //Cmd 0
    roboclaw.ForwardBackwardM2(address, right); //Cmd 5
    
}

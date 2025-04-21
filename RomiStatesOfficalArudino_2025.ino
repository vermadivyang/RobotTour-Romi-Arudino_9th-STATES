//  ---------------------------------------- TO DO ----------------------------------------  //

/*
 *Check Timing with the Delay (for non-jerk)
 *Check the addition of LL & RR
 *Test Backwards Motion
 *Add weight to one side of the robot to stop jerk
 *Test everything with new batteries, prepare scripts for all 3 samples
*/



//   ------------------------------   IMPORTANT LIBARIES   ------------------------------   //
#include <Arduino.h>
#include "Chassis.h"
#include "Romi32U4Buttons.h"

//   ------------------------------   IMPORTANT VARIBLES   ------------------------------   //

// encoder count targets, tune by turning 16 times and changing numbers untill offset is 0
#define NIGHTY_LEFT_TURN_COUNT -725
#define NIGHTY_RIGHT_TURN_COUNT 710

//Adjust the distance with these values (defualt: 50 - AdjustmentDistance)
#define AdjustmentDistance 0.3

// The time the robot waits after a function so the robot doesn't jerk
#define NonJerk 200 
#define NonJerk 200 

//10.16 is the radius of the robot, so the robot travles in the center until the end --(for endDist and startDist)
#define PoleToCenterDistance 10.16


//   ------------------------------   HOW TO CODE   ------------------------------   //

// F and B go forward/backwards 50 cm by default, but other distances can be easily specified by adding a number after the letter
// S and E go the start/end distance
// L and R are left and right
// targetTime is target time (duh)
// Also change the min speed in the Chassis.cpp by the following equation: y = mx + b    | 15.3 cm per sec is the min  |

//   ------------------------------   CHANGE THE FOLLOWING   ------------------------------   //

char moves[200] ="S L F100 R R F200 L F100 L F100 L F R F R F100 R F L L F100 L E";
double targetTime = 81;
double endDist = 100 - PoleToCenterDistance;
double startDist = 25 + PoleToCenterDistance;

//   ------------------------------   CONFIG THE HARDWARE   ------------------------------   //

// Parameters are wheel diam, encoder counts, wheel track (tune these to your own hardware)*
// *Default values of 7, 1440, 14 can't go wrong
Chassis chassis(6.994936972, 1440, 14.0081);

//Define Buttons
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;

//   ------------------------------   ROBOT STATES   ------------------------------   //

// this state machine is not actually useful in any way
enum ROBOT_STATE { ROBOT_IDLE,
                   ROBOT_MOVE,
                   MOVING };
//Set current state
ROBOT_STATE robotState = ROBOT_IDLE;

//   ------------------------------   ROBOT IDLE FUNCTION   ------------------------------   //

// a helper function to stop the motors
void idle(void) {
  Serial.println("idle()");
printBatteryVoltage();
  Serial.println("idle()");
  chassis.idle();
  robotState = ROBOT_IDLE;
}


//   ------------------------------   ROBOT BATTERY VOLTAGE   ------------------------------   //

void printBatteryVoltage() {
  int sensorValue = analogRead(A3); // Read the voltage from the A3 pin
  float voltage = sensorValue * (30.0 / 1024.0);  // Convert to actual voltage
  Serial.println("Battery Voltage: ");
  Serial.println(voltage);  // Print the voltage
  Serial.println(" V");
}


/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */

//   ------------------------------   SETUP   ------------------------------   //

void setup() {
  // This will initialize the Serial at a baud rate of 115200 for prints --115200 is the fastest
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  // Serial1 is used to receive data from K210
  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();


  // these can be undone for the student to adjust
  // tuned like shit, very good numbers to change
  // it's actually a PI controller where first number is P and second is I
  chassis.setMotorPIDcoeffs(5, 0.5);
}


//   ------------------------------   ROBOT TURN FUNCTION   ------------------------------   //

void right(float seconds) {
  chassis.turnWithTimePosPid(NIGHTY_RIGHT_TURN_COUNT, seconds);
}

void left(float seconds) {
  chassis.turnWithTimePosPid(NIGHTY_LEFT_TURN_COUNT, seconds);
}

//   ------------------------------   LOOP   ------------------------------   //

void loop() {
  //Button A = Start Button
  if (buttonA.getSingleDebouncedPress()) {
    delay(500); // wait a little before starting to move so it doesn't hit the pencil or smth idk
    robotState = ROBOT_MOVE;
  }

  //Button B = Voltage Reader Button
  if (buttonB.getSingleDebouncedPress()) {
    delay(50); // transmitting delay
    printBatteryVoltage();
  }

  //Switch States to ROBT_MOVE, so it does stuff
  if (robotState == ROBOT_MOVE) {

    //   ------------------------------   Parseing stuff   ------------------------------   //

    int count = 1; // count the number of moves (turns and straights)
    for (int i = 0; i < strlen(moves); i++)
      if (isSpace(moves[i])) count++;

    // constucts *movesList, each element is pointer to the first character of a move string
    // i.e. if moves is "S R F100 B L E" then *movesList[2] is a pointer to "F" and moveslist[2] is "F100"
    char *movesList[count];
    char *ptr = NULL;

    // tokenize moves with space as delimiter, each token is one move
    byte index = 0;
    ptr = strtok(moves, " ");
    while (ptr != NULL) {
      movesList[index] = ptr;
      index++;
      ptr = strtok(NULL, " ");
    }

    int numTurns = 0;
    int numDrive = 0;
    double totalDist = 0;
    char currentChar;
    String st;

    //   ------------------------------   TIME CALCULATIONS   ------------------------------   //

    // count number of turns and total distance travelled
    // instead of *movesList[i] I could've just done st[0]... but pointers are cool ig
    //Runs Through movesList to figure out total distance and calculates time based on that
    for (int i = 0; i < count; i++) {
      currentChar = *movesList[i];
      st = movesList[i];
      if (currentChar == 'R' || currentChar == 'L') {
        numTurns++;
      }
      if (currentChar == 'RR' || currentChar == 'LL') {
        numTurns++;
        numTurns++;
      }
      else if (currentChar == 'F' || currentChar == 'B') {   
        numDrive++;
        if (st.length() > 1) {
          totalDist += st.substring(1).toDouble();
        } else {
          totalDist += 50;
        }
      } else if (currentChar == 'S') {
        numDrive++;
        totalDist += abs(startDist);
      } else if (currentChar == 'E') {
        numDrive++;
        totalDist += abs(endDist);
      }
    }

    //   ------------------------------   CALCULATED (time) VARIBLES   ------------------------------   //

    double turnTime = 0.6; // target time for a turn is 0.55 seconds
    double totalTurnTime = (turnTime + 0.1 + (NonJerk/1000)) * numTurns; // but the code doesn't work so add 0.1, plus the turnTime and the Wait time for non-jerks
    double totalDriveWaitTime = numDrive * 0.2; // Every time the robot actions, there is a 200 millisecond wait, hence ...
    double totalDriveTime = targetTime - totalTurnTime - totalDriveWaitTime - (0.00297)*totalDist; // this also always went over hence the 0.0029*totalDist
    double dist;

  Serial.println(totalDist);  // Print the distance


    //   ------------------------------   EXECUTION   ------------------------------   //
    //Runs through the movesList for a second time, acually running the code
    for (int i = 0; i < count; i++) {
      currentChar = *movesList[i];
      st = movesList[i];

      //Depending on what charater (i.e. R, L, B, F, S, & E), do different actions 
      //Efficient Switch Case Thingy (based on previous comment)
      switch (currentChar) {
        case 'R':
          right(turnTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'L':
          left(turnTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'LL':
          left(turnTime);
          left(turnTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'RR':
          right(turnTime);
          right(turnTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'B':
          if (st.length() > 1) {dist = ( st.substring(1).toDouble() ) - AdjustmentDistance;} 
          else {dist = 50 - AdjustmentDistance;}
          chassis.driveWithTime(0-dist, dist/totalDist * totalDriveTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'F':
          if (st.length() > 1) {dist = ( st.substring(1).toDouble() ) - AdjustmentDistance;} 
          else {dist = 50 - AdjustmentDistance;}
          chassis.driveWithTime(dist, dist/totalDist * totalDriveTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'S':
          chassis.driveWithTime(startDist, abs(startDist)/totalDist * totalDriveTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

        case 'E':
          chassis.driveWithTime(endDist, abs(endDist)/totalDist * totalDriveTime);
          delay(NonJerk); // Non-Jerk Delay
          break;

      }
    }
    idle(); // go back to idling after finish
  }
}

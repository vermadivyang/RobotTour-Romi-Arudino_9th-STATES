#include "Chassis.h"

float calculateSpeed(float forwardDistance, float targetSeconds, float elapsedSeconds, float minSpeed = 8) { //YOU WANT TO CHANGE THIS AS WELL
   float speedMultiplier = 1.05;
  if (forwardDistance < 0) speedMultiplier = -1.05;
  else if (forwardDistance >= 0 && forwardDistance < 50) speedMultiplier = 1.08;
  else if (forwardDistance >= 50 && forwardDistance < 100) speedMultiplier = 1.06;
  else if (forwardDistance >= 100 && forwardDistance < 150) speedMultiplier = 1.06;
  else if (forwardDistance >= 150 && forwardDistance < 200) speedMultiplier = 1.05;

  float percentComplete = elapsedSeconds / targetSeconds;
  float delta = (fabs(forwardDistance) - minSpeed * targetSeconds) / (0.6 * targetSeconds); // diff between max and min speed
  float s = delta + minSpeed; // s is max speed if it's not at beginning or end

  // if in first or last 40%, calculate speed using trapezoidal motion profile
  if (percentComplete < 0.4) {
    s = delta * percentComplete / 0.4 + minSpeed;
  } else if (percentComplete > 0.6) {
    s = delta * (1 - percentComplete) / 0.4 + minSpeed;
  } else if (percentComplete > 1) s = minSpeed;
  if (percentComplete > 0.98){Serial.println(percentComplete);}
  return s * speedMultiplier;  // tends to be a little slow, friction and stuff ig
   
}
/*
float calculateSpeed(float forwardDistance, float targetSeconds, float elapsedSeconds, float minSpeed = 7) { //YOU WANT TO CHANGE THIS AS WELL
  double PercentOfTrap = 0.3;
  float speedMultiplier = 1.05;
  float MAXpercent = (0.021 * forwardDistance) + 0.51;
  if (forwardDistance < 0) speedMultiplier = -1.05;

  float percentComplete = elapsedSeconds / targetSeconds;
  float delta = (fabs(forwardDistance) - minSpeed * targetSeconds) / ( targetSeconds); // diff between max and min speed
  float s = delta + minSpeed; // s is max speed if it's not at beginning or end

  // if in first or last 40%, calculate speed using trapezoidal motion profile
  if (percentComplete < PercentOfTrap) {
   // s = (delta * percentComplete / PercentOfTrap) + minSpeed;
    s = (delta * (percentComplete/PercentOfTrap)) + minSpeed;
  } else if (percentComplete > (1-PercentOfTrap)) {
    //s = (delta * (1 - percentComplete) / PercentOfTrap) + minSpeed;
    if ((delta * ((1-percentComplete)/PercentOfTrap)) < 0) {s = minSpeed;}
    else {s = (delta * ((1-percentComplete)/PercentOfTrap)) + minSpeed;}
      //Serial.println(s);
  }
  Serial.println(s);
  Serial.println(percentComplete);
  
      
  float RealS = s * speedMultiplier;
  return RealS;  // tends to be a little slow, friction and stuff ig
}//*/

int calculateIntermediateTargetLinear(int target, float finishSeconds, float elapsedSeconds) {
  if (elapsedSeconds >= finishSeconds) return target;
  return int(elapsedSeconds * (float(target) / finishSeconds));
}


void Chassis::init(void) {
  Romi32U4Motor::init();

  // temporarily turn off interrupts while we set the time up
  noInterrupts();

  // sets up timer 4 for a 16 ms loop, which triggers the motor PID controller
  // dt = 1024 (prescaler) * (249 + 1) / 16E6 (clock speed) = 16 ms
  TCCR4A = 0x00;  //disable some functionality
  TCCR4B = 0x0B;  //sets the prescaler to 1024
  TCCR4C = 0x00;  //disable outputs (overridden for the servo class)
  TCCR4D = 0x00;  //fast PWM mode (for servo)

  OCR4C = 249;  //TOP goes in OCR4C

  TIMSK4 = 0x04;  //enable overflow interrupt

  // re-enable interrupts
  interrupts();
}

void Chassis::setMotorPIDcoeffs(float kp, float ki) {
  leftMotor.setSpeedPIDCoeffients(kp, ki);
  rightMotor.setSpeedPIDCoeffients(kp, ki);
}


void Chassis::idle(void) {
  setMotorEfforts(0, 0);
}


void Chassis::setMotorEfforts(int leftEffort, int rightEffort) {
  leftMotor.setMotorEffort(leftEffort);
  rightMotor.setMotorEffort(rightEffort);
}

void Chassis::setWheelSpeeds(float leftSpeed, float rightSpeed) {
  int16_t leftTicksPerInterval = (leftSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;
  int16_t rightTicksPerInterval = (rightSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;

  leftMotor.setTargetSpeed(leftTicksPerInterval);
  rightMotor.setTargetSpeed(rightTicksPerInterval);
}

void Chassis::setTwist(float forwardSpeed, float turningSpeed) {
  int16_t ticksPerIntervalFwd = (forwardSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;
  int16_t ticksPerIntervalTurn = (robotRadius * 3.14 / 180.0) * (turningSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;

  leftMotor.setTargetSpeed(ticksPerIntervalFwd - ticksPerIntervalTurn);
  rightMotor.setTargetSpeed(ticksPerIntervalFwd + ticksPerIntervalTurn);
}

void Chassis::driveFor(float forwardDistance, float forwardSpeed, bool block) {
  // ensure the speed and distance are in the same direction
  forwardSpeed = forwardDistance > 0 ? fabs(forwardSpeed) : -fabs(forwardSpeed);
  setTwist(forwardSpeed, 0);  //sets the speeds

  // calculate the total motion in encoder ticks
  long delta = forwardDistance / cmPerEncoderTick;

  // set both wheels to move the same amount
  leftMotor.moveFor(delta);
  rightMotor.moveFor(delta);

  if (block) {
    while (!checkMotionComplete()) { delay(1); }
  }
}

void Chassis::driveWithTime(float forwardDistance, float targetSeconds) {
  unsigned long startTime = millis();
  float targetSeconds2 = targetSeconds;
  float forwardSpeed = calculateSpeed(forwardDistance, targetSeconds2, 0);
  setTwist(forwardSpeed, 0);
  // calculate the total motion in encoder ticks
  long delta = forwardDistance / cmPerEncoderTick;
  // set both wheels to move the same amount
  leftMotor.moveFor(delta);
  rightMotor.moveFor(delta);

  while (true) {
    delay(1);
    if (checkMotionComplete()) break;
    float elapsedSeconds = (millis() - startTime) / 1000.0;
    forwardSpeed = calculateSpeed(forwardDistance, targetSeconds2, elapsedSeconds);
    int ticksPerIntervalFwd = (forwardSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;
    leftMotor.targetSpeed = ticksPerIntervalFwd;
    rightMotor.targetSpeed = ticksPerIntervalFwd;
  }
  
  delay(1);
}

void Chassis::turnFor(float turnAngle, float turningSpeed, bool block) {
  // ensure angle and speed are in the same direction
  turningSpeed = turnAngle > 0 ? fabs(turningSpeed) : -fabs(turningSpeed);
  setTwist(0, turningSpeed);

  // calculate the total motion in encoder ticks
  long delta = turnAngle * (robotRadius * 3.14 / 180.0) / cmPerEncoderTick;

  // set wheels to drive in opposite directions
  leftMotor.moveFor(-delta);
  rightMotor.moveFor(delta);

  if (block) {
    while (!checkMotionComplete()) { delay(1); }
  }
}

void Chassis::turnWithTimePosPid(int targetCount, float targetSeconds) {
  unsigned long startTime = millis();
  targetSeconds = targetSeconds;
  leftMotor.setTargetCount(0);
  rightMotor.setTargetCount(0);
  while (true) {
    delay(1);
    float elapsedSeconds = (millis() - startTime) / 1000.0;
    int thisTarget = calculateIntermediateTargetLinear(targetCount, targetSeconds - 0.1, elapsedSeconds);
    leftMotor.targetCount = thisTarget;
    rightMotor.targetCount = -thisTarget;
    if (elapsedSeconds > targetSeconds)
      break;
  }
  delay(100);
  setMotorEfforts(0, 0);
}

bool Chassis::checkMotionComplete(void) {
  bool complete = leftMotor.checkComplete() && rightMotor.checkComplete();
  return complete;
}


ISR(TIMER4_OVF_vect) {
  chassis.updateEncoderDeltas();
}

void Chassis::updateEncoderDeltas(void) {
  leftMotor.calcEncoderDelta();
  rightMotor.calcEncoderDelta();

  leftMotor.update();
  rightMotor.update();
}

void Chassis::printSpeeds(void) {
  Serial.print(leftMotor.speed);
  Serial.print('\t');
  Serial.print(rightMotor.speed);
  Serial.print('\n');
}

void Chassis::printEncoderCounts(void) {
  Serial.print(leftMotor.getCount());
  Serial.print('\t');
  Serial.print(rightMotor.getCount());
  Serial.print('\n');
}

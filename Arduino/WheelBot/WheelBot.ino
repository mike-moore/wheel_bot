#include "RobotState.h"
#include "ProtobuffSerial.h"
#include "CommandAndDataHandler.h"
#include "Navigation.h"
#include "Guidance.h"
#include "Control.h"

// - Initialize an instance of the robot's state registry
RobotState robotState;
// - Initialize an instance of the protbuff serial class to do communication
ProtobuffSerial serialComm;
// - Initialize an instance of the command and data handler
CommandAndDataHandler cmdAndDataHandler(serialComm.Commands, serialComm.Telemetry, robotState);
// - Initiliaze an instance of the robot navigation
Navigation navigation(robotState);
// - Initiliaze an instance of the robot guidance
Guidance guidance(robotState);
// - Initiliaze an instance of the robot control
Control control(robotState);

const long cycleTimeMillis = 10;
unsigned long previousMillis = 0;

// - Use volatile and global because encoder counts are modified in an
//   interrupt service routine
volatile long int mtrR_encoderCount = 0;
volatile long int mtrL_encoderCount = 0;
long mtrR_encoderCountPrev = 0;
long mtrL_encoderCountPrev = 0;

// - Motor speeds. Computed here due to use of interrupts
int mtrR_speed = 0; int mtrR_speed_prev = 0;
int mtrL_speed = 0; int mtrL_speed_prev = 0;

bool mtrL_A_set, mtrL_B_set;
bool mtrR_A_set, mtrR_B_set;

void setup(){
  Serial.begin(57600);
  // - Serial comm init
  serialComm.InitHw();
  // - Init sensors
  navigation.InitSensors();
  // - Very Important: attach interrupt service routines for motor encoders
  setup_encoders();
}

void setup_encoders(){
  digitalWrite(mtrR_encoderPinA, HIGH);
  digitalWrite(mtrR_encoderPinB, HIGH);
  digitalWrite(mtrL_encoderPinA, HIGH);
  digitalWrite(mtrL_encoderPinB, HIGH);
  attachInterrupt(digitalPinToInterrupt(mtrR_encoderPinA), ISR_mtrR_encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mtrR_encoderPinB), ISR_mtrR_encoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mtrL_encoderPinA), ISR_mtrL_encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mtrL_encoderPinB), ISR_mtrL_encoderB, CHANGE);
}

void loop(){
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= cycleTimeMillis) {
    /// - Save off current millis. Used for control loop timing.
    previousMillis = currentMillis;
    /// - Read commands from the serial port.
    serialComm.Rx();
    /// - Forward received commands on to C&DH
    if (serialComm.NewCommandsArrived()){
      cmdAndDataHandler.ProcessCmds();
    }
    /// Execute the robot navigation
    navigation.Execute();
    /// Execute the robot guidance
    guidance.Execute();
    /// Execute the robot control logic
    computeMotorSpeeds(); 
    control.Execute();
    /// Have C&DH prepare the robot telemetry for transmission
    cmdAndDataHandler.LoadTelemetry();
    /// - Send the telemetry over the serial port
    serialComm.Tx();
  }
  
}

void computeMotorSpeeds(){
  computeRightMotorSpeed();
  computeLeftMotorSpeed();
}

void computeRightMotorSpeed()  {                                                    
 mtrR_speed = ((mtrR_encoderCount - mtrR_encoderCountPrev)*(60*(1000/cycleTimeMillis)))/(360);          // 360 counts per output shaft rev
 float alpha = 0.3;
 mtrR_speed = (1-alpha)*mtrR_speed + alpha*mtrR_speed_prev;
 mtrR_speed_prev = mtrR_speed;
 mtrR_encoderCountPrev = mtrR_encoderCount;                  
}

void computeLeftMotorSpeed()  {                                                    
 mtrL_speed = ((mtrL_encoderCount - mtrL_encoderCountPrev)*(60*(1000/cycleTimeMillis)))/(360);          // 360 counts per output shaft rev
 float alpha = 0.3;
 mtrL_speed = (1-alpha)*mtrL_speed + alpha*mtrL_speed_prev;
 mtrL_speed_prev = mtrL_speed;
 mtrL_encoderCountPrev = mtrL_encoderCount;                  
}

// - Interrupt Service Routines for left and right motors
void ISR_mtrR_encoderA(){
    mtrR_A_set = digitalRead(mtrR_encoderPinA) == HIGH;
    mtrR_encoderCount += (mtrR_A_set != mtrR_B_set) ? +1 : -1;
}

void ISR_mtrR_encoderB(){
    mtrR_B_set = digitalRead(mtrR_encoderPinB) == HIGH;
    mtrR_encoderCount += (mtrR_A_set == mtrR_B_set) ? +1 : -1;
}

void ISR_mtrL_encoderA(){
    mtrL_A_set = digitalRead(mtrL_encoderPinA) == HIGH;
    mtrL_encoderCount += (mtrL_A_set != mtrL_B_set) ? +1 : -1;
}

void ISR_mtrL_encoderB(){
    mtrL_B_set = digitalRead(mtrL_encoderPinB) == HIGH;
    mtrL_encoderCount += (mtrL_A_set == mtrL_B_set) ? +1 : -1;
}

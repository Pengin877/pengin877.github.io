//***3RPS Parallel Manipulator Ball Balancer Code BY Aaed Musa**
//--------------------------------------------------------------

//libraries
#include <AccelStepper.h>
#include <InverseKinematics.h>
#include <MultiStepper.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <math.h>

Machine machine(2, 3.125, 1.75, 3.669291339);     //(d, e, f, g) object to define the lengths of the machine
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);  //touch screen pins (XGND, YGND, X5V, Y5V)

//stepper motors
AccelStepper stepperA(1, 1, 2);  //(driver type, STEP, DIR) Driver A
AccelStepper stepperB(1, 3, 4);  //(driver type, STEP, DIR) Driver B
AccelStepper stepperC(1, 5, 6);  //(driver type, STEP, DIR) Driver C
MultiStepper steppers;           // Create instance of MultiStepper

//stepper motor variables
int pos[3];                                            // An array to store the target positions for each stepper motor
int ENA = 0;                                           //enable pin for the drivers
double angOrig = 206.662752199;                        //original angle that each leg starts at
double speed[3] = { 0, 0, 0 }, speedPrev[3], ks = 20;  //the speed of the stepper motor and the speed amplifying constant

//touch screen variables
double Xoffset = 495;  //X offset for the center position of the touchpad
double Yoffset = 500;  //Y offset for the center position of the touchpad

//LQR variables
double K[2][4] = {
  {0.0016, 0, 0.0051, 0},
  {0, 0.0016, 0, 0.0051}
};
double out[2];
double pos_b[2] = {0,0};
double vel_b[2] = {0,0};
double posPrev_b[2] = {0,0};
long timeI;                                                                           //variables to capture initial times

//Other Variables
double angToStep = 3200 / 360;  //angle to step conversion factor (steps per degree) for 16 microsteps or 3200 steps/rev
bool detected = 0;              //this value is 1 when the ball is detected and the value is 0 when the ball in not detected

void setup() {
  Serial.begin(115200);
  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  //Enable pin
  pinMode(ENA, OUTPUT);           //define enable pin as output
  digitalWrite(ENA, HIGH);        //sets the drivers off initially
  delay(1000);                    //small delay to allow the user to reset the platform
  digitalWrite(ENA, LOW);         //sets the drivers on
  moveTo(4.25, 0, 0);             //moves the platform to the home position
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
}
void loop() {
  LQR(0, 0);  //(X setpoint, Y setpoint) -- must be looped
}
//moves/positions the platform with the given parameters
void moveTo(double hz, double nx, double ny) {
  //if the ball has been detected
  if (detected) {
    //calculates stepper motor positon
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep);
    }
    //sets calculated speed
    stepperA.setMaxSpeed(speed[A]);
    stepperB.setMaxSpeed(speed[B]);
    stepperC.setMaxSpeed(speed[C]);
    //sets acceleration to be proportional to speed
    stepperA.setAcceleration(speed[A] * 30);
    stepperB.setAcceleration(speed[B] * 30);
    stepperC.setAcceleration(speed[C] * 30);
    //sets target positions
    stepperA.moveTo(pos[A]);
    stepperB.moveTo(pos[B]);
    stepperC.moveTo(pos[C]);
    //runs stepper to target position (increments at most 1 step per call)
    stepperA.run();
    stepperB.run();
    stepperC.run();
  }
  //if the hasn't been detected
  else {
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep);
    }
    //sets max speed
    stepperA.setMaxSpeed(800);
    stepperB.setMaxSpeed(800);
    stepperC.setMaxSpeed(800);
    //moves the stepper motors
    steppers.moveTo(pos);
    steppers.run();  //runs stepper to target position (increments at most 1 step per call)
  }
}
//takes in an X and Y setpoint/position and moves the ball to that position
void LQR(double setpointX, double setpointY){
  TSPoint p = ts.getPoint();
  if(p.x != 0){
    Serial.println((String) "px=" + p.x + " py=" + p.y);
    detected = 1;
    // Update state
    posPrev_b[0] = pos_b[0];
    posPrev_b[1] = pos_b[1];
    pos_b[0] = Xoffset - p.x - setpointX;
    pos_b[1] = Yoffset - p.y - setpointY;
    // Finite-difference velocity (same dt as your loop: 20ms)
    vel_b[0] = 0.8 * vel_b[0] + 0.2 * (pos_b[0] - posPrev_b[0]) / 0.02;
    vel_b[1] = 0.8 * vel_b[1] + 0.2 * (pos_b[1] - posPrev_b[1]) / 0.02;
    if (isnan(vel_b[0]) || isinf(vel_b[0])) vel_b[0] = 0;
    if (isnan(vel_b[1]) || isinf(vel_b[1])) vel_b[1] = 0;

    double state[4] = { pos_b[0], pos_b[1], vel_b[0], vel_b[1] };

    // u = -K * state
    out[0] = 0; out[1] = 0;
    for (int j = 0; j < 4; j++) {
      out[0] -= K[0][j] * state[j];
      out[1] -= K[1][j] * state[j];
    }

    out[0] = constrain(out[0], -0.25, 0.25);
    out[1] = constrain(out[1], -0.25, 0.25);

    for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];
      speed[i] = (i == A) * stepperA.currentPosition() + (i == B) * stepperB.currentPosition() + (i == C) * stepperC.currentPosition();
      speed[i] = abs(speed[i] - pos[i]) * ks;
      speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200);
      speed[i] = constrain(speed[i], 0, 1000);
    }
  }
  else { 
    //double check that there is no ball
    delay(10);                  //10 millis delay before another reading
    TSPoint p = ts.getPoint();  //measure X and Y positions again to confirm no ball
    if (p.x == 0) {             //if the ball is still not detected
      //Serial.println("BALL NOT DETECTED");
      detected = 0;
    } 
  }

  timeI = millis();
  while (millis() - timeI < 20) {
    moveTo(4.25, -out[0], out[1]);
  }

}
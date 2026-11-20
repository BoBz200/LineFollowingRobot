/* ************************************************************************************************* */
// UCSD ECE 5 Lab 4 Code: Line Following Robot with PID 
// V 4.0
// Last Modified 5/14/2025 by MingWei Yeoh, Korey Huynh, and Karcher Morris
/* ************************************************************************************************* */

// Include files needed
#include <L298NX2.h> // Using "L298N" library found through arduino library manager developed by Andrea Lombardo (https://github.com/AndreaLombardo/L298N)

// ************************************************************************************************* //
// ************************************************************************************************* //
// Change Robot Settings here

#define PRINTALLDATA        1  // Turn to 1  to prints ALL the data when changed to 1, Could be useful for debugging =)
                                // !! Turn to 0 when running robot untethered
#define NOMINALSPEED        80 // This is the base speed for both motors, can also be increased by using potentiometers

// ************************************************************************************************* //

// Taken from LEFT TO RIGHT of the robot ****** Orient yourself so that you are looking from the rear of the robot (photoresistors are farthest away from you, wheels are closest to you)
//                  Left Motors   Right motors 
L298NX2 DriveMotors(  42, 41, 40,      37, 39, 38);
//                 ENA, IN1, IN2, ENB, IN3, IN4

enum Side {LEFT, RIGHT};

int LDR_Pin[] = {1,2,3,4,5,6,7};
constexpr int num_photoresistors = sizeof(LDR_Pin) / sizeof(LDR_Pin[0]);

// Potentiometer Pins
#define S_pin 13 // Pin connected to Speed potentiometer
#define P_pin 12 // Pin connected to P term potentiometer
#define I_pin 11 // Pin connected to I term potentiometer
#define D_pin 10 // Pin connected to D term potentiometer

int led_pins[] = {46, 17};
constexpr int num_leds = sizeof(led_pins) / sizeof(led_pins[0]); 

// Variables for Calibration and Error Calculation
float min_values[num_photoresistors] = {}; 
float max_values[num_photoresistors] = {};

// For Motor Control
float lasterror = 0, sumerror = 0;



// ************************************************************************************************* //
// setup - runs once
void setup() {
  Serial.begin(9600);                            // For serial communication set up

  for (int i = 0; i < num_leds; i++)
    pinMode(led_pins[i], OUTPUT);                // Initialize all LEDs to output
  
  calitbrate_photoresistors();                                   // Calibrate black and white sensing

} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {
  int Sp = 0; // speed increase
  float kP = 0; // proportional gain
  float kI = 0; // integral gain
  float kD = 0; // derivative gain

  get_potentiometer_data(&Sp, &kP, &kI, &kD); // Read potentiometers

  int data[num_photoresistors] = {};
  get_photoresistor_data(data); // Read photoresistors 

  int error = calculate_error(data);          // Calculates error

  int turn = get_pid_turn(error, kP, kI, kD);           // PID Control and Output to motors to turn

  int motor_speeds[2] = {};
  calculate_motor_values(turn, Sp, motor_speeds);          // Runs motors
  run_motors(motor_speeds);
  
  if (PRINTALLDATA)     // If PRINTALLDATA Enabled, Print all the data
    Print(Sp, kP, kI, kD, data, error, motor_speeds);
  
} // end loop()

// ************************************************************************************************* //
// function to calibrate

void calitbrate_photoresistors() {

  int num_mesurements = 20;                // set number Of Measurements to take

  calitbrate_photoresistors_helper(num_mesurements, false); // White Calibration

  set_leds(0);                                   // Turn off LEDs to indicate user to calibrate other color
  delay(2000);
  
  calitbrate_photoresistors_helper(num_mesurements, true);  // Black Calibration

  Serial.print("White Vals:  ");
  for (int i = 0; i < num_photoresistors; i++)
    Serial.print(String(min_values[i]) + " ");          // Print the White values that will be used by the robot
  Serial.println();

  Serial.print("Black Vals:  ");
  for (int i = 0; i < num_photoresistors; i++)
    Serial.print(String(max_values[i]) + " ");          // Print the Black values that will be used by the robot
  Serial.println();

  Serial.print("Delta Vals:  ");
  for (int i = 0; i < num_photoresistors; i++)
    Serial.print(String(max_values[i] - min_values[i]) + " ");  // Print the Difference between the White and Black valuess
  Serial.println();

  set_leds(1);                                   // Turn LEDs on
  delay(2000);

} // end calitbrate_photoresistors()

void calitbrate_photoresistors_helper(int num_measurements, boolean is_calibrating_black) {
  if (is_calibrating_black)
    Serial.println("\nCalibrating Black");
  else
    Serial.println("\nCalibrating White");
// Indicate that calibration is starting
  for (int i = 0; i < 4; i++) {
      set_leds(1); // turn the LEDs on
      delay(250); // wait
      set_leds(0); // turn the LEDs off
      delay(250); // wait 
  }
  
  set_leds(1);
  delay(250);

  float light_data[num_photoresistors];
  for (int i = 0; i < num_measurements; i++) {
    for (int pin = 0; pin < num_photoresistors; pin++) {
      light_data[pin] += (float)analogRead(LDR_Pin[pin]);
      delay(2);
    }
    Serial.print(". ");
  }

  for (int pin = 0; pin < num_photoresistors; pin++) {
    if (is_calibrating_black) {                                   // updating cooresponding array based on if we are calibrating black or white
      max_values[pin] = round(light_data[pin] / (float)num_measurements); // take average and store for black
    }
    else {
      min_values[pin] = round(light_data[pin] / (float)num_measurements); // take average and store for white
    }
    light_data[pin] = 0.0;
  }

  Serial.println(" Done!");
  set_leds(0);
  delay(250);
}

// Set all LEDs to a certain brightness
void set_leds(int x) {
  for (int i = 0; i < num_leds; i++)
    digitalWrite(led_pins[i], x);
}

// **********Recall your Challenge #1 Code********************************************************************** //
// function to read and map values from potentiometers
void get_potentiometer_data(int* Sp_out, float* kP_out, float* kI_out, float* kD_out) {
  *Sp_out = read_potentiometer(S_pin, 0, 4095, 0, 100);
  *kP_out = read_potentiometer(P_pin, 0, 4095, 0, 100);
  *kI_out = read_potentiometer(I_pin, 0, 4095, 0, 100);
  *kD_out = read_potentiometer(D_pin, 0, 4095, 0, 100);

  *kP_out = (float)*kP_out * 1.0;
  *kI_out = (float)*kI_out * 0.001;
  *kD_out = (float)*kD_out * 0.01;

} // end get_potentiometer_data()

inline int read_potentiometer(int pin, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
  return map(analogRead(pin), min_resolution, max_resolution, min_potentiometer, max_potentiometer); 
}

// **********Recall your Challenge #2 Code********************************************************************** //
// Function to read photo resistors and map from 0 to 100
void get_photoresistor_data(int* photoresistor_data_out) {
  for (int i = 0; i < num_photoresistors; i++) { 
    int raw_photoresistor_data = analogRead(LDR_Pin[i]);
    photoresistor_data_out[i] = map(raw_photoresistor_data, min_values[i], max_values[i], 0, 100);
  }
} // end get_photoresistor_data()

// ************************************************************************************************* //
// Calculate error from photoresistor readings
float calculate_error(int light_data[num_photoresistors]) {
  int highest_value = -99, highest_value_index = -1;
  float average_value = 0.0;
  for (int i = 0; i < num_photoresistors; i++) {
    if (highest_value < light_data[i]) {
      highest_value = light_data[i];
      highest_value_index = (float)i;
    }
    average_value += (float)light_data[i] / (float)num_photoresistors;
  }

  int max_criteria = 1.5;
  int left_highest_index = -1, right_highest_index = -1;
  if (highest_value > max_criteria * average_value) { // Make sure that the highest_value_index is actually "seeing" a line. What happens if there is no line and we take the photoresistor that happens to have the highest value?

    // Next we assign variables to hold the index of the left and right Photoresistor that has the highest value, though we have to make sure that we aren't checking a Photoresistor that doesn't exist.
    // Ex: To the left of the left most photoresistor or the right of the right most photoresistor
    if (highest_value_index != 0)
      left_highest_index = highest_value_index - 1;
    else
      left_highest_index = highest_value_index;

    if (highest_value_index != num_photoresistors - 1)
      right_highest_index = highest_value_index + 1;
    else
      right_highest_index = highest_value_index;

    // Next we take the percentage of "line" each of our left, middle, and right photoresistors sees and then we take the average, which is our error calculation
    float numerator = (float)(light_data[left_highest_index] * left_highest_index) + (float)(light_data[highest_value_index] * highest_value_index) + (float)(light_data[right_highest_index] * right_highest_index);
    float denominator = (float)light_data[left_highest_index] + (float)light_data[highest_value_index] + (float)light_data[right_highest_index];

    float weighted_average = ((float)numerator) / denominator;

    float error = (weighted_average - num_photoresistors/2.0);
    return error;
  }
  return lasterror;
} // end calculate_error()

// ************************************************************************************************* //
// PID Function
int get_pid_turn(float error, float kP, float kI, float kD) {

  int turn = error * kP + sumerror * kI + (error - lasterror) * kD;

  if (sumerror > 5)   // prevents integrator wind-up
    sumerror = 5;
  else if (sumerror < -5)
    sumerror = -5;

  if (error == 0)     // Reset sumerror if line is centered
    sumerror = 0;

  lasterror = error;
  sumerror = sumerror + error;

  return turn;
} // end get_pid_turn()

void calculate_motor_values(int turn, int speed, int* motor_speeds_out) {
  int M1P = 0, M2P = 0; // NOTE: change at some point
  if (turn < 0) {
    M1P = -turn;
    M2P = turn;
  }
  else if (turn > 0) {
    M1P = -turn;
    M2P = turn;
  }
  else {
    M1P = 0;
    M2P = 0;
  }

  motor_speeds_out[0] = min(NOMINALSPEED + speed + M1P, 255);
  motor_speeds_out[1] = min(NOMINALSPEED + speed + M2P, 255);
}


// **********Recall your Challenge #3 Code********************************************************************** //
// function to start motors using nominal speed + speed addition from potentiometer
void run_motors(int motor_speeds[2]) {
  run_motor(LEFT, motor_speeds[1]);
  run_motor(RIGHT, motor_speeds[0]);
} // end run_motors()

// A function that commands a specified motor to move towards a given direction at a given speed
void run_motor(Side side, int speed) {
  if (side == LEFT) {
    DriveMotors.setSpeedA(abs(speed));
    if (speed > 0)                // swap direction if speed is negative
      DriveMotors.forwardA();           // sets the direction of the motor from arguments
    else
      DriveMotors.backwardA();          // sets the direction of the motor from arguments
  }
  if (side == RIGHT) {
    DriveMotors.setSpeedB(abs(speed));
    if (speed > 0)                // swap direction if speed is negative
      DriveMotors.backwardB();           // sets the direction of the motor from arguments
    else
      DriveMotors.forwardB();          // sets the direction of the motor from arguments
  }
}

// ************************************************************************************************* //
// function to print values of interest
void Print(int Sp, float kP, float kI, float kD, int light_data[num_photoresistors], float error, int motor_speeds[2]) {
  Serial.print(" Sp: " + String(Sp) + " P: " + String(kP) + " I: " + String(kI) + " D: " + String(kD) + "  PResistor Val : "); // Prints PID settings

  for (int i = 0; i < num_photoresistors; i++) { // Printing the photo resistor reading values one by one
    Serial.print(light_data[i]);
    //Serial.print(raw_photoresistors_data[i]); //Uncomment this if you would prefer to see raw photoresistor readings
    Serial.print(" ");
  }

  Serial.print(" Error: " + String(error));      // this will show the calculated error (-3 through 3)

  Serial.println("  LMotor:  " + String(motor_speeds[0]) + "  RMotor:  " + String(motor_speeds[1]));    // This prints the arduino output to each motor so you can see what the values are (0-255)
  set_leds(0); 
  delay(100);                                    // just here to slow down the output for easier reading. Don't comment out or else it'll slow down the processor on the arduino
  set_leds(1); 
  delay(100); 

} // end Print()

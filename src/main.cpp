#include <AccelStepper.h>
#include <Bounce2.h>

// Motor Control Pins
const int PIN_CUT_MOTOR_PUL = 11;
const int PIN_CUT_MOTOR_DIR = 12;
const int PIN_POSITION_MOTOR_PUL = 5;
const int PIN_POSITION_MOTOR_DIR = 6;

// Position Switch Pins
const int PIN_CUT_MOTOR_POSITION_SWITCH = 7;
const int PIN_POSITION_MOTOR_POSITION_SWITCH = 8;

// Debounce time in milliseconds
const int DEBOUNCE_TIME = 10;

// Run Cycle Switch Pin
const int PIN_RUN_CYCLE_SWITCH = 9;

// Add reload switch pin definition at the top with other pins
const int PIN_RELOAD_SWITCH = 10;

// Create stepper instances
AccelStepper cutMotor(AccelStepper::DRIVER, PIN_CUT_MOTOR_PUL, PIN_CUT_MOTOR_DIR);
AccelStepper positionMotor(AccelStepper::DRIVER, PIN_POSITION_MOTOR_PUL, PIN_POSITION_MOTOR_DIR);

// Create Bounce instances for switches
Bounce cutSwitch = Bounce();
Bounce positionSwitch = Bounce();
Bounce runCycleSwitch = Bounce();
Bounce reloadSwitch = Bounce();

// Switch configuration constants
const unsigned long SWITCH_DEBOUNCE_TIME = 20;    // ms
const int STEPS_PER_INCH = 2000;
const float CUT_MOTOR_TRAVEL = 9.5;              // inches
const float POSITION_MOTOR_TRAVEL = 3.3;         // inches
const unsigned long CLAMP_OPERATION_DELAY = 100;  // ms

// Motor speeds
const float CUT_MOTOR_SPEED = 2000;        // Normal speed
const float CUT_MOTOR_RETURN_SPEED = 50000;  // Return speed
const float POSITION_MOTOR_SPEED = 5000;    // Normal speed
const float POSITION_MOTOR_RETURN_SPEED = 3000;  // Return speed

// Motor accelerations
const float CUT_MOTOR_ACCEL = 10000;
const float POSITION_MOTOR_ACCEL = 5000;

// Function declaration (prototype)
void homeMotors();

// Clamp pins
const int PIN_POSITION_CLAMP = 3;
const int PIN_SECURE_WOOD_CLAMP = 4;

void setup() {
    // Configure motors with initial speeds and accelerations
    cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
    
    positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
    positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
    
    // Configure clamp pins as outputs and ensure they start disengaged
    pinMode(PIN_POSITION_CLAMP, OUTPUT);
    pinMode(PIN_SECURE_WOOD_CLAMP, OUTPUT);
    digitalWrite(PIN_POSITION_CLAMP, LOW);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, LOW);
    
    // Configure switches with debouncing
    pinMode(PIN_CUT_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
    pinMode(PIN_POSITION_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
    pinMode(PIN_RUN_CYCLE_SWITCH, INPUT_PULLUP);
    
    cutSwitch.attach(PIN_CUT_MOTOR_POSITION_SWITCH);
    cutSwitch.interval(DEBOUNCE_TIME);
    
    positionSwitch.attach(PIN_POSITION_MOTOR_POSITION_SWITCH);
    positionSwitch.interval(DEBOUNCE_TIME);
    
    runCycleSwitch.attach(PIN_RUN_CYCLE_SWITCH);
    runCycleSwitch.interval(SWITCH_DEBOUNCE_TIME);
    
    // Configure reload switch
    pinMode(PIN_RELOAD_SWITCH, INPUT_PULLUP);
    reloadSwitch.attach(PIN_RELOAD_SWITCH);
    reloadSwitch.interval(SWITCH_DEBOUNCE_TIME);
    
    // Start serial for monitoring
    Serial.begin(115200);
    Serial.println("Starting homing sequence...");
    
    // Ensure clamps are disengaged before homing
    delay(CLAMP_OPERATION_DELAY);
    
    // Home both motors
    homeMotors();
}

// Function definition (implementation)
void homeMotors() {
    Serial.println("Homing both motors...");
    
    // Move both motors until their respective switches are triggered
    while (true) {
        cutSwitch.update();
        positionSwitch.update();
        
        // Move cut motor if not at home
        if (cutSwitch.read() == HIGH) {
            cutMotor.moveTo(10000);
            cutMotor.run();
        } else {
            cutMotor.stop();
            cutMotor.setCurrentPosition(0);
        }
        
        // Move position motor if not at home
        if (positionSwitch.read() == HIGH) {
            positionMotor.moveTo(10000);
            positionMotor.run();
        } else {
            positionMotor.stop();
            positionMotor.setCurrentPosition(0);
        }
        
        // Exit when both motors are homed
        if (cutSwitch.read() == LOW && positionSwitch.read() == LOW) {
            break;
        }
    }
    
    Serial.println("Both motors homed");
}

void loop() {
    runCycleSwitch.update();
    reloadSwitch.update();
    
    // Check for reload switch after cycle completion
    if (reloadSwitch.rose() && positionMotor.currentPosition() < 0) {
        // Disengage both clamps
        digitalWrite(PIN_SECURE_WOOD_CLAMP, LOW);
        digitalWrite(PIN_POSITION_CLAMP, LOW);
        delay(CLAMP_OPERATION_DELAY);
        return;
    }
    
    // Double-check switch state before starting a new cycle
    if (runCycleSwitch.read() == HIGH) {
        delay(SWITCH_DEBOUNCE_TIME);
        runCycleSwitch.update();
        if (runCycleSwitch.read() == LOW) {
            return;  // Exit if switch is no longer active
        }
        
        // Engage both clamps before cycle begins
        digitalWrite(PIN_SECURE_WOOD_CLAMP, HIGH);
        digitalWrite(PIN_POSITION_CLAMP, HIGH);
        delay(CLAMP_OPERATION_DELAY);
        
        // Set cut motor to normal speed for outward movement
        cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
        
        // Move cut motor negative 9.5 inches
        long cutTargetPosition = -STEPS_PER_INCH * CUT_MOTOR_TRAVEL;
        cutMotor.moveTo(cutTargetPosition);
        
        while (cutMotor.distanceToGo() != 0) {
            cutMotor.run();
        }
        
        delay(CLAMP_OPERATION_DELAY);
        
        // Disengage position clamp before homing
        digitalWrite(PIN_POSITION_CLAMP, LOW);
        delay(CLAMP_OPERATION_DELAY);
        
        // Set return speeds for both motors
        cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
        positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
        
        // Return both motors to home position simultaneously
        cutMotor.moveTo(0);
        positionMotor.moveTo(0);
        
        while (cutMotor.distanceToGo() != 0 || positionMotor.distanceToGo() != 0) {
            cutMotor.run();
            positionMotor.run();
        }
        
        // Reset motors to normal speeds
        cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
        positionMotor.setMaxSpeed(POSITION_MOTOR_SPEED);
        
        // Disengage secure wood clamp after both motors reach home
        delay(CLAMP_OPERATION_DELAY);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, LOW);
        
        // Engage position clamp before position motor movement
        digitalWrite(PIN_POSITION_CLAMP, HIGH);
        delay(CLAMP_OPERATION_DELAY);
        
        // Move position motor negative 3.3 inches
        long positionTargetPosition = -STEPS_PER_INCH * POSITION_MOTOR_TRAVEL;
        positionMotor.moveTo(positionTargetPosition);
        
        while (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
        
        // Check switch state one final time before ending cycle
        runCycleSwitch.update();
        if (runCycleSwitch.read() == LOW) {
            delay(CLAMP_OPERATION_DELAY);
        }
    }
}
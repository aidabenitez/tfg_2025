#include <Arduino.h>
#include <Hub.h>
#include <ESP32Encoder.h>
#include <vector>
#include <sstream>
#include "serial_utils/serial_utils.h"


ESP32Encoder encoderL;
ESP32Encoder encoderR;

// PWM configuration
const int frequency = 20000;  // Frequency Hz
const int resolution = 8;     // Resolution in bits (from 1 to 15)

// Enum classes
enum class Motor { Left, Right };
enum class Direction { Forward, Backward };


void stop_motor_right() {
    /*
        Stops the right motor.
    */

    ledcDetachPin(GPIO_IN1_R);
    ledcDetachPin(GPIO_IN2_R);

    pinMode(GPIO_IN1_R, OUTPUT);
    pinMode(GPIO_IN2_R, OUTPUT);

    digitalWrite(GPIO_IN1_R, HIGH);
    digitalWrite(GPIO_IN2_R, HIGH);

    encoderR.clearCount();
}

void stop_motor_left() {
    /*
        Stops the left motor.
    */

    ledcDetachPin(GPIO_IN1_L);
    ledcDetachPin(GPIO_IN2_L);

    pinMode(GPIO_IN1_L, OUTPUT);
    pinMode(GPIO_IN2_L, OUTPUT);

    digitalWrite(GPIO_IN1_L, HIGH);
    digitalWrite(GPIO_IN2_L, HIGH);

    encoderL.clearCount();
}

void motor_right(int speed, Direction dir) {
    /*
        Controls the right motor.

        Arguments:
            speed (int): PWM value to control the motor speed (0–255).
            dir (direction): Direction of rotation.
    */

    int real_speed = speed;
    if (dir == Direction::Forward) { real_speed = 255 - real_speed; }

    // Reload pwm
    ledcAttachPin(GPIO_IN1_R, GPIO_CH1_R);
    ledcAttachPin(GPIO_IN2_R, GPIO_CH2_R);

    if (speed == 0) stop_motor_right();

    // Set velocity and direction
    ledcWrite(GPIO_CH1_R, dir == Direction::Forward ? 0 : speed);
    ledcWrite(GPIO_CH2_R, dir == Direction::Forward ? speed : 0);
}

void motor_left(int speed, Direction dir) {
    /*
        Controls the left motor.
    
        Arguments:
            speed (int): PWM value to control the motor speed (0–255).
            dir (direction): Direction of rotation.
    */

    int real_speed = speed;
    if (dir == Direction::Forward) { real_speed = 255 - real_speed; }

    // Reload pwm
    ledcAttachPin(GPIO_IN1_L, GPIO_CH1_L);
    ledcAttachPin(GPIO_IN2_L, GPIO_CH2_L);

    if (speed == 0) stop_motor_left();

    // Set velocity and direction
    ledcWrite(GPIO_CH1_L, dir == Direction::Forward ? 0 : speed);
    ledcWrite(GPIO_CH2_L, dir == Direction::Forward ? speed : 0);
}

int velocity_to_pwm(float velocity) {
    /*
        Converts the desired velocity (m/s) to a PWM value (0–255).

        Arguments:
            velocity (int): Desired robot velocity in meters per second.

        Returns:
            int: PWM value (0 to MAX_PWM).
    */

    // Convert the velocity to pwm
    float pwm = MOTOR_VEL_MIN_PWM + (velocity / MOTOR_MAX_LINEAL_VEL) * (MOTOR_VEL_MAX_PWM - MOTOR_VEL_MIN_PWM);

    // Clamp the result between 0 and MAX_PWM
    if (pwm > MOTOR_VEL_MAX_PWM) pwm = MOTOR_VEL_MAX_PWM;
    if (pwm < 0) pwm = 0;

    return (int)pwm;
}


void setup() {
    setupSerial(); 


    // Set up PWM channels for motor control
    ledcSetup(GPIO_CH1_L, frequency, resolution);
    ledcSetup(GPIO_CH2_L, frequency, resolution);
    ledcAttachPin(GPIO_IN1_L, GPIO_CH1_L);
    ledcAttachPin(GPIO_IN2_L, GPIO_CH2_L);

    ledcSetup(GPIO_CH1_R, frequency, resolution);
    ledcSetup(GPIO_CH2_R, frequency, resolution);
    ledcAttachPin(GPIO_IN1_R, GPIO_CH1_R);
    ledcAttachPin(GPIO_IN2_R, GPIO_CH2_R);


    // Set motor direction pins as outputs
    pinMode(GPIO_IN1_L, OUTPUT); 
    pinMode(GPIO_IN2_L, OUTPUT);
    pinMode(GPIO_IN1_R, OUTPUT);
    pinMode(GPIO_IN2_R, OUTPUT);

    // Set motor encoder pins as inputs
    pinMode(GPIO_ENC1_L, INPUT_PULLUP);
    pinMode(GPIO_ENC2_L, INPUT_PULLUP);
    pinMode(GPIO_ENC1_R, INPUT_PULLUP);
    pinMode(GPIO_ENC2_R, INPUT_PULLUP);


    // Attach and configure motor encoders
    encoderL.attachFullQuad(GPIO_ENC1_L, GPIO_ENC2_L);
    encoderL.setFilter(1023);
    encoderL.setCount(0);

    encoderR.attachFullQuad(GPIO_ENC1_R, GPIO_ENC2_R);
    encoderR.setFilter(1023);
    encoderR.setCount(0);
}


void loop() {

    // === Send encoders message ===

    // Get encoder counts from both motors
    int64_t left_encoder_val  = encoderL.getCount();    
    int64_t right_encoder_val = encoderR.getCount() * -1;   // Invert count (forward = backward)

    double left_encoder = (int)(left_encoder_val / MOTOR_POLSOS_PER_CM);
    double right_encoder = (int)(right_encoder_val / MOTOR_POLSOS_PER_CM);
    
    // Format encoder values into a message string (in centimeters)
    char encoders_msg[50]; snprintf(
        encoders_msg, sizeof(encoders_msg),
        "EL,%ld,ER,%ld",
        (int)(left_encoder_val / MOTOR_POLSOS_PER_CM),
        (int)(right_encoder_val / MOTOR_POLSOS_PER_CM)
    );

    // Send the message
    sendMessage(encoders_msg);


    // === Read message ===

    std::string message = readMessage().c_str();

    // Get each value of the message and assign motors power
    if (!message.empty()) {
        std::vector<std::string> message_parts;
        std::stringstream ss(message);
        std::string item;

        // Split the message by commas and store parts
        while (std::getline(ss, item, ',')) {
            message_parts.push_back(item);
        }

        // Remove the last part as it's a checksum
        if (!message_parts.empty()) {
            message_parts.pop_back();
        }

        // Process command pairs
        for (int i = 0; i < message_parts.size(); i += 2) {
            std::string motor = message_parts[i];

            float vel = stof(message_parts[i+1]);
            int pwm   = velocity_to_pwm(abs(vel));
            
            Direction dir = Direction::Forward;

            if (motor == "ML") { 
                if (vel < 0) { dir = Direction::Backward; }
                if (pwm == 0) { encoderL.setCount(0); }
                motor_left(pwm, dir);

            } else if (motor == "MR") {
                if (vel < 0) { dir = Direction::Backward; }
                if (pwm == 0) { encoderR.setCount(0); }
                motor_right(pwm, dir);
            }
        }
    }

    delay(20); // Receive 100 messages per second
}
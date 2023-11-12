#include "rotate.h"
#include "rcc_stdlib.h"

using namespace std;

int main() 
{
    // Initialize the sensors and motors
    stdio_init_all();
    sleep_ms(1000);
    //motor using 100 sleep, imu uses 1500 sleep, odom uses 1000 sleep, so which one do I use?
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); // LED on

    //Declare the constant variables here

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    // IMU setup
    rcc_init_i2c(); // Setup I2C
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    // Odom (wheel encoders) setup
    Left_Odom leftOdom; // class left odom 
    Right_Odom rightOdom; // class right odom 

    // Reset wheel encoders count (comment if not needed)
    leftOdom.setZero();
    rightOdom.setZero();

    while (true) {
        // Read IMU data
        imu.update_pico();
        float angvel_z = imu.getAngVelZ();

        // Read wheel encoder data
        int left_count = leftOdom.getCount();
        int right_count = rightOdom.getCount();

        // Calculate the current angle based on wheel encoder data (adjust as needed)
        float currentAngle = (left_count - right_count) * 0.01; // Adjust scaling factor as needed

        // Calculate the error between the target and current angles
        float angleError = targetAngle - currentAngle;

        // Set motor speeds to achieve the rotation
        int motorSpeed = static_cast<int>(angleError * Kp); // Adjust Kp as needed
        MotorPower(&motors, -motorSpeed, motorSpeed); // Rotate the robot

        // Check if the robot has rotated 90 degrees
        if (std::abs(angleError) <= 1.0) {
            MotorPower(&motors, 0, 0); // Stop the motors
            break; // Exit the loop
        }

        // Delay to control the loop frequency
        sleep_ms(100);
    }
}


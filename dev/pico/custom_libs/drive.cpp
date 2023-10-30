#include "drive.h"
#include "rcc_stdlib.h"

using namespace std;

// measure these values and replace these with the actual measurements
const int countsPerRevolution = INSERT_COUNTS_PER_REVOLUTION;
const float mmPerCount = INSERT_MM_PER_COUNT;

// Function to drive forward a specific distance
void driveForward(float distance) {

    Left_Odom leftOdom;  // Initialize the left encoder
    Right_Odom rightOdom;  // Initialize the right encoder
    Motor motors;  // Initialize the motors

    
    int targetCounts = static_cast<int>(distance / mmPerCount);

    // Set the motor speed for forward motion
    int motorSpeed = 100;  // Replace with your desired motor speed
    MotorPower(&motors, motorSpeed, motorSpeed);

    while (leftOdom.getCount() < targetCounts || rightOdom.getCount() < targetCounts) {
        // Check the encoder counts and control the motors to maintain forward motion
        // Insert code here to adjust motor speeds if needed to maintain motion
        // For example:
        // if (leftOdom.getCount() < targetCounts) {
        //     MotorPower(&motors, motorSpeed, motorSpeed - 10);
        // } else if (rightOdom.getCount() < targetCounts) {
        //     MotorPower(&motors, motorSpeed - 10, motorSpeed);
        // }
    }

    // Stop the motors
    MotorPower(&motors, 0, 0);
}

// Function to drive backward a specific distance
void driveBackward(float distance, Left_Odom& leftOdom, Right_Odom& rightOdom, Motor& motors) {
    int targetCounts = static_cast(int)(distance / mmPerCount);

    // Set the motor speed for backward motion
    int motorSpeed = -100;  // Replace with your desired motor speed
    MotorPower(&motors, motorSpeed, motorSpeed);

    while (leftOdom.getCount() > -targetCounts || rightOdom.getCount() > -targetCounts) {
        // Check the encoder counts and control the motors to maintain backward motion
        // Insert code here to adjust motor speeds if needed to maintain motion
        // For example:
        // if (leftOdom.getCount() > -targetCounts) {
        //     MotorPower(&motors, motorSpeed, motorSpeed + 10);
        // } else if (rightOdom.getCount() > -targetCounts) {
        //     MotorPower(&motors, motorSpeed + 10, motorSpeed);
        // }
    }

    // Stop the motors
    MotorPower(&motors, 0, 0);
}

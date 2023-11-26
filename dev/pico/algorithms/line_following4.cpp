//line following 4 is pid control
#include "rcc_stdlib.h"
using namespace std;

// Robot States
typedef enum {
    FORWARD,   // state 1 (010)
    LEFT,      // state 2 (100 or 110)
    RIGHT,     // state 3 (001 or 011)
    JUNCTION,  // state 4 (111 or 101)
    ROBOT_STOP //state 5 (continuation of Junction)
} robot_state_t;

int main() {
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true);  // turns on LED

    // Motors setup
    Motor motors;                    // struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);  // setup
    MotorsOn(&motors);  // enable PWM

    //IR setup
    init_ir();

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //distance
    const int targetDistance = 50; //the distance traveled in mm

    robot_state_t robot_state = FORWARD;

    //PID Alg
    // PID Controller setup for steering
    PID_control pidController(
        /* kp */ 1.0,       // Proportional gain
        /* ki */ 0.1,       // Integral gain
        /* kd */ 0.01,      // Derivative gain
        /* lowerLimit */ -100, // Lower limit of motor power
        /* upperLimit */ 100,  // Upper limit of motor power
        /* sigma */ 1.0,     // Bandwidth for the bandlimited derivative calculation
        /* ts */ 0.01       // Sample time in seconds
    );

   // Differentiator setup for linear distance
    Differentiator distanceDifferentiator(
        /* sigma */ 1.0,     // Bandwidth for the band-limited derivative
        /* ts */ 0.01       // Sample time in seconds
    );

    while (true) {
        bool leftIR_data = gpio_get(LEFT_IR_SENSOR);
        bool centerIR_data = gpio_get(CENTER_IR_SENSOR);
        bool rightIR_data = gpio_get(RIGHT_IR_SENSOR);

        int angle;                           // position of servo 

        //Linear Distance for Odom
        float leftDistance = linear_distance(left.getCount());
        float rightDistance = linear_distance(right.getCount());

        //User Linear Distance Wanted
        float userDistance = distance_convertor(targetDistance);

        //output
        cout << "left distance: " << leftDistance <<  " | right distance: " << rightDistance << "\n"; //Odom


        //PID Alg
        // Linear Distance Differentiation
        float leftLinearVelocity = distanceDifferentiator.differentiate(leftDistance);
        float rightLinearVelocity = distanceDifferentiator.differentiate(rightDistance);

        // Calculate the steering error (difference between left and right linear velocities)
        float steeringError = leftLinearVelocity - rightLinearVelocity;

        // PID control for steering
        float steeringCorrection = pidController.pid(0, steeringError);  // Reference (setpoint) is 0 for straight line following

        // Adjust motor powers based on steering correction
        int leftPower = 80 + static_cast<int>(steeringCorrection);
        int rightPower = 77 - static_cast<int>(steeringCorrection);


        // State Machines
        switch (robot_state) {
            case FORWARD:  // 010
                cout << "FORWARD \n";
                MotorPower(&motors, 80, 77);  // make it so that it goes straight, we know that 100, 97 is straight

                if ((leftIR_data && centerIR_data && !rightIR_data) ||
                    (leftIR_data && !centerIR_data && !rightIR_data))  // left turn
                    robot_state = LEFT;
                if ((rightIR_data && centerIR_data && !leftIR_data) ||
                    (rightIR_data && !centerIR_data && !leftIR_data))  // right turn
                    robot_state = RIGHT;
                if ((leftIR_data && centerIR_data && rightIR_data) ||
                    (rightIR_data && !centerIR_data && leftIR_data))  // junction
                    robot_state = JUNCTION;
            break;

            case LEFT:  // 110 or 100
                cout << "LEFT \n";
                MotorPower(&motors, 0, 80);
                if (centerIR_data && !leftIR_data && !rightIR_data)  // Turn until center is black
                    robot_state = FORWARD;
            break;

            case RIGHT:  // 011 or 001
                cout << "RIGHT \n";
                MotorPower(&motors, 80, 0);
                if (centerIR_data && !leftIR_data && !rightIR_data)  // Turn until center is black
                    robot_state = FORWARD;
            break;

            case JUNCTION:  // 111 or 101
                cout << "JUNCTION \n";

                MotorPower(&motors, 0, 0); //stop

                for (int i = 0; i < 3; i++)  // blink LED 3 times
                {
                    cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0));  // blinks LED (indicating stopped at Junction)
                    sleep_ms(300);
                }
                cyw43_arch_gpio_put(0, true);
                
                //Reset Wheel Encoder Counts before going into drive
                right.setZero();
                left.setZero();

                MotorPower(&motors, 100, 97); //drive
                if(leftDistance >= userDistance && rightDistance >= userDistance) //go forward for a distance
                {   
                    MotorPower(&motors, 0, 0); //stop

                    if (leftIR_data && centerIR_data && rightIR_data) //if all on black
                    robot_state = ROBOT_STOP;

                    if (!(leftIR_data && centerIR_data && rightIR_data)) //if all are on white
                    robot_state = LEFT; //can make this RIGHT
                }
            break;

            case ROBOT_STOP: //only can come here from Junction
                cout << "STOP ROBOT \n";
                MotorPower(&motors, 0, 0); //stop
            break;
        }
    }
}
//line following 3 is just line following
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
    const int targetDistance = 20; //the distance traveled in mm

    robot_state_t robot_state = FORWARD;

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

        // State Machines
        switch (robot_state) {
            case FORWARD:  // 010
                cout << "FORWARD \n";
                MotorPower(&motors, 60, 58);  // make it so that it goes straight, we know that 100, 97 is straight

                if ((leftIR_data && centerIR_data && !rightIR_data) ||
                    (leftIR_data && !centerIR_data && !rightIR_data))  // left turn
                    robot_state = LEFT;
                if ((rightIR_data && centerIR_data && !leftIR_data) ||
                    (rightIR_data && !centerIR_data && !leftIR_data))  // right turn
                    robot_state = RIGHT;
                if ((leftIR_data && centerIR_data && rightIR_data) ||
                    (rightIR_data && !centerIR_data && leftIR_data)){  // junction
                    //Reset Wheel Encoder Counts 
                    right.setZero();
                    left.setZero();
                    robot_state = JUNCTION;
                    } 
                    
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

                // for (int i = 0; i < 3; i++)  // blink LED 3 times
                // {
                //     cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0));  // blinks LED (indicating stopped at Junction)
                //     sleep_ms(300);
                // }
                // cyw43_arch_gpio_put(0, true);

                MotorPower(&motors, 100, 97); //drive
                if(leftDistance >= userDistance || rightDistance >= userDistance) //go forward for a distance
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
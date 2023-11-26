//final line following 
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

// Object Detection
typedef enum {
    UNDETECTED,
    DETECTED,
    OBJECT_STOP
} object_state_t;

const int MID_ANGLE = 50; //servo mid angle

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

    // Lidar setup
    rcc_init_i2c();         // setup pico i2c
    VL53L0X lidar;           // class
    rcc_init_lidar(&lidar);  // setup lidar

    // Servo setup
    Servo s3;                            // struct
    ServoInit(&s3, 18, false, MID_ANGLE);       // setup on pin 18
    ServoOn(&s3);                        // enables PWM

    uint16_t distance;                   // lidar variable
    uint16_t target_distance = 350;     // distance of the object

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //counts
    const int target_counts = 50; //the number of counts needed to travel forward

    //Initial States
    robot_state_t robot_state = FORWARD;
    object_state_t object_state = UNDETECTED;

    while (true) {
        bool leftIR_data = gpio_get(LEFT_IR_SENSOR);
        bool centerIR_data = gpio_get(CENTER_IR_SENSOR);
        bool rightIR_data = gpio_get(RIGHT_IR_SENSOR);

        distance = getFastReading(&lidar);

        //output
        cout << "distance: " << distance << "\n"; //Lidar
        cout << "left count: " << left.getCount() <<  " | right count: " << right.getCount() << "\n"; //Odom

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
                // Indication at Junction
                cout << "JUNCTION \n";

                MotorPower(&motors, 0, 0); //stop
        
                //Reset Wheel Encoder Counts before going into drive
                right.setZero();
                left.setZero();

                MotorPower(&motors, 100, 97); //drive

                if(left.getCount() >= target_counts) //go forward for a distance
                {   
                    MotorPower(&motors, 0, 0); //stop

                    if (leftIR_data && centerIR_data && rightIR_data) //if all on black
                    robot_state = ROBOT_STOP;
                    object_state = OBJECT_STOP;

                    if (!(leftIR_data && centerIR_data && rightIR_data)) //if all are on white
                    robot_state = LEFT; //can make this RIGHT
                }
            break;

            case ROBOT_STOP: //only can come here from Junction
                cout << "STOP ROBOT \n";
                MotorPower(&motors, 0, 0); //stop
            break;
        

        //Object Detection
        switch (object_state) {
                case UNDETECTED:
                    cout << "UNDETECTED \n\n";
                    if (distance <= target_distance)
                        object_state = DETECTED;
                break;

                case DETECTED:
                    // Indication of Object Detection
                    cout << "DETECTED \n\n";

                    MotorPower(&motors, 0, 0); //stop

                    robot_state = LEFT;  // can make this RIGHT (junction goes left as well)
                    object_state = UNDETECTED;

                break;

                case OBJECT_STOP: //can only come here from Junction
                    cout << "STOP OBJECT \n\n";
                    ServoPosition(&s3, MID_ANGLE);
                break;
            }
        }
    }
}
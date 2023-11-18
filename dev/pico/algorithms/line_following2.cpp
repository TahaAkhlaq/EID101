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

void init_ir(void) {
    gpio_init(LEFT_IR_SENSOR);
    gpio_init(CENTER_IR_SENSOR);
    gpio_init(RIGHT_IR_SENSOR);

    gpio_set_dir(LEFT_IR_SENSOR, false);
    gpio_set_dir(CENTER_IR_SENSOR, false);
    gpio_set_dir(RIGHT_IR_SENSOR, false);
}

//Odom
float linear_distance(unsigned long count)
{
  return  (WHEEL_DIAM * PI * count) / (CPR * SLOTS); //I defined WHEEL_DIAM and PI globally 
  //20 is about a foot or about 300 mm 
}

//Distance
float distance_convertor(float userDistance)
{
    // 1 = 18.5mm
    return (userDistance / 18.5); //converts to mm
}

const int MIN_ANGLE = 25;
const int MID_ANGLE = 50;
const int MAX_ANGLE = 75;

int main() {
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true);  // turns on LED

    // Motors setup
    Motor motors;                    // struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);  // setup
    MotorsOn(&motors);  // enable PWM

    init_ir();

    // Lidar setup
    rcc_init_i2c();         // setup pico i2c
    VL53L0X lidar;           // class
    rcc_init_lidar(&lidar);  // setup lidar

    // Servo setup
    Servo s3;                            // struct
    ServoInit(&s3, 18, false, 50);       // setup on pin 18
    ServoOn(&s3);                        // enables PWM

    uint16_t distance;                   // lidar variable
    uint16_t target_distance = 200;     // distance of the object

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //distance
    int targetDistance = 50; //the distance traveled in mm

    robot_state_t robot_state = FORWARD;
    object_state_t object_state = UNDETECTED;

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

        distance = getFastReading(&lidar);

        //output
        cout << "distance: " << distance << "\n";
        cout << "left distance: " << leftDistance <<  " | right distance: " << rightDistance << "\n"; //Odom

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
                    object_state = OBJECT_STOP;

                    if (!(leftIR_data && centerIR_data && rightIR_data)) //if all are on white
                    robot_state = LEFT; //can make this RIGHT
                }
            
            case ROBOT_STOP: //only can come here from Junction
                cout << "STOP ROBOT \n";
                MotorPower(&motors, 0, 0); //stop
            break;
        }

        switch (object_state) {
            case UNDETECTED:
                cout << "UNDETECTED \n\n";

                for (angle = MIN_ANGLE; angle <= MAX_ANGLE; angle++) {
                    ServoPosition(&s3, angle);
                    sleep_ms(20);
                }

                for (angle = MAX_ANGLE; angle >= MIN_ANGLE; angle--) {
                    ServoPosition(&s3, angle);
                    sleep_ms(20);
                }

                if (distance <= target_distance)
                    object_state = DETECTED;
            break;

            case DETECTED:
                cout << "DETECTED \n\n";

                // Gradually return to the mid-angle
                if (angle < MID_ANGLE) {
                    while (angle < MID_ANGLE) {
                        angle++;
                        ServoPosition(&s3, angle);
                        sleep_ms(20);
                    }
                } else if (angle > MID_ANGLE) {
                    while (angle > MID_ANGLE) {
                        angle--;
                        ServoPosition(&s3, angle);
                        sleep_ms(20);
                    }
                }

                MotorPower(&motors, 0, 0); //stop

                // Indication of Object Detection
                for (int i = 0; i < 3; i++)  // blink LED 3 times
                {
                    cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0));  // blinks LED
                    sleep_ms(300);
                }
                cyw43_arch_gpio_put(0, true);

                robot_state = RIGHT;  // can make this LEFT
                object_state = UNDETECTED;

            break;

            case OBJECT_STOP: //can only come here from Junction
                cout << "STOP OBJECT \n\n";
                ServoPosition(&s3, 50);
        }
    }
}

#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum { 
    WAIT,    //state 1
    FORWARD, //state 2
    BACKWARD,//state 3
} robot_state_t;

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

int main(void)
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); //led on

    rcc_init_pushbutton(); //set up button

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //Intial Robot States
    robot_state_t robot_state = WAIT;

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //distance
    int targetDistance = 300; //the distance traveled in mm

    //Counter for the number of cycles
    int cycles = 0;

    while(true)
    {   
        //Linear Distance for Odom
        float leftDistance = linear_distance(left.getCount());
        float rightDistance = linear_distance(right.getCount());

        //User Linear Distance Wanted
        float userDistance = distance_convertor(targetDistance);

        //debugging
        cout << "left distance: " << leftDistance <<  " | right distance: " << rightDistance << "\n"; //Odom
        cout << "left count: " << left.getCount() <<  " | right count: " << right.getCount() << "\n"; //Odom
        cout << robot_state << "\n\n";

        //Switch for Robot_State
        switch(robot_state) //Intial State is WAIT
        {
            case WAIT: //state 1

                MotorPower(&motors, 0, 0); //stop
                if(!gpio_get(RCC_PUSHBUTTON))
                {
                //Reset Wheel Encoder Counts before going into drive
                right.setZero();
                left.setZero();

                robot_state = FORWARD;
                }

            break;

            case FORWARD: //state 2

                MotorPower(&motors, 100, 97); //both at full power

                if(leftDistance >= userDistance && rightDistance >= userDistance) //go forward about a foot
                {
                    MotorPower(&motors, 0, 0);

                    //Reset Wheel Encoder Counts before going into drive
                    right.setZero();
                    left.setZero();

                    sleep_ms(300);

                    robot_state = BACKWARD;
                }
                            
            break; 

        case BACKWARD: // state 2
            MotorPower(&motors, -100, -97); // both at full power in reverse

            // Assuming your target distance is in millimeters
            if (leftDistance >= userDistance && rightDistance >= userDistance)
            {
                // Reset Wheel Encoder Counts before going into drive
                right.setZero();
                left.setZero();

                sleep_ms(300);

                if (cycles < 2) // Check if it's the third cycle
                {
                    robot_state = FORWARD;
                    cycles++;
                }
                else
                {
                    robot_state = WAIT;
                    cycles = 0;
                }
            }
            break;
        }
    }

    return 0;
}
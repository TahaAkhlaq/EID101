#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum { 
    WAIT,    //state 1
    FORWARD, //state 2
    BACKWARD,//state 3
} robot_state_t;

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
    const int targetDistance = 300; //the distance traveled in mm
    const int number_cycles = 3;
    int cycles_completed = 0;

    while(true)
    {   
        // //Linear Distance for Odom
        // const float leftDistance = linear_distance(left.getCount());
        // const float rightDistance = linear_distance(right.getCount());

        //User Linear Distance Wanted
        const float distance_ticks = distance_convertor(targetDistance);

        //debugging
        // cout << "left distance: " << leftDistance <<  " | right distance: " << rightDistance << "\n"; //Odom
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

                MotorPower(&motors, 80, 80); //both at full power

                if(right.getCount() >= distance_ticks) //go forward
                {
                    MotorPower(&motors, 0, 0);

                    //Reset Wheel Encoder Counts before going into drive
                    right.setZero();
                    left.setZero();
                    sleep_ms(500);
                    robot_state = BACKWARD;
                }
                            
            break; 

            case BACKWARD: // state 2
                MotorPower(&motors, -80, -80); // both at full power in reverse

                // Assuming your target distance is in millimeters
                if (left.getCount() >= distance_ticks)
                {
                    // Reset Wheel Encoder Counts before going into drive
                    right.setZero();
                    left.setZero();
                    cycles_completed++;

                    if (cycles_completed >= number_cycles) // Check if it's the third cycle
                    {
                        robot_state = WAIT;
                        cycles_completed = 0;
                    }
                    else
                    {
                        sleep_ms(500);
                        robot_state = FORWARD;
                    }
                }
            break;
        }
    }   
    return 0;
}
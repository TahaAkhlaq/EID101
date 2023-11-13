#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum { 
    WAIT,    
    FORWARD, 
    BACKWARD,  
    STOP     
} robot_state_t;

//Odom
float linear_distance(unsigned long count)
{
  return WHEEL_DIAM*PI/count; //I defined WHEEL_DIAM and PI globally 
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); //led on

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //Intial Robot States
    robot_state_t robot_state = WAIT;

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //Linear Distance
    float leftDistance = linear_distance(left.getCount());
    float rightDistance = linear_distance(right.getCount());

    while(true)
    {   
        //debugging
         cout << "left distance: " << leftDistance <<  " | right distance: " << rightDistance << "\n"; //Odom


        //Switch for Robot_State
        switch(robot_state) //Intial State is WAIT
        {
            case WAIT:

                MotorPower(&motors, 0, 0); //stop
                sleep_ms(300);

                //Reset Wheel Encoder Counts before going into drive
                right.setZero();
                left.setZero();

                robot_state = FORWARD;

            break;

            case FORWARD:

                MotorPower(&motors, 100, 100); //both at full power

                if(leftDistance >= 25 && rightDistance >= 25) //go forward about an inch
                {
                    robot_state = BACKWARD;

                    //Reset Wheel Encoder Counts before going into drive
                    right.setZero();
                    left.setZero();
                }
                            
            break; 

            case BACKWARD:

                MotorPower(&motors, -100, -100); //both at full power in reverse

                if(leftDistance >= 25 && rightDistance >= 25) //go forward about an inch
                {
                    robot_state = STOP;
                }
                            
            break; 

            case STOP:

                MotorPower(&motors, 0, 0);

            break;
            
        }
    }
}
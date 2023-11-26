#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum { 
    WAIT,    
    TURN        
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

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //Turning Right and Left
    const int turn = 30.0; //90 degrees
    const int number_turns = 3; //number of turns
    int turns_completed = 0;

    //Intial Robot States
    robot_state_t robot_state = WAIT;

    while(true)
    {   

        //debugging
        cout << "left count: " << left.getCount() <<  " | right count: " << right.getCount() << "\n"; //Odom

        //Switch for Robot_State
        switch(robot_state) //Intial State is WAIT
        {
            case WAIT:

                MotorPower(&motors, 0, 0); //stop
                if(!gpio_get(RCC_PUSHBUTTON))
                {
                    //Reset Wheel Encoder Counts before going into drive
                    right.setZero();
                    left.setZero();
                    robot_state = TURN;
                }

            break;

            case TURN:
                cout << "Turning \n\n";
                MotorPower(&motors, 80, 0); //rotate 
 
                if(left.getCount() >= turn || right.getCount() >= turn) //rotate 
                { 
                    MotorPower(&motors, 0, 0);
                    //Reset Wheel Encoder Counts before going into drive
                    right.setZero();
                    left.setZero();
                    turns_completed++;

                    if (turns_completed >= number_turns)
                    {
                        robot_state = WAIT;
                        turns_completed = 0;
                    }
                    else
                    sleep_ms(500);
                }
            break;
        }
    }
}
//using time to move forward and backward
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

    //cycle
    const int number_cycles = 3;
    int cycles_completed = 0;

    while(true)
    {   
        switch(robot_state) //Intial State is WAIT
        {
            case WAIT: //state 1
                MotorPower(&motors, 0, 0); //stop
                if(!gpio_get(RCC_PUSHBUTTON))
                robot_state = FORWARD;
            break;

            case FORWARD: //state 2
                MotorPower(&motors, 80, 80); //forward
                sleep_ms(3000); //forward for 3s
                MotorPower(&motors, 0, 0); //stop
                robot_state = BACKWARD;        
            break; 

            case BACKWARD: // state 3
                MotorPower(&motors, -80, -80); // reverse
                sleep_ms(3000); //reverse for 3s
                MotorPower(&motors, 0, 0); //stop
                cycles_completed++;

                    if (cycles_completed >= number_cycles) // Check if it's the third cycle
                    {
                        robot_state = WAIT;
                        cycles_completed = 0;
                    }
                    else
                    {
                        robot_state = FORWARD;
                    }
            break;
        }
    }   
}
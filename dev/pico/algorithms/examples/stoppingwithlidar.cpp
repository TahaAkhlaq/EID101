#include "rcc_stdlib.h"
using namespace std;

/*
In this example, going to have robot drive forwards and then stop based on lidar readings

going to use switch cases to define the state machines
some nice reading here: https://www.w3schools.com/cpp/cpp_switch.asp 
*/

//create enum named state_t 
typedef enum{
    WAIT, 
    FORWARD, 
    STOP
}state_t;


int main()
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0,1); //led on

    rcc_init_pushbutton(); //set up button

    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    uint16_t distance; //lidar variable

    //how far away object will be when we stop
    uint16_t target_distance = 200;

    //setup motors for use
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //default robot state is wait
    state_t robot_state = WAIT;

    while(true){
        //at start of each loop, get most updated lidar reading
        distance = getFastReading(&lidar);

        //for debugging
        cout << "distance: " << distance << "\n";

        //switch case for robot's state 
        switch(robot_state){
            case WAIT:
                //at start of each case, do something (or dont for this state)
                MotorPower(&motors, 0,0); //dont move

                //then, define transition conditions, in this case, 
                //robot goes into turn state when button is pushed
                if(!gpio_get(RCC_PUSHBUTTON)){
                        robot_state = FORWARD;
                    }
                break;

            case FORWARD:
                //do something first in each case
                MotorPower(&motors, 60, 60);

                //define transition conditions
                //in this case if lidar reading is at the target distance
                if(distance <= target_distance){
                    robot_state = STOP;
                }
            break; 

            case STOP:
                //always do something in each state first
                MotorPower(&motors, 0, 0);

                //since this is end of state machine, 
                //theres no transition conditions, 
                //have to reset pico to go back to beginning
            break;
            
        }

    }
}
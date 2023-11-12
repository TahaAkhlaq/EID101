#include "rcc_stdlib.h"
using namespace std;

/*
In this example, going to have 2 state machines running at same time
first state machine will be the /robot state/, the other the calculus timing state machine

going to use switch cases to define the state machines
some nice reading here: https://www.w3schools.com/cpp/cpp_switch.asp 
*/

//create enum named state_t 
typedef enum{
    WAIT, 
    TURN, 
    STOP
}state_t;

//create another enum named integratorstate_t
typedef enum{
    DWELL,
    INTEGRATE
}integratorstate_t;

int main()
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0,1); //led on

    //setup IMU for use 
    rcc_init_i2c(); //setup i2c
    MPU6050 imu; //instantiate class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    rcc_init_pushbutton(); //set up button

    //setup motors for use
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //timing variables for riemann sum
    uint32_t current_time, previous_time;
    uint32_t duration = 10000; //10000 us to be 10ms

    //setup variables to track angle 
    double theta = 0.0;

    //in this case i've hard coded 65 as a "90" degree turn ... not ideal but its okay
    double turn_degrees = 65.0;

    //default robot state is wait
    state_t robot_state = WAIT;

    //default calculus state is dwell
    integratorstate_t imustate = DWELL;

    while(true){
        //at start of each while true loop, update imu data and update current time
        imu.update_pico(); //updates data
        current_time = time_us_32(); //update current time

        //for debugging
        cout << "theta: " << theta << "\n";

        //first switch case for robot's state 
        switch(robot_state){
            case WAIT:
                //at start of each case, do something (or dont for this state)
                MotorPower(&motors, 0,0); //dont move

                //then, define transition conditions, in this case, 
                //robot goes into turn state when button is pushed
                if(!gpio_get(RCC_PUSHBUTTON)){
                        robot_state = TURN;
                        //important to reset theta before going into turn state
                        theta = 0.0; 
                    }
                break;

            case TURN:
                //do something first in each case
                //robot will rotate left in this example
                MotorPower(&motors, -50, 50);

                //define transition conditions
                //if theta, which is being updated by other state machine, 
                //if theta is greater than the degrees required for the turn, 
                //then go to stop state
                if(theta >= turn_degrees){
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

        //in the other state machine, the IMU's angular velocity is always being integrated
        //but we can reset theta in the other state machine to put it back to zero degrees
        switch(imustate){
            case DWELL:
                //doing nothing in this state 

                //transition to the integrate state only after the time has passed
                if(current_time - previous_time >= duration){
                    imustate = INTEGRATE;
                }
                break;

            //in this state, we perform the riemann sum and update the previous time
            //the timer starts when the pico turns on so we always have to bring the 
            //bring the previous time variable forward to match the current time
            //before increasing the current time further

            case INTEGRATE:
                //in this state, we do the math and update theta
                theta = theta + imu.getAngVelZ()*duration/1000000.0;

                //automatically want to go back to the other state
                imustate = DWELL;
                previous_time = current_time; //update time
                break; 
        }

    }
}
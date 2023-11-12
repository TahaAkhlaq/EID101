#include "rcc_stdlib.h"
using namespace std;

//Turn State
typedef enum {
    FORWARD, 
    LEFT,    
    RIGHT,   
    STOP     
} robot_state_t;

//Calculus State
typedef enum{
    DWELL,
    INTEGRATE
}imu_state_t;

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

    // IMU setup
    rcc_init_i2c(); 
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    // State Machine Riemann Sum variables
    uint32_t cur, prev;
    uint32_t duration = 1000000; // 1 second

    //Starting Angle
    double theta = 0.0;

    //Turning Right and Left
    double turn_degrees = 90.0;

    //Intial Robot States
    state_t robot_state = STOP;

    //Initial Calculus State
    integratorstate_t imu_state = DWELL;


    while(true)
    {   
        imu.update_pico(); //updates data
        cur = time_us_32(); //update current time

        //debugging
        cout << "theta: " << theta << "\n";

        //State - STOP 
        switch(robot_state) //Initial State is Stop
            case STOP:
            MotorPower(&motors, 0,0);

            //CHANGE CODE TO USE SWITCH STATEMENTS
        {
            MotorPower(&motors, 100, 100); //both at full power

            theta += imu.getAngVelZ()*duration/1000000.0;

            if(!centerIR_data && !rightIR_data)//RIGHT State - 011  //if NOT gpio (if gpio is low)
            {
                robot_state = RIGHT;
                targetAngle = 90.0;
                prev = cur; //Update time - is this needed here?
            }

            else if (!leftIR_data && !centerIR_data)//LEFT State - 110  //if NOT gpio (if gpio is low)
            {
                robot_state = LEFT;
                targetAngle = -90.0;
                prev = cur; //Update time - is this needed here?
            }

            else if (rightIR_data && centerIR_data && leftIR_data)//STOP State - 111  //if NOT gpio (if gpio is low)
            {
                robot_state = STOP;
                targetAngle = 0;
                prev = cur; //Update time - is this needed here?
            }
        }


        //State 011 - RIGHT
        if(robot_state == RIGHT) 
        {
            MotorPower(&motors, 0, 100); //left at full power

            theta += imu.getAngVelZ()*duration/1000000.0;

            //Transition condition
            if(!centerIR_data) //if the center gpio is low (on the black line)
            {
                robot_state = FORWARD;
                targetAngle = 0 // is this needed here?
                prev = cur; //Update time 
            }
        }

        //State 110 - LEFT
        if (robot_state == LEFT)
        {
            MotorPower(&motors, 100, 0); //right at full power

            theta += imu.getAngVelZ()*duration/1000000.0;

            //Transition condition
            if(!centerIR_data) //if the center gpio is low (on the black line)
            {
                robot_state = FORWARD;
                targetAngle = 0 // is this needed here?
                prev = cur; //Update time
            }
        }

        //State 111 - STOP
        if (robot_state == STOP)
        {
            MotorPower(&motors, 0, 0); //stop

            theta = theta + imu.getAngVelZ()*duration/1000000.0;

        //Transition condition
            if(!centerIR_data) //if the center gpio is low (on the black line)
            {
                robot_state = FORWARD;
                targetAngle = 0 // is this needed here?
                prev = cur; //Update time
            }

        }

    }
}
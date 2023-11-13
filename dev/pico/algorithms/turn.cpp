#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum { 
    WAIT,    
    TURN,   
    STOP     
} robot_state_t;

//IMU States
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
    uint32_t current_time, previous_time;
    uint32_t duration = 1000000; // 1 second

    //Starting Angle
    double theta = 0.0;

    //Turning Right and Left
    double turn_degrees = 90.0; //may need to change this to 65.0

    //Intial Robot States
    robot_state_t robot_state = WAIT;

    //Initial Calculus State
    imu_state_t imu_state = DWELL;


    while(true)
    {   
        imu.update_pico(); //updates data
        current_time = time_us_32(); //update current time

        //debugging
        cout << "theta: " << theta << "\n";

        //Switch for IMU_State
        switch(imu_state)
        {
            case DWELL:

                //Transition condition
                if(current_time - previous_time >= duration)
                {
                    imu_state = INTEGRATE;
                }
                break;

            //riemann sum 
            case INTEGRATE:      
                
                theta += imu.getAngVelZ()*duration/1000000.0; 

                imu_state = DWELL; //go back to initial state
                previous_time = current_time; //update time
                break; 
        }

        //Switch for Robot_State
        switch(robot_state) //Intial State is WAIT
        {
            case WAIT:

                MotorPower(&motors, 0, 0); //stop
                sleep_ms(300);

                robot_state = TURN;
                theta = 0.0;

            break;

            case TURN:

                MotorPower(&motors, 50, -50); //rotate right

                if((3*theta) >= turn_degrees) //rotate right 3 times
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
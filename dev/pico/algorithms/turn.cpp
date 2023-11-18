#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum { 
    WAIT,    
    TURN,        
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

    rcc_init_pushbutton(); //set up button

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
    const double turn_counterclockwise = 1 * 65.0; //may need to change this
    const double turn_clockwise = 3 * -65.0; //may need to change this

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
        cout << robot_state << "\n\n";

        //Switch for Robot_State
        switch(robot_state) //Intial State is WAIT
        {
            case WAIT:

                MotorPower(&motors, 0, 0); //stop
                if(!gpio_get(RCC_PUSHBUTTON))
                {
                    theta = 0.0; 
                    sleep_ms(300);
                    robot_state = TURN;
                }

            break;

            case TURN:

                MotorPower(&motors, 0, 50); //rotate counterclockwise

                if(theta >= turn_counterclockwise) //rotate 
                { 
                    MotorPower(&motors, 0, 0);
                    sleep_ms(300);
                    robot_state = WAIT;
                }

            break;
            
        }

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
    }
}


// //Setup Right IR Sensor ISR 
// void right_ir_sensor_isr(void)
// {   
//     // Check the isr reason for Right IR Sensor (pin, events)
//     if(gpio_get_irq_event_mask(RIGHT_IR_SENSOR) & right_ir_sensor_events) {

//         // Acknowledge the interrupt request (pin, events)
//         gpio_acknowledge_irq(RIGHT_IR_SENSOR, right_ir_sensor_events);

//         // Do something when ISR is activated
//         right_ir_sensor_count++;
//     }
// }

// //Setup Center IR Sensor ISR 
// void center_ir_sensor_isr(void)
// {   
//     // Check the isr reason for Center IR Sensor (pin, events)
//     if(gpio_get_irq_event_mask(CENTER_IR_SENSOR) & center_ir_sensor_events) {

//         // Acknowledge the interrupt request (pin, events)
//         gpio_acknowledge_irq(CENTER_IR_SENSOR, center_ir_sensor_events);

//         // Do something when ISR is activated
//         center_ir_sensor_count++;
//     }
// }

// //Setup Left IR Sensor ISR 
// void left_ir_sensor_isr(void)
// {   
//     // Check the isr reason for Left IR Sensor (pin, events)
//     if(gpio_get_irq_event_mask(LEFT_IR_SENSOR) & left_ir_sensor_events) {

//         // Acknowledge the interrupt request (pin, events)
//         gpio_acknowledge_irq(LEFT_IR_SENSOR, left_ir_sensor_events);

//         // Do something when ISR is activated
//         left_ir_sensor_count++;
//     }

// }
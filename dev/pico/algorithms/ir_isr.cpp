#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum {
    WAIT,
    FORWARD, 
    LEFT,    
    RIGHT,   
    JUNCTION,
    STOP   
} robot_state_t;

//IMU States
typedef enum{
    DWELL,
    INTEGRATE
}imu_state_t;

//Define GPIO events that trigger the ISR

//Right IR
const uint32_t right_ir_sensor_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
volatile uint32_t right_ir_sensor_count = 0; //Volatile because value is edited in ISR

//Center IR
const uint32_t center_ir_sensor_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
volatile uint32_t center_ir_sensor_count = 0; //Volatile because value is edited in ISR

//Left IR
const uint32_t left_ir_sensor_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
volatile uint32_t left_ir_sensor_count = 0; //Volatile because value is edited in ISR

//Intialize the GPIO Pins for the IR sensors
void init_ir(void) {
    gpio_init(LEFT_IR_SENSOR);    // Initialize the left IR sensor pin (I defined this as a global variable already)
    gpio_init(CENTER_IR_SENSOR);  // Initialize the center IR sensor pin (I defined this as a global variable already)
    gpio_init(RIGHT_IR_SENSOR);   // Initialize the right IR sensor pin (I defined this as a global variable already)

    gpio_set_dir(LEFT_IR_SENSOR, false);      // Set the left IR sensor pin as input
    gpio_set_dir(CENTER_IR_SENSOR, false);    // Set the center IR sensor pin as input
    gpio_set_dir(RIGHT_IR_SENSOR, false);     // Set the right IR sensor pin as input
}

//Setup Right IR Sensor ISR 
void right_ir_sensor_isr(void)
{   
    // Check the isr reason for Right IR Sensor (pin, events)
    if(gpio_get_irq_event_mask(RIGHT_IR_SENSOR) & right_ir_sensor_events) {

        // Acknowledge the interrupt request (pin, events)
        gpio_acknowledge_irq(RIGHT_IR_SENSOR, right_ir_sensor_events);

        // Do something when ISR is activated
        right_ir_sensor_count++;
    }
}

//Setup Center IR Sensor ISR 
void center_ir_sensor_isr(void)
{   
    // Check the isr reason for Center IR Sensor (pin, events)
    if(gpio_get_irq_event_mask(CENTER_IR_SENSOR) & center_ir_sensor_events) {

        // Acknowledge the interrupt request (pin, events)
        gpio_acknowledge_irq(CENTER_IR_SENSOR, center_ir_sensor_events);

        // Do something when ISR is activated
        center_ir_sensor_count++;
    }
}

//Setup Left IR Sensor ISR 
void left_ir_sensor_isr(void)
{   
    // Check the isr reason for Left IR Sensor (pin, events)
    if(gpio_get_irq_event_mask(LEFT_IR_SENSOR) & left_ir_sensor_events) {

        // Acknowledge the interrupt request (pin, events)
        gpio_acknowledge_irq(LEFT_IR_SENSOR, left_ir_sensor_events);

        // Do something when ISR is activated
        left_ir_sensor_count++;
    }

}

//Odom
float linear_distance(unsigned long count)
{
  return WHEEL_DIAM*PI/count; //I defined WHEEL_DIAM and PI globally 
}


int main()
{
    stdio_init_all();    
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); //turns on LED

    //Pushbutton setup
    rcc_init_pushbutton(); //for debugging purposes

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //IMU setup
    rcc_init_i2c(); 
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //Linear Distance
    float leftDistance = linear_distance(left.getCount());
    float rightDistance = linear_distance(right.getCount());

    //Lidar Setup
    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    //IR Sensors Setup
    init_ir(); 

    // Setup GPIO pins as an interrupt pins with events for the IR Sensors

    //Right
    gpio_set_irq_enabled(RIGHT_IR_SENSOR, right_ir_sensor_events, true);
    gpio_add_raw_irq_handler(RIGHT_IR_SENSOR, &right_ir_sensor_isr);
    
    //Center
    gpio_set_irq_enabled(CENTER_IR_SENSOR, center_ir_sensor_events, true);
    gpio_add_raw_irq_handler(CENTER_IR_SENSOR, &center_ir_sensor_isr);

    //Left
    gpio_set_irq_enabled(LEFT_IR_SENSOR, left_ir_sensor_events, true);
    gpio_add_raw_irq_handler(LEFT_IR_SENSOR, &left_ir_sensor_isr);

    // State Machine Riemann Sum variables
    uint32_t current_time, previous_time;
    uint32_t duration = 1000000; // 1 second

    //Object Distance
    uint16_t target_distance = 200;

    //Starting Angle
    double theta = 0.0;

    //Turning Right and Left
    double turn_degrees = 90.0; //may need to change this to 65.0

    //Turning Around
    double turn_around = 180.0; //may need to change this too depending on the value of turn_degrees

    //Intial Robot States
    robot_state_t robot_state = WAIT;

    //Initial Calculus State
    imu_state_t imu_state = DWELL;

    while(true)
    {   
        imu.update_pico(); //updates data
        current_time = time_us_32(); //update current time

        distance = getFastReading(&lidar); // lidar reading

        // Read IR Sensor Data
        //True = Black Line
        //False = White Background
        bool leftIR_data = gpio_get(LEFT_IR_SENSOR);       // Left IR sensor data (if left IR sensor is on the line)
        bool centerIR_data = gpio_get(CENTER_IR_SENSOR);   // Center IR sensor data (if center IR sensor is on the line)
        bool rightIR_data = gpio_get(RIGHT_IR_SENSOR);     // Right IR sensor data (if right IR sensor is on the line)


        //For Debugging Purposes
        cout << "theta: " << theta << "\n"; //IMU
        cout << "left distance: " << leftDistance <<  " | right distance: " << rightDistance << "\n"; //Odom
        cout << leftIR_data << " | " << centerIR_data << " | " << rightIR_data << "\n"; //IR 
        cout << "distance: " << distance << "\n"; //Lidar
        cout << "\n";


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
        //State 111 - STOP
        switch(robot_state) //Intial State is Stop
        {
            case WAIT:
                MotorPower(&motors, 0, 0); //stop
            
                //Transition condition
                if (leftIR_data == false && centerIR_data == true && rightIR_data == false) //if the center gpio is on the black line
                {
                    robot_state = FORWARD;
                    theta = 0.0;
                }

                else if (leftIR_data == false && centerIR_data == false && rightIR_data == false) //all are on white
                {
                    robot_state = WAIT;
                }

                else 
                {
                    robot_state = WAIT;
                }
                break;

            case FORWARD:
                MotorPower(&motors, 100, 100); //both at full power

                //Object Detection
                if(distance <= target_distance)
                {
                    cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0)); //blinks LED (indicating detected object)
                    MotorPower(&motors, -100, -100); //reverse

                    //Reset Wheel Encoder Counts
                    right.setZero();
                    left.setZero();

                    if(leftDistance >= 25 && rightDistance >= 25) //go backward about an inch
                    {
                        MotorPower(&motors, 0, 0); //stop
                        robot_state = FORWARD;

                    }
                }

                //Transition condition
                if (leftIR_data == true && centerIR_data == true && rightIR_data == false) //if the left & center gpio on the black line
                {
                    robot_state = LEFT;
                    theta = 0.0;
                }
                
                else if (leftIR_data == false && centerIR_data == true && rightIR_data == true) //if the right & center gpio on the black line
                {
                    robot_state = RIGHT;
                    theta = 0.0;
                }

                else if(leftIR_data == true && centerIR_data == true && rightIR_data == true) //if all are on the black line
                {   
                    robot_state = JUNCTION;
                    theta = 0.0;
                }
                break;

            case LEFT:
                MotorPower(&motors, -50, 50); //turn left

                //Transition condition
                if (abs(theta) >= turn_degrees) //Turn until theta = 90.0 degrees 
                {
                    robot_state = WAIT;
                }
                break;
            
            case RIGHT:
                MotorPower(&motors, 50, -50); //turn right

                //Transition condition
                if (abs(theta) >= turn_degrees) //Turn until theta = 90.0 degrees 
                {
                    robot_state = WAIT;
                }
                break;
            
            case JUNCTION:
                MotorPower(&motors, 100, 100); //move forward

                //Reset Wheel Encoder Counts
                right.setZero();
                left.setZero();

                //Transition condition
                if(leftDistance >= 25 && rightDistance >= 25) //go forward about an inch
                {
                    MotorPower(&motors, 0, 0); //stop

                    if (leftIR_data == false && centerIR_data == false && rightIR_data == false) //if all are on white now
                    {
                        cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0)); //blinks LED (indicating stopped at Junction)
                        //replace this to stop at junction, but test this out to see if it works
                        MotorPower(&motors, 50, -50); //turn right
                        if (abs(theta) >= turn_around) //keep turning until it has turned 180 degrees
                        {
                            robot_state = WAIT;
                        }
                    }

                    else if (leftIR_data == true && centerIR_data == true && rightIR_data == true) //if all are still on black
                        {
                        robot_state = STOP;
                        }
                }
                break;

            case STOP:
                MotorPower(&motors, 0, 0); //stop
                break;
        }
    }
}
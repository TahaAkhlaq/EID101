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


//Intialize the GPIO Pins for the IR sensors
void init_ir(void) {
    gpio_init(LEFT_IR_SENSOR);    // Initialize the left IR sensor pin (I defined this as a global variable already)
    gpio_init(CENTER_IR_SENSOR);  // Initialize the center IR sensor pin (I defined this as a global variable already)
    gpio_init(RIGHT_IR_SENSOR);   // Initialize the right IR sensor pin (I defined this as a global variable already)

    gpio_set_dir(LEFT_IR_SENSOR, false);      // Set the left IR sensor pin as input
    gpio_set_dir(CENTER_IR_SENSOR, false);    // Set the center IR sensor pin as input
    gpio_set_dir(RIGHT_IR_SENSOR, false);     // Set the right IR sensor pin as input
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
    //rcc_init_pushbutton(); //for debugging purposes

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //For IMU and Lidar
    rcc_init_i2c();     

    //IMU setup
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    //Lidar Setup
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    //Odom setup
    Left_Odom left;
    Right_Odom right;

    //Linear Distance
    float leftDistance = linear_distance(left.getCount());
    float rightDistance = linear_distance(right.getCount());


    //IR Sensors Setup
    init_ir(); 

    //Object Detection
    uint16_t distance; //lidar variable
    uint16_t target_distance = 200;


    // State Machine Riemann Sum variables
    uint32_t current_time, previous_time;
    uint32_t duration = 1000000; // 1 second

    //Starting Angle
    double theta = 0.0;

    //Turning Right and Left
    double turn_right = 90.0; //may need to change this to 65.0
    double tunr_left = -90.0; //may need to change this to 65.0

    //Turning Around
    double turn_around = 180.0; //may need to change this too depending on the value of turn_degrees

    //Intial Robot States
    robot_state_t robot_state = WAIT;

    //Initial Calculus State
    imu_state_t imu_state = DWELL;

    // Define blink_count as a global variable for indicating Junction and Obstacle
    int blink_count = 5;

    while(true)
    {   
        imu.update_pico(); //updates data
        current_time = time_us_32(); //update current time

        distance = getFastReading(&lidar); // lidar reading

        leftDistance = linear_distance(left.getCount());
        rightDistance = linear_distance(right.getCount());

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
        cout << robot_state << "\n";
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
        switch(robot_state) //Intial State is WAIT
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
                    robot_state = FORWARD;
                    theta = 0.0;
                }
                
            break;

            case FORWARD: //010
                MotorPower(&motors, 75, 75); //both at full power

                //Object Detection
                // if(distance <= target_distance)
                // {
                //     MotorPower(&motors, 0, 0);//stop

                //     for (int i = 0; i < blink_count; i++) //blink LED 5 times
                //     {
                //     cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0)); //blinks LED (indicating detected object)
                //     sleep_ms(300);
                //     }
                
                //     //Reset Wheel Encoder Counts
                //     right.setZero();
                //     left.setZero();

                //     MotorPower(&motors, -100, -100); //reverse

                //     if(leftDistance >= 25 && rightDistance >= 25) //go backward about an inch
                //     {
                //         MotorPower(&motors, 0, 0); //stop
                //         robot_state = FORWARD;

                //     }
                // }

                //Transition condition


                if (leftIR_data == true && centerIR_data == true && rightIR_data == false) //if the left & center gpio on the black line
                {
                    robot_state = LEFT;
                    theta = 0.0;
                }

                if (leftIR_data == true && centerIR_data == false && rightIR_data == false) //if the left gpio on the black line
                {
                    robot_state = LEFT;
                    theta = 0.0;
                }
                
                else if (leftIR_data == false && centerIR_data == true && rightIR_data == true) //if the right & center gpio on the black line
                {
                    robot_state = RIGHT;
                    theta = 0.0;
                }

                else if (leftIR_data == false && centerIR_data == false && rightIR_data == true) //if the right gpio on the black line
                {
                    robot_state = RIGHT;
                    theta = 0.0;
                }

                else if(leftIR_data == true && centerIR_data == true && rightIR_data == true) //if all are on the black line
                {   
                    robot_state = JUNCTION;
                    theta = 0.0;
                    //can add reset wheel encoders here before going into junction
                }

            break;

            case LEFT: //110
                MotorPower(&motors, -50, 50); //turn left

                //Transition condition
                if (abs(theta) >= turn_right) //Turn until theta = 90.0 degrees 
                {
                    robot_state = FORWARD;
                }

            break;
            
            case RIGHT: //011
                MotorPower(&motors, 50, -50); //turn right

                //Transition condition
                if (abs(theta) >= turn_clockwise) //Turn until theta = 90.0 degrees 
                {
                    robot_state = FORWARD;
                }

            break;
            
            case JUNCTION: //111

                //Reset Wheel Encoder Counts
                right.setZero();
                left.setZero();
                
                MotorPower(&motors, 100, 100); //move forward

                //Transition condition
                if(leftDistance >= 25 && rightDistance >= 25) //go forward about an inch
                {
                    MotorPower(&motors, 0, 0); //stop

                    if (leftIR_data == false && centerIR_data == false && rightIR_data == false) //if all are on white now
                    {
                       
                        for (int i = 0; i < blink_count; i++) //blink LED 5 times
                        {
                            cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0)); //blinks LED (indicating stopped at Junction)
                            sleep_ms(300);
                        }
                        
                        //replace this to stop at junction, but test this out to see if it works

                        robot_state = STOP;

                        // if (abs(theta) >= turn_around) //keep turning until it has turned 180 degrees
                        // {
                        //    //Reset Wheel Encoder Counts
                        //     right.setZero();
                        //     left.setZero();
                                
                        //     MotorPower(&motors, 100, 100); //move forward

                        //     //Transition condition
                        //     if(leftDistance >= 25 && rightDistance >= 25) //go forward about an inch
                        //     {
                        //         robot_state = WAIT;
                        //     }
                           
                        // }
                    }

                    else if (leftIR_data == true && centerIR_data == true && rightIR_data == true) //if all are still on black
                        {
                        robot_state = STOP;
                        }
                    
                    else //anything else
                        {
                    robot_state = WAIT;
                        }

                }

            break;

            case STOP: //111

                MotorPower(&motors, 0, 0); //stop

            break;
        }
    }
}
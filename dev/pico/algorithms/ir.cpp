#include "rcc_stdlib.h"
using namespace std;

typedef enum {
    FORWARD, //011
    LEFT,    //110
    RIGHT,   //011
    STOP     //111
} state_t;

//These pins are defined in pins.h under custom_libs
void init_ir(void) {
    gpio_init(LEFT_IR_SENSOR);    // Initialize the left IR sensor pin
    gpio_init(CENTER_IR_SENSOR);  // Initialize the center IR sensor pin
    gpio_init(RIGHT_IR_SENSOR);   // Initialize the right IR sensor pin

    //gpio_set_function (pin number, GPIO_FUNC_SIO); not needed 

    gpio_set_dir(LEFT_IR_SENSOR, false);      // Set the left IR sensor pin as input
    gpio_set_dir(CENTER_IR_SENSOR, false);    // Set the center IR sensor pin as input
    gpio_set_dir(RIGHT_IR_SENSOR, false);     // Set the right IR sensor pin as input
}

//interupt 
// dont put a print in interupt handler 
//volatile = the value changes

int main(void) {
    stdio_init_all();
    sleep_ms(100);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1); // Turns on LED

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    // IMU setup
    rcc_init_i2c(); 
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    //IR Sensors Setup
    //how can I setup the IR sensors?
    init_ir(); //is this all thats needed?

    // State machine variables
    uint32_t cur, prev;
    uint32_t duration = 1000000; // 1 second
    state_t state = FORWARD; //initial state
    double theta = 0.0;
    
    double targetAngle = 90.0; //target angle in degrees
    double tolerance = 2.0; //tolerance for error

//incorporate interupt 


  while(true) 
    {
        imu.update_pico(); //updates data
        cur = time_us_32(); //update current time

        // Read IR sensor data
        bool leftIR_data = gpio_get(LEFT_IR_SENSOR);       // Left IR sensor data
        bool centerIR_data = gpio_get(CENTER_IR_SENSOR);   // Center IR sensor data
        bool rightIR_data = gpio_get(RIGHT_IR_SENSOR);     // Right IR sensor data

        //State 010 - FORWARD
        if(!rightIR_data && !leftIR_data)
        {
            state == FORWARD;

            MotorPower(&motors, 100, 100);  //both forward at full power

            theta += imu.getAngVelZ()*duration/1000000.0;

            if(!leftIR_data)/*011*/  //the logic is inverted 
            {
                state = RIGHT;
                targetAngle = 90.0;
            }

            else if (!rightIR_data)/*110*/  //the logic is inverted 
            {
                state = LEFT;
                targetAngle = -90.0;
            }

            else if (leftIR_data && centerIR_data && rightIR_data)/*111*/  //the logic is inverted ?
            {
                state = STOP;
                targetAngle = 0;
            }
        }

        //State 011 - RIGHT
        if(state == RIGHT)
        {
            MotorPower(&motors, 0, 100); //left at full power

            theta += imu.getAngVelZ()*duration/1000000.0;

            //Transition condition
            if(!rightIR_data && !leftIR_data)
            {
                state = FORWARD;
                prev = cur; //Update time
            }
        }

        //State 110 - LEFT
        if (state == LEFT)
        {
            MotorPower(&motors, 100, 0); //right at full power

            theta += imu.getAngVelZ()*duration/1000000.0;

            //Transition condition
            if(!rightIR_data)
            {
                state = FORWARD;
                prev = cur; //Update time
            }
        }

        //State 111 - STOP
        if (state == STOP)
        {
            MotorPower(&motors, 0, 0); //stop

            theta = theta + imu.getAngVelZ()*duration/1000000.0;

        //Transition condition
            if(leftIR_data && centerIR_data && rightIR_data)
            {
                state = FORWARD;
                prev = cur; //Update time
            }

        }

    }
}

//how can I use the IMU to get the car to turn exactly 90 degrees?


//Different way to implement state machines? :

//     while(true){

//         switch(state){

// //State 010 - FORWARD
//             case FORWARD: 
//                 MotorPower(&motors, 100, 100);  //both forward at full power

//                 theta = theta + imu.getAngVelZ()*duration/1000000.0;

//                 if(/*011*/)
//                 {
//                     state = RIGHT;
//                 }

//                 if (/*110*/)
//                 {

//                     state = LEFT;
//                 }

//                 if(/*111*/)
//                 {
//                     state = STOP;
//                 }

//                 break;

// //State 011 - RIGHT
//             case RIGHT: 
//                 MotorPower(&motors, 50, -50); //left goes backwards, right goes forward? can this also work?
                
//                 theta = theta + imu.getAngVelZ()*duration/1000000.0;

//                 //transition conditions: 
//                 if(true){ 
//                     state = FORWARD;
//                     prev = cur; //Update time
//                 }
//                 break;

// //State 110 - LEFT
//             case LEFT:
//                 MotorPower(&motors, -50, 50); //right backwards, left forward

//                 theta = theta + imu.getAngVelZ()*duration/1000000.0;
                
//                 //transition conditions:
//                 if(true){ 
//                     state = FORWARD;
//                     prev = cur; //Update time
//                 }
//                 break;

// //State 111 - STOP
//             case STOP:
//                 MotorPower(&motors, 0, 0); //both stop

//                 theta = theta + imu.getAngVelZ()*duration/1000000.0;
                
//                 //transition conditions:
//                 if(true){ 
//                     state = FORWARD;
//                     prev = cur; //Update time
//                 }
//                 break;
//         }   

//     }


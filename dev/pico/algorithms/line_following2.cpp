#include "rcc_stdlib.h"
using namespace std;

//Robot States
typedef enum {
    FORWARD,  //state 1 (010)
    LEFT,    //state 2 (100 or 110)
    RIGHT,   //state 3 (001 or 011)
    JUNCTION, //state 4 (111)
} robot_state_t;

void init_ir(void) {
    gpio_init(LEFT_IR_SENSOR);   
    gpio_init(CENTER_IR_SENSOR);  
    gpio_init(RIGHT_IR_SENSOR); 

    gpio_set_dir(LEFT_IR_SENSOR, false);      
    gpio_set_dir(CENTER_IR_SENSOR, false);   
    gpio_set_dir(RIGHT_IR_SENSOR, false);    
}

int main()
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); // turns on LED

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    init_ir(); 
    
    robot_state_t robot_state = FORWARD;

    while (true) {
        bool leftIR_data = gpio_get(LEFT_IR_SENSOR);     
        bool centerIR_data = gpio_get(CENTER_IR_SENSOR); 
        bool rightIR_data = gpio_get(RIGHT_IR_SENSOR); 

        // State Machines
        switch (robot_state) {
            case FORWARD: // 010
                cout << "FORWARD: \n";
                MotorPower(&motors, 80, 77); //make it so that it goes straight, we know that 100, 97 is straight

                if ((leftIR_data && centerIR_data && !rightIR_data) || (leftIR_data && !centerIR_data && !rightIR_data)) // left turn
                    robot_state = LEFT;
                if ((rightIR_data && centerIR_data && !leftIR_data) || (rightIR_data && !centerIR_data && !leftIR_data)) // right turn
                    robot_state = RIGHT;
                if(leftIR_data && centerIR_data && rightIR_data) //junction
                    robot_state = JUNCTION;
                break;

            case LEFT: // 110 or 100
                cout << "LEFT: \n";
                MotorPower(&motors, 0, 80);
                if (centerIR_data && !leftIR_data && !rightIR_data) // Turn until center is black
                    robot_state = FORWARD;
                break;

            case RIGHT: // 011 or 001
                cout << "RIGHT: \n";
                MotorPower(&motors, 80, 0);
                if (centerIR_data && !leftIR_data && !rightIR_data) // Turn until center is black
                    robot_state = FORWARD;
                break;

            case JUNCTION: // 111
                cout << "JUNCTION: \n";
                MotorPower(&motors, 0, 0);

                 for (int i = 0; i < blink_count; i++) //blink LED 5 times
                        {
                            cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0)); //blinks LED (indicating stopped at Junction)
                            sleep_ms(300);
                        }
                robot_state = FORWARD;
                break;
        }
    }
}

        // if (centerIR_data) {
        //     if (leftIR_data && !rightIR_data)
        //         robot_state = LEFT; // 110 or 100
        //     else if (!leftIR_data && !rightIR_data)
        //         robot_state = FORWARD; // 010
        //     else if (!leftIR_data && rightIR_data)
        //         robot_state = RIGHT; // 011 or 001
        //     else if (leftIR_data && rightIR_data)
        //         robot_state = JUNCTION; // 111
        // } else if (!centerIR_data && rightIR_data) {
        //     robot_state = RIGHT; // 011 or 001
        // }

        //other method

        // if (centerIR_data) {
        //     if (leftIR_data) {
        //         if (rightIR_data)
        //             robot_state = JUNCTION; // 111
        //         else
        //             robot_state = LEFT; // 110 or 100
        //     } else {
        //         robot_state = FORWARD; // 010
        //     }
        // } else if (rightIR_data) {
        //     robot_state = RIGHT; // 011 or 001
        // }

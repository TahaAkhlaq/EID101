#include "rcc_stdlib.h"
using namespace std;

//which pin(s) plugging IR sensor into
#define IRPIN0 0

//helper function for setting up gpio pin
void gpio_input_setup(int pin_number){
    gpio_init(pin_number);
    gpio_set_dir(pin_number, false); //input
}

//helper function for getting IR sensor reading
bool ir_getreading(int pin_number){
    //true = black line
    //false = white background
    return gpio_get(pin_number);
}

//state machine enum setup
typedef enum{
    WAIT,
    FORWARD, 
    JUNCTION
} state_t;


int main()
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0,1); //led on

    rcc_init_pushbutton(); //set up button

    //setup gpio pin using helper function
    gpio_input_setup(IRPIN0);

    //setup motors
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    //default state is wait
    state_t state = WAIT;

    while(true){
        //for debugging
        cout << ir_getreading(IRPIN0) << "\n";

        switch(state){
            case WAIT:
                //do something first in each state
                MotorPower(&motors, 0,0); //dont move

                //check transition conditions
                if(!gpio_get(RCC_PUSHBUTTON)){
                        state = FORWARD;
                    }
                break;

            case FORWARD:
                //do something first in each state
                MotorPower(&motors, 60, 60);

                //for this example, if the IR sensor reads true (so seeing black line)
                //go to junction state
                //this is just example, you'll prob need more than one sensor to do this...
                if(ir_getreading(IRPIN0) == true){
                    state = JUNCTION;
                }
                break;

            case JUNCTION:
                //do something first in each state, in this case stop
                MotorPower(&motors, 0, 0);

                //no transition conditions for this example
                break;
        }
    }
}
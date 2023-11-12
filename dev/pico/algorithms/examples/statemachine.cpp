#include "rcc_stdlib.h"
using namespace std;

int main()
{   
    stdio_init_all();
    sleep_ms(100);
    cyw43_arch_init(); //setup 
    cyw43_arch_gpio_put(0, 1); //turns on led
    rcc_init_potentiometer(); //intialize the potentiometer 

    int state1 = 0; //State machine 1
    int state2 = 0; //State machine 2
    
    int val;
    uint32_t cur, prev;
    uint32_t duration = 1000000;

    while(true)
    {   // Update sensor readings and timing vars
        val = adc_read();
        cur = time_us_32();

//State Machine 1-----------------------------------------------------------------------
        //State 0, 0<=val<=1300
        if(state1 == 0)
        {
            //Task
            cout << "s0\n";
            // Transition conditions
            if(val >= 1301 && val <= 2600) {state1 = 1;}
            if(val >= 2601){state1 = 2;}
            
        }
        //State 1, 1301<=val<=2600

        if(state1 == 1)
        {   //Task
            cout << "s1\n";
            //Set motor power to (100, 100)
            // Transition conditions
            if(val >= 0 && val <= 1300) // State 1 -> State 0
            {
                state1 = 0;
            }
            if(val >= 2601)                 //State 1 -> State 2
            {
                state1 = 2;
            }
        }
        //State 2, val >= 2601
        if(state1 == 2)
        {
            //Task
            cout << "s2\n";
            //Transition conditions
            if(val >= 0 && val <= 1300) // State 2 -> State 0
            {
                state1 = 0;
            }
            if(val >= 1301 && val <= 2600)                        // State 2 -> State 1
            {
                state1 = 1;
            }
        }
//State machine 2 ----------------------------------------------------------------------
        if(state2 == 0)
        {
            //=Task, Dwell
            if(cur - prev >= duration)
            {
                state2 = 1;
            }
        }
        if(state2 == 1)
        {
            //Task, flip led state
            cout << "State2: 1\n";
            cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0));
            if(true)
            {
                state2 = 0;
                prev = cur;
            }
        }
    }
}
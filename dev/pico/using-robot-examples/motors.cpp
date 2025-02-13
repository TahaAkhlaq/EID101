#include "rcc_stdlib.h"
using namespace std;

int main(void)
{
    stdio_init_all();
    sleep_ms(100);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); //led on

    rcc_init_pushbutton(); //set up button

    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    while(true)
    {   
        if(!gpio_get(RCC_PUSHBUTTON)) //if NOT gpio (if gpio is low)
        {
            MotorPower(&motors, 100, 100);  //both forward at full power
            sleep_ms(5000);
            MotorPower(&motors, 0, 0);  //both forward at full power
        }
    }
}
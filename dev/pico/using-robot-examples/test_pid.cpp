#include "rcc_stdlib.h"
using namespace std;

int main(void)
{
    stdio_init_all();
    cyw43_arch_init();
    rcc_init_potentiometer(); //setup potentiometer on ADC2
    sleep_ms(1000);   

    cyw43_arch_gpio_put(0, true); //led on

    Servo s3; //struct
    ServoInit(&s3, 18, false, 50); //attach to pin 18
    ServoOn(&s3); //enable PWM
    
    while(true)
    {   
        PID_control_config_t my_pid;
        my_pid.kp = 0;
        
    }
}
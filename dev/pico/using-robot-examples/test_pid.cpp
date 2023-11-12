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



#include "rcc_stdlib.h"
#include "rotate.h"

using namespace std;

VL530X lidar;

void rotate(VL530X lidar) {

}


int main()
{   
    stdio_init_all();
    sleep_ms(100);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1); //turns on led

    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    while(true)
    {
        rotate(3)
        rotate(lidar);
        uint16_t dist = getFastReading(&lidar);
        cout << "distance: " << dist << "\n";
        if (dist > 500) {
            MotorPower(100, 100);
            sleep_ms(100);
            if (turn_condition) {
                rotate(3);
            }
        }
        sleep_ms(100);
    }
}
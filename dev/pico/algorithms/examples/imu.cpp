#include "rcc_stdlib.h"
using namespace std;

typedef enum{
    DWELL,
    INTEGRATE
}state_t;

int main() {
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1); //turns on led

    rcc_init_i2c(); //setup i2c
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    // State machine variables
    uint32_t cur, prev;
    uint32_t duration = 10000; //10000 us to be 10ms
    state_t state = DWELL;
    double theta = 0.0;

    while(true) 
    {
        imu.update_pico(); //updates data
        cur = time_us_32(); //Update current time

// State machine
        //State 0 - DWELL
        if(state == DWELL)
        {
            // Do nothing
            //Transition conditions
            if(cur - prev >= duration)
            {
                state = INTEGRATE;
            }
        }
        if(state == INTEGRATE)
        {
            //Do our integration (LEFT HAND riemman sum)
            theta = theta + imu.getAngVelZ()*duration/1000000.0;
            //Transition condition
            if(true)
            {
                state = DWELL;
                prev = cur; //Update time
            }
        }
        cout << "Theta: " << theta << "\n";
    }
}
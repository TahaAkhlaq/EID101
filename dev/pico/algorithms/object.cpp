#include "rcc_stdlib.h"
using namespace std;

typedef enum {
    UNDETECTED,
    DETECTED
} object_state_t;

const int MIN_ANGLE = 25;
const int MID_ANGLE = 50;
const int MAX_ANGLE = 75;

int main() {
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); // turns on LED

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    // Lidar setup
    rcc_init_i2c();        // setup pico i2c
    VL53L0X lidar;          // class
    rcc_init_lidar(&lidar); // setup lidar

    // Servo setup
    Servo s3;                            // struct
    ServoInit(&s3, 18, false, 50);       // setup on pin 18
    ServoOn(&s3);                        // enables PWM

    uint16_t distance;                   // lidar variable
    uint16_t target_distance = 200;     // distance of the object

    object_state_t object_state = UNDETECTED;

    while (true) {
        distance = getFastReading(&lidar);
        cout << "distance: " << distance << "\n";
        
        int angle;
        
        switch (object_state) {
        case UNDETECTED:
            cout << "UNDETECTED: \n\n";

            MotorPower (&motors, 80, 77);

            for (angle = MIN_ANGLE; angle <= MAX_ANGLE; angle++) {
                ServoPosition(&s3, angle);
                sleep_ms(20); 
            }
            
            for (angle = MAX_ANGLE; angle >= MIN_ANGLE; angle--) {
                ServoPosition(&s3, angle);
                sleep_ms(20);
            }

            if(distance <= target_distance)
                object_state = DETECTED;

            break;

        case DETECTED:
            cout << "DETECTED: \n\n";

           // Gradually return to the mid-angle
            if (angle < MID_ANGLE) {
                while (angle < MID_ANGLE) {
                    angle++;
                    ServoPosition(&s3, angle);
                    sleep_ms(20);
                }
            } else if (angle > MID_ANGLE) {
                while (angle > MID_ANGLE) {
                    angle--;
                    ServoPosition(&s3, angle);
                    sleep_ms(20);
                }
            }

            MotorPower(&motors, 0, 0);
            
            break;
        }
    }

    return 0;
}
#include "rcc_stdlib.h"
using namespace std;

#define rot360   260
#define rot90     65

// false = Left, True = right
void rotate90(Motor* motorPointer, int current_count, bool dir) {
    while (current_count < (current_count + rot90)) {
        if (dir) {
            MotorPower(motorPointer, 0, 50);
        } else {
            MotorPower(motorPointer, 50, 0);
        }
        sleep_ms(100);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0,1);

    rcc_init_pushbutton();

    Motor motors;
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);
    MotorsOn(&motors);
    printf("AFTER MOTORS");

    Left_Odom left;
    Right_Odom right;
    printf("AFTER ODOM");

    int left_count = 0;
    int right_count = 0;
    int rot360deg = 260; //number counts for 360 degree rotation
    int rot90deg = 65; //number of counts for a 90 degree rotation


    while(true)
    {
        left_count = left.getCount();
        right_count = right.getCount();

        if(!gpio_get(RCC_PUSHBUTTON))
        {
            rotate90(&motors, left_count, false);
            sleep_ms(100);
            rotate90(&motors, right_count, true);
        }
    }

}
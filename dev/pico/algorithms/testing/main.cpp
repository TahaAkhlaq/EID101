##include "rotate.h"
#include "rcc_stdlib.h"

using namespace std;

int main() 
{
    // Initialize the sensors and motors
    stdio_init_all();
    sleep_ms(1000);
    //motor using 100 sleep, imu uses 1500 sleep, odom uses 1000 sleep, so which one do I use?
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); // LED on

    //Declare the constant variables here

    //Motors setup
    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM

    // IMU setup
    rcc_init_i2c(); // Setup I2C
    MPU6050 imu; //class
    imu.begin(i2c1); //adds to i2c1
    imu.calibrate(); //hold robot still

    // Odom (wheel encoders) setup
    Left_Odom leftOdom; // class left odom 
    Right_Odom rightOdom; // class right odom 

    // Reset wheel encoders count (comment if not needed)
    leftOdom.setZero();
    rightOdom.setZero();
    
    //Functions

    // Call the drive functions to move the robot
    driveForward(1000.0, leftOdom, rightOdom, motors);  // Move forward 1000 mm
    driveBackward(500.0, leftOdom, rightOdom, motors);  // Move backward 500 mm

    //Call the rotate functions to turn the robot
    rotate(1) //make 1 clockwise 90 degree turn

    return 0;
}


// converions for the wheel encoders

#include "rcc_stdlib.h"

#define WHEEL_DIAM      0.08
#define CPR 80
#define PI 3.14159
#define DUMMY_TICKS 124

float ticks2meters(unsigned long ticks)
{
  return WHEEL_DIAM*PI/ticks;
}

float ticks2centimeters(unsigned long ticks)
{
  return (WHEEL_DIAM*PI/ticks) * 100.0;
}

int main(void)
{
  stdio_init_all();

  std::cout << "A " << '\n';
  std::cout << DUMMY_TICKS << '\n';
  std::cout << " microsecond duration means an object is " << '\n';
  std::cout << duration2meters(DUMMY_TICKS) << '\n';
  std::cout << " meters or " << '\n';
  std::cout << duration2centimeters(DUMMY_TICKS) << '\n';
  std::cout << " centimeter away\n";
}


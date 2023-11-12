#include "rcc_stdlib.h"
using namespace std;

uint32_t count = 0;

int main()
{
    stdio_init_all();    
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0,1); //turns on LED
    rcc_init_pushbutton(); //set up button

    // Set up a GPIO pin as an input
    gpio_init(27);
    gpio_set_function(27, GPIO_FUNC_SIO);
    gpio_set_dir(27, false);
    
    while(true)
    {   
        if(!gpio_get(27)) //if NOT gpio (if gpio is low)
        {
            //do stuff here when 27 LOW!
            cout << "PIN 27 LOW!\n";
            count++;
        }
            cout << "Count: " << count << "\n"; 
    }
}
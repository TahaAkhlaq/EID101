#include "rcc_stdlib.h"
using namespace std;
//Define GPIO events that trigger the ISR
// Remember to use BITWISE OR operator (|) not the regular OR operator (||)
const uint32_t reflectiveIR_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
const uint32_t button_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
volatile uint32_t reflectiveIR_count=0; //Volatile because value is edited in ISR
volatile uint32_t button_count=0;
void button_isr()
{
    cout << "BUTTON ISR\n";
        // Check if the isr reason for pin 27 is the same as button_events
    if(gpio_get_irq_event_mask(22) & button_events) {
        // Acknowledge the interrupt request (pin, events)
        gpio_acknowledge_irq(22, button_events);
        // Do something when ISR is activated
        button_count++;
    }
}
void reflectiveIR_isr()
{   
    cout << "IR ISR\n";
    // Check if the isr reason for pin 27 is the same as button_events
    if(gpio_get_irq_event_mask(27) & reflectiveIR_events) {
        // Acknowledge the interrupt request (pin, events)
        gpio_acknowledge_irq(27, reflectiveIR_events);
        // Do something when ISR is activated
        reflectiveIR_count++;
    }
}
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
    // Setup a GPIO pin as an interrupt pin with events
    gpio_set_irq_enabled(27, reflectiveIR_events, true);
    gpio_add_raw_irq_handler(27, &reflectiveIR_isr);
    // Setup button GPIIO as IRQ
    gpio_set_irq_enabled(RCC_PUSHBUTTON, button_events, true);
    gpio_add_raw_irq_handler(RCC_PUSHBUTTON, &button_isr);
    Left_Odom left;
    Right_Odom right;
    while(true)
    {   
            cout << "Button count: " << button_count << " | ir_count: " << 
            reflectiveIR_count << "right_count: " << right.getCount() << "\n";
            sleep_ms(100);
    }
}
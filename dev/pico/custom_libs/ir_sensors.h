//Custon IR Sensor Library
void init_ir(void) {
    gpio_init(LEFT_IR_SENSOR);
    gpio_init(CENTER_IR_SENSOR);
    gpio_init(RIGHT_IR_SENSOR);

    gpio_set_dir(LEFT_IR_SENSOR, false);
    gpio_set_dir(CENTER_IR_SENSOR, false);
    gpio_set_dir(RIGHT_IR_SENSOR, false);
}
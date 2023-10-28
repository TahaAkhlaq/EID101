#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#define KP 1.0
#define KI 0.0
#define KD 0.0

typedef struct {

    float sensor1;
    float sensor2;
    float sensor3;

} SensorData;

// Function prototypes
void initializeHardware();
void readSensors(SensorData *sensors);
void controlMotors(float correction);

#endif

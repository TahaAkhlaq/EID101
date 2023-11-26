//--ODOM--//
#define WHEEL_DIAM 80
#define PI 3.14159
#define SLOTS 20
#define CPR 40

//Odom
float linear_distance(unsigned long count)
{
  return  (WHEEL_DIAM * PI * count) / (CPR); 
  //20 is about a foot or about 300 mm 
}

//Distance
float distance_convertor(float targetDistance)
{
    // 1 tick = 6.28318mm
    return (targetDistance / 6.28318);  //converts mm to ticks
}
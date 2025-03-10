#include "VEGA_MLX90614.h"
#include <PID_v1.h>
VEGA_MLX90614 mlx(3,4); // SDA , SCL
#define PIN_OUTPUT 2
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
void setup()
{
  delay(2000);
  Serial.begin(9600); 
  Serial.println("+-----[ MLX90614 Temperature Sensor with VEGA ARIES Boards ]-----+");
    //initialize the variables we're linked to;
  Setpoint = 40;
  Input = mlx.mlx90614ReadTargetTempC();
  //turn the PID on
  myPID.SetMode(AUTOMATIC); 
}
void loop () {
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  myPID.Compute();
  Input = mlx.mlx90614ReadTargetTempC();
  analogWrite(PIN_OUTPUT, Output);
  delay(500);
  //read temp from probe and display to 7segment display...
  Serial.println(Output);
}


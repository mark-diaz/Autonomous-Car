#include "utility/Sensor.h"

// Phototransistor and Sensor Fusion Containers
uint16_t sensorValues[8];
int16_t sensorFusion = 0;
int16_t prevSensorFusion = 0;
int16_t correction = 0;

// Motor Pins
const int left_nslp_pin = 31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin = 11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin = 29;
const int right_dir_pin = 30;
const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

// Bumper and LED Pin
uint16_t bumpSw2_pin = 6;
uint16_t bumpSw2Reading = 0;
const int LED_RF = 75;

// Parameters
const int baseSpeed = 35;
const float kp = 0.002; 
const float kd = 0.001; 
const int delayTime = 1310;
const int weight1 = 2;
const int weight2 = 14;
const int weight3 = 12;
const int weight4 = 10;

// Temporary Containers and Flags
int currLeftSpeed = 0;
int currRightSpeed = 0;
bool hasTurned = false;
bool prevBlackLine = false;

// Function Prototypes
void calibrate();
void uTurn();
bool blackLine();

// setup will run only once when the microcontroller is turned on
void setup()
{
  Sensor_Init();
  
  // motor and bumper pins
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  pinMode(LED_RF, OUTPUT);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
  pinMode(bumpSw2_pin, INPUT_PULLUP);
  pinMode(bumpSw3_pin, INPUT_PULLUP);
  pinMode(LED_RF, OUTPUT); 

  delay(3000);
  
  // set the motors base speed
  analogWrite(left_pwm_pin,baseSpeed);  
  analogWrite(right_pwm_pin,baseSpeed);  

  // set the data rate in bits per second for serial data transmission
  Serial.begin(9600); 
  delay(2000);
}

// loop will run after setup during the entire time the microcontroller is on
void loop()
{
  // read raw sensor values
  Sensor_read_IR(sensorValues);

  // calibrate if the bumper switch has been pressed
  bumpSw2Reading = digitalRead(bumpSw2_pin);
  if (bumpSw2Reading == 0)
  {
    calibrate();
  }

  // this checks for two straight black lines - necessary for phantom black lines
  if (blackLine() && prevBlackLine)
  {
    if(hasTurned) // car has completed track and seen its second black line
    {
      analogWrite(left_pwm_pin, 0);  
      analogWrite(right_pwm_pin, 0);  
    }
    else // first black line
    {
      uTurn();
    }
  }
  
  if (blackLine())
  {
    prevBlackLine = true;
  }
  else
  {
    prevBlackLine = false;
  }

  prevSensorFusion = sensorFusion;

  // Sensor fusion calculated using 2-14-12-10-10-12-14-2 as the weights for the 8 phototransistor sensors
  sensorFusion = (weight1*sensorValues[0] + weight2*sensorValues[1] + weight3*sensorValues[2] + weight4*sensorValues[3]
                  - weight4*sensorValues[4] - weight3*sensorValues[5] - weight2*sensorValues[6] - weight1*sensorValues[7]);

  // calculates the correction using the proportional and derivative error
  correction = (sensorFusion * kp) + ((sensorFusion - prevSensorFusion) * kd);

  // correct the error and keep car on track
  analogWrite(left_pwm_pin,baseSpeed + correction);  
  analogWrite(right_pwm_pin,baseSpeed - correction);
}

void uTurn()
{
    // Stop the car
    analogWrite(left_pwm_pin, 0);  
    analogWrite(right_pwm_pin, 0);  
    delay(200);
    
    // Change the directions of wheels
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, HIGH);
    analogWrite(left_pwm_pin, 50);  
    analogWrite(right_pwm_pin, 50);  
  
    // rotate car 180 degrees using the correct delay time
    delay(delayTime);
    analogWrite(left_pwm_pin, 0);  
    analogWrite(right_pwm_pin, 0);  

    // set right motor to correct direction
    digitalWrite(right_dir_pin, LOW);

    delay(200);
    hasTurned = true;
}

// detects a horizonal black line meaning the car has to turn around or stop
bool blackLine()
{
  int sum = 0;
  for(int i = 0; i < 8; i++)
  {
    sum += sensorValues[i];
  }

  // if the sensor read above a certain threshold, we detect a black line
  if(sum >= 19500)
  {
    return true;
  }

  return false;
}

// calibrate sensors for a new track
void calibrate()
{
  analogWrite(left_pwm_pin, 0);  
  analogWrite(right_pwm_pin, 0);  
  delay(2000);

  digitalWrite(LED_RF, HIGH); 
  Sensor_read_IR(sensorValues);

  delay(2000);
  digitalWrite(LED_RF, LOW);

  analogWrite(left_pwm_pin, baseSpeed);  
  analogWrite(right_pwm_pin, baseSpeed);  
}

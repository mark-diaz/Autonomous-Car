#include <ECE3.h>

// Phototransistor and Sensor Fusion Containers
uint16_t sensorValues[8];
uint16_t sensorValuesMin[8] = {608.4,608.4,565.6,561.2,542.8,552.2,566,684.2};
uint16_t sensorValuesMax[8] = {1791.6,1791.6,1834.4,1262.4,1397.2,1847.8,1834,1715.8};
uint16_t sensorValuesNormalized[8];
int16_t sensorFusion = 0;
int16_t prevSensorFusion = 0;
int16_t correction = 0;
int16_t cummulative_error = 0;

// Motor Pins
const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

// Bump and LED Pins
uint16_t bumpSw2_pin = 6;
uint16_t bumpSw2Reading;
uint16_t bumpSw3_pin = 27;
uint16_t bumpSw3Reading;
const int LED_RF = 75;

// Parameters
const int baseSpeed = 35;
const float kp = 0.002; 
const float kd = 0; 
const int delayTime = 1310;

// Containers
int currLeftSpeed = 0;
int currRightSpeed = 0;
bool hasTurned = false;
bool prevBlackLine = false;

// Function Prototypes
void calibrate();
void uTurn();

void setup()
{
  ECE3_Init();
  
  // motor and umper switch pins
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

  // set our motors base speed
//  ECE3_read_IR(sensorValuesMin);

  delay(3000);
  
  analogWrite(left_pwm_pin,baseSpeed);  
  analogWrite(right_pwm_pin,baseSpeed);  

//  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
//  delay(2000);
  
}


void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
//
//  bumpSw2Reading = digitalRead(bumpSw2_pin);
//  bumpSw3Reading = digitalRead(bumpSw3_pin);
  
//  if (bumpSw2Reading == 0 || bumpSw3Reading == 0)
//  {
//    calibrate();
//  }

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance

//  for (unsigned char i = 0; i < 8; i++)
//  {
//      sensorValuesNormalized[i] = ((sensorValues[i] - sensorValuesMin[i]) * 1000) / (sensorValuesMax[i]); // normalize values between 1-1000
//      Serial.print(sensorValues[i]);
//      Serial.print("\t");
//  }
//  Serial.print("\n");

//    // Outer sensors are reading black outer lines
//  if(sensorValues[0] >= 2400 || sensorValues[7] >= 2400)
//  {
//    sensorValuesNormalized[0] -= 1500;
//    sensorValuesNormalized[7] -= 1500;
//    analogWrite(left_pwm_pin,baseSpeed - 10);  
//    analogWrite(right_pwm_pin,baseSpeed + 10);  
//  }


    if (turnAround() && prevBlackLine && !hasTurned)
  {
//    Serial.print("LOOK AT ME");
    uTurn();
  }

  
  if (turnAround())
  {
    prevBlackLine = true;
  }
  else
  {
    prevBlackLine = false;
  }
           

//  prevSensorFusion = sensorFusion;

  sensorFusion = (2*sensorValues[0] + 14*sensorValues[1] + 12*sensorValues[2] + 10*sensorValues[3]
                        - 10*sensorValues[4] - 12*sensorValues[5] - 14*sensorValues[6] - 2*sensorValues[7]);

//  correction = (sensorFusion * kp) + ((sensorFusion - prevSensorFusion) * kd);
  correction = sensorFusion * kp;


  analogWrite(left_pwm_pin,baseSpeed + correction);  
  analogWrite(right_pwm_pin,baseSpeed - correction);  
  
  //Serial.print(sensorFusion);

}


void uTurn()
{
    analogWrite(left_pwm_pin, 0);  
    analogWrite(right_pwm_pin, 0);  
    delay(200);
    
    
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, HIGH);
  
    
    analogWrite(left_pwm_pin, 50);  
    analogWrite(right_pwm_pin, 50);  
  
    delay(delayTime);
    
    
    analogWrite(left_pwm_pin, 0);  
    analogWrite(right_pwm_pin, 0);  
    
    digitalWrite(right_dir_pin, LOW);

    delay(200);
    hasTurned = true;
}

bool turnAround()
{
  int sum = 0;
  for(int i = 0; i < 8; i++)
  {
    sum += sensorValues[i];
  }
//  Serial.print("Current sum: ");
//  Serial.print(sum);
//  Serial.print("\n");
//  delay(1000);

  if(sum >= 19500)
  {
    return true;
  }

  return false;

}

//void calibrate()
//{
//  analogWrite(left_pwm_pin, 0);  
//  analogWrite(right_pwm_pin, 0);  
//  delay(2000);
//
//  digitalWrite(LED_RF, HIGH); // LED on when reading mins
//  ECE3_read_IR(sensorValuesMin);
//  
//  delay(2000);
//  digitalWrite(LED_RF, LOW);
//  
//  
//  analogWrite(left_pwm_pin, baseSpeed);  
//  analogWrite(right_pwm_pin, baseSpeed);  
//}

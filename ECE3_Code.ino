#include <ECE3.h>

// Phototransistor and Sensor Fusion Containers
uint16_t sensorValues[8];
uint16_t sensorValuesMin[8] = {608.4,608.4,565.6,561.2,542.8,552.2,566,684.2};
uint16_t sensorValuesMax[8] = {1791.6,1791.6,1834.4,1262.4,1397.2,1847.8,1834,1715.8};
uint16_t sensorValuesNormalized[8];
int16_t sensorFusion = 0;

// Motor Constants
const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;
int wheelSpd = 80;
int distance = 300;


void setup()
{
  ECE3_Init();

  // potentially have some sort of white paper calibration to calculate mins later
  // read minimum values by calibrating over paper
  ECE3_read_IR(sensorValuesMin);

  // motor output
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);


  resetEncoderCount_left();
  resetEncoderCount_right();
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

  
}


void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance

  
  for (unsigned char i = 0; i < 8; i++)
  {
      //Serial.print(sensorValues[i]);

      sensorValuesNormalized[i] = ((sensorValues[i] - sensorValuesMin[i]) * 1000) / (sensorValuesMax[i]); // normalize values between 1-1000
    
      //Serial.print(sensorValuesNormalized[i]);

      

           
 
    
      Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

  }

  sensorFusion = (-8*sensorValuesNormalized[0] -4*sensorValuesNormalized[1] -2*sensorValuesNormalized[2] - sensorValuesNormalized[3]
                        + sensorValuesNormalized[4] + 2*sensorValuesNormalized[5] + 4*sensorValuesNormalized[6] + 8*sensorValuesNormalized[7])/4;
  Serial.print(sensorFusion);
  if(sensorFusion < 0)
  {
    
  }
  
  Serial.println();
  delay(50);



  
}

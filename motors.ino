

// ------------------------------------ private data -----------------------------------

// ------------------------------------- public code -----------------------------------

#if defined(ESP_PLATFORM)
const int MotPin0 = 32;  //HR
const int MotPin1 = 33;  //VR
const int MotPin2 = 25;  //HL
const int MotPin3 = 35; //26;  //VL
const int MotChannel0 = 0;
const int MotChannel1 = 1;   
const int MotChannel2 = 2;
const int MotChannel3 = 3;

void motors_write() 
{
  ledcWrite(MotChannel0, servo[0]);
  ledcWrite(MotChannel1, servo[1]);
  ledcWrite(MotChannel2, servo[2]);
  ledcWrite(MotChannel3, servo[3]);
}

void motors_init() 
{
  ledcSetup(MotChannel0, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
}

#elif defined(ADAFRUIT_FEATHER_M0)
#include <Servo.h> //Using servo library to control ESC

Servo esc1, esc2, esc3, esc4;

void motors_write() 
{
  esc1.writeMicroseconds(servo[0]);
  esc2.writeMicroseconds(servo[1]);
  esc3.writeMicroseconds(servo[2]);
  esc4.writeMicroseconds(servo[3]);
}

void motors_init()
{
  int minPulseRate  = 1000;
  int maxPulseRate  = 2000;

  esc1.attach(5, minPulseRate, maxPulseRate);
  esc1.write(0); // init esc with 0 value

  esc2.attach(12, minPulseRate, maxPulseRate);
  esc2.write(0); // init esc with 0 value

  esc3.attach(11, minPulseRate, maxPulseRate);
  esc3.write(0); // init esc with 0 value

  esc4.attach(6, minPulseRate, maxPulseRate);
  esc4.write(0); // init esc with 0 value
}

#endif

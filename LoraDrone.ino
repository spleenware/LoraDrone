

#if defined(ESP_PLATFORM)
  #define LED 2
#else
  #define LED   13
#endif

#define CYCLETIME 2

enum ang { ROLL, PITCH, YAW };

#define MINTHROTTLE 1090
#define MIDRUD 1495
#define THRCORR 19

#define CHANNELS  8

#define THR 0
#define ROL 1
#define PIT 2
#define RUD 3
#define AU1 4
#define AU2 5

// -------------------------------------------- data ------------------------------------------------

uint16_t servo[4] = { 1000, 1000, 1000, 1000 };
float angle[2] = {0,0};

float yawRate = 5.0;
float rollPitchRate = 5.0;

float P_PID = 0.14;    // P8
float I_PID = 0.00;    // I8
float D_PID = 0.08;    // D8

//0.8 0.01 0.5 a little shaky, on the edge
//0.8 0.01 0.9 a little shaky, good to fly

float P_Level_PID = 0.40;   // P8
float I_Level_PID = 0.01;   // I8
float D_Level_PID = 0.05;   // D8

int16_t gyroData[3];
int16_t rcCommand[] = {0,0,0};
int16_t rcThrottle;
int16_t rcValue[CHANNELS];  // in us, center = 1500
uint8_t seqno;

#define GYRO     0
#define STABI    1
int8_t flightmode;
int8_t oldflightmode;
uint8_t armct = 0;

boolean armed = false;

unsigned long flight_timer = 0;

// -------------------------------------------- code ------------------------------------------------

void setup()
{
  Serial.begin(9600);

  radio_init();
  motors_init();
  gyro_init();

  delay(500); 
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop()
{
  radio_loop();

  if (millis() - flight_timer > CYCLETIME) {
    flight_timer = millis();

    flight_modes();
    gyro_getADC();
    gyro_acc_getADC();

    imu_getEstimatedAttitude();

    flight_pid();
    flight_quadx_mix();

    motors_write();
  }
}

#include <SPI.h>
#include <LoRa.h>

#if defined(ESP_PLATFORM)

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define LED 2

#elif defined(ADAFRUIT_FEATHER_M0)

#define SS      8   // GPIO8 -- SX1278's CS
#define RST     4   // GPIO4 -- SX1278's RESET
#define DI0     3   // GPIO3 -- SX1278's IRQ(Interrupt Request)

#define LED   13

#endif

#define BAND    918E6

// ------------------------------------ private data -----------------------------------

// Transmitting power in dBm: 2 to 20, default 17
static const byte tx_power_low = 6;    //  4=>2.5mW; 6=>5mW; 10=>10mW
static const byte tx_power_high = 20;  //  12=>16mW; 14=>25mW; 16=>40mW; 18=>63mW; 20=>100mW
static const byte tx_power_step = 2;

const int power_thr_high = -90;  // testing: 190 (-65); flying:  180 (-75)
const int power_thr_low = -100;   // testing: 180 (-75); flying:  160 (-95)

// Signal bandwidth and frame time 
// full range 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, 500E3.
// measured (16Bytes from Transmitter, 6Bytes from Receiver : 
//    SF6,BW500 => 10ms; SF6,BW250 => 20ms; SF6,BW125 => 40ms; SF6,BW62,5 => 80ms
static const unsigned long BW_low = 125E3;   //Hz
static const unsigned long BW_high = 250E3;  //Hz
static const unsigned long F_rate_low = 45000;  //us
static const unsigned long F_rate_high = 25000; //us

#define frequency_step  100000
static const uint8_t hop_list[] = { 5, 7, 12 };

#define TX_POWER_DELAY_FILTER sizeof(hop_list)*2
#define LOST_FRAMES_COUNT 30  // sizeof(hop_list) * 10

#define FAILSAFE_DELAY_MS 800

static uint8_t current_channel = 0;
static int RX_RSSI;  // RSSI on receiver side
static byte current_power = tx_power_low;
static unsigned long lost_frames = 0;
static byte power_delay_counter = TX_POWER_DELAY_FILTER;

enum stateMachineDef { SETUP = 0, TRANSMIT = 1, RECEIVE = 2, BIND = 3 };

static stateMachineDef stateMachine = RECEIVE;
static unsigned long TX_period = F_rate_low;  // 7700 / 20000 / 40000  us
static unsigned long RX_last_frame_received = 0, RX_hopping_timeout = 0;

static unsigned int pending_response_id = 0;  // if non-zero, the id of command we are responding to

// command codes
#define CMD_ACC_CALIBRATE  0x01

//static uint8_t calculated_rssi = 0;
//static uint8_t calculated_lost_frames_rssi = 0;

// ------------------------------------- private code -----------------------------------

static byte power_increase() {
  if (current_power < tx_power_high)
    current_power = current_power + tx_power_step;
  else
    return current_power;
  LoRa.sleep();
  LoRa.setTxPower(current_power);
  LoRa.idle();
  return current_power;
}

static byte power_decrease() {
  if (current_power > tx_power_low)
    current_power = current_power - tx_power_step;
  else
    return current_power;
  LoRa.sleep();
  LoRa.setTxPower(current_power);
  LoRa.idle();
  return current_power;
}

static void hop_to_next() {
  LoRa.sleep();

  current_channel++;
  if (current_channel >= sizeof(hop_list))
    current_channel = 0;

  long f = BAND + (hop_list[current_channel] * frequency_step);
  #ifdef DEBUG_CH_FREQ
    Serial.print(f);
  #endif

  LoRa.setFrequency(f);
  LoRa.idle();
}

static int receive_data(byte buf[], int max_len) {
  int packetSize = LoRa.parsePacket(max_len);
  if (packetSize) {
    int i = 0;

    while (i < packetSize && LoRa.available()) {
      buf[i++] = (byte) LoRa.read();
    }
    RX_RSSI = LoRa.packetRssi();
  }
  return packetSize;
}

static bool send_data(byte buf[], int len) {
  int sent;
  if (len == 0)
    return false;

  LoRa.beginPacket(true);   // false - explicit, true - implicit
  sent = LoRa.write(buf, len);
  LoRa.endPacket();
  //TX_Buffer_Len = 0;
  return sent > 0;
}

static uint8_t calculate_lost_frames_rssi(unsigned long lost_frames) {
  // calculate RSSI basing on lost frames
  // lost frames may be given as
  // - a counter of lost frames - no filter - short response, first good frame resets to zero, difficult to see when frame losts are shorter than half a second
  // - an average of frames lost in portion of time - slight delay in response, easier to see when conditions are going bad
  return 100 - round(100 * (lost_frames > LOST_FRAMES_COUNT ? LOST_FRAMES_COUNT : lost_frames) / LOST_FRAMES_COUNT); // 30 frames is a history
}

/**
 * Calculate RSSI based RSSI reported by transmitter
 * Return in %.
 * Easy but not reliable - I reported significant packets losts (Failsafe activated) when signal dropped below 55% (-102dB) 
 * It looks like is would be good to adapt the dynamics to sensitivity (SF and BW dependent) and TX power
 *
static uint8_t calculate_rssi(int tr_rssi) {
  // Simpliest method used from Lora-net project
  tr_rssi = 157 + tr_rssi; // entire link budget

  // Method based on OpenLRSng (although not identical)
  // if the result from the link budget is less than 50
  // use lost packets as the remaining  50% calculation
  if (tr_rssi < 50) {
    // TODO
  }
  // Setting frames to show it in "%"
  if (tr_rssi > 100) tr_rssi = 100;
  if (tr_rssi < 0)  tr_rssi = 0;
  return (failsafe_state ? 0 : tr_rssi);
} */

static void decodeRcData(unsigned char RX_Buffer[]) {
  char i = 1;
  rcValue[0]  = (uint16_t) ((RX_Buffer[i+0]    |RX_Buffer[i+1] <<8)                     & 0x07FF);
  rcValue[1]  = (uint16_t) ((RX_Buffer[i+1]>>3 |RX_Buffer[i+2] <<5)                     & 0x07FF);
  rcValue[2]  = (uint16_t) ((RX_Buffer[i+2]>>6 |RX_Buffer[i+3] <<2 |RX_Buffer[i+4]<<10)    & 0x07FF);
  rcValue[3]  = (uint16_t) ((RX_Buffer[i+4]>>1 |RX_Buffer[i+5] <<7)                     & 0x07FF);
  rcValue[4]  = (uint16_t) ((RX_Buffer[i+5]>>4 |RX_Buffer[i+6] <<4)                     & 0x07FF);
  rcValue[5]  = (uint16_t) ((RX_Buffer[i+6]>>7 |RX_Buffer[i+7] <<1 |RX_Buffer[i+8]<<9)     & 0x07FF);
  rcValue[6]  = (uint16_t) ((RX_Buffer[i+8]>>2 |RX_Buffer[i+9] <<6)                     & 0x07FF);
  rcValue[7]  = (uint16_t) ((RX_Buffer[i+9]>>5 |RX_Buffer[i+10] <<3) & 0x07FF);
}

static void processCommand(unsigned char RX_Buffer[]) {

  unsigned int cmd_id = RX_Buffer[2] | (RX_Buffer[3] << 8);  // extract the command-id

  switch (RX_Buffer[1]) {
    case CMD_ACC_CALIBRATE:
      if (radioMode == DISARMED) {
        gyro_acc_start_calibrating(cmd_id);
      }
      break;
  }
}

// ------------------------------------- public code -----------------------------------

void radio_init() {
#if defined(ESP_PLATFORM)
  SPI.begin(SCK, MISO, MOSI, SS);
#elif defined(ADAFRUIT_FEATHER_M0)
  SPI.begin();
#endif
  
  LoRa.setPins(SS, RST, DI0);  

  while (!LoRa.begin(BAND)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setTxPower(current_power);
  LoRa.setSpreadingFactor(6); // 6 to 12 - 6 requires IMPLICIT  
  LoRa.setSignalBandwidth(BW_high);
  LoRa.enableCrc();

  rcValue[THR] = 1000;
  rcValue[AU1] = 1500;
  rcValue[AU2] = 1500;
}

void radio_send_response_to(unsigned int cmd_id)
{
  pending_response_id = cmd_id;
}

void radio_loop()
{
  static unsigned char  rx_buf[12];
  static unsigned char  tx_buf[6];

  switch (stateMachine) {
    case RECEIVE:
      //Serial.println("Test");
      if (int s = receive_data(rx_buf, 12)) {
          #if defined(DEBUG_ANALYZER) || defined(DEBUG_CH_FREQ)
          Serial.println(""); Serial.print("\t");
          #endif

        if (rx_buf[0] == 'S') {   // RC channel data
          decodeRcData(rx_buf);

          if (rcValue[AU2] >= 1800 && radioMode != ARMED) {
            radioMode = ARMED;
            digitalWrite(LED, HIGH);
          } else if (rcValue[AU2] < 1800 && radioMode != DISARMED) {
            radioMode = DISARMED;
            digitalWrite(LED, LOW);
          }
        } else if (rx_buf[0] == 'C') {   // 'command' data
          processCommand(rx_buf);
        }
        stateMachine = TRANSMIT;   // transmit 'ack' next

        #ifdef DEBUG_ANALYZER
          Serial.print(millis() - RX_last_frame_received); Serial.print("ms\t");
        #endif
        RX_last_frame_received = millis();
        
        RX_hopping_timeout = micros()  + (TX_period * 3 / 2);
        if (RX_RSSI < power_thr_low && power_delay_counter-- == 0) {
          power_increase();
          power_delay_counter = TX_POWER_DELAY_FILTER;
        }
        if (RX_RSSI > power_thr_high && power_delay_counter-- == 0) {
          power_decrease();
          power_delay_counter = TX_POWER_DELAY_FILTER;
        }
      }
      break;
    case TRANSMIT:
      tx_buf[0] = RX_RSSI;
      tx_buf[1] = current_power;

      if (pending_response_id) {
          #ifdef DEBUG_STATE
            Serial.println("Sending response ");
          #endif
        tx_buf[2] = (unsigned char)(pending_response_id & 0xFF);
        tx_buf[3] = (unsigned char)(pending_response_id >> 8);
        tx_buf[4] = 0;
        tx_buf[5] = 'R';   // a command response
        pending_response_id = 0;   // reset
      } else {
          #ifdef DEBUG_STATE
            Serial.println("Sending ack ");
          #endif
        tx_buf[2] = 0;
        tx_buf[3] = 0;
        tx_buf[4] = 0;
        tx_buf[5] = 'A';   // an 'ack'
      }
      //packet_timer = micros();
      send_data(tx_buf, 6);
      hop_to_next();
      stateMachine = RECEIVE;

      //calculated_rssi = calculate_rssi(RX_RSSI);
      //calculated_lost_frames_rssi= calculate_lost_frames_rssi(lost_frames);
      
      //Serial.println(micros() - packet_timer);
      #ifdef DEBUG_ANALYZER
      Serial.print("\t"); Serial.print(radioMode);
      #endif
      break;
  }
  // forced hopping if no data received
  // TODO: Fast recover then slow recover
  if (micros() > RX_hopping_timeout) {

    RX_hopping_timeout = micros()  + (TX_period * 2);
      
    hop_to_next();
    
    #ifdef DEBUG_CH_FREQ
    Serial.print(" No FR: ");
    Serial.print(millis() - RX_last_frame_received);
    Serial.println("ms\thop");
    #endif
  }

  // Failsafe
  if (radioMode == ARMED && millis() - RX_last_frame_received > FAILSAFE_DELAY_MS) {
    radioMode = FAILSAFE;

    // TODO: flash the LED
    for (int i = 0; i < CHANNELS; i++) {
      rcValue[i] = 1500;
    }
    rcValue[THR] = MINTHROTTLE;
  }  
}

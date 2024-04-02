/* ========================================
 *
 *  loopian::ORBIT main
 *    description: Main Loop
 *    for Raspberry Pi pico
 *
 *  Copyright(c)2024 Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"      //https://github.com/khoih-prog/RPI_PICO_TimerInterrupt

#include  "constants.h"
#include  "i2cdevice.h"
#include  "global_timer.h"
#include  "white_led.h"
#include  "touch.h"

/*----------------------------------------------------------------------------*/
//     Constants
/*----------------------------------------------------------------------------*/
#define PIN_WHITELED_EN 15
#define LED_ERR       16
#define LED1          17
#define LED2          22
#define JOYSTICK_X    26
#define JOYSTICK_Y    27
#define JOYSTICK_SW   14

/*----------------------------------------------------------------------------*/
//     Variables
/*----------------------------------------------------------------------------*/
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer(0);

bool setup_mode = false;
constexpr int HOLD_TIME = 10;  // *10msec この間、一度でもonならonとする。離す時少し鈍感にする。最大16
int counter = 0;
unsigned long seconds_old = 0;
GlobalTimer gt;
WhiteLed wled;
bool availableEachDevice[MAX_KAMABOKO_NUM];
uint16_t sw[MAX_KAMABOKO_NUM][MAX_EACH_SENS] = {0};
int holdtime_cnt = 0; // 指を離したときの感度を弱めに（反応を遅めに）にするためのカウンタ
bool available_each_device[MAX_KAMABOKO_NUM+4] = {false};
TouchEvent ev[MAX_TOUCH_EV];
SwitchEvent swevt[MAX_KAMABOKO_NUM];
uint8_t velocity = 100;

/*----------------------------------------------------------------------------*/
//     setup
/*----------------------------------------------------------------------------*/
#define TIMER_INTERVAL_MS        2L
void setup() {
  // GPIO
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(PIN_WHITELED_EN, OUTPUT);
  pinMode(JOYSTICK_SW,INPUT);
  
  digitalWrite(LED_ERR, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  int joystick_sw = digitalRead(JOYSTICK_SW);
  if (joystick_sw == 0){setup_mode = true;}
  digitalWrite(LED_BUILTIN, HIGH);

  // Init Vari
  for (int i=0; i<MAX_KAMABOKO_NUM; ++i){
    availableEachDevice[i] = true;
  }

  //  I2C/device settings
  wireBegin();   // Join I2C bus
  for (int k=0; k<MAX_KAMABOKO_NUM; k++){
    PCA9685_init(16+k);
  }
  ada88_init();
  // Opening
  for (int i=0; i<4; i++) {
      ada88_write(26 + i);
      delay(300);
  }
  for (int i=0; i<3; i++) {
      ada88_write(28 - i);
      delay(300);
  }
  ada88_writeNumber(109); // version No. 一の位は9固定
  delay(500);


  // USB & MIDI
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  TinyUSB_Device_Init(0);
#endif

  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleProgramChange(handleProgramChange);
  usb_midi.setStringDescriptor("TinyUSB MIDI");
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);

  // Interval in unsigned long microseconds
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS*1000, TimerHandler)) {
    // success
  }

  wled.clear_all();

  // check touch sensor & initialize
  if (setup_mode){check_and_setup_board();}
  else {normal_mode();}
}
/*----------------------------------------------------------------------------*/
void normal_mode(void) {
  // Normal Mode
  int exist_err = 0;
  for (int i=0; i<MAX_KAMABOKO_NUM; i++) {
      int ret = MBR3110_init(i);
      if (ret == 0) {
        available_each_device[i] = true;
        digitalWrite(LED1,HIGH);
        ada88_write(i * 10);
      }
      else {
        available_each_device[i] = false;
        exist_err = ret;
        digitalWrite(LED1,LOW);
        ada88_write(25); //--
      }
  }
  int disp_num = 0;
  if (exist_err != 0) {
      digitalWrite(LED_ERR,HIGH);
      disp_num = 20 + exist_err; // Error: 19:き, 18:ま,
      if (disp_num >= 23) {
          disp_num = 23;
      }
      // Er
      else if (disp_num < 0) {
          disp_num = 0;
      }
  } else {
      disp_num = 22; // OK
  }
  ada88_write(disp_num);
  delay(2000);
  ada88_write_org_bit(available_each_device);
  delay(2000);
}
/*----------------------------------------------------------------------------*/
void check_and_setup_board(void) {
  int err;
  int selmode = 0;
  int incdec_old = 0;
  const int SELMODE_MAX = 13;

  ada88_write(21); // SU
  delay(5000);
  while(1) {
      uint16_t adval = get_joystick_position_x();
      int incdec = 0;
      if (adval > 3000) {
          incdec = 1;
      } else if (adval < 1000) {
          incdec = -1;
      }
      if (incdec != incdec_old) {
          if (incdec > 0) {
              selmode += 1;
              if (selmode >= SELMODE_MAX) {
                  selmode = 0;
              }
          } else if (incdec < 0) {
              selmode -= 1;
              if (selmode < 0) {
                  selmode = SELMODE_MAX - 1;
              }
          }
          incdec_old = incdec;
      }
      uint16_t cnt = gt.globalTime();
      display_setup(selmode, cnt);
      if (digitalRead(JOYSTICK_SW) == HIGH) {
          break;
      }
  }

  //  Check White LED
  if (selmode == 12){
    ada88_write(24);//"LE"
    delay(200);
    for(int f=0; f<MAX_EACH_LIGHT; f++){wled.light_led_each(f,0);}
    digitalWrite(PIN_WHITELED_EN, LOW);  //  All White LED available
    while(1){
      for(int l=0; l<2; l++){
        for(int e=0; e<MAX_EACH_LIGHT; e++){
          uint16_t bright = (e%2)==0?l:(l+1)%2;
          wled.light_led_each(e,bright*200);
        }
        delay(200);
      }
    } //  無限ループ
  }

  // CapSense Setup Mode
  ada88_writeNumber(selmode * 10);
  int success = setup_mbr(selmode);
  if (success == 0) {
      ada88_write(22); // Ok
  } else if (success == 1) {
      ada88_write(25); // --
  } else {
      ada88_write(23); // Er
      digitalWrite(LED_ERR, HIGH); // Err LED on
  }

  while(1);
}
void display_setup(int sup, uint16_t counter) {
    if ((counter % 400) > 200) {
        if (sup < 12) {
            ada88_writeNumber(sup * 10);
        } else {
            ada88_write(24); // "LE"
        }
    } else {
        ada88_write(0);
    }
}
int setup_mbr(size_t num) {
    int v = MBR3110_setup(num);
    if (v == 0) {
        // 書き込みしてOKだった場合
        for (int i=0; i<3; ++i) {
            // when finished, flash 3times.
            digitalWrite(LED_ERR,HIGH);
            delay(100);
            digitalWrite(LED_ERR,LOW);
            delay(100);
        }
        delay(500);
    }
    return v;
}
/*----------------------------------------------------------------------------*/
//     loop
/*----------------------------------------------------------------------------*/
void loop() {
  //  Global Timer 
  long difftm = generateTimer();

  // read any new MIDI messages
  MIDI.read();

  if ( gt.timer10msecEvent() ){
    // check active
    if (gt.timer100ms()%2){digitalWrite(LED_BUILTIN, LOW);}
    else {digitalWrite(LED_BUILTIN, HIGH);}
    holdtime_cnt += 1;
    if (holdtime_cnt>=HOLD_TIME){holdtime_cnt=0;}

    //  Touch Sensor
    int errNum = 0;
    bool light_someone = false;
    for (int i=0; i<MAX_KAMABOKO_NUM; ++i){
      if (availableEachDevice[i] == true){
        uint8_t swtmp[2] = {0};
        int err = MBR3110_readTouchSw(swtmp,i);
        if (err){
          errNum += 0x01<<i;
        }
        else {
          light_someone = swevt[i].update_sw_event(swtmp, gt.timer10ms()*10);
        }
      }
    }
    if (light_someone){digitalWrite(LED1, LOW);}
    else {digitalWrite(LED1, HIGH);}
    int target_num = update_touch_target();
    if (target_num>1) {digitalWrite(LED2, HIGH);}
    else {digitalWrite(LED2,LOW);}
  }

  //  update touch location
  interporate_location(difftm);
  int tchev[MAX_TOUCH_EV];
  for (int i=0; i<MAX_TOUCH_EV; i++){
    tchev[i] = ev[i]._locate_current;
  }

  // Light White LED
  int max_ev = wled.gen_lighting_in_loop(difftm, tchev);

  // Dispay position
  int position = ev[0]._locate_current;
  if (position < 0){ada88_write(0);}
  else {ada88_writeNumber(position);}

  // Read Joystick
  velocity = get_velocity_from_adc();
}
/*----------------------------------------------------------------------------*/
//     Timer
/*----------------------------------------------------------------------------*/
bool TimerHandler(repeating_timer* rt)
{
  gt.incGlobalTime();
  return true;
}
long generateTimer( void )
{
  uint16_t  gTime = gt.globalTime();
  long diff = gTime - gt.gtOld();
  gt.setGtOld(gTime);
  if ( diff < 0 ){ diff += 0x10000; }

  gt.clearAllTimerEvent();
  gt.updateTimer(diff);
  return diff;
}
/*----------------------------------------------------------------------------*/
//     MIDI/Other Hardware
/*----------------------------------------------------------------------------*/
void handleNoteOn(byte channel, byte pitch, byte velocity) {
  digitalWrite(LED_BUILTIN, HIGH);
}
void handleNoteOff(byte channel, byte pitch, byte velocity) {
  digitalWrite(LED_BUILTIN, LOW);
}
void handleProgramChange(byte channel , byte number) {
  //
}
/*----------------------------------------------------------------------------*/
uint8_t get_velocity_from_adc(void) {
    // adc: 0-4095
    uint16_t adc = get_joystick_position_x();
    uint8_t ret;
    if (adc > 2100) {
        ret = (adc / 70) + 69;
    } else if (adc < 1900) {
        ret = (adc / 25) + 20;
    } else {
        ret = 98;
    }
    return ret;
}
/*----------------------------------------------------------------------------*/
uint16_t get_joystick_position_x(void) {
    // 0-4095
    uint16_t pin_adx_value = analogRead(JOYSTICK_X);
    return pin_adx_value;
}
/*----------------------------------------------------------------------------*/
void setMidiNoteOn(uint8_t note, uint8_t vel)
{
  MIDI.sendNoteOn(note, vel, 1);
}
/*----------------------------------------------------------------------------*/
void setMidiNoteOff(uint8_t note)
{
  MIDI.sendNoteOff(note, 64, 1);
}
/*----------------------------------------------------------------------------*/
void setMidiControlChange(uint8_t controller, uint8_t value)
{
  MIDI.sendControlChange(controller, value, 1);
}
/*----------------------------------------------------------------------------*/
void setMidiProgramChange(uint8_t pcn , uint8_t chnl )
{
  MIDI.sendProgramChange(pcn, chnl);
}
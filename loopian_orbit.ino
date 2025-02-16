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
#include <Adafruit_NeoPixel.h>
// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"      //https://github.com/khoih-prog/RPI_PICO_TimerInterrupt

#include  "constants.h"
#include  "i2cdevice.h"
#include  "global_timer.h"
#include  "white_led.h"
#include  "touch.h"

//#define UART_MIDI
/*----------------------------------------------------------------------------*/
//     Constants
/*----------------------------------------------------------------------------*/
//#define SW1           10
//#define SW2           11
//#define SW3           12
//#define SW4           13
#define PIN_WHITELED_EN D10 // P11
#define LED_ERR       20    // no connect
#define LED1          16    // LED
#define LED2          17    // LED
#define LED3          25 // LED
#define JOYSTICK_X    26    // P1(A0)
#define JOYSTICK_Y    27    // P2(A1)
#define JOYSTICK_SW   D7    // P8
#define POWER_PIN     11    //NeoPixelの電源

/*----------------------------------------------------------------------------*/
//     Variables
/*----------------------------------------------------------------------------*/
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;
Adafruit_NeoPixel pixels(1, 12, NEO_GRB + NEO_KHZ800);

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);
#ifdef UART_MIDI
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI_UART);
#endif

// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);

bool normal_mode = true;
bool play_mode = true;  // false: command_mode
unsigned long seconds_old = 0;
int holdtime_cnt = 0; // 指を離したときの感度を弱めに（反応を遅めに）にするためのカウンタ
uint8_t velocity_byjoy = 100;
uint8_t damper_byjoy = 0;
int disp_auto_clear = 0;
int disp_notch_counter = 0;
bool available_each_device[MAX_KAMABOKO_NUM+4] = {false};
TouchEvent tchev[MAX_TOUCH_EV];
SwitchEvent swevt[MAX_KAMABOKO_NUM];
int externalNoteState[MAX_MIDI_NOTE] = {0};
int loop_counter = 0;
int neo_pixel_red = 0;
int neo_pixel_green = 0;
int neo_pixel_blue = 0;

GlobalTimer gt;
WhiteLed wled;

/*----------------------------------------------------------------------------*/
//     setup
/*----------------------------------------------------------------------------*/
#define TIMER_INTERVAL_MS        2L
void setup() {
  // GPIO
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(PIN_WHITELED_EN, OUTPUT);
  pinMode(JOYSTICK_SW,INPUT);

  gpio_put(PIN_WHITELED_EN, LOW);  //  All White LED disable
  gpio_put(LED_ERR, LOW);
  gpio_put(LED1, HIGH); // 初期化の間は点灯
  gpio_put(LED2, HIGH);
  gpio_put(LED3, HIGH);
  normal_mode = gpio_get(JOYSTICK_SW);

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
#ifdef UART_MIDI
  MIDI_UART.setHandleNoteOn(handleNoteOn_UART);
  MIDI_UART.setHandleNoteOff(handleNoteOff_UART);
  MIDI_UART.setHandleControlChange(handleControlChange_UART);
  MIDI_UART.setHandleProgramChange(handleProgramChange_UART);
  MIDI_UART.begin(MIDI_CHANNEL_OMNI);
  MIDI_UART.turnThruOff();
#endif

  //  I2C/device settings
  wireBegin();   // Join I2C bus
  for (uint8_t k=0; k<MAX_KAMABOKO_NUM; k++){
    PCA9685_init(k + PCA9685_OFSADRS);
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
  // version No. ex) 129 => ver.1.2 (一の位は9固定)
  ada88_writeNumber(ORBIT_VERSION*10+9);
  delay(500);


  // Interval in unsigned long microseconds
  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS*1000, TimerHandler)) {
    // success
  }

  // check touch sensor & initialize
  if (normal_mode){check_for_normal_mode();}
  else {check_and_setup_board();}

  //Neopixelの電源供給開始
  pinMode(POWER_PIN, OUTPUT);
  gpio_put(POWER_PIN, HIGH);
  pixels.begin();             //NeoPixel制御開始

  //  White LED
  wled.clear_all();
  gpio_put(LED1, LOW);
  gpio_put(PIN_WHITELED_EN, HIGH);  //  All White LED available
}
/*----------------------------------------------------------------------------*/
void check_for_normal_mode(void) {
  // Normal Mode
  int errnum = 0;
  for (int i=0; i<MAX_KAMABOKO_NUM; i++) {
    int ret = MBR3110_init(i);
    if (ret == 0) {
      available_each_device[i] = true;
      ada88_writeNumber(i * 10);
    }
    else {
      available_each_device[i] = false;
      ada88_write(25); //--
      errnum = ret;
      // ada88_writeNumber(ret + 20);// Error: 19:き, 18:ま,
      // break;
    }
    delay(200);
  }
  int disp_num = 0;
  if (errnum != 0) {
    gpio_put(LED_ERR,HIGH); // Err LED 点灯
    for (int i=0; i<3; i++){
      ada88_write(23); // Er
      delay(500);
      ada88_write_org_bit(available_each_device); // どのKAMABOKOがOKか？
      delay(500);
    }
  } else {
    ada88_write(22); // OK
    delay(2000);
  }
  ada88_write(0);
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
      if (adval > 750) {
          incdec = 1;
      } else if (adval < 250) {
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
      if (!gpio_get(JOYSTICK_SW)) {
          break;
      }
  }

  //  Check White LED
  if (selmode == 12){
    ada88_write(24);//"LE"
    delay(200);
    wled.clear_all();
    gpio_put(PIN_WHITELED_EN, HIGH);  //  All White LED available
    while(1){
      for(int l=0; l<2; l++){
        for(int k=0; k<MAX_KAMABOKO_NUM; k++){
          for(int e=0; e<MAX_EACH_LIGHT; e++){
            uint16_t bright = (e%2)==0?l:(l+1)%2;
            wled.set_led(k, e, bright*200);
          }
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
      gpio_put(LED_ERR, HIGH); // Err LED on
  }

  while(1);
}
void display_setup(int sup, uint16_t cnt) {
    if ((cnt % 400) > 200) {
        if (sup < 12) {
            ada88_writeNumber(sup * 10);
        } else {
            ada88_write(24); // "LE"
        }
    } else {
        ada88_write(0);
    }
}
/*----------------------------------------------------------------------------*/
int setup_mbr(size_t num) {
    int v = MBR3110_setup(num);
    if (v == 0) {
        // 書き込みしてOKだった場合
        for (int i=0; i<3; ++i) {
            // when finished, flash 3times.
            gpio_put(LED_ERR,HIGH);
            delay(100);
            gpio_put(LED_ERR,LOW);
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
  loop_counter += 1;

  //  Global Timer 
  long difftm = generateTimer();
  if ((gt.timer100ms()%10)<5){
    gpio_put(LED1, LOW);
    gpio_put(LED2, LOW);
    gpio_put(LED3, LOW);
  }
  else {
    gpio_put(LED1, HIGH);
    gpio_put(LED2, HIGH);
    gpio_put(LED3, HIGH);
  }
  //ada88_writeNumber(difftm); // Loop周期の計測（パフォーマンス測定時に使用）

  // read any new MIDI messages
  MIDI.read();
#ifdef UART_MIDI
  MIDI_UART.read();
#endif

  // mode check
  check_if_play_mode();

  if (gt.timer10msecEvent() && play_mode){
    // check active
    holdtime_cnt += 1;
    if (holdtime_cnt>=HOLD_TIME){holdtime_cnt=0;}

    //  Touch Sensor
    uint16_t errNum = 0;
    bool light_someone = false;
    for (int i=0; i<MAX_KAMABOKO_NUM; ++i){
      if (available_each_device[i] == true){
        uint8_t swtmp[2] = {0};
        int err = MBR3110_readTouchSw(swtmp,i);
        if (err){
          errNum += 0x01<<i;
        }
        //else {
          //バグ出し用にメチャメチャに押したイベント生成
          //uint16_t sw = 0x0003<<(loopCounter%9);
          //swtmp[1] = static_cast<uint8_t>(sw >> 8);
          //swtmp[0] = static_cast<uint8_t>(sw & 0xfff0);
        light_someone |= swevt[i].update_sw_event(swtmp, static_cast<int>(gt.timer10ms())*10);
        //}
      }
    }
    if (light_someone){neo_pixel_green = 255;}
    else {neo_pixel_green = 0;}
    int target_num = update_touch_target(swevt);
    if (target_num>1) {neo_pixel_blue = 255;}
    else {neo_pixel_blue = 0;}
    if (errNum) {neo_pixel_red = 255;}
    else {neo_pixel_red = 0;}
  }

  //  update touch location
  interporate_location(difftm);
  int tchev_copy[MAX_TOUCH_EV];
  for (int i=0; i<MAX_TOUCH_EV; i++){
    tchev_copy[i] = tchev[i]._locate_current;
  }

  // Light White LED
  int max_ev = wled.gen_lighting_in_loop(difftm, tchev_copy, externalNoteState);
  wled.lighten_led();

  // Display position
  display_88matrix();

  // Read Joystick
  joy_stick();

  //  Display Full Color LED
  pixels.setPixelColor(0, pixels.Color(neo_pixel_red, neo_pixel_green, neo_pixel_blue));
  pixels.show();
}
/*----------------------------------------------------------------------------*/
bool stk_jsw = true;
long pushed_time = 0;
int command_mode_incdec_old = 0;
int command_mode_com = 0;
/*----------------------------------------------------------------------------*/
void check_if_play_mode(void) {
  bool jsw = gpio_get(JOYSTICK_SW);
  if (jsw!=stk_jsw) { // false:on, true:off
    if (!jsw) {
      // スイッチが押された時 : いろいろリセット
      gpio_put(LED_ERR, LOW); // とりあえず Err LED を消去
      clear_external_note();
      for (int i=0; i<MAX_KAMABOKO_NUM; i++) { // センサをリセット
        int ret = MBR3110_init(i);
      }
      pushed_time = gt.globalTime();
      if (!play_mode){
        setMidiProgramChange(command_mode_com);
        play_mode = true;
        ada88_write(0); // 表示を消す
      }
    } else {
      // スイッチが離された時
      if (play_mode){
        // 時間が足りなかったので元に戻す
        gpio_put(LED2, LOW);
      }
    }
    stk_jsw = jsw;
  } else if (!jsw && play_mode) {
    // スイッチが押されている間
    long diff = gt.globalTime() - pushed_time;
    if (diff > JSTICK_LONG_HOLD_TIME) {
      play_mode = false;
      gpio_put(LED2, HIGH);
      setMidiProgramChange(0);
    }
  }
}
/*----------------------------------------------------------------------------*/
void display_auto_clear(void) {
  if (gt.timer100msecEvent() && play_mode) {
    disp_notch_counter += 1;
    if (disp_auto_clear > 0){
      disp_auto_clear -= 1;
      if (disp_auto_clear == 0){
        //ada88_write(0);
        disp_notch_counter = 0;
      }
    }
  }
}
/*----------------------------------------------------------------------------*/
#if 0
void display_88matrix(void) {
  uint8_t raw_data[2] = {0};
  int err = mbr3110_read_rawdata( 0, raw_data );
  if (err == 0) {
    int position = raw_data[0] + (raw_data[1] << 8);
    ada88_writeNumber(position/50);
  } else {
    ada88_write(25); // --
  }
}
#else
void display_88matrix(void) {
  // Display auto clear 
  display_auto_clear();

  const int COMMODE_MAX = 18;
  if (play_mode) {
    // 通常の表示モード
    int position = tchev[0]._locate_target;
    if (position >= 0){
      // Touchされているとき
      ada88_writeNumber(position/10);
      disp_auto_clear = 2;
      disp_notch_counter = 0;
    } else {
      ada88_anime(disp_notch_counter/2); // Time per one frame: 200msec
    }
  } else {
    // Joystick長押しで設定モード
    uint16_t adval = get_joystick_position_x();
    int incdec = 0;
    if (adval > 750) {
        incdec = 1;
    } else if (adval < 250) {
        incdec = -1;
    }
    if (incdec != command_mode_incdec_old) {
        if (incdec > 0) {
            command_mode_com += 1;
            if (command_mode_com >= COMMODE_MAX) {
                command_mode_com = 0;
            }
        } else if (incdec < 0) {
            command_mode_com -= 1;
            if (command_mode_com < 0) {
                command_mode_com = COMMODE_MAX - 1;
            }
        }
        command_mode_incdec_old = incdec;
    }
    if ((gt.globalTime() % 400) > 200) {
        if (command_mode_com < COMMODE_MAX-2) {
            ada88_writeNumber(command_mode_com * 10);
        } else if (command_mode_com == COMMODE_MAX-2) {
            ada88_write(5); // "E"
        } else if (command_mode_com == COMMODE_MAX-1) {
            ada88_write(3); // "C"
        }
    } else {
        ada88_write(0);
    }
  }
}
#endif
/*----------------------------------------------------------------------------*/
void joy_stick(void) {
  uint8_t new_vel = get_velocity_from_adc();
  if ((new_vel != velocity_byjoy) && play_mode) {
    ada88_writeNumber(new_vel);
    disp_auto_clear = 5;
    velocity_byjoy = new_vel;
  }
  uint8_t new_dmp = get_damper_from_adc();
  if ((new_dmp != damper_byjoy) && play_mode) {
    ada88_writeNumber(new_dmp);
    setMidiControlChange(64, new_dmp);
    disp_auto_clear = 5;
    damper_byjoy = new_dmp;
  }
}
/*----------------------------------------------------------------------------*/
uint8_t get_velocity_from_adc(void) {
    // get_joystick_position_x(): 0-1023 左が値が大きい
    const uint16_t HUKAN = 64;
    const uint16_t UP_RESO = ((1023 - HUKAN) - (512 + HUKAN))/27;
    const uint16_t DOWN_RESO = ((512 - HUKAN) - HUKAN)/100;
    uint16_t adc = 1023 - get_joystick_position_x();
    uint8_t ret;
    if (adc > 512 + HUKAN) {  // bigger
        ret = (adc - (512 + HUKAN))/UP_RESO + (512 + HUKAN);
    } else if (adc < 512 - HUKAN) { // smaller
        ret = (adc - HUKAN)/DOWN_RESO + HUKAN;
    } else {
        ret = 100;
    }
    return ret;
}
/*----------------------------------------------------------------------------*/
uint8_t get_damper_from_adc(void) {
    // get_joystick_position_y(): 0-1023 左が値が大きい
    uint16_t adc = get_joystick_position_y();
    uint8_t ret;
    if (adc > 545) {
        ret = (adc / 3) - 182;
    } else if (adc < 480) {
        ret = 160 - (adc / 3);
    } else {
        ret = 0;
    }
    if (ret > 127){ret=127;}
    return ret;
}
/*----------------------------------------------------------------------------*/
uint16_t get_joystick_position_x(void) {
    // 0-1023
    uint16_t pin_adx_value = analogRead(JOYSTICK_X);
    return pin_adx_value;
}
/*----------------------------------------------------------------------------*/
uint16_t get_joystick_position_y(void) {
    // 0-1023
    uint16_t pin_ady_value = analogRead(JOYSTICK_Y);
    return pin_ady_value;
}
/*----------------------------------------------------------------------------*/
void clear_external_note(void) {
  for (int i=0; i<MAX_MIDI_NOTE; ++i) {
    externalNoteState[i] = 0;
  }
}
/*----------------------------------------------------------------------------*/
void clear_my_note(void) {
  for (int i=0; i<MAX_MIDI_NOTE; ++i) {
    externalNoteState[i] = 0;
  }
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
void handleNoteOn(byte channel, byte pitch, byte velocity) { // orbit_main
  if (channel == 16){
    externalNoteState[pitch] = velocity;
#ifdef UART_MIDI
    MIDI_UART.sendNoteOn(pitch, velocity, 16); // to orbit_sub
#endif
  } else if (channel == 15) {
#ifdef UART_MIDI
    MIDI_UART.sendNoteOn(pitch, velocity, 15); // to orbit_sub
#endif
  } else if (channel == 14) {
    externalNoteState[pitch] = velocity;
  }
}
void handleNoteOff(byte channel, byte pitch, byte velocity) {
  if (channel == 16){
    externalNoteState[pitch] = 0;
#ifdef UART_MIDI
    MIDI_UART.sendNoteOff(pitch, velocity, 16); // to orbit_sub
#endif
  } else if (channel == 15) {
#ifdef UART_MIDI
    MIDI_UART.sendNoteOff(pitch, velocity, 15); // to orbit_sub
#endif
  } else if (channel == 14) {
    externalNoteState[pitch] = 0; 
  }
}
void handleProgramChange(byte channel , byte number) {
  //
}
/*----------------------------------------------------------------------------*/
#if defined(UART_MIDI)
void handleNoteOn_UART(byte channel, byte pitch, byte velocity) {
  MIDI.sendNoteOn(pitch, velocity, channel);
}
void handleNoteOff_UART(byte channel, byte pitch, byte velocity) {
  MIDI.sendNoteOff(pitch, velocity, channel);
}
void handleControlChange_UART(byte channel , byte number , byte value ) {
  MIDI.sendControlChange(number, value, channel);
}
void handleProgramChange_UART(byte channel , byte number) {
  MIDI.sendProgramChange(number, channel);
}
#endif
/*----------------------------------------------------------------------------*/
void setMidiNoteOn(uint8_t note, uint8_t vel) {
  MIDI.sendNoteOn(note, vel, 12);
#ifdef UART_MIDI
  MIDI_UART.sendNoteOn(note, vel, 13);
#endif
}
void setMidiNoteOff(uint8_t note) {
  MIDI.sendNoteOff(note, 64, 12);
#ifdef UART_MIDI
  MIDI_UART.sendNoteOff(note, 64, 13);
#endif
}
void setMidiControlChange(uint8_t controller, uint8_t value) {
  MIDI.sendControlChange(controller, value, 12);
#ifdef UART_MIDI
  MIDI_UART.sendControlChange(controller, value, 13);
#endif
}
void setMidiProgramChange(uint8_t pcn)
{
  MIDI.sendProgramChange(pcn, 12);
#ifdef UART_MIDI
  MIDI_UART.sendProgramChange(pcn, 13);
#endif
}
/* ========================================
 *
 *  loopian::ORBIT touch.cpp
 *    description: TouchEvent
 *    for Raspberry Pi pico
 *
 *  Copyright(c)2024 Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#include    <Arduino.h>

#include    "touch.h"
#include    "constants.h"
#include    "loopian_common.h"

/*----------------------------------------------------------------------------*/
//     Constants
/*----------------------------------------------------------------------------*/
constexpr int LED_CHASE_SPEED = 10; // locate diff / 2msec
constexpr int SAME_FINGER = 210;  // 100 means next value (200means next sensor)/10msec
                                  // 同じ指とみなす速さ

/*----------------------------------------------------------------------------*/
//     SwitchEvent Class
/*----------------------------------------------------------------------------*/
void SwitchEvent::clear_event(int time, size_t ele) {
    if (_sw[ele] == OFF) {
        return;
    }
    if (_sw[ele] + CHATTERING_TIME < time) {
        _sw[ele] = OFF;
    }
}
bool SwitchEvent::update_sw_event(uint8_t sw[2], int time) {
    if (time == 0) {
        return false;
    }
    bool light_someone = false;
    uint16_t swh = static_cast<uint16_t>(sw[1]);
    uint16_t swl = static_cast<uint16_t>(sw[0]);
    uint16_t bptn = (swh << 8) + swl;
    for (int j=0; j<MAX_ELECTRODE_PER_DEV; ++j) {
        if ((bptn & (0x0001 << j)) != 0) {
            if (_sw[j] == OFF) {
                _sw[j] = time;
            }
            light_someone = true;
        } else {
            clear_event(time, j);
        }
    }
    return light_someone;
}

/*----------------------------------------------------------------------------*/
//     calcurate finger location
/*----------------------------------------------------------------------------*/
// 連続するタッチオンの検出
void extract_finger(TouchEvent (&new_ev)[MAX_TOUCH_EV])
{
  bool start=false;
  int start_i = 0;

  // new_ev の生成
  for (int e=0; e<MAX_TOUCH_EV; e++){
    //  下から上まで端子の状態を走査しながら、タッチされた端子が連続している箇所を探す
    int i=start_i;
    while (i<MAX_KAMABOKO_NUM*MAX_EACH_SENS) {
      int which_dev=i/MAX_EACH_SENS;
      int each_sw=i%MAX_EACH_SENS;
      if (sw[which_dev][each_sw] != 0){
        if (!start){
          start = true;
          new_ev[e]._mintch_locate = which_dev*MAX_EACH_SENS + each_sw;
         }
      }
      else {
        if (start){
          start = false;
          new_ev[e]._maxtch_locate = which_dev*MAX_EACH_SENS + each_sw - 1;
          new_ev[e]._locate_target = (new_ev[e]._mintch_locate + new_ev[e]._maxtch_locate)*100; // *200/2
          start_i = i+1;
          break;
        }
      }
      i+=1;
    }
    if (start){ // 最後のセンサ
      new_ev[e]._maxtch_locate = MAX_KAMABOKO_NUM*MAX_EACH_SENS - 1;
      new_ev[e]._locate_target = (new_ev[e]._mintch_locate + new_ev[e]._maxtch_locate)*100; // *200/2
    }
  }
}
int update_touch_target(void)
{
  int target_num = 0;
  TouchEvent new_ev[MAX_TOUCH_EV];

  // 指と判断できるイベント抽出
  extract_finger(new_ev);

  // ev[]とnew_ev[]を照合して、Note Event を生成
  for (int x=0; x<MAX_TOUCH_EV; x++){
    int new_target = new_ev[x]._locate_target;
    if (new_target == NOTHING){break;}
    bool found = false;
    for (int y=0; y<MAX_TOUCH_EV; y++){
      int crnt_target = ev[y]._locate_target;
      if (crnt_target == NOTHING){break;}
      if (crnt_target == COLLATED){continue;}
      if ((crnt_target-SAME_FINGER < new_target) && (new_target < crnt_target+SAME_FINGER)){
        new_ev[x]._locate_current = ev[y]._locate_current;
        new_ev[x]._time = ev[y]._time;
        ev[y]._locate_target = COLLATED;
        found = true;
        new_ev[x]._last_midi = new_ev[x]._locate_target/100;
        if (new_ev[x]._last_midi != ev[y]._last_midi){
          generate_midi(1, new_ev[x]._last_midi, ev[y]._last_midi);
        }
        target_num = x;
        break;
      }
    }
    if (!found){ // on:new, off:old -> note on
      new_ev[x]._locate_current = new_target;
      new_ev[x]._time = 0;
      new_ev[x]._last_midi = new_ev[x]._locate_current/100;
      generate_midi(0, new_ev[x]._last_midi, NOTHING);
    }
  }
  for (int z=0; z<MAX_TOUCH_EV; z++){ // off:new, on:old -> note off
    int crnt_target = ev[z]._locate_target;
    if (crnt_target == NOTHING){break;}
    if (crnt_target == COLLATED){continue;}
    else {generate_midi(2, ev[z]._last_midi, NOTHING);}
  }
  // copy
  memcpy(ev,new_ev,sizeof(TouchEvent)*MAX_TOUCH_EV);
  //for (int c=0; c<MAX_TOUCH_EV; c++){ev[c] = new_ev[c];}
  return target_num;
}
//current を target に近づける
void interporate_location(long difftm)
{
  for (int i=0; i<MAX_TOUCH_EV; i++){
    int target = ev[i]._locate_target;
    if (target==-1){break;}
    int diff = target - ev[i]._locate_current;
    if (diff>0){
      diff = difftm*LED_CHASE_SPEED>diff? diff:difftm*LED_CHASE_SPEED;
    }
    else if (diff<0){
      diff = difftm*LED_CHASE_SPEED>(-diff)? diff:-difftm*LED_CHASE_SPEED;
    }
    ev[i]._locate_current += diff;
    ev[i]._time += difftm;
  }
}
void generate_midi(int type, int locate, int last_locate){
  constexpr int CHECK_NOTE = 0;
  switch(type){
    case 0:{
      setMidiNoteOn(locate+CHECK_NOTE, velocity);
      break;
    }
    case 1:{
      setMidiNoteOn(locate+CHECK_NOTE, velocity);
      setMidiNoteOff(last_locate+CHECK_NOTE);
      break;
    }
    case 2:{
      setMidiNoteOff(locate+CHECK_NOTE);
      break;
    }
  }
}
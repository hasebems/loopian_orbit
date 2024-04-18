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
    for (int j=0; j<MAX_EACH_SENS; ++j) {
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
void extract_finger(TouchEvent (&new_ev)[MAX_TOUCH_EV], SwitchEvent (&se)[MAX_KAMABOKO_NUM])
{
  constexpr int MAKE_POSITION = MAX_LOCATE / (MAX_KAMABOKO_NUM*MAX_EACH_SENS*2); // 50

  // new_ev の生成
  int locate = 0;
  int fng_num = 0;
  bool start=false;

  //  下から上まで端子の状態を走査しながら、タッチされた端子が連続している箇所を探す
  while (locate < MAX_KAMABOKO_NUM*MAX_EACH_SENS) {
    int which_dev = locate/MAX_EACH_SENS;
    int each_sw = locate%MAX_EACH_SENS;

    if (se[which_dev].sw(each_sw) != 0){
      if (!start){
        start = true;
        new_ev[fng_num]._mintch_locate = which_dev*MAX_EACH_SENS + each_sw;
        }
    } else if (start){
      start = false;
      new_ev[fng_num]._maxtch_locate = which_dev*MAX_EACH_SENS + each_sw - 1;
      new_ev[fng_num]._locate_target = 
          (new_ev[fng_num]._mintch_locate + new_ev[fng_num]._maxtch_locate) * MAKE_POSITION;
      fng_num += 1;
    }
    locate += 1;
  }
  if (start){ // 最後のセンサもONであれば
    new_ev[fng_num]._maxtch_locate = MAX_KAMABOKO_NUM*MAX_EACH_SENS - 1;
    new_ev[fng_num]._locate_target = 
          (new_ev[fng_num]._mintch_locate + new_ev[fng_num]._maxtch_locate) * MAKE_POSITION;
  }
}
int update_touch_target(SwitchEvent (&se)[MAX_KAMABOKO_NUM])
{
  int target_num = 0;
  TouchEvent new_ev[MAX_TOUCH_EV];

  // 指と判断できるイベント抽出
  extract_finger(new_ev, se); // maxtch,mintch,target に値が入る

  // ev[]とnew_ev[]を照合して、Note Event を生成
  for (int x=0; x<MAX_TOUCH_EV; x++){
    int new_target = new_ev[x]._locate_target;
    if (new_target == NOTHING){break;}
    bool found = false;

    for (int y=0; y<MAX_TOUCH_EV; y++){
      int crnt_target = tchev[y]._locate_target;
      if (crnt_target == NOTHING){break;}
      if (crnt_target == COLLATED){continue;}
      if ((crnt_target-SAME_FINGER < new_target) && (new_target < crnt_target+SAME_FINGER)){
        // 過去と現在の位置が近いデータを同じ指とみなす
        // 残りのメンバーをコピー
        new_ev[x]._locate_current = tchev[y]._locate_current;
        new_ev[x]._time = tchev[y]._time;
        tchev[y]._locate_target = COLLATED;
        found = true;
        new_ev[x]._last_midi = new_ev[x]._locate_target/100;
        if (new_ev[x]._last_midi != tchev[y]._last_midi){
          // move
          generate_midi(TYP_SLIDE, new_ev[x]._last_midi, tchev[y]._last_midi);
        }
        target_num = x + 1;
        break;
      }
    }
    if (!found){ // off:old, on:new -> note on
      new_ev[x]._locate_current = new_target;
      new_ev[x]._time = 0;
      new_ev[x]._last_midi = new_ev[x]._locate_current/100;
      generate_midi(TYP_NOTE_ON, new_ev[x]._last_midi, NOTHING);
    }
  }

  for (int z=0; z<MAX_TOUCH_EV; z++){ // on:old, off:new -> note off
    int crnt_target = tchev[z]._locate_target;
    if (crnt_target == NOTHING){break;}
    if (crnt_target == COLLATED){continue;}
    else {generate_midi(TYP_NOTE_OFF, tchev[z]._last_midi, NOTHING);}
  }

  // copy
  //for (int c=0; c<MAX_TOUCH_EV; c++){ev[c] = new_ev[c];}
  memcpy(tchev,new_ev,sizeof(TouchEvent)*MAX_TOUCH_EV);

  return target_num;
}
//current を target に近づける
void interporate_location(long difftm)
{
  for (int i=0; i<MAX_TOUCH_EV; i++){
    int target = tchev[i]._locate_target;
    if (target==-1){break;}
    int diff = target - tchev[i]._locate_current;
    if (diff>0){
      diff = difftm*LED_CHASE_SPEED>diff? diff:difftm*LED_CHASE_SPEED;
    }
    else if (diff<0){
      diff = difftm*LED_CHASE_SPEED>(-diff)? diff:-difftm*LED_CHASE_SPEED;
    }
    tchev[i]._locate_current += diff;
    tchev[i]._time += difftm;
  }
}
void generate_midi(NOTE_TYP type, int locate, int last_locate){
  constexpr int CHECK_NOTE = 0;

  if ((locate >= MAX_KAMABOKO_NUM*MAX_EACH_SENS) ||
    (last_locate >= MAX_KAMABOKO_NUM*MAX_EACH_SENS)){
    // ありえない数字
    return;
  }

  switch(type){
    case TYP_NOTE_ON:{
      setMidiNoteOn(locate+CHECK_NOTE, velocity_byjoy);
      break;
    }
    case TYP_SLIDE:{
      setMidiNoteOn(locate+CHECK_NOTE, velocity_byjoy-1);
      setMidiNoteOff(last_locate+CHECK_NOTE);
      break;
    }
    case TYP_NOTE_OFF:{
      setMidiNoteOff(locate+CHECK_NOTE);
      break;
    }
  }
}
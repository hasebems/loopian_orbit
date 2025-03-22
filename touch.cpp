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
#include    "i2cdevice.h"

/*----------------------------------------------------------------------------*/
//     Constants
/*----------------------------------------------------------------------------*/
constexpr int LED_CHASE_SPEED = 10; // locate diff / 2msec
constexpr int SAME_FINGER = 410;  // 100 means next value (200means next sensor)/10msec
                                  // 同じ指とみなす速さ

/*----------------------------------------------------------------------------*/
//     SwitchEvent Class
/*----------------------------------------------------------------------------*/
/// @brief かまぼこ筐体一個あたりのスイッチの状態を更新
/// @param sw[2] [0]:下位, [1]:上位
/// @param time [msec]
/// @return どこか一つでも ON なら true
bool SwitchEvent::update_kama_event(uint8_t sw[2], int kama, uint32_t time) {
    if (time == 0) {
        return false;
    }
    bool light_someone = false;
    uint16_t swh = static_cast<uint16_t>(sw[1]);
    uint16_t swl = static_cast<uint16_t>(sw[0]);
    uint16_t bptn = (swh << 8) + swl;
    size_t which_dev = kama*MAX_EACH_SENS;

    // 一つのかまぼこ筐体について、各スイッチの状態を更新
    for (int j=0; j<MAX_EACH_SENS; ++j) {
      uint16_t evt_ptn = 0x0001 << j;
      size_t each_sw = which_dev + j;
      if ((bptn & evt_ptn) != 0) {
        // Error Check
        if (_eachSw[each_sw]._timeWhenOn == HOLD_ERROR) {
          continue;
        }

        // SW ON Event 
        if (!_eachSw[each_sw]._sw) {
          _eachSw[each_sw]._sw = true;
          _eachSw[each_sw]._timeWhenOn = time;
        }
        light_someone = true;

        // Sensor Error で、SW ON が続いている場合 
        if (duration(each_sw, time) > 10*1000) { // 10sec 以上 SW ON ならばエラー扱いとする
          _eachSw[each_sw]._sw = false;
          _eachSw[each_sw]._timeWhenOn = HOLD_ERROR;
        }

      } else {
        // Error Check
        if (_eachSw[each_sw]._timeWhenOn == HOLD_ERROR) {
          _eachSw[each_sw]._timeWhenOn = NO_TOUCH;
        }

        if (_eachSw[each_sw]._sw && (duration(each_sw, time) > CHATTERING_TIME)) {
          _eachSw[each_sw]._sw = false;
        }
      }
    }
    return light_someone;
}
/// @brief スイッチの状態を更新
/// @param time [msec]
/// @return どこか一つでも ON なら true
bool SwitchEvent::update_allsw_event(uint32_t time, bool (&available_dev)[MAX_KAMABOKO_NUM]) {
  bool light_someone = false;
  uint16_t errNum = 0;
  for (int i=0; i<MAX_KAMABOKO_NUM; ++i){
    if (available_dev[i] == true){
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
      light_someone |= update_kama_event(swtmp, i, time);
      //}
    }
  }
  return light_someone;
}
/*----------------------------------------------------------------------------*/
//     calcurate finger location
/*----------------------------------------------------------------------------*/
/// @brief 指によるタッチの状態を更新
/// @param SwitchEvent &se
/// @param bool &no_sensor_error
/// @return タッチされた指の数
int update_touch_target(SwitchEvent &se, bool &sensor_error)
{
  int target_num = 0;
  TouchEvent new_ev[MAX_TOUCH_EV];

  // 指と判断できるイベント抽出
  sensor_error = !extract_finger(new_ev, se); // maxtch,mintch,target に値が入る

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
/// @brief 連続するタッチの検出
/// @param TouchEvent (&new_ev)[MAX_TOUCH_EV] : Touch Event の配列への参照
/// @param SwitchEvent &se : スイッチの状態
/// @return Error の有無
bool extract_finger(TouchEvent (&new_ev)[MAX_TOUCH_EV], SwitchEvent &se)
{
  constexpr int MAKE_POSITION = MAX_LOCATE / (MAX_KAMABOKO_NUM*MAX_EACH_SENS*2); // 50

  // new_ev の生成
  size_t locate = 0;
  int fng_num = 0;
  bool start = false;
  bool normal = true;

  //  下から上まで端子の状態を走査しながら、タッチされた端子が連続している箇所を探す
  //  MAX_TOUCH_EV 以上になったら走査終了
  while (locate < MAX_KAMABOKO_NUM*MAX_EACH_SENS) {
    TOUCH_STATE state = se.sw(locate);
    // エラーがあれば、その前のセンサの状態をコピーする
    size_t locate_bk = locate;
    while (state == ST_ERROR) {
      normal = false;
      locate_bk -= 1;
      state = se.sw(locate_bk);
    }

    if (state == ST_TOUCH){
      if (!start){
        start = true;
        new_ev[fng_num]._mintch_locate = locate;
      }
    } else if (start){
      start = false;
      new_ev[fng_num]._maxtch_locate = locate - 1;
      new_ev[fng_num]._locate_target = 
          (new_ev[fng_num]._mintch_locate + new_ev[fng_num]._maxtch_locate) * MAKE_POSITION;
      if (fng_num < MAX_TOUCH_EV) {fng_num += 1;}
      else {break;}
    }
    locate += 1;
  }
  if (start){ // 最後のセンサもONであれば
    new_ev[fng_num]._maxtch_locate = MAX_KAMABOKO_NUM*MAX_EACH_SENS - 1;
    new_ev[fng_num]._locate_target = 
          (new_ev[fng_num]._mintch_locate + new_ev[fng_num]._maxtch_locate) * MAKE_POSITION;
  }
  return normal;
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
  constexpr int CHECK_NOTE = 18;

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
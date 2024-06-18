/* ========================================
 *
 *  loopian: white led
 *    description: white led
 *    for Arduino Leonardo / Sparkfun pro micro
 *
 *  Copyright(c)2023- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#include "white_led.h"
#include "i2cdevice.h"

constexpr int FADE_RATE = 4;
constexpr int HAIKEI_SPEED = 16; // 大きいほど遅い

/*----------------------------------------------------------------------------*/
//     White LED Control
/*----------------------------------------------------------------------------*/
void WhiteLed::clear_all(void) {
  for (int j=0; j<MAX_KAMABOKO_NUM; j++){
    for (int i=0; i<MAX_EACH_LIGHT; i++){
      _kama[j].light_led_each(i, 0);
    }
  }
}
void WhiteLed::set_led(int kama, int lednum, uint16_t lvl) {
  _kama[kama].light_led_each(lednum, lvl);
}
int WhiteLed::gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV], int (&extkbd)[128]) {
  _total_time += difftm;  // difftm は MINIMUM_RESOLUTION [ms] （今は2ms）
  // light変数の0クリア
  for (int i=0; i<MAX_KAMABOKO_NUM; i++) {
    _kama[i].clr_light();
  }

  // tchev : (0..1599) + 1600*kamanum で絶対位置が表現され、イベントごとにその数値が入力される
  int max_ev = 0;
  constexpr int MAKE_FRAC = MAX_LOCATE/MAX_LIGHT; // 50
  for (int i=0; i<MAX_TOUCH_EV; i++){
    if (tchev[i] < 0){break;}
    else if (tchev[i] >= MAX_LOCATE){continue;}
    int frac = (tchev[i]%MAKE_FRAC)*2;  // 0-98
    int pos = tchev[i]/MAKE_FRAC;
    for (int j=0; j<2; ++j){
      //  触った箇所の前後二つのLEDが点灯する
      if (pos+1+j < MAX_LIGHT) {
        int kamanum = (pos+1+j)/MAX_EACH_LIGHT;
        int lednum = (pos+1+j)%MAX_EACH_LIGHT;
        int lvl = (frac+100)>j*100? (frac+100)-j*100: 0;    // 0-198
        _kama[kamanum].add_light_lvl(lednum, lvl);
      }
      if (pos>=j){
        int kamanum = (pos+1+j)/MAX_EACH_LIGHT;
        int lednum = (pos+1+j)%MAX_EACH_LIGHT;
        int lvl = (199-frac)>j*100? (199-frac)-j*100: 0;
        _kama[kamanum].add_light_lvl(lednum, lvl);
      }
    }
    max_ev += 1;
  }

  // 外部演奏ノートの反映
  constexpr int MM = MAX_LIGHT/MAX_NOTE;
  for (int i=0; i<MAX_NOTE; i++) {
    int extlight = extkbd[i+KEYBD_LO-4];
    if (extlight) {
      for (int j=0; j<MM; j++) {
        int kamanum = (i*MM+j)/MAX_EACH_LIGHT;
        int lednum = (i*MM+j)%MAX_EACH_LIGHT;
        _kama[kamanum].set_light_lvl(lednum, extlight);// velocity値をそのまま入れる
      }
    }
  }
  return max_ev;
}
void WhiteLed::lighten_led(void) {
  // Fade job on/off の作成
  _kama[_crnt_kama].one_kamaboco(_total_time);

  // 0..MAX_KAMABOKO_NUM-1 通るたびにインクリメント
  _crnt_kama += 1;
  if (_crnt_kama >= MAX_KAMABOKO_NUM) {
    _crnt_kama = 0;
  }
}

/*----------------------------------------------------------------------------*/
//     Kamaboco Class
/*----------------------------------------------------------------------------*/
void Kamaboco::clr_light(void) {
  memset(&_light_lvl[0], 0, sizeof(int)*MAX_EACH_LIGHT);
}
void Kamaboco::add_light_lvl(int lednum, int lvl) {
  _light_lvl[lednum] += lvl;
}
void Kamaboco::set_light_lvl(int lednum, int lvl) {
  if (_light_lvl[lednum] < lvl) {
    _light_lvl[lednum] = lvl;
  }
}
void Kamaboco::one_kamaboco(long total_time) {
  long diff = total_time - _kama_fade_counter;
  bool fade_ena = false;
  if (diff > FADE_RATE) {
    _kama_fade_counter += FADE_RATE;
    fade_ena = true;
  }

  // 背景で薄く光っている
  int ptn = (total_time/HAIKEI_SPEED+(4*_kama_crnt_led))%64;
  ptn = ptn<32? ptn:64-ptn;
  light_led_each(_kama_crnt_led, ptn);

  _kama_crnt_led += 1;
  if (_kama_crnt_led >= MAX_EACH_LIGHT) {
    _kama_crnt_led = 0;
  }

  // Touch したところを光らせる（薄く光った後に上書き）
  for (int i=0; i<MAX_EACH_LIGHT; i++){
    if ((_light_lvl[i] > 0) || (_light_lvl_itp[i] > 0)){
      if (_light_lvl[i]>_light_lvl_itp[i]) {
        _light_lvl_itp[i] = _light_lvl[i];
      } else if (fade_ena){
        // だんだん暗くなるとき
        _light_lvl_itp[i] = (_light_lvl_itp[i]-_light_lvl[i])*3/4 + _light_lvl[i];
      }
      light_led_each(i, _light_lvl_itp[i]*20);
    }
  }
}
void Kamaboco::light_led_each(const int num, uint16_t strength) { // strength=0-4095
  int err;
  uint8_t adrs = num * 4 + 0x06;
  uint8_t dev = static_cast<uint8_t>(_mynum + PCA9685_OFSADRS);
  if (strength > 4000){strength = 4000;}
	err = PCA9685_write( dev, adrs, 0 );          // ONはtime=0
	err = PCA9685_write( dev, adrs+1, 0 );        // ONはtime=0
	err = PCA9685_write( dev, adrs+2, static_cast<uint8_t>(strength & 0x00ff) );// OFF 0-4095 (0-0x0fff) の下位8bit
	err = PCA9685_write( dev, adrs+3, static_cast<uint8_t>(strength>>8) );      // OFF 上位4bit
}
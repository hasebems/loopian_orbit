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

/*----------------------------------------------------------------------------*/
//     White LED Control
/*----------------------------------------------------------------------------*/
void WhiteLed::clear_all(void) {
  for (int j=0; j<MAX_KAMABOKO_NUM; j++){
    for (int i=0; i<MAX_EACH_LIGHT; i++){
      light_led_each(i, j, 0);
    }
  }
}
int WhiteLed::gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV], int (&extkbd)[128]) {
  _total_time += difftm;
  _fade_counter += difftm;
  if (_fade_counter > FADE_RATE){_fade_counter = 0;}

  //for (int x=0; x<MAX_LIGHT; x++){_light_lvl[x]=0;}
  // 0クリア
  memset(&_light_lvl[0], 0, sizeof(int)*MAX_LIGHT);

  // tchev : (0..1599) + 1600*kamanum で絶対位置が表現され、イベントごとにその数値が入力される
  int max_ev = 0;
  constexpr int MAKE_FRAC = MAX_LOCATE/MAX_LIGHT; // 50
  for (int i=0; i<MAX_TOUCH_EV; i++){
    if (tchev[i] < 0){break;}
    else if (tchev[i] >= MAX_LOCATE){continue;}
    int frac = (tchev[i]%MAKE_FRAC)*2;  // 0-98
    int pos = tchev[i]/MAKE_FRAC;
    for (int j=0; j<2; j++){
      //  触った箇所の前後二つのLEDが点灯する
      if (pos+1+j < MAX_LIGHT) {
        _light_lvl[pos+1+j] += (frac+100)>j*100? (frac+100)-j*100: 0;    // 0-198
      }
      if (pos>=j){
        _light_lvl[pos-j] += (199-frac)>j*100? (199-frac)-j*100: 0;
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
        if (extlight > _light_lvl[i*MM+j]){
          _light_lvl[i*MM+j] = extlight; // velocity値をそのまま入れる
        }
      }
    }
  }

  // 0..MAX_KAMABOKO_NUM-1 通るたびにインクリメント
  _update_counter += 1;
  if (_update_counter >= MAX_KAMABOKO_NUM) {
    _update_counter = 0;
  }

  uint16_t time10 = static_cast<uint16_t>(_total_time/10);
  for (int j=0; j<MAX_KAMABOKO_NUM; j++){
    uint16_t time = 0;
    if (j == _update_counter) {time = time10;}
    one_kamaboco(j, time);
  }
  if (_fade_counter == 0){_fade_counter = 1;}

  return max_ev;
}
void WhiteLed::one_kamaboco(int kamanum, uint16_t time) {
  const int offset_num = kamanum*MAX_EACH_LIGHT;

  for (int i=0; i<MAX_EACH_LIGHT; i++){
    int x = i+offset_num;
    if ((_light_lvl[x] > 0) || (_light_lvl_itp[x] > 0)){
      if (_light_lvl[x]>_light_lvl_itp[x]) {
        _light_lvl_itp[x] = _light_lvl[x];
      } else if (_fade_counter == 0){
        // だんだん暗くなるとき
        _light_lvl_itp[x] = (_light_lvl_itp[x]-_light_lvl[x])*3/4 + _light_lvl[x];
      }
      light_led_each(i, kamanum, _light_lvl_itp[x]*20);
    } else if (time != 0){
      // 背景で薄く光っている
      int ptn = (time+(4*i))%64;
      ptn = ptn<32? ptn:64-ptn;
      light_led_each(i, kamanum, ptn);
    }
  }
}
void WhiteLed::light_led_each(const int num, const int dev_num, uint16_t strength) { // strength=0-4095
  int err;
  uint8_t adrs = num * 4 + 0x06;
  uint8_t dev = static_cast<uint8_t>(dev_num + PCA9685_OFSADRS);
  if (strength > 4000){strength = 4000;}
	err = PCA9685_write( dev, adrs, 0 );          // ONはtime=0
	err = PCA9685_write( dev, adrs+1, 0 );        // ONはtime=0
	err = PCA9685_write( dev, adrs+2, static_cast<uint8_t>(strength & 0x00ff) );// OFF 0-4095 (0-0x0fff) の下位8bit
	err = PCA9685_write( dev, adrs+3, static_cast<uint8_t>(strength>>8) );      // OFF 上位4bit
}
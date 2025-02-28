/* ========================================
 *
 *  loopian::ORBIT touch_ad.cpp
 *    description: TouchEvent
 *    for Seeed XIAO RP2350
 *
 *  Copyright(c)2025 Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#include <Arduino.h>
#include "touch_ad.h"
#include "constants.h"
#include "i2cdevice.h"

/*--------------------------------------------------------*/
//     EachTerminal Class
/*--------------------------------------------------------*/
uint32_t EachTerminal::get_ad_value(uint32_t time, uint16_t raw_count) {

        if (_lowest_raw_count == 0) {
            //  初回
            _lowest_raw_count = raw_count;
            _highest_raw_count = _lowest_raw_count + INITIAL_LO_HI_DEFFERENCE;
        } else {
            if (raw_count <= _lowest_raw_count) {
                _lowest_raw_count = raw_count;
                _last_lowest_time = time;
            } else {
                if (time < _last_lowest_time) {
                    time += 0x10000;
                }
                if (time - _last_lowest_time > 500) {
                    // raw_count > _lowest_raw_count
                    // のまま、ある程度時間が経ったら 最低値の補正のため
                    // _lowest_raw_count を少しずつ大きくする
                    _lowest_raw_count += LOWEST_ADJUST_PACE;
                    if (time >= 0x10000) {
                        _last_lowest_time = time - 0x10000;
                    } else {
                        _last_lowest_time = time;
                    }
                }
            }
            if (raw_count > _highest_raw_count) {
                _highest_raw_count = raw_count;
            }
        }
        _current_touch = (raw_count - _lowest_raw_count) * 255 /
                         (_highest_raw_count - _lowest_raw_count);
        //return _current_touch;
        return raw_count/16;
}

/*--------------------------------------------------------*/
//     TouchAd Class
/*--------------------------------------------------------*/
void TouchAd::update_touch_event(EachTouch (&tch)[MAX_TOUCH_EV],
                                 bool (&available_dev)[MAX_KAMABOKO_NUM],
                                 const uint32_t time) {
    uint32_t ad_val[MAX_KAMABOKO_NUM * MAX_EACH_SENS] = {0};

    // すべてのタッチセンサのAD値を更新
#if 0
    ad_val[_kamaboko*MAX_EACH_SENS+_sensor] = get_ad_value(_kamaboko, _sensor, time);
    _sensor += 1;
    if (_sensor >= MAX_EACH_SENS) {
        _sensor = 0;
        //_kamaboko += 1;
        //if (_kamaboko >= MAX_KAMABOKO_NUM) {
        //    _kamaboko = 0;
        //}
    }
    mbr3110_set_terminal(_kamaboko, _sensor);
#else
    int terminal = (time/2)%4;
    mbr3110_set_terminal(0, terminal);
    unsigned char raw_data[8] = {0};
    int err = mbr3110_read_rawdata(0, terminal, raw_data);
    if (err) {
        return;
    }
    uint16_t raw_count = raw_data[8] + (raw_data[7] << 8);
    terminal = raw_data[0];
    ad_val[terminal] = raw_count/16;//_eachTerminal[0][terminal].get_ad_value(time, raw_count);
    //get_ad_value(0, terminal, time, raw_count);
#endif
    //  AD値が連続するタッチセンサの検索と、位置の算出
    tch[terminal]._position = ad_val[terminal];  // とりあえず 2番目の位置にAD値を入れる
}

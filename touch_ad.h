/* ========================================
 *
 *  loopian::ORBIT touch_ad.h
 *    description: TouchEvent
 *    for Seeed XIAO RP2350
 *
 *  Copyright(c)2025 Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#ifndef TOUCH_AD_H
#define TOUCH_AD_H

#include "constants.h"

/*--------------------------------------------------------*/
//     EachTouch Class
/*--------------------------------------------------------*/
struct EachTouch {
   public:
    EachTouch(void) : _active(false), _position(0), _depth(0) {}
    bool _active;
    uint32_t _position;
    uint32_t _depth;
};
/*--------------------------------------------------------*/
//     EachTerminal Class
/*--------------------------------------------------------*/
constexpr uint32_t INITIAL_LO_HI_DEFFERENCE = 200;
constexpr uint32_t LOWEST_ADJUST_PACE = 10;
class EachTerminal {
   public:
    EachTerminal(void)
        : _device(0),
          _number(0),
          _lowest_raw_count(0),
          _highest_raw_count(0),
          _current_touch(0),
          _last_lowest_time(0) {}
    void set_device_number(size_t device, size_t number) {
        _device = device;
        _number = number;
    }
    uint32_t get_ad_value(uint32_t time, uint16_t raw_count);
    void reset_highest_raw_count(void) {
        _highest_raw_count = _lowest_raw_count + INITIAL_LO_HI_DEFFERENCE;
    }

   private:
    size_t _device;
    size_t _number;
    uint32_t _lowest_raw_count;
    uint32_t _highest_raw_count;
    uint32_t _current_touch;  // 0 - 255
    uint32_t _last_lowest_time;
};
/*--------------------------------------------------------*/
//     TouchAd Class
/*--------------------------------------------------------*/
class TouchAd {
   public:
    TouchAd(void) :
        _kamaboko(0),
        _sensor(0) {}
    void set_device_number(void) {
        for (size_t i = 0; i < MAX_KAMABOKO_NUM; i++) {
            for (size_t j = 0; j < MAX_EACH_SENS; j++) {
                _eachTerminal[i][j].set_device_number(i, j);
            }
        }
    }
    void update_touch_event(EachTouch (&tch)[MAX_TOUCH_EV],
                            bool (&available_dev)[MAX_KAMABOKO_NUM],
                            const uint32_t time);

   private:
    //uint32_t get_ad_value(size_t device, size_t number, uint32_t time) {
    //    return _eachTerminal[device][number].get_ad_value(time);
    //}

    EachTerminal _eachTerminal[MAX_KAMABOKO_NUM][MAX_EACH_SENS];
    size_t   _kamaboko;
    size_t   _sensor;
};
#endif
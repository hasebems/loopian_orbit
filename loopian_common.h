/* ========================================
 *
 *  loopian::ORBIT loopian_common.h
 *    description: common function/variables
 *    for Raspberry Pi pico
 *
 *  Copyright(c)2024 Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#ifndef LOOPIAN_HARDWARE_H
#define LOOPIAN_HARDWARE_H
#include    <Arduino.h>
#include    "constants.h"
#include    "touch.h"

void setMidiNoteOn(uint8_t note, uint8_t vel);
void setMidiNoteOff(uint8_t note);
void setMidiControlChange(uint8_t controller, uint8_t value);
void setMidiProgramChange(uint8_t pcn , uint8_t chnl );

extern TouchEvent tchev[MAX_TOUCH_EV];
extern uint8_t velocity;

#endif
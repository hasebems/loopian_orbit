/* ========================================
 *
 *	constants.h
 *		description: TouchMidi Configuration
 *
 *	Copyright(c)2017- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt.
 *
 * ========================================
*/
#ifndef CONSTANTS_H
#define CONSTANTS_H

#define   MAX_KAMABOKO_NUM      12
#define   MAX_TOUCH_EV          8
#define   MAX_EACH_SENS         8

constexpr size_t MAX_NOTE = 96; // 1system が取りうる最大 Note 番号
constexpr int MAX_LOCATE = MAX_NOTE*100;

#define USE_CY8CMBR3110
#define USE_ADA88
#define USE_PCA9685

#endif

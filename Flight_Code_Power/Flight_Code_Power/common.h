/*
 * common.h
 *
 * Created: 29-05-2019 12:38:15
 *  Author: PRASHANT KURREY
 */ 

#ifndef COMMON_H
#define COMMON_H
#include "avr_compiler.h"
// operational mode
#define preflight_mode 0X4C
#define launch_mode 0x70
#define detumb_pre_mode 0x85
#define detumb_post_mode 0x86
#define nom_transmit_mode 0x89
#define nom_uplink_mode 0x8A
#define nom_idle_mode 0x88
#define low_power_mode 0x93
#define extreme_low_power_mode 0xA3
// #define emergency_mode 0x9X  //to be decided
#define restart_mode 0xB2
#define kill_mode 0xBE


#endif 

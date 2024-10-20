#ifndef __CONFIG_H
#define __CONFIG_H

// #define ADV
// #define GTV
// #define PINTV
// #define XRV

#ifdef ADV
#define HAS_CHARGING
#endif

#if !defined(XRV)
#define HAS_WS2812
#endif

// #define USE_BUZZER
#ifdef USE_BUZZER
#define BEEP_ON_POWER
#define BEEP_ON_DUTY_CYCLE
#endif

#endif

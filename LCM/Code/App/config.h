#ifndef __CONFIG_H
#define __CONFIG_H

// NOTE: These are defined in the uVision target settings
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

#define USE_BUZZER
#ifdef USE_BUZZER
// #define BEEP_ON_POWER_AND_LIGHTS
#endif

#define CONTROL_POWER

#endif

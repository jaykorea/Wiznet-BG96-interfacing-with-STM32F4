/*
 * Copyright (C) WIZnet, Inc. All rights reserved.
 * Use is subject to license terms.
 */
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>


// Change _SKT_LTE_M1_LOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#define _SKT_LTE_M1_LOGLEVEL_ 3

#if defined(ARDUINO_ARCH_SAMD) 

#define LOGERROR(x)    if(_SKT_LTE_M1_LOGLEVEL_>0) { SerialUSB.print("[SKT_LTE_M1] "); SerialUSB.println(x); }
#define LOGERROR1(x,y) if(_SKT_LTE_M1_LOGLEVEL_>0) { SerialUSB.print("[SKT_LTE_M1] "); SerialUSB.print(x); SerialUSB.print(" "); SerialUSB.println(y); }
#define LOGWARN(x)     if(_SKT_LTE_M1_LOGLEVEL_>1) { SerialUSB.print("[SKT_LTE_M1] "); SerialUSB.println(x); }
#define LOGWARN1(x,y)  if(_SKT_LTE_M1_LOGLEVEL_>2) { SerialUSB.print("[SKT_LTE_M1] "); SerialUSB.print(x); SerialUSB.print(" "); SerialUSB.println(y); }
#define LOGINFO(x)     if(_SKT_LTE_M1_LOGLEVEL_>2) { SerialUSB.print("[SKT_LTE_M1] "); SerialUSB.println(x); }
#define LOGINFO1(x,y)  if(_SKT_LTE_M1_LOGLEVEL_>2) { SerialUSB.print("[SKT_LTE_M1] "); SerialUSB.print(x); SerialUSB.print(" "); SerialUSB.println(y); }

#define LOGDEBUG(x)      if(_SKT_LTE_M1_LOGLEVEL_>3) { Serial.print(F("[SKT_LTE_M1] ")); SerialUSB.println(x); }
#define LOGDEBUG0(x)     if(_SKT_LTE_M1_LOGLEVEL_>3) { Serial.print(F("[SKT_LTE_M1] ")); SerialUSB.print(x); }
#define LOGDEBUG1(x,y)   if(_SKT_LTE_M1_LOGLEVEL_>3) { Serial.print(F("[SKT_LTE_M1] ")); SerialUSB.print(x); SerialUSB.print(" "); SerialUSB.println(y); }
#define LOGDEBUG2(x,y,z) if(_SKT_LTE_M1_LOGLEVEL_>3) { Serial.print(F("[SKT_LTE_M1] ")); SerialUSB.print(x); SerialUSB.print(" "); SerialUSB.print(y); SerialUSB.print(" "); SerialUSB.println(z); }
#define LOGDEBUG_TYPE(x,y)      if(_SKT_LTE_M1_LOGLEVEL_>3) { Serial.print(F("[SKT_LTE_M1] ")); SerialUSB.print(x,y); }
#define LOGDEBUG_LN_TYPE(x,y)   if(_SKT_LTE_M1_LOGLEVEL_>3) { Serial.print(F("[SKT_LTE_M1] ")); SerialUSB.println(x,y); }
#else
    
#define LOGERROR(x)    if(_SKT_LTE_M1_LOGLEVEL_>0) { printf("[SKT_LTE_M1] "); printf(x\r\n); }
#define LOGERROR1(x,y) if(_SKT_LTE_M1_LOGLEVEL_>0) { printf("[SKT_LTE_M1] "); printf(x); printf(" "); printf(y); }
#define LOGWARN(x)     if(_SKT_LTE_M1_LOGLEVEL_>1) { printf("[SKT_LTE_M1] "); printf(x); }
#define LOGWARN1(x,y)  if(_SKT_LTE_M1_LOGLEVEL_>1) { printf("[SKT_LTE_M1] "); printf(x); printf(" "); printf(y); }
#define LOGINFO(x)     if(_SKT_LTE_M1_LOGLEVEL_>2) { printf("[SKT_LTE_M1] "); printf(x); }
#define LOGINFO1(x,y)  if(_SKT_LTE_M1_LOGLEVEL_>2) { printf("[SKT_LTE_M1] "); printf(x); printf(" "); printf(y); }

#define LOGDEBUG(x)      if(_SKT_LTE_M1_LOGLEVEL_>3) {  printf("[SKT_LTE_M1_Debug] "); printf(x); }
#define LOGDEBUG0(x)     if(_SKT_LTE_M1_LOGLEVEL_>3) {  printf("[SKT_LTE_M1_Debug] "); printf(x); }
#define LOGDEBUG1(x,y)   if(_SKT_LTE_M1_LOGLEVEL_>3) {  printf("[SKT_LTE_M1_Debug] ");  printf(x);  printf(" ");  printf(y); }
#define LOGDEBUG2(x,y,z) if(_SKT_LTE_M1_LOGLEVEL_>3) {  printf("[SKT_LTE_M1_Debug] ");  printf(x);  printf(" "); printf(y);  printf(F(" "));  printf(z); }
#define LOGDEBUG_TYPE(x,y)      if(_SKT_LTE_M1_LOGLEVEL_>3) {  printf("[SKT_LTE_M1_Debug] ");  printf(x,y); }
#define LOGDEBUG_LN_TYPE(x,y)   if(_SKT_LTE_M1_LOGLEVEL_>3) {  printf("[SKT_LTE_M1_Debug] ");  printf(x,y); }

#endif // defined(ARDUINO_ARCH_SAMD) 

#endif // __DEBUG_H__

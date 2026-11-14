#pragma once

#include <Arduino.h>
#include <WiFi.h>  // the real ESP32 WiFi library (this file used to be named wifi.h, which shadowed it on Windows)


void wifi_set_sta(void);  //Access the WiFi network in STA mode
String get_wifi_state(void);
void wifi_init(void);
void wifi_loop(void);
void wifi_restart(void);



extern char wifi_ssid[50];
extern char wifi_password[30];

typedef struct WIFI_STATE_TypeDef {
  String SERVER = "server";
  String CLIENT = "client";
  String CLOSE = "close";
} WIFI_STATE_TypeDef;

extern WIFI_STATE_TypeDef WIFI_STATE;
extern bool wifi_restart_flag ;






/*
 * cmd_parser.c
 *
 *  Created on: Feb 8, 2026
 *      Author: Nuria
 */


#include "cmd_parser.h"
#include <string.h>
#include <stdio.h>

static void trim_crlf(char *s)
{
  size_t n = strlen(s);
  while (n && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' ' || s[n-1] == '\t')) {
    s[n-1] = 0;
    n--;
  }
}

int Cmd_ParseLine(const char *line_in, SystemCommandSource_t src, SystemCommandMsg_t *out)
{
  if (!line_in || !out) return 0;

  char line[UART_LINE_MAX];
  strncpy(line, line_in, sizeof(line)-1);
  line[sizeof(line)-1] = 0;
  trim_crlf(line);

  memset(out, 0, sizeof(*out));
  out->src = src;

  // CONT ON / CONT OFF
  if (strcmp(line, "CONT ON") == 0) { out->type = CMD_START_CONTINUOUS; return 1; }
  if (strcmp(line, "CONT OFF") == 0){ out->type = CMD_STOP_CONTINUOUS;  return 1; }

  // READ
  if (strcmp(line, "READ") == 0)    { out->type = CMD_FORCE_READ;       return 1; }

  // WIFI <ssid> <pass>
  if (strncmp(line, "WIFI ", 5) == 0) {
    out->type = CMD_SET_WIFI;
    char ssid[WIFI_SSID_MAX] = {0};
    char pass[WIFI_PASS_MAX] = {0};
    if (sscanf(line + 5, "%31s %63s", ssid, pass) == 2) {
      strncpy(out->u.wifi.ssid, ssid, WIFI_SSID_MAX);
      strncpy(out->u.wifi.pass, pass, WIFI_PASS_MAX);
      return 1;
    }
    return 0;
  }

  // RTC 2026-02-08 19:30:00
  if (strncmp(line, "RTC ", 4) == 0) {
    out->type = CMD_SET_RTC;
    int y, mo, d, hh, mm, ss;
    if (sscanf(line + 4, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &hh, &mm, &ss) == 6) {
      out->u.rtc.year  = (uint16_t)y;
      out->u.rtc.month = (uint8_t)mo;
      out->u.rtc.day   = (uint8_t)d;
      out->u.rtc.hour  = (uint8_t)hh;
      out->u.rtc.min   = (uint8_t)mm;
      out->u.rtc.sec   = (uint8_t)ss;
      return 1;
    }
    // formato alternativo: RTC y m d hh mm ss
    if (sscanf(line + 4, "%d %d %d %d %d %d", &y, &mo, &d, &hh, &mm, &ss) == 6) {
      out->u.rtc.year  = (uint16_t)y;
      out->u.rtc.month = (uint8_t)mo;
      out->u.rtc.day   = (uint8_t)d;
      out->u.rtc.hour  = (uint8_t)hh;
      out->u.rtc.min   = (uint8_t)mm;
      out->u.rtc.sec   = (uint8_t)ss;
      return 1;
    }
    return 0;
  }

  return 0; // desconocido
}

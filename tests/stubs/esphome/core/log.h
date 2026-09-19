#pragma once
#include <cstdio>
extern bool g_log;
#define ESP_LOG_(lvl, tag, fmt, ...) do { if (g_log) printf("      [" lvl "] " fmt "\n", ##__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, ...) ESP_LOG_("E", tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) ESP_LOG_("W", tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) ESP_LOG_("I", tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) ESP_LOG_("D", tag, __VA_ARGS__)
#define ESP_LOGV(tag, ...) do {} while (0)
#define ESP_LOGCONFIG(tag, ...) do {} while (0)
#define LOG_PIN(a, b)
#define LOG_SENSOR(a, b, c)
#define LOG_BINARY_SENSOR(a, b, c)
#define LOG_UPDATE_INTERVAL(a)
#define YESNO(b) ((b) ? "YES" : "NO")

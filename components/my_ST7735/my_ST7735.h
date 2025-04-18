#pragma once

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "ili9340.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"

#include "driver/gpio.h"


#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

TickType_t FillTest(TFT_t * dev, int width, int height);

TickType_t ColorBarTest(TFT_t * dev, int width, int height);

TickType_t ArrowTest(TFT_t * dev, FontxFile *fx, uint16_t model, int width, int height);

TickType_t DirectionTest(TFT_t * dev, FontxFile *fx, int width, int height);

TickType_t HorizontalTest(TFT_t * dev, FontxFile *fx, int width, int height);

TickType_t VerticalTest(TFT_t * dev, FontxFile *fx, int width, int height);

TickType_t LineTest(TFT_t * dev, int width, int height);

TickType_t CircleTest(TFT_t * dev, int width, int height);

TickType_t RectAngleTest(TFT_t * dev, int width, int height);

TickType_t TriangleTest(TFT_t * dev, int width, int height);

TickType_t RoundRectTest(TFT_t * dev, int width, int height);

TickType_t FillRectTest(TFT_t * dev, int width, int height);

TickType_t ColorTest(TFT_t * dev, int width, int height) ;

TickType_t ScrollTest(TFT_t * dev, FontxFile *fx, int width, int height);

void ScrollReset(TFT_t * dev, int width, int height);

TickType_t BMPTest(TFT_t * dev, char * file, int width, int height);

TickType_t JPEGTest(TFT_t * dev, char * file, int width, int height);

TickType_t PNGTest(TFT_t * dev, char * file, int width, int height);

TickType_t IconTest(TFT_t * dev, char * file, int width, int height, int xpos, int ypos);

TickType_t TextBoxTest(TFT_t * dev, FontxFile *fx, int width, int height);

void TFT(void *pvParameters);

void listSPIFFS(char * path);

esp_err_t mountSPIFFS(char * path, char * label, int max_files);















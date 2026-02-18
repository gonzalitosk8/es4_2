#pragma once
#include <stdint.h>
#include "engine_s.h"

typedef struct {
  int sample_rate;
  int channels;      // 1 o 2
  float seconds;
} bake_desc_t;

int bake_wav(engine_s* e,
             const bake_desc_t* d,
             const char* out_path,
             float throttle_start,
             float throttle_end,
             float rpm_start,
             float rpm_end);
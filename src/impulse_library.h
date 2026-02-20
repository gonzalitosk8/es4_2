#pragma once
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// Tipo de preset
typedef struct {
    const char* name;
    const double* samples;
    size_t size;
} impulse_preset_t;

// Referencia al array que está en convo_filter_s.h
extern const double g_convo_filter_impulse[];
extern const size_t g_convo_filter_impulse_size;

// Variables globales (definidas UNA sola vez aquí)
const double* g_active_impulse = NULL;
size_t        g_active_impulse_size = 0;

// Presets
static const impulse_preset_t impulse_presets[] = {
    { "auto_4cil", g_convo_filter_impulse, g_convo_filter_impulse_size },
    { NULL, NULL, 0 }
};

const size_t num_impulse_presets = 1;

// Funciones que usa hotreload_engine.h
const impulse_preset_t* find_impulse_preset(const char* name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < num_impulse_presets; i++) {
        if (impulse_presets[i].name && strcmp(impulse_presets[i].name, name) == 0) {
            return &impulse_presets[i];
        }
    }
    return NULL;
}

void list_impulse_presets(void)
{
    printf("[hr] Preset disponible: auto_4cil (%zu coeficientes)\n", g_convo_filter_impulse_size);
}
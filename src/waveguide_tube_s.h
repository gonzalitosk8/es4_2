#ifndef WAVEGUIDE_TUBE_S_H
#define WAVEGUIDE_TUBE_S_H

#include <stdlib.h>
#include <string.h>

typedef struct {
    float* buf_fwd;     // p+: onda que avanza hacia la salida
    float* buf_rev;     // p-: onda que vuelve hacia el motor
    int    length;      // Retardo en muestras (N = L * fs / c)
    int    write_pos;   // Cabeza de escritura del buffer circular
} waveguide_tube_t;

// Inicializa el tubo con una longitud fija de muestras
void waveguide_tube_init(waveguide_tube_t* wg, int delay_samples) {
    wg->length = delay_samples;
    wg->write_pos = 0;

    // Asignación de memoria limpia
    wg->buf_fwd = (float*)calloc(delay_samples, sizeof(float));
    wg->buf_rev = (float*)calloc(delay_samples, sizeof(float));
}

// Libera la memoria
void waveguide_tube_free(waveguide_tube_t* wg) {
    if (wg->buf_fwd) { free(wg->buf_fwd); wg->buf_fwd = NULL; }
    if (wg->buf_rev) { free(wg->buf_rev); wg->buf_rev = NULL; }
}

// Procesa una muestra a través del tubo
void waveguide_tube_step(waveguide_tube_t* wg, float in_fwd, float in_rev, float* out_fwd, float* out_rev) {
    // Leer el extremo opuesto (lo que entró hace 'length' muestras)
    int read_pos = (wg->write_pos + 1) % wg->length;

    *out_fwd = wg->buf_fwd[read_pos];
    *out_rev = wg->buf_rev[wg->write_pos];

    // Escribir la nueva entrada en la cabeza actual
    wg->buf_fwd[wg->write_pos] = in_fwd;
    wg->buf_rev[read_pos] = in_rev;

    // Avanzar la cabeza de escritura
    wg->write_pos = read_pos;
}

#endif // WAVEGUIDE_TUBE_S_H
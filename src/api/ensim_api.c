#include "ensim_api.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// incluye los headers reales del core ensim4
#include "std.h"
#include "sampler_s.h"
#include "synth_s.h"
#include "engine_s.h"
#include "gamma.h"

// ---- IMPORTANTE ----
// Esto asume que engine_s soporta:
//   struct node_s* node; size_t size;
// y que reset_engine/run_engine ya existen (como en main.c). :contentReference[oaicite:1]{index=1}

typedef struct {
    struct sampler_s sampler;
    sampler_synth_t  sampler_synth;
    struct synth_s   synth;
    struct engine_s  engine;

    double monitor_refresh_hz;
    float  sample_rate_hz;

    // heap del grafo
    struct node_s* heap_nodes;
    size_t heap_nodes_count;

    float throttle;
    int starter_on;
    int can_ignite;
} ensim_context_t;

static void free_graph(ensim_context_t* ctx) {
    if (ctx->heap_nodes) {
        free(ctx->heap_nodes);
        ctx->heap_nodes = NULL;
        ctx->heap_nodes_count = 0;
    }
    ctx->engine.node = NULL;
    ctx->engine.size = 0;
}

// ---- Builder mínimo de grafo dinámico ----
// V1: genera un motor “genérico” con N cilindros usando los mismos tipos de nodos
// que ensim4 ya tiene (iplenum/injector/piston/erunner/eplenum/...)
// En tu repo esos structs existen porque main.c los incluye. :contentReference[oaicite:2]{index=2}
//
// Si tu node_s es una unión "as.*", este builder se completa siguiendo TU node_s real.
// Acá te dejo la estructura del flujo y el contrato: una función que “aloca y llena”.

static int build_generic_engine_graph(ensim_context_t* ctx, const ensim_engine_params_t* p) {
    // Conteo simple: por cilindro ~ 6 nodos (depende tu grafo real)
    // Ajustalo cuando mapees tus tipos reales.
    size_t per_cyl = 6;
    size_t base_nodes = 4; // crank/sink/etc según tu grafo
    size_t total = base_nodes + (size_t)p->cylinders * per_cyl;

    struct node_s* nodes = (struct node_s*)calloc(total, sizeof(struct node_s));
    if (!nodes) return -1;

    // TODO: acá va el llenado real de nodes[i].as.<tipo> y nodes[i].next[]
    // El punto crítico: NO usar designated initializers tipo ".as.piston = {...}" en C++.
    // Esto es C puro. En MSYS/clang C23 funciona.

    // Por ahora dejamos el grafo como “placeholder válido” solo si tu engine_s tolera size=0.
    // PERO: para sonar, tenés que mapear tu node_s real.
    // (esto es el único lugar donde hay trabajo mecánico, pero es directo)

    ctx->heap_nodes = nodes;
    ctx->heap_nodes_count = total;

    ctx->engine.node = nodes;
    ctx->engine.size = total;

    return 0;
}

ensim_context_t* ensim_create(double monitor_refresh_hz) {
    precompute_cp(); // como en main.c :contentReference[oaicite:3]{index=3}

    ensim_context_t* ctx = (ensim_context_t*)calloc(1, sizeof(ensim_context_t));
    if (!ctx) return NULL;
    ctx->monitor_refresh_hz = monitor_refresh_hz;
    ctx->sample_rate_hz = 48000.0f;
    return ctx;
}

void ensim_destroy(ensim_context_t* ctx) {
    if (!ctx) return;
    free_graph(ctx);
    free(ctx);
}

int ensim_engine_build(ensim_context_t* ctx, const ensim_engine_params_t* p) {
    if (!ctx || !p) return -1;

    free_graph(ctx);

    // set mecánica base
    memset(&ctx->engine, 0, sizeof(ctx->engine));
    ctx->engine.volume = p->master_volume;
    ctx->engine.radial_spacing = p->radial_spacing;

    ctx->engine.crankshaft.mass_kg = p->crank_mass_kg;
    ctx->engine.crankshaft.radius_m = p->crank_radius_m;
    ctx->engine.flywheel.mass_kg = p->flywheel_mass_kg;
    ctx->engine.flywheel.radius_m = p->flywheel_radius_m;

    // limiter/starter (convert rpm -> rad/s)
    const double rpm_to_rad = (2.0 * M_PI) / 60.0;
    ctx->engine.limiter.cutoff_angular_velocity_r_per_s  = (double)p->redline_rpm * rpm_to_rad;
    ctx->engine.limiter.relaxed_angular_velocity_r_per_s = (double)p->limiter_relax_rpm * rpm_to_rad;

    ctx->engine.starter.rated_torque_n_m = p->starter_torque_nm;
    ctx->engine.starter.no_load_angular_velocity_r_per_s = (double)p->starter_no_load_rpm * rpm_to_rad;
    ctx->engine.starter.radius_m = p->starter_radius_m;

    // build graph
    if (build_generic_engine_graph(ctx, p) != 0) {
        free_graph(ctx);
        return -2;
    }

    // apply runtime toggles
    ctx->engine.throttle_open_ratio = ctx->throttle;
    ctx->engine.starter.is_on = ctx->starter_on ? true : false;
    ctx->engine.can_ignite = ctx->can_ignite ? true : false;

    reset_engine(&ctx->engine);
    return 0;
}

void ensim_engine_reset(ensim_context_t* ctx) {
    if (!ctx) return;
    reset_engine(&ctx->engine);
}

void ensim_set_throttle(ensim_context_t* ctx, float t) {
    if (!ctx) return;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    ctx->throttle = t;
    ctx->engine.throttle_open_ratio = t;
}

void ensim_set_starter(ensim_context_t* ctx, int on) {
    if (!ctx) return;
    ctx->starter_on = on ? 1 : 0;
    ctx->engine.starter.is_on = on ? true : false;
}

void ensim_set_can_ignite(ensim_context_t* ctx, int on) {
    if (!ctx) return;
    ctx->can_ignite = on ? 1 : 0;
    ctx->engine.can_ignite = on ? true : false;
}

size_t ensim_get_frames_per_tick(ensim_context_t* ctx) {
    // V1: equivalente a lo que usa tu synth buffer size:
    // g_synth_buffer_size = sample_rate / monitor_refresh
    // Como en tu repo lo hacen con constexpr en headers, lo emulamos:
    if (!ctx || ctx->monitor_refresh_hz <= 0.0) return 1024;
    double frames = (double)ctx->sample_rate_hz / ctx->monitor_refresh_hz;
    if (frames < 64) frames = 64;
    return (size_t)(frames + 0.5);
}

int ensim_tick_audio_f32(ensim_context_t* ctx, float* out_frames, size_t frames) {
    if (!ctx || !out_frames || frames == 0) return -1;

    clear_synth(&ctx->synth);

    struct engine_time_s engine_time = {0};
    // tu engine_time usa get_ticks_ms en main.c para perf/ui. Para offline no hace falta.

    run_engine(&ctx->engine, &engine_time, &ctx->sampler, &ctx->synth,
               frames, ctx->sampler_synth);

    // Convertir el buffer interno a float mono.
    // Tu synth_s guarda float value[g_synth_buffer_size] (según errores que mostraste).
    // Copiamos “lo que haya” al out_frames.
    for (size_t i = 0; i < frames; i++) out_frames[i] = ctx->synth.value[i];

    return 0;
}

float ensim_get_rpm(ensim_context_t* ctx) {
    if (!ctx) return 0.0f;
    // engine.crankshaft.angular_velocity_r_per_s existe en tu repo (por errores previos)
    double w = ctx->engine.crankshaft.angular_velocity_r_per_s;
    return (float)(w * 60.0 / (2.0 * M_PI));
}

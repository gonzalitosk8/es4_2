/*
 * hotreload_engine.h
 *
 * Sistema de hot-reload completo para ensim4.
 *
 * CÓMO FUNCIONA:
 *   En lugar de usar constexpr (que son literales grabadas a fuego en el binario),
 *   aquí declaramos variables globales mutables que el loop principal puede
 *   actualizar en cualquier momento leyendo el JSON.
 *
 *   Los nodos del motor (g_hr_nodes[]) se reconstruyen desde cero con los nuevos
 *   valores cada vez que el JSON cambia, y luego se hace apuntar g_engine a ellos.
 *
 * USO EN main.c:
 *   1. #include "hotreload_engine.h"   (en lugar o además de engine_3_cyl.h)
 *   2. Inicializar: hr_init("configs/engine_current.json");
 *   3. En el loop: hr_tick() -- detecta cambios y recarga si hace falta.
 *   4. hr_apply_to_engine(&g_engine) -- apunta g_engine a los nodos nuevos.
 *
 * PARÁMETROS HOT-RELOADABLES (sin recompilar):
 *   - sound_volume
 *   - crankshaft: mass_kg, radius_m
 *   - flywheel: mass_kg, radius_m
 *   - limiter_cutoff_r_per_s, limiter_relaxed_r_per_s
 *   - starter: rated_torque_n_m, no_load_r_per_s, radius_m
 *   - piston: diameter_m, crank_throw_m, connecting_rod_m,
 *             connecting_rod_mass_kg, compression_height_m
 *   - chamber_volume_m3, throttle_volume_m3, irunner_volume_m3,
 *     injector_volume_m3, erunner_volume_m3, eplenum_volume_m3,
 *     exhaust_volume_m3, max_flow_area_m2
 *   - topología de nodos completa (array "nodes" en el JSON)
 */

#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "impulse_library.h"

// ─────────────────────────────────────────────────────────────
// Límites
// ─────────────────────────────────────────────────────────────
#define HR_MAX_NODES      64
#define HR_MAX_CYLINDERS  16
#define HR_MAX_CONNECTIONS 16

// ─────────────────────────────────────────────────────────────
// Parámetros "en vivo" del motor (todos mutables)
// ─────────────────────────────────────────────────────────────
typedef struct {
    // Meta
    char name[128];
    double sound_volume;

    // Cigüeñal / volante
    double crank_mass_kg;
    double crank_radius_m;
    double fly_mass_kg;
    double fly_radius_m;

    // Limitador
    double limiter_cutoff_r_per_s;
    double limiter_relaxed_r_per_s;

    // Starter
    double starter_rated_torque_n_m;
    double starter_no_load_r_per_s;
    double starter_radius_m;

    // Pistón
    double piston_diameter_m;
    double piston_crank_throw_m;
    double piston_connecting_rod_m;
    double piston_connecting_rod_mass_kg;
    double piston_compression_height_m;

    // Volúmenes de cámaras
    double chamber_volume_m3;
    double throttle_volume_m3;
    double irunner_volume_m3;
    double injector_volume_m3;
    double erunner_volume_m3;
    double eplenum_volume_m3;
    double exhaust_volume_m3;
    double source_sink_volume_m3;

    // Áreas de flujo
    double max_flow_area_m2;

    // ── Acústica del tubo CFD (eplenum) ─────────────────────
    // Defaults usados si el nodo eplenum no los sobreescribe.
    // pipe_length_m: longitud equivalente del tubo de escape.
    //   Determina la frecuencia de resonancia junto con la velocidad
    //   numérica de la onda (v = dx/dt = pipe_length/(128*dt_substep)).
    //   Para que f_fundamental ~ 150-250 Hz en un auto, usar 1.2-1.6m.
    //   Para una moto de 1 cil, 0.4-0.6m da f ~ 450-700 Hz.
    double default_pipe_length_m;
    // mic_position_ratio: [0,1] donde 0=entrada del tubo, 1=extremo abierto.
    //   0.08-0.15: sonido directo, agresivo (cerca de la fuente).
    //   0.85-0.95: sonido irradiado, más grave y resonante (tipo escape real).
    double default_mic_position_ratio;
    // velocity_low_pass_cutoff_frequency_hz: suaviza la excitación antes
    //   de entrar al CFD. Más bajo = más suave y grave. 1500-3000 para autos,
    //   4000-7000 para motos.
    double default_velocity_low_pass_hz;

    // ── Timing de válvulas e ignición ────────────────────────
    // Expresados en radianes relativos al TDC del pistón.
    // Permiten variar el carácter: válvulas tempranas = más solapamiento
    // (suena "sucio"), tardías = más limpio y agudo.
    double irunner_valve_engage_r;    // default: -0.25π  (apertura admisión)
    double irunner_valve_ramp_r;      // default:  1.00π  (duración rampa)
    double piston_valve_engage_r;     // default:  2.70π  (apertura escape)
    double piston_valve_ramp_r;       // default:  0.95π  (duración rampa)
    double sparkplug_engage_r;        // default:  2.05π  (avance encendido)
    double sparkplug_on_r;            // default:  0.25π  (duración chispa)

    // ── Fricción / cabeza de pistón ─────────────────────────
    double piston_head_clearance_m;   // default: 0.007m
    double gas_damping_tau_s;         // default: 0.53e-3 s

    // ── Convolución acústica configurable ───────────────────
    // Si impulse_size > 0, se usa impulse[] en vez del hardcodeado.
    // Permite definir la respuesta impulsional por tipo de vehículo:
    // auto, moto, camión, etc. — sin recompilar.
    double  impulse[16384];
    size_t  impulse_size;             // 0 = usar convo_filter_s.h hardcodeado
} hr_params_t;

// ─────────────────────────────────────────────────────────────
// Descripción JSON de un nodo (antes de construirlo en C)
// ─────────────────────────────────────────────────────────────
typedef struct {
    int  id;
    char type[32];
    int  connections[HR_MAX_CONNECTIONS];
    int  num_connections;
    double piston_theta_r;
    int  parent;       // para injector: id del irunner al que espía
    int  bank;         // para eplenum
    double volume_m3;  // override por nodo (si está en el JSON)
    double initial_pressure_pa;
    double initial_temp_k;
    // Per-nodo acústico (solo para eplenum; -1 = usar el default global)
    double pipe_length_m;          // -1 = usar default_pipe_length_m
    double mic_position_ratio;     // -1 = usar default_mic_position_ratio
    double velocity_low_pass_hz;   // -1 = usar default_velocity_low_pass_hz
} hr_node_desc_t;

// ─────────────────────────────────────────────────────────────
// Estado interno del hot-reloader
// ─────────────────────────────────────────────────────────────
static hr_params_t     g_hr_params  = {};
static hr_node_desc_t  g_hr_desc[HR_MAX_NODES] = {};
static int             g_hr_num_nodes = 0;
static struct node_s   g_hr_nodes[HR_MAX_NODES] = {};   // nodos reconstruidos
static char            g_hr_filepath[512] = {};
static long            g_hr_last_filesize = -1;

// ─────────────────────────────────────────────────────────────
// Valores por defecto de los parámetros de válvulas / ignición
// (estos se pueden exponer al JSON más adelante si se necesita)
// ─────────────────────────────────────────────────────────────
#define HR_IRUNNER_VALVE_ENGAGE_R   (-0.25 * 3.14159265359)
#define HR_IRUNNER_VALVE_RAMP_R     ( 1.00 * 3.14159265359)
#define HR_PISTON_VALVE_ENGAGE_R    ( 2.70 * 3.14159265359)
#define HR_PISTON_VALVE_RAMP_R      ( 0.95 * 3.14159265359)
#define HR_SPARKPLUG_ENGAGE_R       ( 2.05 * 3.14159265359)
#define HR_SPARKPLUG_ON_R           ( 0.25 * 3.14159265359)
#define HR_GAS_DAMPING_TAU_S        ( 0.53e-3 )
#define HR_PISTON_HEAD_DENSITY      ( 9500.0 )
#define HR_PISTON_HEAD_CLEARANCE_M  ( 0.007 )
#define HR_PISTON_DYN_FRICTION      ( 0.029 )
#define HR_PISTON_STAT_FRICTION     ( 0.90 )
#define HR_EPLENUM_PIPE_LEN_M       ( 1.1 )
#define HR_MIC_POS_RATIO            ( 0.1 )
#define HR_VEL_LPF_HZ               ( 6000.0 )

// ─────────────────────────────────────────────────────────────
// Ratios de área de flujo (relativos a max_flow_area_m2)
// ─────────────────────────────────────────────────────────────
#define HR_SOURCE_AREA_RATIO    0.300
#define HR_THROTTLE_AREA_RATIO  0.250
#define HR_IRUNNER_AREA_RATIO   0.600
#define HR_INJECTOR_AREA_RATIO  0.005
#define HR_PISTON_AREA_RATIO    0.500
#define HR_ERUNNER_AREA_RATIO   0.450
#define HR_EPLENUM_AREA_RATIO   1.800
#define HR_EXHAUST_AREA_RATIO   0.900

// ─────────────────────────────────────────────────────────────
// Helpers de construcción de cámara
// ─────────────────────────────────────────────────────────────
static struct chamber_s
hr_make_chamber(double vol, double area, double tau)
{
    struct chamber_s c = {};
    c.volume_m3 = vol;
    c.nozzle_max_flow_area_m2 = area;
    c.gas_momentum_damping_time_constant_s = tau;
    return c;
}

// ─────────────────────────────────────────────────────────────
// Construcción de nodos desde g_hr_desc[] y g_hr_params
// ─────────────────────────────────────────────────────────────
static void
hr_build_nodes(void)
{
    hr_params_t* p = &g_hr_params;
    double max_a = p->max_flow_area_m2;
    double src_vol = p->source_sink_volume_m3;
    double tau = p->gas_damping_tau_s > 0.0 ? p->gas_damping_tau_s : HR_GAS_DAMPING_TAU_S;

    // Defaults acústicos: si no están en params, usar los #define originales
    double def_pipe  = p->default_pipe_length_m      > 0.0 ? p->default_pipe_length_m      : HR_EPLENUM_PIPE_LEN_M;
    double def_mic   = p->default_mic_position_ratio  > 0.0 ? p->default_mic_position_ratio  : HR_MIC_POS_RATIO;
    double def_vlpf  = p->default_velocity_low_pass_hz > 0.0 ? p->default_velocity_low_pass_hz : HR_VEL_LPF_HZ;

    // Defaults de timing: si no están en params, usar los #define originales
    double ir_engage = p->irunner_valve_engage_r  != 0.0 ? p->irunner_valve_engage_r  : HR_IRUNNER_VALVE_ENGAGE_R;
    double ir_ramp   = p->irunner_valve_ramp_r    != 0.0 ? p->irunner_valve_ramp_r    : HR_IRUNNER_VALVE_RAMP_R;
    double pv_engage = p->piston_valve_engage_r   != 0.0 ? p->piston_valve_engage_r   : HR_PISTON_VALVE_ENGAGE_R;
    double pv_ramp   = p->piston_valve_ramp_r     != 0.0 ? p->piston_valve_ramp_r     : HR_PISTON_VALVE_RAMP_R;
    double sp_engage = p->sparkplug_engage_r      != 0.0 ? p->sparkplug_engage_r      : HR_SPARKPLUG_ENGAGE_R;
    double sp_on     = p->sparkplug_on_r          != 0.0 ? p->sparkplug_on_r          : HR_SPARKPLUG_ON_R;

    double head_clr  = p->piston_head_clearance_m > 0.0 ? p->piston_head_clearance_m  : HR_PISTON_HEAD_CLEARANCE_M;

    memset(g_hr_nodes, 0, sizeof(g_hr_nodes));

    int wave_idx = 0;

    for (int i = 0; i < g_hr_num_nodes; i++) {
        hr_node_desc_t* d = &g_hr_desc[i];
        struct node_s* n  = &g_hr_nodes[i];

        for (int j = 0; j < d->num_connections && j < HR_MAX_CONNECTIONS; j++) {
            n->next[j] = (uint8_t)d->connections[j];
        }

        if (strcmp(d->type, "source") == 0) {
            n->type = g_is_source;
            n->as.source.chamber = hr_make_chamber(src_vol, HR_SOURCE_AREA_RATIO * max_a, tau);

        } else if (strcmp(d->type, "throttle") == 0) {
            n->type = g_is_throttle;
            double vol = d->volume_m3 > 0.0 ? d->volume_m3 : p->throttle_volume_m3;
            n->as.throttle.chamber = hr_make_chamber(vol, HR_THROTTLE_AREA_RATIO * max_a, tau);

        } else if (strcmp(d->type, "irunner") == 0) {
            n->type = g_is_irunner;
            double vol = d->volume_m3 > 0.0 ? d->volume_m3 : p->irunner_volume_m3;
            n->as.irunner.chamber = hr_make_chamber(vol, HR_IRUNNER_AREA_RATIO * max_a, tau);
            n->as.irunner.valve.engage_r = d->piston_theta_r + ir_engage;
            n->as.irunner.valve.ramp_r   = ir_ramp;

        } else if (strcmp(d->type, "injector") == 0) {
            n->type = g_is_injector;
            double vol = d->volume_m3 > 0.0 ? d->volume_m3 : p->injector_volume_m3;
            n->as.injector.chamber = hr_make_chamber(vol, HR_INJECTOR_AREA_RATIO * max_a, tau);
            n->as.injector.nozzle_index = (d->parent >= 0) ? (size_t)d->parent : 0;

        } else if (strcmp(d->type, "piston") == 0) {
            n->type = g_is_piston;
            struct piston_s* ps = &n->as.piston;
            ps->chamber = hr_make_chamber(0.0, HR_PISTON_AREA_RATIO * max_a, tau);
            ps->valve.engage_r     = d->piston_theta_r + pv_engage;
            ps->valve.ramp_r       = pv_ramp;
            ps->sparkplug.engage_r = d->piston_theta_r + sp_engage;
            ps->sparkplug.on_r     = sp_on;
            ps->diameter_m                   = p->piston_diameter_m;
            ps->theta_r                      = -d->piston_theta_r;
            ps->crank_throw_length_m         = p->piston_crank_throw_m;
            ps->connecting_rod_length_m      = p->piston_connecting_rod_m;
            ps->connecting_rod_mass_kg       = p->piston_connecting_rod_mass_kg;
            ps->head_mass_density_kg_per_m3  = HR_PISTON_HEAD_DENSITY;
            ps->head_compression_height_m    = p->piston_compression_height_m;
            ps->head_clearance_height_m      = head_clr;
            ps->dynamic_friction_n_m_s_per_r = HR_PISTON_DYN_FRICTION;
            ps->static_friction_n_m_s_per_r  = HR_PISTON_STAT_FRICTION;

        } else if (strcmp(d->type, "erunner") == 0) {
            n->type = g_is_erunner;
            double vol = d->volume_m3 > 0.0 ? d->volume_m3 : p->erunner_volume_m3;
            n->as.erunner.chamber = hr_make_chamber(vol, HR_ERUNNER_AREA_RATIO * max_a, tau);

        } else if (strcmp(d->type, "eplenum") == 0) {
            n->type = g_is_eplenum;
            double vol = d->volume_m3 > 0.0 ? d->volume_m3 : p->eplenum_volume_m3;
            n->as.eplenum.chamber = hr_make_chamber(vol, HR_EPLENUM_AREA_RATIO * max_a, tau);
            n->as.eplenum.wave_index  = (size_t)wave_idx++;
            // Per-nodo o default global
            n->as.eplenum.pipe_length_m = (d->pipe_length_m > 0.0)
                ? d->pipe_length_m : def_pipe;
            n->as.eplenum.mic_position_ratio = (d->mic_position_ratio >= 0.0)
                ? d->mic_position_ratio : def_mic;
            n->as.eplenum.velocity_low_pass_cutoff_frequency_hz = (d->velocity_low_pass_hz > 0.0)
                ? d->velocity_low_pass_hz : def_vlpf;

        } else if (strcmp(d->type, "exhaust") == 0) {
            n->type = g_is_exhaust;
            double vol = d->volume_m3 > 0.0 ? d->volume_m3 : p->exhaust_volume_m3;
            n->as.exhaust.chamber = hr_make_chamber(vol, HR_EXHAUST_AREA_RATIO * max_a, tau);

        } else if (strcmp(d->type, "sink") == 0) {
            n->type = g_is_sink;
            n->as.sink.chamber = hr_make_chamber(src_vol, 0.0, tau);

        } else {
            fprintf(stderr, "[hr] nodo %d: tipo desconocido '%s', ignorado\n", d->id, d->type);
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Parseo JSON → g_hr_params + g_hr_desc[]
// ─────────────────────────────────────────────────────────────
static bool
hr_parse_json(const char* filepath)
{
    FILE* f = fopen(filepath, "r");
    if (!f) {
        fprintf(stderr, "[hr] no se pudo abrir '%s'\n", filepath);
        return false;
    }
    fseek(f, 0, SEEK_END);
    long flen = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* data = (char*)malloc((size_t)flen + 1);
    if (!data) { fclose(f); return false; }
    fread(data, 1, (size_t)flen, f);
    fclose(f);
    data[flen] = '\0';

    cJSON* root = cJSON_Parse(data);
    free(data);
    if (!root) {
        fprintf(stderr, "[hr] JSON inválido: %s\n", cJSON_GetErrorPtr());
        return false;
    }

    hr_params_t* p = &g_hr_params;

    // ── Nombre ────────────────────────────────────────────────
    cJSON* j;
    j = cJSON_GetObjectItem(root, "engine_name");
    if (j && j->valuestring) strncpy(p->name, j->valuestring, sizeof(p->name)-1);

    // ── Volumen de sonido ─────────────────────────────────────
    j = cJSON_GetObjectItem(root, "sound_volume");
    if (j) p->sound_volume = j->valuedouble;

    // ── Cigüeñal ──────────────────────────────────────────────
    cJSON* crank = cJSON_GetObjectItem(root, "crankshaft");
    if (crank) {
        j = cJSON_GetObjectItem(crank, "mass_kg");   if(j) p->crank_mass_kg  = j->valuedouble;
        j = cJSON_GetObjectItem(crank, "radius_m");  if(j) p->crank_radius_m = j->valuedouble;
    }

    // ── Volante ───────────────────────────────────────────────
    cJSON* fly = cJSON_GetObjectItem(root, "flywheel");
    if (fly) {
        j = cJSON_GetObjectItem(fly, "mass_kg");   if(j) p->fly_mass_kg  = j->valuedouble;
        j = cJSON_GetObjectItem(fly, "radius_m");  if(j) p->fly_radius_m = j->valuedouble;
    }

    // ── Limitador ─────────────────────────────────────────────
    j = cJSON_GetObjectItem(root, "limiter_cutoff_r_per_s");   if(j) p->limiter_cutoff_r_per_s  = j->valuedouble;
    j = cJSON_GetObjectItem(root, "limiter_relaxed_r_per_s");  if(j) p->limiter_relaxed_r_per_s = j->valuedouble;

    // ── Starter ───────────────────────────────────────────────
    cJSON* starter = cJSON_GetObjectItem(root, "starter");
    if (starter) {
        j = cJSON_GetObjectItem(starter, "rated_torque_n_m");  if(j) p->starter_rated_torque_n_m = j->valuedouble;
        j = cJSON_GetObjectItem(starter, "no_load_r_per_s");   if(j) p->starter_no_load_r_per_s  = j->valuedouble;
        j = cJSON_GetObjectItem(starter, "radius_m");          if(j) p->starter_radius_m          = j->valuedouble;
    }

    // ── Pistón ────────────────────────────────────────────────
    cJSON* piston = cJSON_GetObjectItem(root, "piston");
    if (piston) {
        j = cJSON_GetObjectItem(piston, "diameter_m");             if(j) p->piston_diameter_m            = j->valuedouble;
        j = cJSON_GetObjectItem(piston, "crank_throw_m");          if(j) p->piston_crank_throw_m          = j->valuedouble;
        j = cJSON_GetObjectItem(piston, "connecting_rod_m");       if(j) p->piston_connecting_rod_m       = j->valuedouble;
        j = cJSON_GetObjectItem(piston, "connecting_rod_mass_kg"); if(j) p->piston_connecting_rod_mass_kg = j->valuedouble;
        j = cJSON_GetObjectItem(piston, "compression_height_m");   if(j) p->piston_compression_height_m  = j->valuedouble;
    }

    // ── Volúmenes globales ────────────────────────────────────
    j = cJSON_GetObjectItem(root, "chamber_volume_m3");  if(j) p->chamber_volume_m3  = j->valuedouble;
    j = cJSON_GetObjectItem(root, "throttle_volume_m3"); if(j) p->throttle_volume_m3 = j->valuedouble;
    j = cJSON_GetObjectItem(root, "irunner_volume_m3");  if(j) p->irunner_volume_m3  = j->valuedouble;
    j = cJSON_GetObjectItem(root, "injector_volume_m3"); if(j) p->injector_volume_m3 = j->valuedouble;
    j = cJSON_GetObjectItem(root, "erunner_volume_m3");  if(j) p->erunner_volume_m3  = j->valuedouble;
    j = cJSON_GetObjectItem(root, "eplenum_volume_m3");  if(j) p->eplenum_volume_m3  = j->valuedouble;
    j = cJSON_GetObjectItem(root, "exhaust_volume_m3");  if(j) p->exhaust_volume_m3  = j->valuedouble;
    j = cJSON_GetObjectItem(root, "max_flow_area_m2");   if(j) p->max_flow_area_m2   = j->valuedouble;

    // ── Impulse response configurable ────────────────────────
    // "impulse_preset": "auto_4cil"  → busca en impulse_library.h
    // Si no está en el JSON, se mantiene lo que había (o 0 = hardcodeado)
    j = cJSON_GetObjectItem(root, "impulse_preset");
    if (j && j->valuestring) {
        const impulse_preset_t* preset = find_impulse_preset(j->valuestring);
        if (preset) {
            size_t copy_count = preset->size;
            if (copy_count > 16384) copy_count = 16384;
            memcpy(p->impulse, preset->samples, copy_count * sizeof(double));
            p->impulse_size = copy_count;
            printf("[hr] Impulso acústico: '%s' (%zu coeficientes)\n",
                   j->valuestring, copy_count);
        } else {
            printf("[hr] AVISO: impulse_preset '%s' no encontrado — usando default hardcodeado\n",
                   j->valuestring);
            list_impulse_presets();
            p->impulse_size = 0;
        }
    }

    // source/sink es "infinito" — no se expone al JSON
    p->source_sink_volume_m3 = 1.00e20;

    // ── Nodos ─────────────────────────────────────────────────
    cJSON* nodes_arr = cJSON_GetObjectItem(root, "nodes");
    g_hr_num_nodes = 0;

    if (nodes_arr && cJSON_IsArray(nodes_arr)) {
        int n_count = cJSON_GetArraySize(nodes_arr);
        if (n_count > HR_MAX_NODES) n_count = HR_MAX_NODES;

        for (int ni = 0; ni < n_count; ni++) {
            cJSON* nd = cJSON_GetArrayItem(nodes_arr, ni);
            hr_node_desc_t* d = &g_hr_desc[g_hr_num_nodes++];
            memset(d, 0, sizeof(*d));
            d->parent = -1;
            d->pipe_length_m      = -1.0;
            d->mic_position_ratio = -1.0;
            d->velocity_low_pass_hz = -1.0;

            j = cJSON_GetObjectItem(nd, "id");
            if(j) d->id = j->valueint;

            j = cJSON_GetObjectItem(nd, "type");
            if(j && j->valuestring) strncpy(d->type, j->valuestring, sizeof(d->type)-1);

            j = cJSON_GetObjectItem(nd, "piston_theta_r");
            if(j) d->piston_theta_r = j->valuedouble;

            j = cJSON_GetObjectItem(nd, "parent");
            if(j) d->parent = j->valueint;

            j = cJSON_GetObjectItem(nd, "bank");
            if(j) d->bank = j->valueint;

            j = cJSON_GetObjectItem(nd, "volume_m3");
            if(j) d->volume_m3 = j->valuedouble;

            j = cJSON_GetObjectItem(nd, "initial_pressure_pa");
            if(j) d->initial_pressure_pa = j->valuedouble;

            j = cJSON_GetObjectItem(nd, "initial_temp_k");
            if(j) d->initial_temp_k = j->valuedouble;

            // Campos acústicos per-nodo (solo para eplenum)
            j = cJSON_GetObjectItem(nd, "pipe_length_m");
            if(j) d->pipe_length_m = j->valuedouble;

            j = cJSON_GetObjectItem(nd, "mic_position_ratio");
            if(j) d->mic_position_ratio = j->valuedouble;

            j = cJSON_GetObjectItem(nd, "velocity_low_pass_cutoff_frequency_hz");
            if(j) d->velocity_low_pass_hz = j->valuedouble;

            cJSON* conns = cJSON_GetObjectItem(nd, "connections");
            if (conns && cJSON_IsArray(conns)) {
                int nc = cJSON_GetArraySize(conns);
                if (nc > HR_MAX_CONNECTIONS) nc = HR_MAX_CONNECTIONS;
                for (int ci = 0; ci < nc; ci++) {
                    cJSON* cv = cJSON_GetArrayItem(conns, ci);
                    if (cv) d->connections[d->num_connections++] = cv->valueint;
                }
            }
        }
    }

    cJSON_Delete(root);
    return true;
}

// ─────────────────────────────────────────────────────────────
// Detectar cambio de archivo (por tamaño + mtime simple)
// ─────────────────────────────────────────────────────────────
static bool
hr_file_changed(const char* filepath)
{
    FILE* f = fopen(filepath, "r");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fclose(f);
    if (sz != g_hr_last_filesize) {
        g_hr_last_filesize = sz;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
// Aplicar los parámetros y nodos reconstruidos al g_engine
// ─────────────────────────────────────────────────────────────
static void
hr_apply_to_engine(struct engine_s* e)
{
    hr_params_t* p = &g_hr_params;

    // Nombre (la cadena vive en g_hr_params.name)
    e->name = p->name;

    // Volumen de audio
    e->volume = p->sound_volume;

    // Cigüeñal
    e->crankshaft.mass_kg  = p->crank_mass_kg;
    e->crankshaft.radius_m = p->crank_radius_m;

    // Volante
    e->flywheel.mass_kg  = p->fly_mass_kg;
    e->flywheel.radius_m = p->fly_radius_m;

    // Limitador
    e->limiter.cutoff_angular_velocity_r_per_s  = p->limiter_cutoff_r_per_s;
    e->limiter.relaxed_angular_velocity_r_per_s = p->limiter_relaxed_r_per_s;

    // Starter
    e->starter.rated_torque_n_m                = p->starter_rated_torque_n_m;
    e->starter.no_load_angular_velocity_r_per_s= p->starter_no_load_r_per_s;
    e->starter.radius_m                        = p->starter_radius_m;

    // Nodos reconstruidos
    if (g_hr_num_nodes > 0) {
        e->node = g_hr_nodes;
        e->size = (size_t)g_hr_num_nodes;
    }

    // Impulso acústico: si se cargó un preset, activarlo globalmente
    // (g_active_impulse y g_active_impulse_size están en convo_filter_s.h)
    if (p->impulse_size > 0) {
        g_active_impulse      = p->impulse;
        g_active_impulse_size = p->impulse_size;
        printf("[hr] Impulso activo: %zu coeficientes (%.1f ms)\n",
               p->impulse_size, (double)p->impulse_size / 48000.0 * 1000.0);
    } else {
        // Restaurar default hardcodeado
        g_active_impulse      = g_convo_filter_impulse;
        g_active_impulse_size = g_convo_filter_impulse_size;
    }

    // Throttle presets (valores razonables fijos; se pueden exponer si se quiere)
    e->no_throttle   = 0.000;
    e->low_throttle  = 0.001;
    e->mid_throttle  = 0.050;
    e->high_throttle = 1.000;
    e->radial_spacing = 3.0;
}

// ─────────────────────────────────────────────────────────────
// API pública
// ─────────────────────────────────────────────────────────────

/*
 * hr_init() — llamar una vez al inicio, antes de reset_engine().
 * Carga el JSON, construye los nodos, aplica al engine.
 * Devuelve true si todo OK.
 */
static bool
hr_init(const char* filepath, struct engine_s* e)
{
    strncpy(g_hr_filepath, filepath, sizeof(g_hr_filepath)-1);

    if (!hr_parse_json(filepath)) return false;
    hr_build_nodes();
    hr_apply_to_engine(e);

    printf("[hr] Motor cargado: '%s'  (nodos: %d)\n",
           g_hr_params.name, g_hr_num_nodes);
    return true;
}

/*
 * hr_tick() — llamar cada frame (o cada N ms).
 * Si el JSON cambió: recarga params, reconstruye nodos, aplica al engine,
 * y llama a reset_engine() para re-inicializar el estado termodinámico.
 * Devuelve true si hubo recarga.
 *
 * IMPORTANTE: esta función llama reset_engine() internamente,
 * por lo que el motor se reinicia (RPM a 0, presiones a ambiente).
 * Si solo querés cambiar el volumen sin reiniciar, revisá la nota
 * en hotreload_volume_only() abajo.
 */
static bool
hr_tick(struct engine_s* e)
{
    if (!hr_file_changed(g_hr_filepath)) return false;

    // Pequeña pausa para dejar que el editor termine de escribir
    // (en Windows/WSL los writes no son atómicos)
    SDL_Delay(50);

    if (!hr_parse_json(g_hr_filepath)) {
        printf("[hr] JSON inválido, ignorando cambio\n");
        return false;
    }

    hr_build_nodes();
    hr_apply_to_engine(e);

    // Preservar el estado del starter/throttle que tenga el usuario
    // (no queremos que el motor se "apague" cada vez que editamos el JSON)
    bool was_starter = e->starter.is_on;
    bool was_ignite  = e->can_ignite;
    double was_throttle = e->throttle_open_ratio;

    reset_engine(e);

    e->starter.is_on       = was_starter;
    e->can_ignite          = was_ignite;
    e->throttle_open_ratio = was_throttle;

    printf("[hr] Recargado: '%s' | vol=%.3f | nodos=%d\n",
           g_hr_params.name, g_hr_params.sound_volume, g_hr_num_nodes);
    return true;
}

/*
 * hr_volume_only() — actualiza SOLO el volumen sin reiniciar el motor.
 * Útil si cambiás solo sound_volume y no querés perder las RPM actuales.
 * (el hr_tick normal también reinicia el motor completo)
 */
static void
hr_volume_only(struct engine_s* e, double vol)
{
    g_hr_params.sound_volume = vol;
    e->volume = vol;
    // g_current_volume se actualiza aquí también para synth_s.h
    extern double g_current_volume;
    g_current_volume = vol;
}

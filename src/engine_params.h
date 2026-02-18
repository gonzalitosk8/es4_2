// src/engine_params.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    // -----------------------------
    // Límites (para arrays fijos)
    // -----------------------------
#ifndef ENSIM4_MAX_CYLINDERS
#define ENSIM4_MAX_CYLINDERS 16
#endif

#ifndef ENSIM4_MAX_RPM_POINTS
#define ENSIM4_MAX_RPM_POINTS 64
#endif

#ifndef ENSIM4_MAX_BAND_POINTS
#define ENSIM4_MAX_BAND_POINTS 32
#endif

// -----------------------------
// Enums
// -----------------------------
    typedef enum {
        ENSIM4_LAYOUT_INLINE = 0,
        ENSIM4_LAYOUT_V = 1,
        ENSIM4_LAYOUT_BOXER = 2,
        ENSIM4_LAYOUT_W = 3,
        ENSIM4_LAYOUT_RADIAL = 4
    } ensim4_engine_layout_e;

    typedef enum {
        ENSIM4_CYCLE_OTTO = 0,
        ENSIM4_CYCLE_DIESEL = 1,
        ENSIM4_CYCLE_ATKINSON = 2,
        ENSIM4_CYCLE_MILLER = 3
    } ensim4_engine_cycle_e;

    typedef enum {
        ENSIM4_INTAKE_NA = 0,
        ENSIM4_INTAKE_TURBO = 1,
        ENSIM4_INTAKE_SUPERCHARGER = 2
    } ensim4_induction_e;

    typedef enum {
        ENSIM4_THROTTLE_BODY = 0,
        ENSIM4_THROTTLE_ITB = 1
    } ensim4_throttle_e;

    typedef enum {
        ENSIM4_FUEL_GASOLINE = 0,
        ENSIM4_FUEL_E85 = 1,
        ENSIM4_FUEL_METHANOL = 2,
        ENSIM4_FUEL_DIESEL = 3
    } ensim4_fuel_e;

    typedef enum {
        ENSIM4_IGNITION_WASTED_SPARK = 0,
        ENSIM4_IGNITION_COP = 1,   // coil-on-plug
        ENSIM4_IGNITION_DISTRIBUTOR = 2
    } ensim4_ignition_e;

    typedef enum {
        ENSIM4_FIRE_AUTO = 0,
        ENSIM4_FIRE_MANUAL_DEGREES = 1  // usa firing_deg[ ]
    } ensim4_firing_mode_e;

    typedef enum {
        ENSIM4_VALVE_PROFILE_SIMPLE = 0,   // rampa + plateau
        ENSIM4_VALVE_PROFILE_SMOOTH = 1    // más suave (curva)
    } ensim4_valve_profile_e;

    typedef enum {
        ENSIM4_EXHAUST_HEADER = 0,
        ENSIM4_EXHAUST_LOG = 1,
        ENSIM4_EXHAUST_TURBO_MANIFOLD = 2
    } ensim4_exhaust_e;

    // -----------------------------
    // Curva genérica (por RPM)
    // -----------------------------
    typedef struct {
        uint32_t count; // <= ENSIM4_MAX_RPM_POINTS
        float rpm[ENSIM4_MAX_RPM_POINTS];
        float value[ENSIM4_MAX_RPM_POINTS];
    } ensim4_curve_rpm_f32_t;

    // -----------------------------
    // Banda / EQ (para shaping del audio)
    // -----------------------------
    typedef struct {
        uint32_t count; // <= ENSIM4_MAX_BAND_POINTS
        float hz[ENSIM4_MAX_BAND_POINTS];
        float gain_db[ENSIM4_MAX_BAND_POINTS];
        float q[ENSIM4_MAX_BAND_POINTS];
    } ensim4_eq_bands_t;

    // -----------------------------
    // Parámetros de mezcla/audio de salida
    // -----------------------------
    typedef struct {
        // Output
        uint32_t sample_rate_hz;   // 48000 típico
        uint32_t channels;         // 1=mono, 2=stereo (si tu engine soporta)
        float master_gain;         // 0..inf (recomendado 0..2)
        float limiter_threshold;   // 0..1 (si aplicás limitador)
        float limiter_release_s;   // 0.01..1

        // Filtros globales (DC / Convolution)
        float dc_highpass_hz;      // ej 20..60
        float convo_mix;           // 0..1
        float convo_predelay_ms;   // 0..50

        // Shaping post (si lo hacés fuera del solver)
        ensim4_eq_bands_t eq;

        // Ruido/artefactos controlados
        float hiss_gain;           // 0..1
        float crackle_gain;        // 0..1
    } ensim4_audio_params_t;

    // -----------------------------
    // Geometría y masa/inercia mecánica
    // -----------------------------
    typedef struct {
        // Geometría básica
        float bore_m;                 // diámetro pistón
        float stroke_m;               // carrera
        float rod_length_m;           // biela
        float compression_ratio;      // CR
        float deck_height_m;          // opcional: para coherencia geométrica
        float piston_pin_offset_m;    // offset perno (si querés asimetría)

        // Masas
        float piston_mass_kg;
        float rod_mass_kg;
        float reciprocating_mass_kg;  // si preferís consolidar
        float crank_mass_kg;
        float crank_radius_m;         // radio efectivo para inercia (si tu modelo lo usa)
        float flywheel_mass_kg;
        float flywheel_radius_m;

        // Fricción (modelo simple)
        float static_friction_n_m_s_per_r;   // tu unidad actual (según el repo)
        float dynamic_friction_n_m_s_per_r;
        float bearing_friction_scale;        // 0..2
        float pumping_loss_scale;            // 0..2
    } ensim4_mech_params_t;

    // -----------------------------
    // Aire/termo/flujo
    // -----------------------------
    typedef struct {
        // Ambiente
        float ambient_pressure_pa;      // 101325
        float ambient_temperature_k;    // 293.15
        float ambient_humidity;         // 0..1 (si lo usás)
        float altitude_m;              // alternativa

        // Combustible / mezcla
        ensim4_fuel_e fuel;
        float afr_target;              // air-fuel ratio target (gasolina ~14.7)
        float lambda_target;           // alternativa
        float fuel_energy_j_per_kg;    // si querés parametrizar (o lo derivás del fuel)
        float stoich_afr;              // por fuel (o fijo)

        // Modelo de admisión
        ensim4_induction_e induction;
        float manifold_volume_m3;      // plenum (intake)
        float runner_volume_m3;        // runner
        float runner_nozzle_max_area_m2;
        float plenum_nozzle_max_area_m2;
        float intake_flow_loss_k;      // pérdidas
        float intake_temp_gain_k;      // calentamiento por compresión/cercanía

        // Modelo de escape
        ensim4_exhaust_e exhaust_type;
        float exhaust_volume_m3;
        float exhaust_nozzle_max_area_m2;
        float exhaust_flow_loss_k;

        // Turbo/supercharger (si induction != NA)
        float boost_target_kpa;        // objetivo (sobre atmósfera o absoluto, definilo en tu implementación)
        float wastegate_kpa;           // control
        float compressor_efficiency;   // 0..1
        float turbine_efficiency;      // 0..1
        float intercooler_efficiency;  // 0..1
        float turbo_inertia;           // escala (si modelás spool)
        float turbo_spool_rpm;         // escala
    } ensim4_flow_params_t;

    // -----------------------------
    // Encendido / chispa / corte
    // -----------------------------
    typedef struct {
        ensim4_ignition_e ignition;

        // RPM
        float idle_rpm;
        float redline_rpm;
        float limiter_rpm;          // corte duro/blando
        float limiter_hysteresis_rpm;
        float starter_rpm;          // velocidad objetivo de arranque (si aplica)

        // Avance encendido (por RPM y/o carga)
        // value: grados BTDC (positivo) / ATDC (negativo) según convención
        ensim4_curve_rpm_f32_t spark_advance_deg_by_rpm;

        // Energía chispa / duración
        float spark_energy_j;
        float spark_duration_ms;
        float spark_min_rpm;        // por debajo, deshabilitar
        float spark_max_rpm;        // por encima, si querés

        // Misfire / variación (para realismo)
        float misfire_base_prob;    // 0..1
        float misfire_prob_at_redline;
        float timing_jitter_deg;    // 0..N
    } ensim4_ignition_params_t;

    // -----------------------------
    // Válvulas (admisión/escape)
    // -----------------------------
    typedef struct {
        ensim4_valve_profile_e profile;

        // Ventanas (en grados de cigüeñal)
        float intake_open_deg;
        float intake_close_deg;
        float exhaust_open_deg;
        float exhaust_close_deg;

        // Rampas y lift (si tu modelo lo traduce a nozzle_open_ratio)
        float intake_ramp_deg;
        float exhaust_ramp_deg;
        float intake_max_lift_m;
        float exhaust_max_lift_m;

        // Area efectiva máxima (si querés bypass al lift)
        float intake_max_flow_area_m2;
        float exhaust_max_flow_area_m2;

        // VVT (si querés animarlo en tiempo real)
        float vvt_intake_phase_deg;   // -N..+N
        float vvt_exhaust_phase_deg;
        float vvt_intake_max_deg;
        float vvt_exhaust_max_deg;
    } ensim4_valve_params_t;

    // -----------------------------
    // Throttle / carga
    // -----------------------------
    typedef struct {
        ensim4_throttle_e throttle_type;

        // Entrada principal
        float throttle_open_ratio;        // 0..1 (tiempo real)
        float throttle_response_s;        // suavizado (0 = directo)

        // Modelo de placa / ITB
        float throttle_body_area_m2;      // TB
        float itb_area_m2;               // ITB por cilindro (si aplica)
        float idle_air_bypass_area_m2;    // IAC

        // Control de ralentí
        float idle_controller_kp;
        float idle_controller_ki;
        float idle_controller_max;        // clamp

        // Carga/torque shaping (opcional)
        ensim4_curve_rpm_f32_t ve_by_rpm; // volumetric efficiency 0..1.2
    } ensim4_throttle_params_t;

    // -----------------------------
    // Firing order (dinámico sin recompilar)
    // -----------------------------
    typedef struct {
        ensim4_firing_mode_e mode;

        // Si mode == ENSIM4_FIRE_MANUAL_DEGREES:
        // ángulo de ignición por evento (0..720 en 4T), tamaño = cylinders
        float firing_deg[ENSIM4_MAX_CYLINDERS];

        // Alternativa: orden por índice de cilindro (1..cylinders)
        // Si preferís esta representación en UI:
        uint8_t firing_order[ENSIM4_MAX_CYLINDERS];

        // Phase global (para alinear)
        float global_phase_deg;
    } ensim4_firing_params_t;

    // -----------------------------
    // Parámetros del solver/estabilidad/CPU
    // -----------------------------
    typedef struct {
        // Paso temporal
        float dt_s;                  // si el solver depende (si no, lo derivás del refresh)
        uint32_t monitor_refresh_hz;  // el famoso ENSIM4_MONITOR_REFRESH_RATE_HZ
        uint32_t substeps;            // 1..N (calidad vs CPU)

        // Damping / estabilidad de ondas
        float gas_momentum_damping_time_constant_s;
        float wave_reflection_loss;        // 0..1
        float wave_numerical_viscosity;    // 0..1

        // Tamaños internos (si migrás a dinámico; si son fijos, esto sirve para validar)
        uint32_t wave_cells;
        uint32_t max_waves;
    } ensim4_solver_params_t;

    // -----------------------------
    // Paquete completo
    // -----------------------------
    typedef struct {
        // Identidad/estructura
        uint32_t cylinders;                 // 1..ENSIM4_MAX_CYLINDERS
        ensim4_engine_layout_e layout;      // inline/V/boxer/...
        ensim4_engine_cycle_e cycle;        // otto/diesel/...

        // Bancos (para V/Boxer/W/ Radial)
        uint32_t banks;                     // 1 para inline, 2 para V/boxer, etc.
        float bank_angle_deg;               // V-angle
        float crankpin_offset_deg;          // para V8 crossplane/flatplane, etc.

        // Espaciado (para radial/visual o para pequeñas variaciones de fase)
        float radial_spacing;               // escala o deg
        float cylinder_phase_offset_deg[ENSIM4_MAX_CYLINDERS]; // opcional: micro-ajustes

        // Sub-sistemas
        ensim4_mech_params_t mech;
        ensim4_flow_params_t flow;
        ensim4_valve_params_t valves;
        ensim4_ignition_params_t ignition;
        ensim4_throttle_params_t throttle;
        ensim4_firing_params_t firing;
        ensim4_solver_params_t solver;
        ensim4_audio_params_t audio;

        // Flags
        uint32_t enable_waves;              // 0/1
        uint32_t enable_convolution;        // 0/1
        uint32_t enable_turbo;              // 0/1 (si induction==turbo)
        uint32_t enable_idle_control;        // 0/1
        uint32_t enable_misfire;            // 0/1
        uint32_t enable_debug_panels;        // 0/1
    } ensim4_engine_params_t;

#ifdef __cplusplus
}
#endif

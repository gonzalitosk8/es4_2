#include <stdlib.h>
#include <string.h>
#include <math.h>

// Asumimos 255 como el marcador de "no hay más conexiones"
constexpr uint8_t END_OF_LINKS = 255;

static void
link_node(struct node_s* nodes, size_t from_idx, size_t to_idx)
{
    for (size_t i = 0; i < g_nodes_node_children; i++)
    {
        if (nodes[from_idx].next[i] == END_OF_LINKS)
        {
            nodes[from_idx].next[i] = (uint8_t)to_idx;
            return;
        }
    }
    // Si llegás acá, te quedaste sin slots en g_nodes_node_children (16)
    g_panic_message = "Node children limit exceeded";
}

struct node_s*
    build_dynamic_engine_nodes(const ensim4_engine_params_t* params, size_t* out_node_count)
{
    // 1. Calcular tamaño (1 Source + 1 Throttle + 1 iplenum + N*(runner, inj, piston, erunner) + 1 eplenum + 1 exhaust + 1 sink)
    size_t n = params->cylinders;
    size_t total_nodes = 6 + (n * 4);

    struct node_s* nodes = (struct node_s*)malloc(total_nodes * sizeof(struct node_s));

    // Inicializar memoria a 0, pero forzar los arrays "next" a END_OF_LINKS
    memset(nodes, 0, total_nodes * sizeof(struct node_s));
    for (size_t i = 0; i < total_nodes; i++) {
        memset(nodes[i].next, END_OF_LINKS, g_nodes_node_children);
    }

    // 2. Asignar Índices
    size_t current = 0;
    size_t idx_source = current++;
    size_t idx_throttle = current++;
    size_t idx_iplenum = current++; // Centralizamos el aire aquí antes de los runners

    size_t idx_eplenum = total_nodes - 3;
    size_t idx_exhaust = total_nodes - 2;
    size_t idx_sink = total_nodes - 1;

    // 3. Configurar Nodos Globales (Ajustar volúmenes según tu lógica de normalización)
    nodes[idx_source].type = g_is_source;

    nodes[idx_throttle].type = g_is_throttle;
    nodes[idx_throttle].as.chamber.volume_m3 = 2.0e-4; // O tomarlo de params->flow
    nodes[idx_throttle].as.chamber.nozzle_max_flow_area_m2 = params->throttle.throttle_body_area_m2;

    nodes[idx_iplenum].type = g_is_iplenum;
    nodes[idx_iplenum].as.chamber.volume_m3 = params->flow.manifold_volume_m3;

    nodes[idx_eplenum].type = g_is_eplenum;
    nodes[idx_eplenum].as.chamber.volume_m3 = params->flow.exhaust_volume_m3;
    nodes[idx_eplenum].as.chamber.nozzle_max_flow_area_m2 = params->flow.exhaust_nozzle_max_area_m2;
    // Ojo: Acá tendrías que inicializar el wave_index si params->enable_waves es true

    nodes[idx_exhaust].type = g_is_exhaust;
    nodes[idx_sink].type = g_is_sink;

    // 4. Conexiones Globales
    link_node(nodes, idx_source, idx_throttle);
    link_node(nodes, idx_throttle, idx_iplenum);
    link_node(nodes, idx_eplenum, idx_exhaust);
    link_node(nodes, idx_exhaust, idx_sink);

    // 5. Construcción y Firing Order Dinámico de Cilindros
    for (size_t i = 0; i < n; i++)
    {
        size_t idx_irunner = current++;
        size_t idx_injector = current++;
        size_t idx_piston = current++;
        size_t idx_erunner = current++;

        // I-Runner
        nodes[idx_irunner].type = g_is_irunner;
        nodes[idx_irunner].as.irunner.chamber.volume_m3 = params->flow.runner_volume_m3;
        nodes[idx_irunner].as.irunner.chamber.nozzle_max_flow_area_m2 = params->flow.runner_nozzle_max_area_m2;
        nodes[idx_irunner].as.irunner.valve.engage_r = params->valves.intake_open_deg * (g_std_pi_r / 180.0);
        nodes[idx_irunner].as.irunner.valve.ramp_r = params->valves.intake_ramp_deg * (g_std_pi_r / 180.0);

        // Injector
        nodes[idx_injector].type = g_is_injector;
        // Inyector es un "reservoir", normalizar después

        // Piston (Aplicamos termodinámica geométrica)
        nodes[idx_piston].type = g_is_piston;
        struct piston_s* p = &nodes[idx_piston].as.piston;
        p->diameter_m = params->mech.bore_m;
        p->crank_throw_length_m = params->mech.stroke_m / 2.0;
        p->connecting_rod_length_m = params->mech.rod_length_m;
        p->connecting_rod_mass_kg = params->mech.rod_mass_kg;
        p->head_compression_height_m = 0.025; // Opcional: Extraer a params
        p->head_clearance_height_m = (params->mech.stroke_m) / (params->mech.compression_ratio - 1.0);
        p->dynamic_friction_n_m_s_per_r = params->mech.dynamic_friction_n_m_s_per_r;
        p->static_friction_n_m_s_per_r = params->mech.static_friction_n_m_s_per_r;

        // Asignación de fase dinámica (modo manual vs auto)
        if (params->firing.mode == ENSIM4_FIRE_MANUAL_DEGREES) {
            p->theta_r = params->firing.firing_deg[i] * (g_std_pi_r / 180.0);
        }
        else {
            // Lógica auto fallback
            p->theta_r = (720.0 / n * i) * (g_std_pi_r / 180.0);
        }

        // E-Runner
        nodes[idx_erunner].type = g_is_erunner;
        nodes[idx_erunner].as.chamber.volume_m3 = params->flow.exhaust_volume_m3 / n; // Aproximación simple

        // 6. Cableado del Cilindro
        link_node(nodes, idx_iplenum, idx_irunner);
        link_node(nodes, idx_irunner, idx_piston);
        link_node(nodes, idx_injector, idx_piston);
        link_node(nodes, idx_piston, idx_erunner);
        link_node(nodes, idx_erunner, idx_eplenum);
    }

    // Retorno de datos
    *out_node_count = total_nodes;
    return nodes;
}
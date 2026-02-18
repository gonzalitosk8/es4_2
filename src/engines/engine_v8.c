#include "engines.h"
#include "node_s.h" // tu node real

static struct node_s g_v8_node[] = {

    [0] = source_d({1}),
    [1] = throttle_d({2, 6, 10, 14, 18, 22, 26, 30}),

    [2] = irunner_d(g_engine_piston_0_theta_r, {4}),
    [3] = injector_d(2, {4}),
    [4] = piston_d(g_engine_piston_0_theta_r, {5}),
    [5] = erunner_d({34}),

    [6] = irunner_d(g_engine_piston_1_theta_r, {8}),
    [7] = injector_d(6, {8}),
    [8] = piston_d(g_engine_piston_1_theta_r, {9}),
    [9] = erunner_d({36}),

    [10] = irunner_d(g_engine_piston_2_theta_r, {12}),
    [11] = injector_d(10, {12}),
    [12] = piston_d(g_engine_piston_2_theta_r, {13}),
    [13] = erunner_d({34}),

    [14] = irunner_d(g_engine_piston_3_theta_r, {16}),
    [15] = injector_d(14, {16}),
    [16] = piston_d(g_engine_piston_3_theta_r, {17}),
    [17] = erunner_d({36}),

    [18] = irunner_d(g_engine_piston_4_theta_r, {20}),
    [19] = injector_d(18, {20}),
    [20] = piston_d(g_engine_piston_4_theta_r, {21}),
    [21] = erunner_d({34}),

    [22] = irunner_d(g_engine_piston_5_theta_r, {24}),
    [23] = injector_d(22, {24}),
    [24] = piston_d(g_engine_piston_5_theta_r, {25}),
    [25] = erunner_d({36}),

    [26] = irunner_d(g_engine_piston_6_theta_r, {28}),
    [27] = injector_d(26, {28}),
    [28] = piston_d(g_engine_piston_6_theta_r, {29}),
    [29] = erunner_d({34}),

    [30] = irunner_d(g_engine_piston_7_theta_r, {32}),
    [31] = injector_d(30, {32}),
    [32] = piston_d(g_engine_piston_7_theta_r, {33}),
    [33] = erunner_d({36}),

    [34] = eplenum_d(0, {35}),
    [35] = exhaust_d({38}),

    [36] = eplenum_d(1, {37}),
    [37] = exhaust_d({38}),

    [38] = sink_d(),
};


static const engine_blueprint_t g_v8_bp = {
    .name = "V8",
    .nodes = g_v8_nodes,
    .node_count = sizeof(g_v8_nodes)/sizeof(g_v8_nodes[0]),
};

const engine_blueprint_t* engine_v8_bp(void) { return &g_v8_bp; }

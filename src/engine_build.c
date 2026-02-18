#include <stdlib.h>
#include <string.h>
#include "engine_blueprint.h"
#include "engine_s.h"  // tu engine_s real

int engine_build_from_blueprint(engine_s* e, const engine_blueprint_t* bp) {
    if (!e || !bp || !bp->nodes || bp->node_count == 0) return 0;

    // si ya había uno, lo destruís
    engine_destroy(e);

    e->node = (node_s*)malloc(sizeof(node_s) * bp->node_count);
    if (!e->node) return 0;

    memcpy(e->node, bp->nodes, sizeof(node_s) * bp->node_count);
    e->node_count = bp->node_count;

    // IMPORTANTE:
    // si tus nodos referencian "next[]" por índice, esto funciona directo
    // mientras el blueprint haya sido armado con índices absolutos.
    // Si hoy dependés de macros/offsets, acá se normaliza.

    // reset/clear de estado del motor si aplica:
    // engine_reset(e);
    return 1;
}

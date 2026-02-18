#pragma once
#include "engine_blueprint.h"
#include "node_s.h"

typedef struct {
  struct node_s* nodes;
  unsigned node_count;
} engine_graph_t;

engine_graph_t engine_build_graph_from_blueprint(const engine_blueprint_t* bp);
void engine_free_graph(engine_graph_t* g);

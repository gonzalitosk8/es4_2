#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct node_s node_s; // tu struct real

typedef struct {
    const char* name;
    const node_s* nodes;      // template "readonly"
    size_t node_count;
} engine_blueprint_t;

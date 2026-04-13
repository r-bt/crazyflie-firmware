#include "vec.h"
#include <math.h>

vec3_t vec3_add(vec3_t a, vec3_t b) {
    return (vec3_t){ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
}

vec3_t vec3_sub(vec3_t a, vec3_t b) {
    return (vec3_t){ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
}

vec3_t vec3_scale(vec3_t v, float s) {
    return (vec3_t){ .x = v.x * s, .y = v.y * s, .z = v.z * s };
}

float vec3_dot(vec3_t a, vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3_t vec3_cross(vec3_t a, vec3_t b) {
    return (vec3_t){
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };
}

float vec3_norm(vec3_t v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3_t vec3_normalize(vec3_t v) {
    return vec3_scale(v, 1.0f / vec3_norm(v));
}
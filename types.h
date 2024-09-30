#ifndef TYPES_H
#define TYPES_H

// Quaternion
typedef struct {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} quat_t;

// 3D Vector with timestamp
typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} vec_t;

// Attitude
typedef struct {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} attitude_t;

// GPS
typedef struct {
    double lat = 0.0f;
    double lon = 0.0f;
    double alt = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} gps_t;

// PID
typedef struct {
    attitude_t p;
    attitude_t i;
    attitude_t d;
    attitude_t iPrev;
    attitude_t pPrev;
    attitude_t dPrev;
    attitude_t out;
    attitude_t outPrev;
} PID_t;

// Barometer
typedef struct {
    float pressure = 0.0f;
    float temperature = 0.0f;
    float altitude = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} bar_t;

#endif
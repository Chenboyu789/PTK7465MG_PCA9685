#pragma once

/* Quintic time scaling, normalized input in [0, 1]. */
static inline double trajectory_quintic(double t)
{
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

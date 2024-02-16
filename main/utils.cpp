#include "utils.h"

/// @brief Maps a value from one range to another
/// @param x Variable to map
/// @param in_min Original minimum value
/// @param in_max Original maximum value
/// @param out_min Output minimum value
/// @param out_max Output maximum value
/// @return Maped value
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
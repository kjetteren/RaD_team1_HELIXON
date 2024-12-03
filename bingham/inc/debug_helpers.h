#ifndef DEBUG_HELPERS_H
#define DEBUG_HELPERS_H
#include <bingham.h>

/***
 * input validation before bingham distribution multiplication
 */
void validate_bingham_params(const bingham_t* B, const char* label);

/***
 * numerical stability checks after bingham distribution multiplication
 */
void check_numerical_stability(const bingham_t* B, const char* label);

/***
 * debugging output
 */
void print_bingham_params(const bingham_t* B, const char* label);

#endif //DEBUG_HELPERS_H

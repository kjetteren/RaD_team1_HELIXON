#include "../inc/debug_helpers.h"

#include <bingham.h>
#include <stdio.h>
#include <tgmath.h>

void validate_bingham_params(const bingham_t* B, const char* label) {
    printf("\n\t\tValidate Bingham %s parameters...\n", label);
    // check Z parameters
    for(int i = 0; i < B->d-1; i++) {
        if (B->Z[i] >= 0) {
            printf("\t\t\tFUCKUP: Concentration parameter Z[%d]=%.3f >= 0\n", i, B->Z[i]);
        }
    }

    // check V matrix orthogonality
    for(int i = 0; i < B->d-1; i++) {
        double norm = 0;
        for(int j = 0; j < B->d; j++) {
            norm += B->V[i][j] * B->V[i][j];
        }
        if (fabs(norm - 1.0) > 1e-6) {
            printf("\t\t\tFUCKUP: V[%d] is not unit length\n", i);
        }
    }
    printf("\t\t...Validated Bingham %s parameters.\n", label);

}

void check_numerical_stability(const bingham_t* B, const char* label) {
    printf("\n\t\tCheck numerical stability %s parameters...\n", label);
    for(int i = 0; i < B->d-1; i++) {
        if (fabs(B->Z[i]) > 1000.0) {
            printf("\t\t\tFUCKUP: Large concentration parameter %d\n", i);
        }
    }
    printf("\t\t...Checked numerical stability %s parameters.\n\n", label);
}

void print_bingham_params(const bingham_t* B, const char* label) {
    printf("\n\t\tBingham %s parameters:\n", label);
    printf("\t\t\tConcentration params (Z):\n\t\t\t ");
    for(int i = 0; i < B->d-1; i++)
        printf("%.6f ", B->Z[i]);
    printf("\n\t\t");

    printf("\tPrincipal axes (V):\n");
    for(int i = 0; i < B->d-1; i++) {
        printf("\t\t\t");
        for(int j = 0; j < B->d; j++) {
            printf("%.6f ", B->V[i][j]);
        }
        printf("\n");
    }
}


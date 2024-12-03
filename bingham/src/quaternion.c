#include "../inc/quaternion.h"

#include <stdio.h>
#include <stdlib.h>
#include <tgmath.h>

double* quaternion_conjugate(const double* q) {
    double* conjugate = malloc(sizeof(double) * 4);
    conjugate[0] = q[0];
    conjugate[1] = -q[1];
    conjugate[2] = -q[2];
    conjugate[3] = -q[3];

    return conjugate;
}

void quaternion_multiply(double *product, const double *p, const double *q) {
    product[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    product[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    product[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    product[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
}

void normalize_quaternion(double *q) {
    const double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm < 1e-10) {  // Avoid division by zero
        return;
    }
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}

void attitude_propagation(double *q, const double *omega, const double dt) {
    const double q_omega[4] = {0, omega[0], omega[1], omega[2]};
    normalize_quaternion(q_omega);
    double* q_dot = malloc(4 * sizeof(double));
    quaternion_multiply(q_dot, q, q_omega);

    for(int i = 0; i < 4; i++) {
        q[i] += 0.5 * q_dot[i] * dt;
    }

    free(q_dot);
    normalize_quaternion(q);
}

void compute_orientation_quaternion(const double *acc, const double *mag, double *q) {
    // normalise measurement vectors
    const double acc_norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    const double mag_norm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);

    double acc_normalised[3] = {acc[0]/acc_norm, acc[1]/acc_norm, acc[2]/acc_norm};
    double mag_normalised[3] = {mag[0]/mag_norm, mag[1]/mag_norm, mag[2]/mag_norm};

    // compute angles from measurements
    double roll = atan2(acc_normalised[1], acc_normalised[2]);
    double pitch = -asin(acc_normalised[0]);
    double yaw = atan2(mag_normalised[1]*cos(roll) - mag_normalised[2]*sin(roll),
        mag_normalised[0]*cos(pitch) + mag_normalised[1]*sin(pitch)*sin(roll) + mag_normalised[2]*sin(pitch)*cos(roll));

    // convert the angles into a quaternion
    double cr = cos(roll/2);
    double sr = sin(roll/2);
    double cp = cos(pitch/2);
    double sp = sin(pitch/2);
    double cy = cos(yaw/2);
    double sy = sin(yaw/2);

    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;

    // normalise quaternion
    normalize_quaternion(q);
}

void compute_orientation_matrix(const double* q, double** matrix) {
    for (int i = 0; i < 3; i++) {
        matrix[i] = malloc(4 * sizeof(double));
    }
    // first row is the measured quaternion
    normalize_quaternion(q);
    matrix[0][0] = q[0];
    matrix[0][1] = q[1];
    matrix[0][2] = q[2];
    matrix[0][3] = q[3];

    // second row is constructed to have unit length
    int max_idx = 0;
    double max_val = fabs(q[0]);
    for(int i = 1; i < 4; i++) {
        if(fabs(q[i]) > max_val) {
            max_idx = i;
            max_val = fabs(q[i]);
        }
    }
    for(int i = 0; i < 4; i++) { // create orthogonal vector
        matrix[1][i] = (i == max_idx) ? -q[(i+1)%4] : q[max_idx];
    }

    // third row is the cross product of previous rows
    matrix[2][0] = matrix[0][1]*matrix[1][2] - matrix[0][2]*matrix[1][1];
    matrix[2][1] = matrix[0][2]*matrix[1][0] - matrix[0][0]*matrix[1][2];
    matrix[2][2] = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
    matrix[2][3] = 0;
}
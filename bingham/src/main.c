#define _XOPEN_SOURCE 700  // Must be defined before any includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/limits.h>
#include <unistd.h>
#include <time.h>

#include <bingham.h>
#include <tgmath.h>

#include "../inc/quaternion.h"
#include "../inc/imu_data.h"
#include "../inc/debug_helpers.h"


typedef struct {
    bingham_t* filter;
    double* quaternion;
} bingham_filter_t;

bingham_filter_t* init_bingham_filter() {
    bingham_init();

    bingham_filter_t* bf = malloc(sizeof(bingham_filter_t));
    if (!bf) return NULL;
    bingham_t* filter = malloc(sizeof(bingham_t));
    if (!filter) return NULL;

    int d = 4;


    double *raw, **V;
    safe_calloc(raw, (d-1)*d, double);
    safe_malloc(V,d-1,double*);
    for (int i = 0; i < d-1; i++) {
        V[i] = raw + i*d;
    }

    double *Z = malloc(3*sizeof(double));
    Z[0] = -20.0;
    Z[1] = -20.0;
    Z[2] = -20.0;

    bingham_new(filter, d, V, Z);
    bf->filter = filter;

    double *q = malloc(4 * sizeof(double));
    q[0] = 0;
    q[1] = 0;
    q[2] = 0;
    q[3] = 1;

    bf->quaternion = q;
    return bf;
}

void most_likely_orientation(double* q, const bingham_t* distribution) {
    printf("\t\tConcentration parameters Z -> ");
    for (int i = 0; i < distribution->d-1; i++) {
        printf(" %f", distribution->Z[i]);
    }
    printf("\n");

    double max_value = -INFINITY;
    int max_index = 0;
    for (int i = 0; i < distribution->d-1; i++) {
        if (distribution->Z[i] > max_value) {
            max_value = distribution->Z[i];
            max_index = i;
        }
    }
    printf("\t\tSelected index: %d -> ", max_index);
    for (int i = 0; i < distribution->d-1; i++) {
        q[i] = distribution->V[max_index][i];
        printf("%f, ", distribution->V[max_index][i]);
    }
    printf("\n");
}

void predict_step(const bingham_filter_t* bf, const imu_data_t* imu, const double dt) {
    printf("\t\tUpdate concentration parameters Z -> ");
    for (int i = 0; i < 3; i++) {
        // if we use dt as difference between timestamps, the Z decay too quickly
        // so i divide dt by sampling rate
        bf->filter->Z[i] *= exp(-dt/100); // updates concentration parameters
        printf("%f, ", bf->filter->Z[i]);
    }
    printf("\n");
    printf("\t\tConcentration parameters Z with noise -> ");
    const double process_noise = 0.1;  // adjust based on your system
    for (int i = 0; i < 3; i++) {
        if (fabs(bf->filter->Z[i]) > process_noise) {
            bf->filter->Z[i] = -process_noise;
            printf("%f, ", bf->filter->Z[i]);
        }
    }
    printf("\n");

    int d = 4;

    bingham_t prediction; // creates prediction distribution for prediction step
    bingham_alloc(&prediction, d);

    const double omega[3] = {imu->gyro[0], imu->gyro[1], imu->gyro[2]};
    printf("\t\tQuaternion before predict: %.4f, %.4f, %.4f, %.4f\n",
        bf->quaternion[0], bf->quaternion[1], bf->quaternion[2], bf->quaternion[3]);
    attitude_propagation(bf->quaternion, omega, dt); // converts gyroscope measurement to quaternion rate
    printf("\t\tQuaternion after predict: %.4f, %.4f, %.4f, %.4f\n",
        bf->quaternion[0], bf->quaternion[1], bf->quaternion[2], bf->quaternion[3]);

    double *raw, **V;
    safe_calloc(raw, (d-1)*d, double);
    safe_malloc(V,d-1,double*);
    for (int i = 0; i < d-1; i++) {
        V[i] = raw + i*d;
    }
    V[0][0] = V[1][1] = V[2][2] = 1;

    bingham_new(&prediction, d, V, bf->filter->Z);
    print_bingham_params(&prediction, "for prediction step");

    validate_bingham_params(&prediction, "for prediction step");
    bingham_mult(bf->filter, bf->filter, &prediction); // multiplies distributions
    check_numerical_stability(bf->filter, "for prediction step");

    bingham_mode(bf->quaternion, bf->filter); // extracts orientation from Bingham distribution
    printf("\t\tPredicted orientation:\t[%f, %f, %f, %f]\n",
        bf->quaternion[0], bf->quaternion[1], bf->quaternion[2], bf->quaternion[3]);

    bingham_free(&prediction);
    free(V[0]);
    free(V);
}

void update_step(const bingham_filter_t* bf, const imu_data_t* imu) {
    bingham_t measurement; // creates measurement distribution
    bingham_alloc(&measurement, 4);

    double* q_measurement = malloc( 4 * sizeof(double));
    q_measurement[0] = imu->q[0];
    q_measurement[1] = imu->q[1];
    q_measurement[2] = imu->q[2];
    q_measurement[3] = imu->q[3];
    printf("\t\tMeasured Quaternion = %.3f, %.3f, %.3f, %.3f\n",
               q_measurement[0], q_measurement[1],
               q_measurement[2], q_measurement[3]);

    double Z[3] = {-100.0, -100.0, -100.0}; // lower concentration parameters for measurement
    double** V = malloc(3 * sizeof(double*));
    compute_orientation_matrix(q_measurement, V);
    bingham_new(&measurement, 4, V, Z); // creates distribution
    print_bingham_params(&measurement, "for measurement step");

    validate_bingham_params(&measurement, "for update step");
    bingham_mult(bf->filter, bf->filter, &measurement); // multiplies distributions
    check_numerical_stability(bf->filter, "for measurement step");


    bingham_mode(bf->quaternion, bf->filter); // extracts orientation
    printf("\t\tMost likely quaternion = %.3f, %.3f, %.3f, %.3f\n",
                   bf->quaternion[0], bf->quaternion[1],
                   bf->quaternion[2], bf->quaternion[3]);
    normalize_quaternion(bf->quaternion);
    printf("\t\tNormalised quaternion = %.3f, %.3f, %.3f, %.3f\n",
                   bf->quaternion[0], bf->quaternion[1],
                   bf->quaternion[2], bf->quaternion[3]);
    bingham_free(&measurement);
    free(V[0]);
    free(V);
}



int main() {

    printf("===========================================================\n\n");
    // declares variable to store number of measurements
    int num_measurements;

    // read IMU data
    char* filename = "../../utils/head_rotation_data/sensor_data_noddings_b.csv";
    printf("Attempting to read file: %s\n", filename);
    printf("Preparing imu_data array...");
    //TODO: problem is that we need to know how many measurements we have to alocate enough memory
    //otherwise segmentation fault
    imu_data_t* imu_data = malloc(320*sizeof(imu_data_t));
    printf("\t... allocated memory for imu_data array.\n");
    read_imu_csv(filename, &num_measurements, imu_data);
    printf("Successfully read %d measurements\n", num_measurements);

    // check if data was read successfully
    if (imu_data == NULL) {
        fprintf(stderr, "Error reading file: %s\n", filename);
        return EXIT_FAILURE;
    }

    //print_imu_data(imu_data, 2);
    printf("============================================================\n\n");
    /*printf("First measurement:\n");
    printf("\tTime: %.3f\n", imu_data[0].timestamp);
    printf("\tAcc: [%.3f, %.3f, %.3f]\n",
           imu_data[0].acc[0], imu_data[0].acc[1], imu_data[0].acc[2]);
    printf("\tMag: [%.3f, %.3f, %.3f]\n",
          imu_data[0].mag[0], imu_data[0].mag[1], imu_data[0].mag[2]);
    printf("\tOmega: [%.3f, %.3f, %.3f]\n",
          imu_data[0].omega[0], imu_data[0].omega[1], imu_data[0].omega[2]);*/


    printf("Initialising Bingham filter...\t");
    bingham_filter_t *bf = init_bingham_filter();
    printf("initialised the filter.\n");
    printf("============================================================\n\n");

    double position[3] = {0,0,0};
    double velocity[3] = {0,0,0};

    // for (int i = 0; i < num_measurements; i++) {
    for (int i = 0; i < num_measurements; i++) {
        printf("Processing measurement %d/%d\n", i+1, num_measurements);
        const char* output_file = "../bingham_filter_results.csv";
        printf("\ttime stamp: %.f\t",imu_data[i].timestamp);
        double dt = (i>0) ? imu_data[i].timestamp-imu_data[i-1].timestamp : 0.1;
        printf("\tdt: %.3f\n", dt);

        printf("\t===========================================================\n");
        printf("\tCalling predict step...\n");
        predict_step(bf, &imu_data[i], dt);
        printf("\t...predict step completed.\n");
        printf("\t===========================================================\n");
        printf("\tCalling update_step...\n");
        update_step(bf, &imu_data[i]);
        printf("\t...update step completed\n");
        printf("\t===========================================================\n");

        printf("\tCURRENT QUATERNION: [%.3f, %.3f, %.3f, %.3f]\n",
               bf->quaternion[0], bf->quaternion[1],
               bf->quaternion[2], bf->quaternion[3]);

        const double gravity[3] = {0, 0, -9.81};

        // rotate acceleration to global frame
        double acc_quaternion[4] = {0, imu_data[i].accel[0], imu_data[i].accel[1], imu_data[i].accel[2]};
        quaternion_conjugate(bf->quaternion);
        double temp[4];
        quaternion_multiply(&temp[0], bf->quaternion, acc_quaternion);
        double rotated[4];
        quaternion_multiply(rotated, &temp[0], quaternion_conjugate(bf->quaternion));

        double acc_global[3] = {rotated[1], rotated[2], rotated[3]};

        acc_global[0] -= gravity[0];
        acc_global[1] -= gravity[1];
        acc_global[2] -= gravity[2];

        for(int j = 0; j < 3; j++) { // updates velocity and position
            velocity[j] += acc_global[j] * dt;
            position[j] += velocity[j] * dt;
        }

        save_to_csv(output_file, position, velocity, bf->quaternion);

        printf("============================================================\n\n");

    }

    free(imu_data);
    //bingham_free(bf->filter);
    free(bf->quaternion);
    free(bf);

    printf("===========================================\n\n");

    return EXIT_SUCCESS;
}

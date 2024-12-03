#include "../inc/imu_data.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tgmath.h>

#define MAX_LINE_LENGTH 1024

void read_imu_csv(const char* filename, int* num_measurements, imu_data_t* imu_data) {
    FILE* fp = fopen(filename, "r");
    if (fp == NULL) {
        fprintf(stderr,"Error opening file %s\n", filename);
        *num_measurements = 0;
        return;
    }

    char line[MAX_LINE_LENGTH];
    int count = 0;

    if (fgets(line, MAX_LINE_LENGTH, fp) == NULL) {
        fprintf(stderr, "Error reading header line\n");
        fclose(fp);
        *num_measurements = 0;
        return;
    }

    while(fgets(line, MAX_LINE_LENGTH, fp)) {
        char* token = strtok(line, ",");
        int field = 0;

        while(token != NULL && field < 19) {
            float value = atof(token);

            switch(field) {
                case 0: imu_data[count].accel[0] = value; break;
                case 1: imu_data[count].accel[1] = value; break;
                case 2: imu_data[count].accel[2] = value; break;
                case 3: imu_data[count].magneto[0] = value; break;
                case 4: imu_data[count].magneto[1] = value; break;
                case 5: imu_data[count].magneto[2] = value; break;
                case 6: imu_data[count].gyro[0] = value; break;
                case 7: imu_data[count].gyro[1] = value; break;
                case 8: imu_data[count].gyro[2] = value; break;
                case 9: imu_data[count].gravity[0] = value; break;
                case 10: imu_data[count].gravity[1] = value; break;
                case 11: imu_data[count].gravity[2] = value; break;
                case 12: imu_data[count].q[0] = value; break;
                case 13: imu_data[count].q[1] = value; break;
                case 14: imu_data[count].q[2] = value; break;
                case 15: imu_data[count].q[3] = value; break;
                case 16: imu_data[count].pressure = value; break;
                case 17: imu_data[count].temperature = value; break;
                case 18: imu_data[count].timestamp = value; break;
                default: break;
            }
            token = strtok(NULL, ",\n");
            field++;
        }
        if (field == 19) { // Only increment if we got all fields
            count++;
        }
    }
    *num_measurements = count;
    fclose(fp);
}

void print_imu_data(const imu_data_t* imu_data, const int num_measurements) {
    for (int i = 0; i < num_measurements; i++) {
        printf("Measurement %d:\n", i);
        printf("\tTime: %.3f\n", imu_data[i].timestamp);
        printf("\tAcc: [%.3f, %.3f, %.3f]\n",
               imu_data[i].accel[0], imu_data[i].accel[1], imu_data[i].accel[2]);
        printf("\tMag: [%.3f, %.3f, %.3f]\n",
               imu_data[i].magneto[0], imu_data[i].magneto[1], imu_data[i].magneto[2]);
        printf("\tOmega: [%.3f, %.3f, %.3f]\n",
               imu_data[i].gyro[0], imu_data[i].gyro[1], imu_data[i].gyro[2]);
        printf("\tG: [%.3f, %.3f, %.3f]\n",
               imu_data[i].gravity[0], imu_data[i].gravity[1], imu_data[i].gravity[2]);
        printf("\tQuaternion: [%.3f, %.3f, %.3f, %.3f]\n",
               imu_data[i].q[0], imu_data[i].q[1], imu_data[i].q[2], imu_data[i].q[3]);
        printf("\tG: [%.3f, %.3f, %.3f]\n",
               imu_data[i].gravity[0], imu_data[i].gravity[1], imu_data[i].gravity[2]);
        printf("\tPressure: %.3f\n",
               imu_data[i].pressure);
        printf("\tTemperature: %.3f\n",
               imu_data[i].temperature);
        printf("-------------------\n");
    }
}

void save_to_csv(
    const char* filename,
    const double* position,
    const double* velocity,
    const double* quaternion) {

    static FILE* fp;
    if (!fp) {
        fp = fopen(filename, "w");
        if (!fp) {
            fprintf(stderr, "Error opening file %s\n", filename);
            return;
        }
    }

    fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

    fflush(fp);
}

double compute_measurement_confidence(const imu_data_t* imu) {
    // Based on acceleration magnitude compared to gravity
    const double g = 9.81;
    double acc_mag = sqrt(imu->accel[0]*imu->accel[0] +
                         imu->accel[1]*imu->accel[1] +
                         imu->accel[2]*imu->accel[2]);
    return exp(-fabs(acc_mag - g)/g);  // Returns 1.0 when acc_mag ≈ g
}

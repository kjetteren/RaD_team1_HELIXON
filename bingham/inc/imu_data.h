#ifndef IMU_DATA_H
#define IMU_DATA_H

typedef struct {
    double accel[3];
    double magneto[3];
    double gyro[3];
    double gravity[3];
    double q[4];
    double pressure;
    double temperature;
    double timestamp;
} imu_data_t;

/***
* the method below reads IMU measurements from a file @param filename
* and parses each into @struct imu_data_t and adds it so @param imu_data array,
* @param num_measurements is to specify how many measurements we want to read from file
*/
void read_imu_csv(const char* filename, int* num_measurements, imu_data_t* imu_data);

/***
 * the method below prints @param num_measurements of parsed IMU measurements from @param imu_data,
 * for testing and debugging purposes
 */
void print_imu_data(const imu_data_t* imu_data, const int num_measurements);

/***
 * this method saves estimates to a file,
 * then we can plot them in python
 */
void save_to_csv(const char* filename, const double* position, const double* velocity, const double* quaternion);

/***
 * get IMU measurement confidence for concentration parameters Z
 */
double compute_measurement_confidence(const imu_data_t* imu);

#endif //IMU_DATA_H

#ifndef QUATERNION_H
#define QUATERNION_H

/***
 * the method below returns conjugate of @param q
 */
double* quaternion_conjugate(const double* q);

/***
 * the method below is used to perform quaternion multiplication
 * @param product (self-explanatory)
 * @param q is a quaternion used as multiplicand
 * @param p is a quaternion used as multiplicator
 */
void quaternion_multiply(double* product, const double* q, const double* p);

/***
 * normalisation of @param q quaternion (I think) can be brought down to
 * normalising a 4-dimensional vector
 */
void normalize_quaternion(double* q);

/***
 * @param q quaternion update using angular velocity @param omega,
 * this is equivalent to calculating the infinitesimal rotation
 * that the object undergoes in a small-time step @param dt
 * q(t+dt) = q(t) + dt * (dq/dt) (quaternion integration)
 */
void attitude_propagation(double* q, const double* omega, double dt);

/***
 * the method below converts accelerometer @param acc and magnetometer readings @parammag
 * into a quaternion representing the current orientation,
 * this is done in 3 steps:
 * 1. normalizing accelerometer and magnetometer vectors
 * 2. computing the rotation from the sensor frame to the world frame
 * 3. converting this rotation to a quaternion
 */
void compute_orientation_quaternion(const double* acc, const double* mag, double* q);

/***
 * rotations can also be expressed within the three-dimensional euclidean space,
 * the method below converts @param q into rotation @param matrix
 * SOURCE: https://ahrs.readthedocs.io/en/latest/filters/aqua.html#quaternion-as-orientation
 */
void compute_orientation_matrix(const double* q, double** matrix);

#endif //QUATERNION_H

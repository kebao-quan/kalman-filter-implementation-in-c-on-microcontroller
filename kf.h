#ifndef KF_H
#define KF_H

#include <stdio.h>
#include <stdint.h>

#define M(m, i, j)   *((m)->matrix + (i)*((m)->cols) + (j))

typedef struct {
	uint8_t rows;
	uint8_t cols;
	double* matrix;
} Matrix;

// Kalman filter for 2d parameters, H is always [1, 0]
typedef struct {
	Matrix P;
	Matrix R;
	Matrix Q;
	Matrix F;
	Matrix x;
} KalmanInput2D;


/**
 * Example Usage:
 * double F_[2 * 2] = {
 *	    1, 1,
 *      0, 1
 * };
 * Matrix F;
 * createMatrix(2, 2, &F, F_);
*/
void createMatrix(uint8_t rows, uint8_t cols, Matrix* A, double* matrix);
void predict(KalmanInput2D *kf);
void update(KalmanInput2D *kf, double z);


#endif
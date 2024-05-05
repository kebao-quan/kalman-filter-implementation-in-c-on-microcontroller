#include <stdio.h>
#include <stdint.h>
#include "kf.h"

/**
   Return a Matrix with given rows and cols;

   @param A      A pointer to the Matrix that will be created;
   @param rows   The rows of the given matrix;
   @param cols   The cols of the given matrix;
   @param matrix The 1d array that contain all the elements in the matrix;
 */
void createMatrix(uint8_t rows, uint8_t cols, Matrix* A, double* matrix) {
	A->rows = rows;
	A->cols = cols;
	A->matrix = matrix;
}

/**
 * Multiply two matrices together and store the result in a third matrix.
 * 
 * @param A   		Pointer to the first matrix.
 * @param B   		Pointer to the second matrix.
 * @param result    Pointer to the matrix where the result will be stored.
 * 
 * @return result if the multiplication was successful, NULL otherwise.
 */
Matrix* multiply(Matrix *A, Matrix *B, Matrix *result) {
	if (A->cols != B->rows) return NULL;
	if ((result->rows != A->rows) || (result->cols != B->cols)) return NULL;

	uint8_t r = A->rows;
	uint8_t c = B->cols;
	uint8_t i, j;
	for (i = 0; i < r; i++) {
		for (j = 0; j < c; j++) {
			double sum = 0;
			uint8_t k;
			for (k = 0; k < A->cols; k++) {
				sum += M(A, i, k) * M(B, k, j); 
			}
			M(result, i, j) = sum;
		}
	}
	return result;
}

/**
 * Copy the contents of one matrix to another.
 * 
 * @param src    Pointer to the source matrix.
 * @param dst    Pointer to the destination matrix.
 */
void copy(Matrix *src, Matrix *dst) {
	if (src->rows != dst->rows || src->cols != dst->cols) return;

	uint16_t a = src->rows * src->cols;
	uint16_t i;
	for (i = 0; i < a; i++) {
		dst->matrix[i] = src->matrix[i];
	}
}

/**
 * Transposes a matrix A and stores the result in Matrix result.
 * 
 * @param A   		Pointer to the matrix to transpose.
 * @param result    Pointer to the matrix where the transposed result will be stored.
 */
Matrix* transpose(Matrix *A, Matrix *result) {
    uint8_t r = A->rows;
	uint8_t c = A->cols;
    uint8_t i, j;
    for (i = 0; i < r; i++) {
		for (j = 0; j < c; j++) {         
			M(result, j, i) = M(A, i, j);
		}
	}
	return result;
}

/**
 * Add two matrices together. Result is stored in the first matrix.
 * 
 * @param a    Pointer to the first matrix.
 * @param b    Pointer to the second matrix.
 */
void add(Matrix *a, Matrix *b){
    //check if matrices are compatible
    if(a->rows != b->rows || a->cols != b->cols){
	    return;
    }
    //create result matrix
    int r = a->rows;
    int c = a->cols;
    //add matrices
    int i,j;
    for(i = 0; i < r ; i++){
	    for(j = 0; j < c; j++){
		    M(a, i, j) += M(b, i, j);
	    }
    }
}

/**
    Subtracts two matrices together, result is stored in the first matrix.

    @param *a pointer to first matrix (A);
    @param *b pointer to second matrix (B);
*/
void sub(Matrix *a, Matrix *b){
    //check if matrices are compatible
    if(a->rows != b->rows || a->cols != b->cols){
	    return;
    }
    int r = a->rows;
    int c = a->cols;
    int i,j;
    for(i = 0; i < r ; i++){
	    for(j = 0; j < c; j++){
		    M(a, i, j) -= M(b, i, j);
	    }
    }
}

void printMatrix(Matrix *m) {
	uint8_t i, j;
	for (i = 0; i < m->rows; i++) {
		for (j = 0; j < m->cols; j++) {
			printf("%.3f ", M(m, i, j));
		}
		printf("\n");
	}
	
}

/**
 * Predict the next state of the Kalman filter and updated the state vector in kf->x
 * 2D state vector implementation of Kalman filter. x = [position, velocity];
 * 
 * @param kf    Pointer to the KalmanInput2D.
 */
void predict(KalmanInput2D *kf) {
	// Update State Vector
	double Fx_[2] = { 0 };
	Matrix Fx;
	createMatrix(2, 1, &Fx, Fx_);
	multiply(&kf->F, &kf->x, &Fx);

	// Calculate F transpose
	double FT_[2 * 2] = { 0 };
	Matrix FT;
	createMatrix(2, 2, &FT, FT_);
	transpose(&kf->F, &FT);
	
	// F * P
	double FP_[4];
	Matrix FP;
	createMatrix(2, 2, &FP, FP_);
	multiply(&kf->F, &kf->P, &FP);

	// FP * FT
	double FPFt_[2 * 2] = { 0 };
	Matrix FPFt;
	createMatrix(2, 2, &FPFt, FPFt_);
	multiply(&FP, &FT, &FPFt);

	// P = FPFt + Q
	add(&FPFt, &kf->Q);

	// update
	copy(&Fx, &kf->x);
	copy(&FPFt, &kf->P);
}


/**
 * Update the Kalman filter with the new measurement z. The new state vector is stored in kf->x.
 * Note always use H = [1, 0] for 2D Kalman filter.
 * Note always call predict before calling update.
 * 
 * @param kf    Pointer to the KalmanInput2D.
 * @param z     The new measurement.
 */
void update(KalmanInput2D *kf, double z) {
	// Simplifing y = z - Hx
	double y0 = z - kf->x.matrix[0];
	double y_[1] = { y0 };
	Matrix y;
	createMatrix(1, 1, &y, y_);
	
	// Simplifing S = H * P * Ht + R; S^-1 = 1/S
	double S_inv_[1] = { 0 };
	Matrix S_inv;
	createMatrix(1, 1, &S_inv, S_inv_);
	M(&S_inv, 0, 0) = M(&kf->P, 0, 0);
	M(&S_inv, 0, 0) += M(&kf->R, 0, 0);
	M(&S_inv, 0, 0) = 1 / M(&S_inv, 0, 0);

	// Simplifing P * Ht by taking the first column of P
	double PHt_[2] = {
		M(&kf->P, 0, 0),
		M(&kf->P, 1, 0)
	};
	Matrix PHt;
	createMatrix(2, 1, &PHt, PHt_);

	// Calculate the Kalman Gain, K = P * Ht * S^-1
	double K_[2] = { 0 };
	Matrix K;
	createMatrix(2, 1, &K, K_);
	multiply(&PHt, &S_inv, &K);

	// Ky = K * y
	double Ky_[2] = { 0 };
	Matrix Ky;
	createMatrix(2, 1, &Ky, Ky_);
	multiply(&K, &y, &Ky);

	// Update x --> x = x + Ky
	add(&kf->x, &Ky);

	// A = I
	double A_[4] = {
		1, 0,
		0, 1
	};
	Matrix A;
	createMatrix(2, 2, &A, A_);

	double KH_[4] = {
		M(&K, 0, 0),    0,
		M(&K, 1, 0),    0
	};
	Matrix KH;
	createMatrix(2, 2, &KH, KH_);

	// A = I-KH
	sub(&A, &KH);

	// new_P = (I - KH) P
	double new_P_[4] = { 0 };
	Matrix new_P;
	createMatrix(2, 2, &new_P, new_P_);
	multiply(&A, &kf->P, &new_P);

	// update P from new_P
	copy(&new_P, &kf->P);
}

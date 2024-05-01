#include <stdio.h>
#include <stdint.h>


#define M(m, i, j)   *((m)->matrix + (i)*((m)->cols) + (j))

#define X_DIM 2
#define FX_DIM X_DIM



typedef struct {
	uint8_t rows;
	uint8_t cols;
	double* matrix;
} Matrix;

typedef struct {
	Matrix P; //Control input
	Matrix R; //Observation cov
	Matrix Q; //Process cov
	Matrix F; //State transition
	Matrix H; //Observation
	Matrix x; //current state
} KalmanInput;

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

void multiply(Matrix *A, Matrix *B, Matrix *result) {
	if (A->cols != B->rows) return;
	if ((result->rows != A->rows) || (result->cols != B->cols)) return;

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
	
}

void add(Matrix *A, Matrix *B, Matrix *result) {
    uint8_t r = A->rows;
	uint8_t c = A->cols;
    uint8_t i, j = 0;
    for (i = 0; i < r; i++) {
		for (j = 0; j < c; j++) {         
			M(result, i, j) = M(A, i, j) + M(B, i, j);
		}
	}
}

void transpose(Matrix *A, Matrix *result) {
    uint8_t r = A->rows;
	uint8_t c = A->cols;
    uint8_t i, j = 0;
    for (i = 0; i < r; i++) {
		for (j = 0; j < c; j++) {         
			M(result, j, i) = M(A, i, j);
		}
	}
}

void twoDimInverse(Matrix *A, Matrix *result) {
    uint8_t a = M(A, 0, 0);
    uint8_t b = M(A, 0, 1);
    uint8_t c = M(A, 1, 0);
    uint8_t d = M(A, 1, 1);
    
    double det = a*d - b*c;
    
    M(result, 0, 0) = d/det;
    M(result, 0, 1) = (-b)/det;
    M(result, 1, 0) = (-c)/det;
    M(result, 1, 1) = a/det;
}

void copy(Matrix *src, Matrix *dst) {
	if (src->rows != dst->rows || src->cols != dst->cols) return;

	uint16_t a = src->rows * src->cols;
	uint16_t i;
	for (i = 0; i < a; i++) {
		dst->matrix[i] = src->matrix[i];
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

// 2D state vector implementation of Kalman filter. x = [position, velocity];
void predict(KalmanInput *kf) {
	// Update State Vector
	double Fx_[X_DIM] = { 0 };
	Matrix Fx;
	createMatrix(X_DIM, 1, &Fx, Fx_);
	multiply(&kf->F, &kf->x, &Fx);
	copy(&Fx, &kf->x);

	// Update P
	double FP_[kf->F.rows * kf->P.cols];
	Matrix FP;
	createMatrix(kf->F.rows, kf->P.cols, &FP, FP_);

	multiply(&kf->F, &kf->P, &FP);
    
    //Create F^T (F Transpose)
    double Ft_[kf->F.cols * kf->F.rows];
    Matrix Ft;
    createMatrix(kf->F.cols, kf->F.rows, &Ft, Ft_);
    transpose(&kf->F, &Ft);
    
    //Multiply FP by F^T
    double FPFt_[FP.rows * Ft.cols];
    Matrix FPFt;
    createMatrix(FP.rows, Ft.cols, &FPFt, FPFt_);
    multiply(&FP, &Ft, &FPFt);
    
    //Add Q and update P
    add(&FPFt, &kf->Q, &kf->P);
}

void update(KalmanInput *kf) {
	
}

int main() {
	double F_[X_DIM * X_DIM] = {
		1, 1,
		0, 1
	};

	double P_[X_DIM * X_DIM] = {
		10,    0,
		0,     1000
	};

	double H_[X_DIM] = {
		1, 0
	};

	double R_[X_DIM * X_DIM] = {
		0.5, 0,
		0, 0
	};


	KalmanInput kf;
	createMatrix(X_DIM, X_DIM, &kf.F, F_);
	createMatrix(X_DIM, X_DIM, &kf.P, P_);
	createMatrix(1, X_DIM, &kf.H, H_);
	createMatrix(X_DIM, X_DIM, &kf.R, R_);

        


	/* double H_[1][2] = { */
	/* 	{1, 0} */
	/* }; */
	/* Matrix H; */
	/* createMatrix(&H, 1, 2, H_); */

	/* const double HT[4][2] = { */
	/* 	{1, 0}, */
	/* 	{0, 1}, */
	/* 	{0, 0}, */
	/* 	{0, 0} */
	/* }; */

	/* const double R[2][2] = { */
	/* 	{10, 0}, */
	/* 	{0, 10} */
	/* }; */

	/* double P[4][4] = { */
	/* 	{1000, 0, 0, 0}, */
	/* 	{0, 1000, 0, 0}, */
	/* 	{0, 0, 1000, 0}, */
	/* 	{0, 0, 0, 1000} */
	/* }; */

}

/* double x_prev[4] = {0, 0, 0, 0}; */
/* double x_post[4] = {0, 0, 0, 0}; */




/* //Input: Previous State Vector. {x, y, x', y'} */
/* void predict() { */
/* 	x_post[0] = F[0][0] * x_prev[0] + F[0][2] * x_prev[2]; */
/* 	x_post[1] = F[1][1] * x_prev[1] + F[1][3] * x_prev[3]; */
/* 	x_post[2] = F[2][2] * x_prev[2]; */
/* 	x_post[3] = F[3][3] * x_prev[3]; */

/* 	double res[4][4]; */
/* 	double P_new[4][4]; */
/* 	multiply(F, P, res); */
/* 	multiply(res, FT, P_new); */

/* 	for (int i = 0; i < 4; i++) { */
/* 		for (int j = 0; j < 4; j++) { */
/* 			P[i][j] = P_new[i][j]; */
/* 		} */
/* 	} */
/* } */


/* void update(double z[2], double x_est[4]) { */
/* 	double residual[2] = {z[0] - x_post[0], z[1] - x_post[1]}; */
/* 	double S[2][2] = { */
/* 		{P[0][0] + R[0][0], 0}, */
/* 		{0, P[1][1] + R[1][1]} */
/* 	}; */
/* 	double S_INVERSE[2][2] = { */
/* 		{S[1][1] / (S[0][0] * S[1][1]), 	0							 }, */
/* 		{0, 								S[0][0] / (S[0][0] * S[1][1])} */
/* 	}; */
/* 	double K[4][2]; */
/* 	memset(K, 0, sizeof(K)); */
/* 	K[0][0] = P[0][0] * S_INVERSE[0][0]; */
/* 	K[1][1] = P[1][1] * S_INVERSE[1][1]; */
	
/* 	x_est[0] = x_post[0] + K[0][0] * residual[0]; */
/* 	x_est[1] = x_post[1] + K[1][1] * residual[1]; */
/* 	x_est[2] = x_post[2] + K[0][0] * (x_est[0] - x_prev[0]); */
/* 	x_est[3] = x_post[3] + K[1][1] * (x_est[1] - x_prev[1]); */


/* 	// Update P and x_prev */
/* 	P[0][0] = (1 - K[0][0]) * P[0][0]; */
/* 	P[1][1] = (1 - K[1][1]) * P[1][1]; */
/* 	x_prev[0] = x_est[0]; */
/* 	x_prev[1] = x_est[1]; */
/* 	x_prev[2] = x_est[2]; */
/* 	x_prev[3] = x_est[3]; */
/* } */

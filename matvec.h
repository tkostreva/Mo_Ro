#ifndef _matvec_
#define _matvec_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//the functions and structures defined herein are designed to work on 3x3 matrices and vectors. 
typedef struct vector 
{
    float v[3];
} vector;

typedef struct matrix
{
    float v[3][3];
} matrix;

void AddVectors(vector* pA, vector* pB, vector* pResult);//add 2 vectors
void DotVectors(vector* a, vector* b, vector* result);//find the dot product of 2 vectors and store scalar to result->v[0]
void CrossVectors ( vector* a, vector* b, vector* result);//find the cross product of 2 vectors
void MultMatVec ( matrix* m, vector *v, vector* result);//multiply a matrix by a vector
void AddMatrices (matrix* a, matrix* b, matrix *result);//add 2 matrices
void MultMatrices (matrix* a, matrix* b, matrix *result);//Multiply 2 matrices
void PrintMatrix( matrix* mat );//print the contents of a matrix
void PrintVector( vector* pVec );//print the contents of a vector

#endif
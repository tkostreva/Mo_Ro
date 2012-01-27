#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "matvec.h"
void AddVectors( vector* pA, vector* pB, vector* pResult )
{
  int i;
  /* This is error-checking, and wasn't required */
  if(pA == NULL || pB == NULL || pResult == NULL)
    return;
  /* Loop through the 3 elements of the vectors and add them; store in 
     the vector pointed to by pResult */
  for(i = 0; i < 3; i++)
    pResult->v[i] = pA->v[i] + pB->v[i];
}

void DotVectors ( vector* a, vector* b, vector* result)//store scalar to result->v[0]
{
	int i;//cursor
	result->v[0] = 0;//set to 0 since it will be accumulating.
	for(i=0; i<3; i++){
		result->v[0] += a->v[i] * b->v[i];
	}
}

void CrossVectors ( vector* a, vector* b, vector* result)
{
	result->v[0] = (a->v[1]*b->v[2]) - (a->v[2]*b->v[1]); 
	result->v[1] = (a->v[2]*b->v[0]) - (a->v[0]*b->v[2]);
	result->v[2] = (a->v[0]*b->v[1]) - (a->v[1]*b->v[0]);
}

void MultMatVec ( matrix* m, vector *v, vector* result)
{
	int i, j;//cursors
	for(i=0; i<3; i++){
		result->v[i]=0;
		for(j=0; j<3; j++){
			result->v[i] += m->v[i][j]*v->v[j];
		
		}
	}
}

void AddMatrices (matrix* a, matrix* b, matrix *result)
{
	int i , j;//cursors
	for ( i=0; i<3; i++ ) {
		for ( j=0; j<3; j++){
			result->v[i][j] = a->v[i][j] + b->v[i][j];
		} 
	}
}

void MultMatrices (matrix* a, matrix* b, matrix *result)
{
	int i , j, k;//cursors
	for ( i=0; i<3; i++ ) {//row
		for ( j=0; j<3; j++ ){//col
			result->v[i][j] = 0;
			for ( k=0; k<3; k++ ){
				result->v[i][j] += a->v[i][k]*b->v[k][j];//I assume [row][col]
			}
		} 
	}
}

void PrintMatrix( matrix* mat )
{
  /* Put newlines (\n's) in strategic places to make it
     look pretty.  The 5 in %.5f makes each number take
     print exactly 5 digits following the "."
  */
  printf("%.5f %.5f %.5f\n%.5f %.5f %.5f\n%.5f %.5f %.5f\n",
	 mat->v[0][0], mat->v[0][1], mat->v[0][2],
	 mat->v[1][0], mat->v[1][1], mat->v[1][2],
	 mat->v[2][0], mat->v[2][1], mat->v[2][2]);
}

void PrintVector( vector* pVec )
{
  /* %f prints a float.
     pVec is a pointer to a vector, so you must dereference using the
     -> operator, then access the member array v
  */
  printf("%f %f %f\n", 
	 pVec->v[0],
	 pVec->v[1],
	 pVec->v[2] );                
}
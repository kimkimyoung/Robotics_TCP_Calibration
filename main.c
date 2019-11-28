#include"svd.h"

#define M 9
#define N 3
#define K 6

typedef struct FK
{
    double ori[N][N];
    double pos[N];
}ForwardKinematics;

#if(0)
// Panda3
#define a2 0.264
#define a3 0.236
#define d1 0.111
#define d3 0.013
#define d4 0.144
#define d5 0.114
#define d6 0.082

void Fkine( ForwardKinematics *state, double *point)
{

    // Orientaion
    state->ori[0][0] = cos(point[5])*(sin(point[0])*sin(point[4]) + cos(point[1]+point[2]+point[3])*cos(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*cos(point[0])*sin(point[5]);
    state->ori[0][1] = -sin(point[5])*(sin(point[0])*sin(point[4]) + cos(point[1]+point[2]+point[3])*cos(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*cos(point[0])*cos(point[5]);
    state->ori[0][2] = cos(point[4])*sin(point[0]) - cos(point[1]+point[2]+point[3])*cos(point[0])*sin(point[4]);

    state->ori[1][0] = -cos(point[5])*(cos(point[0])*sin(point[4]) - cos(point[1]+point[2]+point[3])*sin(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*sin(point[0])*sin(point[5]);
    state->ori[1][1] = sin(point[5])*(cos(point[0])*sin(point[4]) - cos(point[1]+point[2]+point[3])*sin(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*sin(point[0])*cos(point[5]);
    state->ori[1][2] = -cos(point[4])*cos(point[0]) - cos(point[1]+point[2]+point[3])*sin(point[0])*sin(point[4]);

    state->ori[2][0] = cos(point[1]+point[2]+point[3])*sin(point[5]) + sin(point[1]+point[2]+point[3])*cos(point[4])*cos(point[5]);
    state->ori[2][1] = -sin(point[1]+point[2]+point[3])*cos(point[4])*sin(point[5]) + cos(point[1]+point[2]+point[3])*cos(point[5]);
    state->ori[2][2] = -sin(point[1]+point[2]+point[3])*sin(point[4]);

    // Position
    state->pos[0] = -a2*cos(point[0])*cos(point[1]) + d6*(cos(point[4])*sin(point[0]) - cos(point[1]+point[2]+point[3])*cos(point[0])*sin(point[4])) + (d3 + d4)*sin(point[0]) + d5*sin(point[1]+point[2]+point[3])*cos(point[0]) - a3*cos(point[0])*cos(point[1])*cos(point[2]) + a3*cos(point[0])*sin(point[1])*sin(point[2]);
    state->pos[1] = -d6*(cos(point[0])*cos(point[4]) + cos(point[1]+point[2]+point[3])*sin(point[0])*sin(point[4])) - (d3 + d4)*cos(point[0]) - a2*cos(point[1])*sin(point[0]) + d5*sin(point[1]+point[2]+point[3])*sin(point[0]) - a3*sin(point[0])*cos(point[1])*cos(point[2]) + a3*sin(point[0])*sin(point[1])*sin(point[2]);
    state->pos[2] = d1 - a3*sin(point[1]+point[2]) - a2*sin(point[1]) - d5*(cos(point[1]+point[2])*cos(point[3]) -sin(point[1]+point[2])*sin(point[3])) - d6*sin(point[4])*(cos(point[1] + point[2])*sin(point[3]) + sin(point[1] + point[2])*cos(point[3]));
}

#else
// UR5
#define a2 0.425
#define a3 0.3922
#define d1 0.1625
#define d4 0.1333
#define d5 0.0997
#define d6 0.0996

void Fkine( ForwardKinematics *state, double *point)
{    
    // Orientaion
    state->ori[0][0] = cos(point[5])*(sin(point[0])*sin(point[4]) + cos(point[1]+point[2]+point[3])*cos(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*cos(point[0])*sin(point[5]);
    state->ori[0][1] = -sin(point[5])*(sin(point[0])*sin(point[4]) + cos(point[1]+point[2]+point[3])*cos(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*cos(point[0])*cos(point[5]);
    state->ori[0][2] = cos(point[4])*sin(point[0]) - cos(point[1]+point[2]+point[3])*cos(point[0])*sin(point[4]);
    state->ori[1][0] = -cos(point[5])*(cos(point[0])*sin(point[4]) - cos(point[1]+point[2]+point[3])*sin(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*sin(point[0])*sin(point[5]);
    state->ori[1][1] = sin(point[5])*(cos(point[0])*sin(point[4]) - cos(point[1]+point[2]+point[3])*sin(point[0])*cos(point[4])) - sin(point[1]+point[2]+point[3])*sin(point[0])*cos(point[5]);
    state->ori[1][2] = -cos(point[4])*cos(point[0]) - cos(point[1]+point[2]+point[3])*sin(point[0])*sin(point[4]);
    state->ori[2][0] = cos(point[1]+point[2]+point[3])*sin(point[5]) + sin(point[1]+point[2]+point[3])*cos(point[4])*cos(point[5]);
    state->ori[2][1] = -sin(point[1]+point[2]+point[3])*cos(point[4])*sin(point[5]) + cos(point[1]+point[2]+point[3])*cos(point[5]);
    state->ori[2][2] = -sin(point[1]+point[2]+point[3])*sin(point[4]);

    // Position
    state->pos[0] = -a2*cos(point[0])*cos(point[1]) + d6*(cos(point[4])*sin(point[0]) - cos(point[1]+point[2]+point[3])*cos(point[0])*sin(point[4])) + d4*sin(point[0]) + d5*sin(point[1]+point[2]+point[3])*cos(point[0]) - a3*cos(point[0])*cos(point[1])*cos(point[2]) + a3*cos(point[0])*sin(point[1])*sin(point[2]);
    state->pos[1] = -d6*(cos(point[0])*cos(point[4]) + cos(point[1]+point[2]+point[3])*sin(point[0])*sin(point[4])) - d4*cos(point[0]) - a2*cos(point[1])*sin(point[0]) + d5*sin(point[1]+point[2]+point[3])*sin(point[0]) - a3*sin(point[0])*cos(point[1])*cos(point[2]) + a3*sin(point[0])*sin(point[1])*sin(point[2]);
    state->pos[2] = d1 - a3*sin(point[1]+point[2]) - a2*sin(point[1]) - d5*(cos(point[1]+point[2])*cos(point[3]) -sin(point[1]+point[2])*sin(point[3])) - d6*sin(point[4])*(cos(point[1] + point[2])*sin(point[3]) + sin(point[1] + point[2])*cos(point[3]));
}
#endif

void SubMatrix(double *dst, double *A, double *B, int row, int col)
{
    int i;
    int j;
    for(i=0; i<row; i++)
    {
        for(j=0; j<col; j++)
        {
            *(dst+i*col+j) = *(A+i*col+j) - *(B+i*col+j) ;
        }
    }
}

void SubVector(double *dst, double *A, double *B, int row)
{
    int i;
    int j;
    for(i=0; i<row; i++)
    {
        *(dst+i) = *(A+i) - *(B+i) ;
    }
}

void Merge3Matrix(double *dst, double *A, double *B, double *C, int row, int col) // 请注意输入row&col是对于输出矩阵的还是输入矩阵的
{
    int i;
    int j;
    if (col=1)
    {
        for(i=0; i<row; i++)
        {
            *(dst+i) = *(A+i);

        }
        for(i=0; i<row; i++)
        {
            *(dst+(i+row)) = *(B+i);
        }
        for(i=0; i<row; i++)
        {
            *(dst+(i+2*row)) = *(C+i);
        }
    }
    else
    {
        for(i=0; i<row; i++)
        {
            if(i<2){
                for(j=0; j<col; j++){ *(dst+i*col+j) = *(A+i*col+j);}
            }
            else if(i<4){
                for(j=0; j<col; j++){ *(dst+i*col+j) = *(B+(i-2)*col+j);}
            }
            else{
                for(j=0; j<col; j++){ *(dst+i*col+j) = *(C+(i-4)*col+j);}
            }
        }
    }
    
}

double Norm(double* A, int row)
{
    double sum = 0;
    for(int i=0; i<row; i++)
    {
        sum = sum + pow(*(A+i),2);
    }
    return sqrt(sum);
}

void MatrixMulConstant(double *dst, double *A, double n, int row, int col)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            *(dst+i*col+j) = *(A+i*col+j) * n;
        }
    }
}

void AddZeroToMatrix(double *A, double *B, double *C, int row, int col)
{
    int i, j;
    for(i=0; i<row; i++)
    {
        for(j=0; j<col; j++)
        {
            *(A+2*i*col+j) = *(B+i*col+j);
        }
    }
    for(i=0; i<row; i++)
    {
        for(j=0; j<col; j++)
        {
            *(A+2*(i + row)*col+j + col) = *(C+i*col+j);
        }
    }       
}

void Merge2Vector(double *dst, double *A, double *B, int row)
{
    int i;
    {
        for(i=0; i<row; i++)
        {
            *(dst+i) = *(A+i);
        }
        for(i=0; i<row; i++)
        {
            *(dst+(i+row)) = *(B+i);
        }
    }
}

void DeMergeVector(double *T, double *n, double *a, int row)
{
    for(int i=0; i<row; i++)
    {
        *(n+i) = *(T+i);
    }
    for(int i=0; i<row; i++)
    {
        *(a+i) = *(T+i+row);
    }    
}
 
void Cross3dVector(double *o, double *a, double *n)
{
    *o = *(a+1) * *(n+2) - *(a+2) * *(n+1);
    *(o+1) = *(a+2) * *n - *a * *(n+2);
    *(o+2) = *a * *(n+1) - *n * *(a+1);
}

void SVD(double *dst, double *A, double *B, double *U, double *V,
            double *singular_values, int row, int col)
{
    double* dummy_array;

    dummy_array = (double*) malloc(col * sizeof(double));
    if (dummy_array == NULL) {printf(" No memory available\n"); exit(0); }

    int err = Singular_Value_Decomposition( A, row, col, U, 
                    singular_values, V, dummy_array);
    free(dummy_array);

    // Position Calibration
    double tolerance;

    Singular_Value_Decomposition_Solve( U, singular_values, V,
                    tolerance, row, col, B, dst);
}

void PrintMatrix(double *A, int row, int col)
{
    for (int i=0; i<row; i++)
    {
        for( int j=0; j<col; j++)
        {
            printf("%f\t", *(A+i*col+j));
        }
        printf("\n");
    }
    printf("\n");
}

int main()
{
    // data input
    double point1[6] = {-2.0331, -0.7404, 1.8401, -1.0997, -2.0331, 0};
    double point2[6] = {-1.9780, -1.0521, 1.8949, -1.7221, -2.1696, 0.0269};
    double point3[6] = {-2.2777, -0.7273,  1.4519, -1.4235, -2.3429, 0.6740};
    double point4[6] = {-2.4548, -1.6277, 1.7076, -1.6972, -2.7185, 1.9017};
    double point5[6] = {-2.7276, -1.0468, 1.1090, -2.2049, -2.6321, 1.3174};
    double point6[6] = {-2.4548, -1.3584, 0.5725, -0.8313, -2.7185, 1.9017};

    // Forward Kinematics
    ForwardKinematics state1;
    ForwardKinematics state2;
    ForwardKinematics state3;
    ForwardKinematics state4;
    ForwardKinematics state5;
    ForwardKinematics state6;
    
    Fkine( &state1, point1);
    Fkine( &state2, point2);
    Fkine( &state3, point3);
    Fkine( &state4, point4);
    Fkine( &state5, point5);
    Fkine( &state6, point6);

    // Print Forward Kinematics
    printf("-----------------------\nRotMat_be1 & PosVec_be1:\n-----------------------\n");
    PrintMatrix((double*)(&state1)->ori, 3, 3);
    PrintMatrix((double*)(&state1)->pos, 3, 1);
    printf("-----------------------\nRotMat_be2 & PosVec_be2:\n-----------------------\n");
    PrintMatrix((double*)(&state2)->ori, 3, 3);
    PrintMatrix((double*)(&state2)->pos, 3, 1);
    printf("-----------------------\nRotMat_be3 & PosVec_be3:\n-----------------------\n");
    PrintMatrix((double*)(&state3)->ori, 3, 3);
    PrintMatrix((double*)(&state3)->pos, 3, 1);
    printf("-----------------------\nRotMat_be4 & PosVec_be4:\n-----------------------\n");
    PrintMatrix((double*)(&state4)->ori, 3, 3);
    PrintMatrix((double*)(&state4)->pos, 3, 1);
    printf("-----------------------\nRotMat_be5 & PosVec_be5:\n-----------------------\n");
    PrintMatrix((double*)(&state5)->ori, 3, 3);
    PrintMatrix((double*)(&state5)->pos, 3, 1);
    printf("-----------------------\nRotMat_be6 & PosVec_be6:\n-----------------------\n");
    PrintMatrix((double*)(&state6)->ori, 3, 3);
    PrintMatrix((double*)(&state6)->pos, 3, 1);

    // Find Matrix A and B and C and D
    double sub_ori1[N][N];
    double sub_ori2[N][N];
    double sub_ori3[N][N];

    double sub_pos1[N];
    double sub_pos2[N];
    double sub_pos3[N];

    double sub_pos5[N];
    double sub_pos6[N];

    SubMatrix((double*)sub_ori1, (double*)(&state1)->ori, (double*)(&state2)->ori, N, N);
    SubMatrix((double*)sub_ori2, (double*)(&state2)->ori, (double*)(&state3)->ori, N, N);
    SubMatrix((double*)sub_ori3, (double*)(&state3)->ori, (double*)(&state4)->ori, N, N);
    
    SubVector(sub_pos1, (&state2)->pos, (&state1)->pos, N);
    SubVector(sub_pos2, (&state3)->pos, (&state2)->pos, N);
    SubVector(sub_pos3, (&state4)->pos, (&state3)->pos, N);
    
    SubVector(sub_pos5, (&state5)->pos, (&state4)->pos, N);
    SubVector(sub_pos6, (&state6)->pos, (&state4)->pos, N);
    
    // Merge A = [sub_ori1; sub_ori2; sub_ori3]
    double A[M][N];
    Merge3Matrix((double*)A, (double*)sub_ori1, (double*)sub_ori2, (double*)sub_ori3, M, N);
    // Merge B = [sub_pos1; sub_pos2; sub_pos3]
    double B[M];
    Merge3Matrix((double*)B, (double*)sub_pos1, (double*)sub_pos2, (double*)sub_pos3, N, 1);
    // Merge C = [delta_x * state4->ori 0 0 0 ; 0 0 0 delta_z * state4->ori];
    double delta_x = Norm(sub_pos5, N);
    double delta_z = Norm(sub_pos6, N);
    double ori_x[N][N];
    double ori_z[N][N];
    double C[K][K];
    MatrixMulConstant((double*)ori_x, (double*)(&state4)->ori, delta_x, N, N);
    MatrixMulConstant((double*)ori_z, (double*)(&state4)->ori, delta_z, N, N);
    AddZeroToMatrix((double*)C, (double*)ori_x, (double*)ori_z, N, N);
    // Merge D = [sub_pos5; sub_pos6]
    double D[K];
    Merge2Vector(D, sub_pos5, sub_pos6, 3);

    // Position Calibration
    double U[M][N];
    double V[N][N];
    double singular_values_pos[N];
    double pos_cal[N];

    // Solve Sigular Values Decompose
    SVD(pos_cal, (double*)A, (double*)B, (double*)U, (double*)V,
                singular_values_pos, M, N);
    printf("---------------------------\nPosition Calibration Result:\n---------------------------\n%f\t%f\t%f\t\n\n",pos_cal[0],pos_cal[1],pos_cal[2]);

    // Orientation Calibration
    // Rot = [n_et, o_et, a_et]
    double O[K][K];
    double P[K][K];
    double singular_values_ori[K];
    double ori_cal[K];
    double n_et[N];
    double a_et[N];
    double o_et[N];
    // Solve Sigular Values Decompose
    SVD(ori_cal, (double*)C, (double*)D, (double*)O, (double*)P,
                singular_values_ori, K, K);
    DeMergeVector(ori_cal, n_et, a_et, N);
    
    Cross3dVector(o_et, a_et, n_et);
    Cross3dVector(a_et, n_et, o_et);

    printf("------------------------------\nOrientation Calibration Result:\n------------------------------\n");
    printf("%f\t%f\t%f\t\n",n_et[0],o_et[0],a_et[0]);
    printf("%f\t%f\t%f\t\n",n_et[1],o_et[1],a_et[1]);
    printf("%f\t%f\t%f\t\n\n",n_et[2],o_et[2],a_et[2]);

}


#include "BirdEyeView.h"
#include "math.h"


// Solve 8x8 linear system using Gaussian Elimination
void solveLinearSystem(float A[8][9], float* solution) {
    for (int i = 0; i < 8; i++) {
        // Find max row in column i
        int maxRow = i;
        for (int k = i + 1; k < 8; k++) {
            if (fabs(A[k][i]) > fabs(A[maxRow][i])) {
                maxRow = k;
            }
        }
        // Swap rows
        for (int k = i; k < 9; k++) {
            float tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }

        // Normalize row
        float divisor = A[i][i];
        for (int k = i; k < 9; k++) {
            A[i][k] /= divisor;
        }

        // Eliminate column
        for (int j = 0; j < 8; j++) {
            if (j != i) {
                float factor = A[j][i];
                for (int k = i; k < 9; k++) {
                    A[j][k] -= factor * A[i][k];
                }
            }
        }
    }

    // Extract solution
    for (int i = 0; i < 8; i++) {
        solution[i] = A[i][8];
    }
}

// Compute Perspective Transformation Matrix
void getPerspectiveTransform(Point2D src[4], Point2D dst[4], float H[3][3]) {
    float A[8][9] = { 0 };
    float h[8];

    for (int i = 0; i < 4; i++) {
        float x = src[i].x, y = src[i].y;
        float x_p = dst[i].x, y_p = dst[i].y;

        A[i * 2][0] = x;
        A[i * 2][1] = y;
        A[i * 2][2] = 1;
        A[i * 2][6] = -x * x_p;
        A[i * 2][7] = -y * x_p;
        A[i * 2][8] = x_p;

        A[i * 2 + 1][3] = x;
        A[i * 2 + 1][4] = y;
        A[i * 2 + 1][5] = 1;
        A[i * 2 + 1][6] = -x * y_p;
        A[i * 2 + 1][7] = -y * y_p;
        A[i * 2 + 1][8] = y_p;
    }

    solveLinearSystem(A, h);

    // Construct Homography Matrix
    H[0][0] = h[0]; H[0][1] = h[1]; H[0][2] = h[2];
    H[1][0] = h[3]; H[1][1] = h[4]; H[1][2] = h[5];
    H[2][0] = h[6]; H[2][1] = h[7]; H[2][2] = 1.0;
}

// Apply Perspective Transformation
Point2D applyPerspectiveTransform(float H[3][3], Point2D p) {
    Point2D result;
    float w = H[2][0] * p.x + H[2][1] * p.y + H[2][2];
    result.x = (H[0][0] * p.x + H[0][1] * p.y + H[0][2]) / w;
    result.y = (H[1][0] * p.x + H[1][1] * p.y + H[1][2]) / w;
    return result;
}


//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1,m2) {
    // returns 2D array that is the result of m1*m2
    var mat = [];
    var i,j,k;
    for (i=0; i<m1.length; i++)
    {
        mat[i] = [];
        for (j=0; j<m2[0].length; j++)
        {
            var sum = 0;
            for (k=0; k<m2.length; k++)
            {
                sum += m1[i][k]*m2[k][j];
            }
            mat[i][j] = sum;
        }
    }    
    return mat;
}

function matrix_transpose(m) {
    // returns 2D array that is the result of m1*m2
    var mat = [];
    var i,j;
    
    for (j=0; j<m[0].length; j++)
    {
        mat[j] = [];
        for (i=0; i<m.length; i++)
        {
            mat[j][i] = m[i][j];
        }
    }
    return mat;
}

function matrix_pseudoinverse(m) {
    // returns pseudoinverse of matrix m
    m_T = matrix_transpose(m);

    return matrix_multiply(numeric.inv(matrix_multiply(m_T, m)), m_T);
}

function matrix_invert_affine(a) {
    // returns 2D array that is the invert affine of 4-by-4 matrix m
    var s0 = a[0][0] * a[1][1] - a[1][0] * a[0][1];
    var s1 = a[0][0] * a[1][2] - a[1][0] * a[0][2];
    var s2 = a[0][0] * a[1][3] - a[1][0] * a[0][3];
    var s3 = a[0][1] * a[1][2] - a[1][1] * a[0][2];
    var s4 = a[0][1] * a[1][3] - a[1][1] * a[0][3];
    var s5 = a[0][2] * a[1][3] - a[1][2] * a[0][3];

    var c5 = a[2][2] * a[3][3] - a[3][2] * a[2][3];
    var c4 = a[2][1] * a[3][3] - a[3][1] * a[2][3];
    var c3 = a[2][1] * a[3][2] - a[3][1] * a[2][2];
    var c2 = a[2][0] * a[3][3] - a[3][0] * a[2][3];
    var c1 = a[2][0] * a[3][2] - a[3][0] * a[2][2];
    var c0 = a[2][0] * a[3][1] - a[3][0] * a[2][1];

    var mat = [[],[],[],[]];
    var invdet = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
    mat[0][0] = ( a[1, 1] * c5 - a[1, 2] * c4 + a[1, 3] * c3) * invdet;
    mat[0][1] = (-a[0, 1] * c5 + a[0, 2] * c4 - a[0, 3] * c3) * invdet;
    mat[0][2] = ( a[3, 1] * s5 - a[3, 2] * s4 + a[3, 3] * s3) * invdet;
    mat[0][3] = (-a[2, 1] * s5 + a[2, 2] * s4 - a[2, 3] * s3) * invdet;

    mat[1][0] = (-a[1, 0] * c5 + a[1, 2] * c2 - a[1, 3] * c1) * invdet;
    mat[1][1] = ( a[0, 0] * c5 - a[0, 2] * c2 + a[0, 3] * c1) * invdet;
    mat[1][2] = (-a[3, 0] * s5 + a[3, 2] * s2 - a[3, 3] * s1) * invdet;
    mat[1][3] = ( a[2, 0] * s5 - a[2, 2] * s2 + a[2, 3] * s1) * invdet;

    mat[2][0] = ( a[1, 0] * c4 - a[1, 1] * c2 + a[1, 3] * c0) * invdet;
    mat[2][1] = (-a[0, 0] * c4 + a[0, 1] * c2 - a[0, 3] * c0) * invdet;
    mat[2][2] = ( a[3, 0] * s4 - a[3, 1] * s2 + a[3, 3] * s0) * invdet;
    mat[2][3] = (-a[2, 0] * s4 + a[2, 1] * s2 - a[2, 3] * s0) * invdet;

    mat[3][0] = (-a[1, 0] * c3 + a[1, 1] * c1 - a[1, 2] * c0) * invdet;
    mat[3][1] = ( a[0, 0] * c3 - a[0, 1] * c1 + a[0, 2] * c0) * invdet;
    mat[3][2] = (-a[3, 0] * s3 + a[3, 1] * s1 - a[3, 2] * s0) * invdet;
    mat[3][3] = ( a[2, 0] * s3 - a[2, 1] * s1 + a[2, 2] * s0) * invdet;

    return mat;
}

function vector_normalize(v) {
    // returns normalized vector for v
    var tmp = 0;
    var i;
    for (i=0; i<v.length; i++)
        tmp += v[i]*v[i];
    var norm = Math.sqrt(tmp);

    var normalized_v = [];
    var j;
    for (j=0; j<v.length; j++)
        normalized_v[j] = v[j]/norm;
    return normalized_v;
}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions
    var cross_product = [];
    cross_product[0] = a[1]*b[2] - a[2]*b[1];
    cross_product[1] = a[2]*b[0] - a[0]*b[2];
    cross_product[2] = a[0]*b[1] - a[1]*b[0];   
    return cross_product; 
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    var mat = [];
    
    mat[0] = [1,0,0,0];
    mat[1] = [0,1,0,0];
    mat[2] = [0,0,1,0];
    mat[3] = [0,0,0,1];
    return mat;
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    var mat = [];
    mat[0] = [1,0,0,tx];
    mat[1] = [0,1,0,ty];
    mat[2] = [0,0,1,tz];
    mat[3] = [0,0,0,1];
    return mat;
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var mat = [];
    mat[0] = [1,0,0,0];
    mat[1] = [0,Math.cos(angle),-Math.sin(angle),0];
    mat[2] = [0,Math.sin(angle),Math.cos(angle),0];
    mat[3] = [0,0,0,1];
    return mat;
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var mat = [];
    mat[0] = [Math.cos(angle),0,Math.sin(angle),0];
    mat[1] = [0,1,0,0];
    mat[2] = [-Math.sin(angle),0,Math.cos(angle),0];
    mat[3] = [0,0,0,1];
    return mat;
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var mat = [];
    mat[0] = [Math.cos(angle),-Math.sin(angle),0,0];
    mat[1] = [Math.sin(angle),Math.cos(angle),0,0];
    mat[2] = [0,0,1,0];
    mat[3] = [0,0,0,1];
    return mat;
}
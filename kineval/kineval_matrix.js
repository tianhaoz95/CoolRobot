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

function matrix_multiply(m1,m2) {
    var mat = [];
    var i,j,k;

    if (m1[0].length !== m2.length) {
        return false;
    }

    for (i=0;i<m1.length;i++) {
        mat[i] = [];
        for (j=0;j<m2[0].length;j++) {
            mat[i][j] = 0;
            for (k=0;k<m1[0].length;k++) {
                mat[i][j] += m1[i][k]*m2[k][j];
            }
        }
    }
    return mat;
}

function matrix_transpose(m) {
    var i;
    mat = [];
    for (i=0;i<m[0].length;i++) {
        mat[i] = [];
        for (j=0;j<m.length;j++) {
            mat[i][j] = Number(m[j][i]);
        }
    }
    return mat;
}

function vector_normalize(v) {
    var i;
    var sum = 0;
    for (i=0;i<v.length;i++) {
        sum += Number(v[i])*Number(v[i]);
    }
    var mag = Math.sqrt(sum);

    vec = [];
    for (i=0;i<v.length;i++) {
        if (typeof v[0][0] === 'undefined')
            vec[i] = v[i]/mag;
        else {
            vec[i] = [];
            vec[i][0] = v[i]/mag;
        }
    }
    return vec;
}

function vector_cross(a,b) {
    c = [];
    if (typeof a[0][0] === 'undefined') {
        c[0] = a[1]*b[2] - a[2]*b[1];
        c[1] = a[2]*b[0] - a[0]*b[2];
        c[2] = a[0]*b[1] - a[1]*b[0];
    }
    else {
        c[0] = [a[1]*b[2] - a[2]*b[1]];
        c[1] = [a[2]*b[0] - a[0]*b[2]];
        c[2] = [a[0]*b[1] - a[1]*b[0]];
    }
    return c;
}

function generate_identity() {
    var mat = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1] ];
    return mat;
}


function generate_translation_matrix(tx, ty, tz) {
    var mat = [
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1] ];
    return mat;
}

function generate_rotation_matrix_X(angle) {
    var mat = [
        [1, 0, 0, 0],
        [0, Math.cos(angle), -Math.sin(angle), 0],
        [0, Math.sin(angle), Math.cos(angle), 0],
        [0, 0, 0, 1] ];
    return mat;
}


function generate_rotation_matrix_Y(angle) {
    var mat = [
        [Math.cos(angle), 0, Math.sin(angle), 0],
        [0, 1, 0, 0],
        [-Math.sin(angle), 0, Math.cos(angle), 0],
        [0, 0, 0, 1] ];
    return mat;
}

function generate_rotation_matrix_Z(angle) {
    var mat = [
        [Math.cos(angle), -Math.sin(angle), 0, 0],
        [Math.sin(angle), Math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1] ];
    return mat;
}
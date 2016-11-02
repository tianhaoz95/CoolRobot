//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

    kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
        var q = {
          a : Math.cos(angle/2),
          b : axis[0] * Math.sin(angle/2),
          c : axis[1] * Math.sin(angle/2),
          d : axis[2] * Math.sin(angle/2),
        }
        return q;
    }

    kineval.quaternionNormalize = function quaternion_normalize(q1) {
        var q1_norm = Math.sqrt(q1.a*q1.a + q1.b*q1.b + q1.c*q1.c + q1.d*q1.d);
        var q = {
            a : q1.a/q1_norm,
            b : q1.b/q1_norm,
            c : q1.c/q1_norm,
            d : q1.d/q1_norm,
        };
        return q;
    }

    kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
        var q = {
          a : q1.a*a2.a - q1.b*a2.b - q1.c*a2.c - q1.d*a2.d,
          b : q1.a*a2.b + q1.b*a2.a + q1.c*a2.d - q1.d*a2.c,
          c : q1.a*a2.c - q1.b*a2.d + q1.c*a2.a + q1.d*a2.b,
          d : q1.a*a2.d + q1.b*a2.c - q1.c*a2.b + q1.d*a2.a,
        }
        return q;
    }

    kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
        var a = q.a; var b = q.b; var c = q.c; var d = q.d;
        var mat = [
            [(a*a)+(b*b)-(c*c)-(d*d),  2*(b*c)-2*(a*d),  2*(b*d)+2*(a*c),  0],
            [2*(b*c)+2*(a*d),  (a*a)-(b*b)+(c*c)-(d*d),  2*(c*d)-2*(a*b),  0],
            [2*(b*d)-2*(a*c),  2*(c*d)+2*(a*b),  (a*a)-(b*b)-(c*c)+(d*d),  0],
            [0,  0,  0,  1]
        ]
        return mat;
    }

fs = require('fs');
util = require('util');

// tempate stencil format
/* STENCIL START */ 
    // reference implementation
/* STENCIL REPLACE START
    // STENCIL: stencil replacement comment
STENCIL REPLACE */ 
/* STENCIL END */ 

//node stencilize.js pendularm1.html > pendularm1_stencil.html
//node ../pendularm/stencilize.js rrt_canvas.html > rrt_canvas_stencil.html
//node ../pendularm/stencilize.js kineval_collision.js > kineval_collision_stencil.js
//node ../pendularm/stencilize.js kineval_rrt_connect.js > kineval_rrt_connect_stencil.js
//node ../pendularm/stencilize.js kineval_inverse_kinematics.js > kineval_inverse_kinematics_stencil.js
//node ../pendularm/stencilize.js kineval_forward_kinematics.js > kineval_forward_kinematics_stencil.js
//node ../pendularm/stencilize.js kineval_matrix.js > kineval_matrix_stencil.js
//node ../pendularm/stencilize.js kineval_quaternion.js > kineval_quaternion_stencil.js
//node ../pendularm/stencilize.js kineval_robot_init_quaternion.js > kineval_robot_init_stencil.js

var i;

//filename = 'pendularm1.html';
filename = process.argv[2];
//console.log(JSON.stringify(process.argv));

var data = fs.readFileSync(filename, 'ascii');
var array = data.toString().split('\n');

in_stencil = false; 
in_replace = false; 

for (i=0;i<array.length;i++) {  // hacky first attempt

    cur_in_stencil = in_stencil;
    cur_in_replace = in_replace;

    regexres = /STENCIL ([A-Z]*) (.*)/g.exec(array[i]);
    if (regexres) {    
        //console.log(i + ": " + regexres[1] + " - " + regexres[2]);
        if (regexres[1] === 'START') in_stencil = true;
        if (regexres[1] === 'END') in_stencil = false;
        if (regexres[1] === 'REPLACE') in_replace = !in_replace;
    }

    if (!in_stencil&&!cur_in_stencil) console.log(array[i]); 
    if (in_stencil&&in_replace&&cur_in_replace) console.log(array[i]); 

}

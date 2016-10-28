
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

/* STENCIL START */ 
    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
/* STENCIL REPLACE START
    // STENCIL: see instructor for random time trial code
STENCIL REPLACE */ 
/* STENCIL END */ 
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

/* STENCIL START */ 
    var x = endeffector_joint;
    temp1_chain = [];
    var i = 0;
    var base_not_reached = true;

    // form chain of links from endeffector link to robot base
    while (base_not_reached) {
        temp1_chain[i] = x;
        if (robot.joints[x].parent !== robot.base) {
            x = robot.links[robot.joints[x].parent].parent; // move up kinematics to next joint
            i += 1;
        }
        else
            base_not_reached = false;
    }  

    // reverse indicies to remain consistent
    temp_chain = [];
    for (i=0;i<temp1_chain.length;i++) {
        temp_chain[temp1_chain.length-(i+1)] = temp1_chain[i];
    }

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local);
    
    // get target Cartesian position in the world
    target_world = matrix_multiply(generate_identity(),endeffector_target_world.position); // just a copy


    // build jacobian for rotational joints (Spong eq 4.81 4.82)
    //J = [[],[],[]]; // initialize Jacobian with 6 cols for Cartesian dofs
    J = [[],[],[],[],[],[]]; // initialize Jacobian with 6 cols for Cartesian dofs
    var zero_vec = [[0],[0],[0],[1]];

    for (i=0;i<temp_chain.length;i++) {

        // transform current joint origin into world frame
        joint_origin_world = matrix_multiply(robot.joints[temp_chain[i]].xform,zero_vec);  // at joint i

        // current joint axis in joint frame
        tempjvec = [
            [ Number(robot.joints[temp_chain[i]].axis[0]) ],
            [ Number(robot.joints[temp_chain[i]].axis[1]) ],
            [ Number(robot.joints[temp_chain[i]].axis[2]) ],
            [ 0 ]
        ];

        // rotate joint axis in world frame, remains centered on joint origin
        vec_joint_axis = matrix_multiply(robot.joints[temp_chain[i]].xform,tempjvec);

        // compute vector from joint origin to endeffector location
        vec_to_endeff = [
            [ endeffector_world[0][0]-joint_origin_world[0][0] ],
            [ endeffector_world[1][0]-joint_origin_world[1][0] ],
            [ endeffector_world[2][0]-joint_origin_world[2][0] ]
            ]; 
      
        // compute cross product for linear component of Jacobian column
        Jv = vector_cross(vec_joint_axis,vec_to_endeff);

        // Jacobian rows: Cartesian dofs  cols: joints
        J[0][i] = Jv[0];  // Jv : linear
        J[1][i] = Jv[1];
        J[2][i] = Jv[2];
        J[3][i] = vec_joint_axis[0][0]; // Jw : angular
        J[4][i] = vec_joint_axis[1][0]; 
        J[5][i] = vec_joint_axis[2][0]; 

        if (typeof robot.joints[temp_chain[i]].type !== 'undefined')
            if ((robot.joints[temp_chain[i]].type === 'prismatic')) {
                J[0][i] = vec_joint_axis[0][0]; // linear: joint axis
                J[1][i] = vec_joint_axis[1][0]; 
                J[2][i] = vec_joint_axis[2][0]; 
                J[3][i] = 0;  // angular : independent of orientation
                J[4][i] = 0;
                J[5][i] = 0;
            }
    }

    // convert endeffector rotation matrix to Euler angles
    temp_euler = new THREE.Euler(0,0,0,'XYZ');
    //temp_euler.setFromRotationMatrix(matrix_2Darray_to_threejs(robot.joints[endeffector_joint].xform,'XYZ'));
    temperot = matrix_multiply(generate_identity(),robot.joints[endeffector_joint].xform);
    temperot[0][3] = 0;  
    temperot[1][3] = 0;  
    temperot[2][3] = 0;  
    //temp_euler.setFromRotationMatrix(matrix_2Darray_to_threejs(temperot,'XYZ'));
    temp_euler.setFromRotationMatrix(matrix_2Darray_to_threejs(temperot,'ZYX'));

    // get delta vector from current to target endeffector pose
    if (kineval.params.ik_orientation_included)
        dx = [
            [ endeffector_target_world.position[0][0]-endeffector_world[0][0] ],
            [ endeffector_target_world.position[1][0]-endeffector_world[1][0] ],
            [ endeffector_target_world.position[2][0]-endeffector_world[2][0] ]
            ,[ endeffector_target_world.orientation[0]-temp_euler.x ]
            ,[ endeffector_target_world.orientation[1]-temp_euler.y ]
            ,[ endeffector_target_world.orientation[2]-temp_euler.z ]
        ]; 
   else
        dx = [
            [ endeffector_target_world.position[0][0]-endeffector_world[0][0] ],
            [ endeffector_target_world.position[1][0]-endeffector_world[1][0] ],
            [ endeffector_target_world.position[2][0]-endeffector_world[2][0] ]
            ,[0], [0], [0]  // assume desired orientation is irrelavent
        ]; 
 
    // get change in joint angles by multiplying Cartesian delta vector by Jacobian 
    // resolved-rate inverse kinematics with geometric jacobian
    // dq = J^T * dx

    //console.log(JSON.stringify(matrix_pseudoinverse(J)));
    //console.log(JSON.stringify(J));

    //console.log(matrix_multiply(matrix_transpose(J),J));
    if (kineval.params.ik_pseudoinverse) {
        dq = matrix_multiply(matrix_pseudoinverse(J),dx);
    }
    else {
        //console.log(J);
        //console.log(dx);
        dq = matrix_multiply(matrix_transpose(J),dx);
    }

    ik_scale = kineval.params.ik_steplength;

    // update controls by add dq to joint angles
    for (i=0;i<temp_chain.length;i++) {
        robot.joints[temp_chain[i]].control += ik_scale*dq[i];
    }
/* STENCIL REPLACE START
    // STENCIL: implement inverse kinematics iteration
STENCIL REPLACE */ 
/* STENCIL END */ 
}



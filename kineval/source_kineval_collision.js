
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
/* STENCIL START */ 
    return robot_collision_forward_kinematics(q);
/* STENCIL REPLACE START
    // STENCIL: implement forward kinematics for collision detection
    //return robot_collision_forward_kinematics(q);
STENCIL REPLACE */ 
/* STENCIL END */ 

}

/* STENCIL START */ 
function robot_collision_forward_kinematics (q) { 

    // transform robot base into the global world coordinates
    var mstack = matrix_multiply(generate_translation_matrix(q[0],q[1],q[2]),matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(q[5]),generate_rotation_matrix_Y(q[4])),generate_rotation_matrix_X(q[3])));

    // handle conversion to ROS coordinate convention
    if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        return traverse_collision_forward_kinematics_link(robot.links[robot.base],matrix_multiply(mstack,offset_xform),q);
    }
    else // recurse kinematics, testing collisions at each link
        return traverse_collision_forward_kinematics_link(robot.links[robot.base],mstack,q);

}



function traverse_collision_forward_kinematics_joint(joint,mstack,q) {

    var mstack_top = matrix_multiply(mstack,generate_identity());

    // compute matrix transform origin of joint in the local space of the parent link
    var local_xform = matrix_multiply(generate_translation_matrix(joint.origin.xyz[0],joint.origin.xyz[1],joint.origin.xyz[2]),matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(joint.origin.rpy[2]),generate_rotation_matrix_Y(joint.origin.rpy[1])),generate_rotation_matrix_X(joint.origin.rpy[0])));

    // push local transform to origin to the top of the matrix stack
    var mstack_origin_top = matrix_multiply(mstack,local_xform)

    // transform motor rotation by quaternion or prismatic displacement
    var joint_local_xform = computeLocalJointTransform(joint,q[q_names[joint.name]]); 

    // push joint angle transform to the top of the matrix stack
    var mstack_top = matrix_multiply(mstack_origin_top,joint_local_xform); 

    // recursively traverse child link with the current_xform being top of matrix stack 
    return traverse_collision_forward_kinematics_link(robot.links[joint.child],mstack_top,q);

}


function computeLocalJointTransform(joint,joint_state) {

    var local_joint_xform = [];

    // compute rotation matrix for current position of joint
    if (typeof joint.type === 'undefined') {
        // assume joints are continuous by default
        current_quat = kineval.quaternionNormalize(kineval.quaternionFromAxisAngle([joint.axis[0],joint.axis[1],joint.axis[2]],joint_state));
        local_joint_xform = kineval.quaternionToRotationMatrix(current_quat);
    }
    else if ((joint.type === 'revolute')||(joint.type === 'continuous')) {
        current_quat = kineval.quaternionNormalize(kineval.quaternionFromAxisAngle([joint.axis[0],joint.axis[1],joint.axis[2]],joint_state));
        local_joint_xform = kineval.quaternionToRotationMatrix(current_quat);
    }
    else if (joint.type === 'prismatic') {
        local_joint_xform = generate_translation_matrix(
            joint_state*joint.axis[0],
            joint_state*joint.axis[1],
            joint_state*joint.axis[2]
        );
    }
    else local_joint_xform = generate_identity();

    return local_joint_xform;
}
/* STENCIL REPLACE START

STENCIL REPLACE */ 
/* STENCIL END */ 

function traverse_collision_forward_kinematics_link(link,mstack,q) {

    // test collision by transforming obstacles in world to link space
/*
    mstack_inv = matrix_invert_affine(mstack);
*/
    mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}



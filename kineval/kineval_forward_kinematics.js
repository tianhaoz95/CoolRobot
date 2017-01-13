
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

/* STENCIL START */ 
    kineval.buildFKTransforms();
/* STENCIL REPLACE START
    // STENCIL: implement kineval.buildFKTransforms();
STENCIL REPLACE */ 
/* STENCIL END */ 

}

/* STENCIL START */ 
kineval.buildFKTransforms = function buildFKTransforms() {

    // transform robot base into the global world coordinates
    robot.origin.xform = matrix_multiply(generate_translation_matrix(robot.origin.xyz[0],robot.origin.xyz[1],robot.origin.xyz[2]),matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(robot.origin.rpy[2]),generate_rotation_matrix_Y(robot.origin.rpy[1])),generate_rotation_matrix_X(robot.origin.rpy[0])));

    // compute robot base heading for user input of forward/backward commands
    var heading_local = [[0],[0],[1],[1]];
    heading_world = matrix_multiply(robot.origin.xform,heading_local);
    robot_heading = heading_world;

    // compute robot base heading for user input of strafing commands
    var lateral_local = [[1],[0],[0],[1]];
    lateral_world = matrix_multiply(robot.origin.xform,lateral_local);
    robot_lateral = lateral_world;

    // draw robot by recursively traversing coordinates of robot kinematics, starting from base link
    // recursion alternates between links and joints

    // handle conversion from ROS coordinate convention
    if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        kineval.traverseFKLink(robot.links[robot.base],matrix_multiply(robot.origin.xform,offset_xform));
    }
    else {
        kineval.traverseFKLink(robot.links[robot.base],robot.origin.xform);
    }
}

kineval.traverseFKLink = function traverse_forward_kinematics_link(link,parent_xform) {
   
    // copy joint transform to matrix stack transform
    // KE T: crude copy using mat mult
    var current_xform = matrix_multiply(parent_xform,generate_identity());

    // copy joint transform to matrix stack transform; crude copy using mat mult
    //link.xform = current_xform;
    link.xform = matrix_multiply(current_xform,generate_identity());

    // recursively traverse each child joint with current_xform as the top of the matrix stack
    var i;
    if (typeof link.children !== 'undefined') { // return if there are no children
        for (i=0;i<link.children.length;i++) {
            // KE T: crude copy using mat mult
            kineval.traverseFKJoint(robot.joints[link.children[i]],current_xform);
        }
    }

    // pop from matrix stack is implicit in returning from this function
}


kineval.traverseFKJoint = function traverse_forward_kinematics_joint(joint,parent_xform) {

    // copy current matrix stack transform to local variable; crude copy using mat mult
    var current_xform = matrix_multiply(parent_xform,generate_identity());

    // compute matrix transform origin of joint in the local space of the parent link
    local_origin_xform = matrix_multiply(generate_translation_matrix(joint.origin.xyz[0],joint.origin.xyz[1],joint.origin.xyz[2]),matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(joint.origin.rpy[2]),generate_rotation_matrix_Y(joint.origin.rpy[1])),generate_rotation_matrix_X(joint.origin.rpy[0])));

    // compute rotation matrix for current position of joint
    if (typeof joint.type === 'undefined') {
        // assume joints are continuous by default
        current_quat = kineval.quaternionNormalize(kineval.quaternionFromAxisAngle([joint.axis[0],joint.axis[1],joint.axis[2]],joint.angle));
        //current_quat = kineval.quaternionFromAxisAngle([joint.axis[0]*1.1,joint.axis[1],joint.axis[2]*2],joint.angle);
        local_joint_xform = kineval.quaternionToRotationMatrix(current_quat);
    }
    else if ((joint.type === 'revolute')||(joint.type === 'continuous')) {
        current_quat = kineval.quaternionNormalize(kineval.quaternionFromAxisAngle([joint.axis[0],joint.axis[1],joint.axis[2]],joint.angle));
        local_joint_xform = kineval.quaternionToRotationMatrix(current_quat);
    }
    else if (joint.type === 'prismatic') {
        local_joint_xform = generate_translation_matrix(
            joint.angle*joint.axis[0],
            joint.angle*joint.axis[1],
            joint.angle*joint.axis[2]
        );
    }
    else local_joint_xform = generate_identity();

    // compute xform for joint
    joint.xform = matrix_multiply(matrix_multiply(current_xform,local_origin_xform),local_joint_xform); 

    // copy joint transform to matrix stack transform; crude copy using mat mult
    //current_xform = joint.origin.xform; 
    //var current_xform = matrix_multiply(joint.xform,generate_identity()); 

    // recursively traverse child link with the current_xform being top of matrix stack 
    // KE T: crude copy using mat mult
    kineval.traverseFKLink(robot.links[joint.child],matrix_multiply(joint.xform,generate_identity()));

    // pop from matrix stack is implicit in returning from this function
}

/* STENCIL REPLACE START
    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
STENCIL REPLACE */ 
/* STENCIL END */ 

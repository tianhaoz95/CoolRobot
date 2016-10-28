
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

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();
}

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

kineval.buildFKTransforms = function build_FK_Transforms() {
    var trans_matrix = generate_translation_matrix(robot.origin.xyz[0],robot.origin.xyz[1], robot.origin.xyz[2]);
    var rotX = generate_rotation_matrix_X(robot.origin.rpy[0]);
    var rotY = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    var rotZ = generate_rotation_matrix_Z(robot.origin.rpy[2]);
    robot.origin.xform = matrix_multiply(trans_matrix,matrix_multiply(matrix_multiply(rotZ, rotY),rotX));
    heading_world = matrix_multiply(robot.origin.xform,[[0],[0],[1],[1]]);
    robot_heading = heading_world;
    lateral_world = matrix_multiply(robot.origin.xform,[[1],[0],[0],[1]]);
    robot_lateral = lateral_world;
    kineval.traverseFKBase();
}

kineval.traverseFKBase = function traverse_FK_Base() {
    if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        kineval.traverseFKLink(robot.links[robot.base],matrix_multiply(robot.origin.xform,offset_xform));
    }
    else {
        kineval.traverseFKLink(robot.links[robot.base],robot.origin.xform);
    }
}

kineval.traverseFKLink = function traverse_FK_Link(link,parent_xform) {
    var current_xform = matrix_copy(parent_xform);
    link.xform = matrix_copy(current_xform);
    var i;
    if (typeof link.children !== 'undefined') {
        for (i=0;i<link.children.length;i++) {
            kineval.traverseFKJoint(robot.joints[link.children[i]],current_xform);
        }
    }
}

kineval.traverseFKJoint = function traverse_forward_kinematics_joint(joint,parent_xform) {
    var current_xform = matrix_copy(parent_xform);
    joint.xform = matrix_copy(current_xform);
    var trans = generate_translation_matrix(joint.origin.xyz[0],joint.origin.xyz[1], joint.origin.xyz[2]);
    var rotZ = generate_rotation_matrix_Z(joint.origin.rpy[2]);
    var rotX = generate_rotation_matrix_X(joint.origin.rpy[0]);
    var rotY = generate_rotation_matrix_Y(joint.origin.rpy[1]);
    local_origin_xform = matrix_multiply(trans,matrix_multiply(matrix_multiply(rotZ, rotY),rotX));
    joint.xform = matrix_multiply(current_xform,local_origin_xform);
    kineval.traverseFKLink(robot.links[joint.child],matrix_copy(joint.xform));
}
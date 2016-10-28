
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

/* STENCIL START */ 
    var setpoint_reached = true;
    for (x in robot.joints) {
        if (Math.abs(robot.joints[x].angle-kineval.params.setpoint_target[x])>0.01)
            setpoint_reached = false;
    }

    if (setpoint_reached) {
        kineval.params.dance_pose_index = (kineval.params.dance_pose_index+1)%kineval.params.dance_sequence_index.length;
    }
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_sequence_index[kineval.params.dance_pose_index]][x];
    }
/* STENCIL REPLACE START
    // STENCIL: implement FSM to cycle through dance pose setpoints
STENCIL REPLACE */ 
/* STENCIL END */ 
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

/* STENCIL START */ 
    var x;  
    for (x in robot.joints) {

        robot.joints[x].servo.p_desired = kineval.params.setpoint_target[x] 

        // hacky check for angular discontinuity
        if ((robot.joints[x].servo.p_desired < 0.4) && (robot.joints[x].angle > 2*Math.PI-0.4))
            robot.joints[x].servo.p_desired += 2*Math.PI;
        if ((robot.joints[x].angle < 0.4) && (robot.joints[x].servo.p_desired > 2*Math.PI-0.4))
            robot.joints[x].servo.p_desired -= 2*Math.PI;

        // proportional term
        robot.joints[x].control += robot.joints[x].servo.p_gain * (robot.joints[x].servo.p_desired - robot.joints[x].angle);

        // derivative term (!!commented out but angle_dot will be available with dyanmics)
        //robot.joints[x].control += robot.joints[x].servo.d_gain * (robot.joints[x].servo.d_desired - robot.joints[x].angle_dot);
    }
/* STENCIL REPLACE START
    // STENCIL: implement P servo controller over joints
STENCIL REPLACE */ 
/* STENCIL END */ 
}


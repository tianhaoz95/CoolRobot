/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics
     in HTML5/JavaScript and threejs

     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

*/

kineval.initRobot = function initRobot() {

    // ASSUME: robot kinematics are described separate js file (eg., "robot_urdf_example.js")

    // initialize and create threejs mesh objects for robot links
    kineval.initRobotLinks();

    // initialize robot joints and create threejs mesh objects for robot joints and form kinematic hiearchy
    kineval.initRobotJoints();

    // initialize robot collision state
    robot.collision = false;

}

kineval.initRobotLinks = function initRobotLinks() {

    for (x in robot.links) {
        robot.links[x].name = x;
    }

    // initialize controls for robot base link
    robot.control = {xyz: [0,0,0], rpy:[0,0,0]};
}

kineval.initRobotJoints = function initRobotJoints() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert three js scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not three js

    var x,tempmat;

    for (x in robot.joints) {

        // give the joint its name as an id
        robot.joints[x].name = x;

        // initialize joint angle value and control input value
        robot.joints[x].angle = 0;
        robot.joints[x].control = 0;
        robot.joints[x].servo = {};
    // STENCIL: set appropriate servo gains for arm setpoint control
        robot.joints[x].servo.p_gain = 0.1;
        robot.joints[x].servo.p_desired = 0;
        robot.joints[x].servo.d_gain = 0.01;

    // STENCIL: complete kinematic hierarchy of robot for convenience.
    //   robot description only specifies parent and child links for joints.
    //   additionally specify parent and child joints for each link

        robot.links[robot.joints[x].child].parent = x;

        if (typeof robot.links[robot.joints[x].parent].children === 'undefined') {
            robot.links[robot.joints[x].parent].children = [];
        }
        robot.links[robot.joints[x].parent].children.push(x);

    }
    /*
    for (x in robot.links) {
        if (!(typeof robot.links[x].child === 'undefined') && !(typeof robot.links[x].parent === 'undefined')) {
            robot.joints[robot.links[x].child].parent = x;
        }

        if (!(typeof robot.links[x].child === 'undefined') && !(typeof robot.links[x].parent === 'undefined')) {
            if (typeof robot.joints[robot.links[x].parent].children === 'undefined') {
                robot.joints[robot.links[x].parent].children = [];
            }
            robot.joints[robot.links[x].parent].children.push(x);
        }
    }
    */

}


/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


/* STENCIL START */ 
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        console.log(x);
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        //q_start_config = q_start_config.concat(robot.joints[x].angle);
        
        // normalize joint state and enforce joint limits
        if (typeof robot.joints[x].type !== 'undefined')
            if ((robot.joints[x].type === 'prismatic')||(robot.joints[x].type === 'revolute'))
                q_start_config = q_start_config.concat(normalize_joint_limit(robot.joints[x].angle,robot.joints[x].limit.lower,robot.joints[x].limit.upper));
            else 
                q_start_config = q_start_config.concat(normalize_angle_0_2pi(robot.joints[x].angle));
        else 
            q_start_config = q_start_config.concat(normalize_angle_0_2pi(robot.joints[x].angle));

    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // initialize tree from start configuration
    q_init = q_start_config;
    T_a = tree_init(q_init);

    // initialize tree from goal configuration
    q_goal = q_goal_config;
    T_b = tree_init(q_goal);

    // keep track of which tree is connected to the start configuration
    a_start_tree = true;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    verbose_console_rrt = false;
    if (verbose_console_rrt)
        console.log("planner initialized");
}
/* STENCIL REPLACE START
    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}
STENCIL REPLACE */ 
/* STENCIL END */ 



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

/* STENCIL START */ 

        var q_rand = random_config(1000,T_a.vertices[0].vertex.length);

        if (!(rrt_alg)) {

            // basic rrt
            if (rrt_iter_count < 1000) {
                extend_result = rrt_extend(T_a,q_rand);
                // KE 2 : should check for goal and generate path back to start
            }

        }
        else {

            // rrt-connect
            extend_result = rrt_extend(T_a,q_rand);
            if ((extend_result !=="trapped")) {
                connect_result = rrt_connect(T_b,T_a.vertices[T_a.newest].vertex);
                if (connect_result==="reached") {   //q_new coming from inside extend, maybe global

                    // if trees connect, planner is done
                    if (verbose_console_rrt)
                        console.log("done");
                    rrt_iterate = false;

                    // draw paths for Tree A and Tree B to respective goal or start, assume connection between trees can be traversed
                    path_a = find_path(T_a,T_a.vertices[0],T_a.vertices[T_a.newest]);
                    path_b = find_path(T_b,T_b.vertices[0],T_b.vertices[T_b.newest]);

                    // order configurations of robot path for navigation
                    kineval.motion_plan = [];
                    var robot_path_idx = 0;
                    kineval.motion_plan_traversal_index = 0;
                    if (a_start_tree) { // if Tree A is connected to start config
                        for (i=path_a.length-1;i>=0;i--) {
                            kineval.motion_plan[robot_path_idx] = path_a[i];
                            robot_path_idx += 1;
                        }
                        for (i=0;i<path_b.length;i++) {
                            kineval.motion_plan[robot_path_idx] = path_b[i];
                            robot_path_idx += 1;
                        }
                    }
                    else { // if Tree B is connected to start config
                        for (i=path_b.length-1;i>=0;i--) {
                            kineval.motion_plan[robot_path_idx] = path_b[i];
                            robot_path_idx += 1;
                        }
                        for (i=0;i<path_a.length;i++) {
                            kineval.motion_plan[robot_path_idx] = path_a[i];
                            robot_path_idx += 1;
                        }
                    }
 
                    //if (verbose_console_rrt)
                    //    console.log("rrt_iterate: " +rrt_iterate );

                    // return true to indicate path was found
                    return connect_result;
                }
            }

            // swap trees to extend and connect from other side in next rrt iteration
            var temp = T_a;
            T_a = T_b;
            T_b = temp;
            a_start_tree = !a_start_tree;

            rrt_iter_count++; 
        }
       
        if (1 == 1) { // normalize all angles
        for (i=0;i<T_a.vertices.length;i++) {
            for (j=3;j<T_a.vertices[i].length;j++) {
                // normalize joint state and enforce joint limits for non-base
                if (j > 5) {
                    if (typeof robot.joints[q_index[j]].type !== 'undefined')
                        if ((robot.joints[q_index[j]].type === 'prismatic')||(robot.joints[q_index[j]].type === 'revolute'))
                            T_a.vertices[i].vertex[j] = normalize_joint_limit(T_a.vertices[i].vertex[j],robot.joints[q_index[j]].limit.lower,robot.joints[q_index[j]].limit.upper);
                        else 
                            T_a.vertices[i].vertex[j] = normalize_angle_0_2pi(T_a.vertices[i].vertex[j]);
                    else 
                            T_a.vertices[i].vertex[j] = normalize_angle_0_2pi(T_a.vertices[i].vertex[j]);
                }
                else
                    T_a.vertices[i].vertex[j] = normalize_angle_0_2pi(T_a.vertices[i].vertex[j]);
            }
        }

        for (i=0;i<T_b.vertices.length;i++) {
            for (j=3;j<T_b.vertices[i].length;j++) {
                // normalize joint state and enforce joint limits for non-base
                if (j > 5) {
                    if (typeof robot.joints[q_index[j]].type !== 'undefined')
                        if ((robot.joints[q_index[j]].type === 'prismatic')||(robot.joints[q_index[j]].type === 'revolute'))
                            T_b.vertices[i].vertex[j] = normalize_joint_limit(T_b.vertices[i].vertex[j],robot.joints[q_index[j]].limit.lower,robot.joints[q_index[j]].limit.upper);
                        else 
                            T_b.vertices[i].vertex[j] = normalize_angle_0_2pi(T_b.vertices[i].vertex[j]);
                    else 
                            T_b.vertices[i].vertex[j] = normalize_angle_0_2pi(T_b.vertices[i].vertex[j]);
                }
                else
                    T_b.vertices[i].vertex[j] = normalize_angle_0_2pi(T_b.vertices[i].vertex[j]);
            }

        }
        } //normalize all angles
        
/* STENCIL REPLACE START
    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
STENCIL REPLACE */ 
/* STENCIL END */ 
    }

/* STENCIL START */ 
    // return path not currently found
    return false;
/* STENCIL REPLACE START
STENCIL REPLACE */ 
/* STENCIL END */ 
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {

/* STENCIL START */ 
    // restrict joint angles to [0..2*PI]
    var i;
    var global_dof_end_index = 5;
    for (i=0;i<q.length;i++) {
        if (!(i <= global_dof_end_index)) {
            if (typeof robot.joints[q_index[i]].type !== 'undefined')
                if ((robot.joints[q_index[i]].type === 'continuous')&&((q[i] < 0)||(q[i] > 2*Math.PI)))
                    console.warn("kineval: tree_add_vertex: joint angle out of range");
                else if ( ((robot.joints[q_index[i]].type === 'prismatic')||(robot.joints[q_index[i]].type === 'revolute'))&& ((q[i] < robot.joints[q_index[i]].limit.lower)||(q[i] > robot.joints[q_index[i]].limit.upper))) {
                    console.warn("kineval: tree_add_vertex: normalizing joint state out of range " + q_index[i] + " " + q[i]);
                    q[i] = normalize_joint_limit(q[i],robot.joints[q_index[i]].limit.lower,robot.joints[q_index[i]].limit.upper);
                }
         else if ((q[i] < 0)||(q[i] > 2*Math.PI)) 
             console.warn("kineval: tree_add_vertex: joint angle out of range");
        }
    }
/* STENCIL REPLACE START
STENCIL REPLACE */ 
/* STENCIL END */ 

    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


/* STENCIL START */ 
function random_config(scale, length) {
    // generate random configuration along each dimension of C-space
    q_rand = [];
    var i;
    for (i=0;i<length;i++) {
        // add constraints for robot base to stay in x-z plane
        if ((i==1) || (i==3) || (i==5))
            q_rand.push(0);
        else if (i==0)
            q_rand.push(((robot_boundary[1][0]+5)-(robot_boundary[0][0]-5))*(Math.random())+(robot_boundary[0][0]-5));
        else if (i==2)
            q_rand.push(((robot_boundary[1][2]+5)-(robot_boundary[0][2]-5))*(Math.random())+(robot_boundary[0][2]-5));
        else if (i<6)
            q_rand.push(Math.random()*2*Math.PI);
        else if (typeof robot.joints[q_index[i]].type !== 'undefined')
            if ((robot.joints[q_index[i]].type === 'prismatic')||(robot.joints[q_index[i]].type === 'revolute'))
                q_rand.push(Math.random()*((robot.joints[q_index[i]].limit.upper+1)-(robot.joints[q_index[i]].limit.lower-1))+(robot.joints[q_index[i]].limit.lower-1));
            else
                q_rand.push(Math.random()*2*Math.PI);
        else
            q_rand.push(Math.random()*2*Math.PI);

    }
    return q_rand;
}

function compute_configuration_distance(qd1,qd2) {
    var i;
    var dist = 0;
    var global_dof_end_index = 2;
    for (i=0;i<qd1.length;i++) {
        if (i <= global_dof_end_index)
            dist += (qd1[i]-qd2[i])*(qd1[i]-qd2[i]);
        else 
            dist += 1.0*(qd1[i]-qd2[i])*(qd1[i]-qd2[i]);
    }
    return Math.sqrt(dist);
}

function normalize_angle_0_2pi(angle) {
    
    if (angle > 2*Math.PI) 
        return angle%(2*Math.PI);
    else if (angle < 0)
        return (2*Math.PI)+(angle%(2*Math.PI));
    else return angle;
}

function normalize_joint_limit(value,lower,upper) {
    return Math.min(Math.max(value,lower),upper);
}

function nearest_neighbor(q, tree) {

    // find the nearest neighbor to configuration q in tree

    var i, j, min_dist, min_idx;
    min_idx = 0;
    var dist = 0;

    //console.log("compute_configuration_distance 0");
    // compute the distance between q and first vertex in tree, serves as initial minimum distance
    dist = compute_configuration_distance(q,tree.vertices[0].vertex);

    min_dist = dist;
    
    // iterate over all vertices in tree to find min distance to q
    for (j=1;j<tree.vertices.length;j++) {

        // compute distance
        dist = compute_configuration_distance(q,tree.vertices[j].vertex);

        // update minimum distance, if less than current min distance
        if (dist < min_dist) {
            min_idx = j;
            min_dist = dist;
        }
    }

    // return index and configuration of nearest neighbor
    var q_near = {}; 
    q_near.index = min_idx; 
    q_near.config = tree.vertices[min_idx].vertex; 
    return q_near;
}


function find_path(T,q_start,q_end) {
    // assume T[0] is goal and T.newest is start

    // initialize visit array for depth-first traversal of tree
    var i; 
    visited = new Array(T.length);
    for (i=0;i<T.length;i++) {
        T.vertices[i].visited = false;
    }

    // perform depth-first traversal of tree
    // (is T_a a mistake? seems to work regardless)
    //one_path = path_dfs(T_a,q_start,q_end);
    one_path = path_dfs(T,q_start,q_end);

    // highlight found path by coloring red base origin indicators of path vertices
    for (i=0;i<one_path.length;i++) {
        one_path[i].geom.material.color = {r:1,g:0,b:0};
    }

    return one_path;

}

function path_dfs(T,v,q_goal) {

    // recursive depth first traversal of tree to find path to a specific vertex

    var i;
    var return_val;

    // if goal vertex found, return an array initialized with the vertex
    if (v === q_goal) {
        return [v];
    } 

    // label vertex as visited
    v.visited = true;

    // traverse each adjacent vertex that has not been visited
    for (i=0;i<v.edges.length;i++) {
        if (!v.edges[i].visited) {
            return_val = path_dfs(T,v.edges[i],q_goal);

            // if the goal vertex was found through an adjacent vertex, return an rray that concatenate this vertex to the array returned by the adjacent vertex
            if (return_val) {
                return return_val.concat(v);
            }
        } 
    }

    // if path was not found through this vertex, return false
    return false;
}

function new_config(q_end, q_start, eps) {
    // generate new configuration from q_start towards q_end 

    var q_return = [];
    var q_disp_mag = 0;
    var q_disp = [];
    var i;

    // compute vector from start to end vertex and its magnitude
    for (i=0;i<q_end.length;i++) {
        q_disp[i] = (q_end[i]-q_start[i]); 
        q_disp_mag += (q_end[i]-q_start[i])*(q_end[i]-q_start[i]);
    }
    q_disp_mag = Math.sqrt(q_disp_mag);

    // generate new configuration with unit step from start to end scaled by eps
    for (i=0;i<q_end.length;i++)
        q_return[i] = q_start[i] + eps*q_disp[i]/q_disp_mag; 

    // non-normalized step
    //for (i=0;i<q_end.length;i++)
    //    q_return[i] = q_start[i] + eps*(q_end[i]-q_start[i]); 

    // return generated configuration and whether it is in collision 
    return_config = {}; 
    return_config.q = q_return; 
    return_config.collision = kineval.poseIsCollision(q_return); 
    return return_config;
}

function rrt_extend(tree, q) {

    // find nearest neighbor of q on tree 
    var q_near_obj = nearest_neighbor(q,tree);
    var q_near_idx = q_near_obj.index;
    var q_near = q_near_obj.config;

    // length of step for exploration
    //var eps = 0.3;
    var eps = 0.5;
    //var eps = 0.9;

    // generate new configuration on tree in direction of q
    var generated_config = new_config(q,q_near,eps);

    // if generated configuration not in collision, add it to tree
    if (!generated_config.collision) {

        var q_new = generated_config.q; 

        // add q_new to tree
        tree_add_vertex(tree,q_new);
        tree_add_edge(tree,tree.vertices.length-1,q_near_idx);

        // check for "reached" condition, where q_new on tree is within eps of q
        dist = compute_configuration_distance(q_new,q);
        if (dist < eps) {
            return "reached"; // if new config is valid and reaches tree
        }
        else
        {
            return "advanced"; // if new config is valid, but does not reach tree
        }
    }

    return "trapped";  // if collision detected
}




function rrt_connect(tree, q) {

    // just an initialization, for while loop instead of repeat loop
    extend_result = "advanced";

    // build to rrt while it successfully advances to other tree without collosion
    while (extend_result === "advanced") {
        extend_result = rrt_extend(tree, q);
        if (verbose_console_rrt)
            console.log(tree.newest);
    }
    if (verbose_console_rrt)
        console.log(extend_result);
    return extend_result;
}

/* STENCIL REPLACE START
    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs
STENCIL REPLACE */ 

/* STENCIL END */ 










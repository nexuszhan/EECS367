
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
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
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
    
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    //swap = 0;
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) 
    {
        cur_time = Date.now();

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

        q_rand = randomConfig();

        var nearest_index = findNearest(T_a, q_rand);
        var q_near = T_a.vertices[nearest_index].vertex;

        var q_new = newConfig(q_rand, q_near);

        var state = "trapped";
        if (!kineval.poseIsCollision(q_new))
        {
            tree_add_vertex(T_a, q_new);
            tree_add_edge(T_a, nearest_index, T_a.newest);

            T_a.vertices[T_a.newest].parent = nearest_index;

            if (checkReach(q_rand, q_new))
                state = "reached";
            else
                state = "advanced";
        }

        if (!(state === "trapped"))
        {
            do
            {
                nearest_index = findNearest(T_b, q_new);
                q_near = T_b.vertices[nearest_index].vertex;

                var q_new2 = newConfig(q_new, q_near);

                state = "trapped";
                if (!kineval.poseIsCollision(q_new2))
                {
                    tree_add_vertex(T_b, q_new2);
                    tree_add_edge(T_b, nearest_index, T_b.newest);

                    T_b.vertices[T_b.newest].parent = nearest_index;

                    if (checkReach(q_new, q_new2))
                        state = "reached";
                    else
                        state = "advanced";
                    q_new = q_new2;
                }
            } while(state == "advanced") 

            if (state === "reached")
            {
                if (Math.abs((rrt_iter_count % 2) - 1) < 0.1)
                {
                    var tmp = Object.assign({}, T_a);
                    Object.assign(T_a, T_b);
                    Object.assign(T_b, tmp);
                }
                var cur_node = Object.assign({}, T_a.vertices[T_a.newest]);
                T_a.vertices[T_a.newest].geom.material.color = {r:1,g:0,b:0};
                T_b.vertices[T_b.newest].geom.material.color = {r:1,g:0,b:0};
                kineval.motion_plan = [T_a.vertices[T_a.newest], T_b.vertices[T_b.newest]];
                
                while (typeof cur_node.parent != "undefined")
                {
                    T_a.vertices[cur_node.parent].geom.material.color = {r:1,g:0,b:0};
                    kineval.motion_plan.unshift(T_a.vertices[cur_node.parent]);
                    cur_node = Object.assign({}, T_a.vertices[cur_node.parent]);
                }

                cur_node = Object.assign({}, T_b.vertices[T_b.newest]);
                while (typeof cur_node.parent != "undefined")
                {
                    T_b.vertices[cur_node.parent].geom.material.color = {r:1,g:0,b:0};
                    kineval.motion_plan.push(T_b.vertices[cur_node.parent]);
                    cur_node = Object.assign({}, T_b.vertices[cur_node.parent]);
                }
                
                rrt_iterate = false;
                return "reached";
            }
        }

        var tmp = Object.assign({}, T_a);
        Object.assign(T_a, T_b);
        Object.assign(T_b, tmp);

        rrt_iter_count++;

        /* var nearest_index = findNearest(T_a, q_rand);
        var q_near = T_a.vertices[nearest_index].vertex;

        var q_new = newConfig(q_rand, q_near);

        if (!kineval.poseIsCollision(q_new))
        {
            tree_add_vertex(T_a, q_new);
            tree_add_edge(T_a, nearest_index, T_a.newest);

            T_a.vertices[T_a.newest].parent = nearest_index;

            var nearest_index2 = findNearest(T_b, q_new);
            var q_near2 = T_b.vertices[nearest_index2].vertex;

            var q_new2 = newConfig(q_new, q_near2);

            if (!kineval.poseIsCollision(q_new2))
            {
                tree_add_vertex(T_b, q_new2);
                tree_add_edge(T_b, nearest_index2, T_b.newest);

                T_b.vertices[T_b.newest].parent = nearest_index2;

                do
                {
                    var q_new3 = newConfig(q_new, q_new2);
                    if (!kineval.poseIsCollision(q_new3))
                    {
                        tree_add_vertex(T_b, q_new3);
                        tree_add_edge(T_b, T_b.newest-1, T_b.newest);

                        T_b.vertices[T_b.newest].parent = T_b.newest-1;

                        q_new2 = q_new3;
                    }
                    else
                        break;
                } while (!checkReach(q_new, q_new2))
            }

            if (checkReach(q_new, q_new2))
            {
                if (Math.abs((swap % 2) - 1) < 0.1)
                {
                    var tmp = Object.assign({}, T_a);
                    Object.assign(T_a, T_b);
                    Object.assign(T_b, tmp);
                }
                var cur_node = Object.assign({}, T_a.vertices[T_a.newest]);
                T_a.vertices[T_a.newest].geom.material.color = {r:1,g:0,b:0};
                T_b.vertices[T_b.newest].geom.material.color = {r:1,g:0,b:0};
                kineval.motion_plan = [T_a.vertices[T_a.newest], T_b.vertices[T_b.newest]];
                
                while (typeof cur_node.parent != "undefined")
                {
                    T_a.vertices[cur_node.parent].geom.material.color = {r:1,g:0,b:0};
                    kineval.motion_plan.unshift(T_a.vertices[cur_node.parent]);
                    cur_node = Object.assign({}, T_a.vertices[cur_node.parent]);
                }

                cur_node = Object.assign({}, T_b.vertices[T_b.newest]);
                while (typeof cur_node.parent != "undefined")
                {
                    T_b.vertices[cur_node.parent].geom.material.color = {r:1,g:0,b:0};
                    kineval.motion_plan.push(T_b.vertices[cur_node.parent]);
                    cur_node = Object.assign({}, T_b.vertices[cur_node.parent]);
                }
                
                rrt_iterate = false;
                return "reached";
            }
        }

        if (T_b.vertices.length < T_a.vertices.length)
        {
            console.log(swap);
            var tmp = Object.assign({}, T_a);
            Object.assign(T_a, T_b);
            Object.assign(T_b, tmp);
            swap++;
            console.log(swap);
        }
        rrt_iter_count++;
        
        return "extended"; */
    }
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


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function randomConfig()
{
    var origin_xMin = robot_boundary[0][0]; 
    var origin_xMax = robot_boundary[1][0]; 
    var origin_zMin = robot_boundary[0][2]; 
    var origin_zMax = robot_boundary[1][2]; 

    var origin_xRand = Math.random() * (origin_xMax - origin_xMin) + origin_xMin;  // Random value for x coordinate
    var origin_zRand = Math.random() * (origin_zMax - origin_zMin) + origin_zMin;  // Random value for y coordinate

    var rot_min = 0;
    var rot_max = 2*Math.PI;

    var tmp = [origin_xRand, 0, origin_zRand];
    
    var i;
    for (i=0; i<q_goal_config.length-3; i++)
        tmp.push(Math.random() * (rot_max - rot_min) + rot_min);
    tmp[3] = 0;
    tmp[5] = 0;

    return tmp; 
}

function findNearest(tree, q)
{
    var i;
    var nearest_index = -1;
    var nearest_dist = Infinity;
    for (i=0; i<tree.vertices.length; i++)
    {
        var vertex = tree.vertices[i].vertex;
        var dist = 0;
        var j;

        /* for (j=0; j<q_goal_config.length; j++)
            dist += (q[j]-vertex[j]) * (q[j]-vertex[j]); */
        for (j=0; j<3; j++)
            dist += (q[j]-vertex[j]) * (q[j]-vertex[j]);
        for (j=3; j<q_goal_config.length; j++)
        {
            var tmp1 = q[j];
            var tmp2 = vertex[j];
            while (tmp1 >= 2*Math.PI)
                tmp1 -= 2*Math.PI;
            while (tmp1 < 0)
                tmp1 += 2*Math.PI;
            while (tmp2 >= 2*Math.PI)
                tmp2 -= 2*Math.PI;
            while (tmp2 < 0)
                tmp2 += 2*Math.PI;
            dist += (tmp1-tmp2) * (tmp1-tmp2);
        }
           
        dist = Math.sqrt(dist);
        if (dist < nearest_dist)
        {
            nearest_index = i;
            nearest_dist = dist;
        }
    }
    return nearest_index;
}

function newConfig(q, q_near)
{
    var i;
    var q_new = [];

    //var dist = 0;
    var dist1 = 0;
    var dist2 = 0;
    var j;
    /* for (j=0; j<q_goal_config.length; j++)
        dist += (q[j]-q_near[j]) * (q[j]-q_near[j]); */
    for (j=0; j<3; j++)
        dist1 += (q[j]-q_near[j]) * (q[j]-q_near[j]);
    dist1 = Math.sqrt(dist1);
    for (i=0; i<3; i++)
        q_new.push(q_near[i] + 0.5 * (q[i] - q_near[i]) / dist1);
    for (j=3; j<q_goal_config.length; j++)
    {
        /* var tmp1 = q[j];
        var tmp2 = q_near[j]; */
        while (q[j] >= 2*Math.PI)
            q[j] -= 2*Math.PI;
        while (q[j] < 0)
            q[j] += 2*Math.PI;
        while (q_near[j] >= 2*Math.PI)
            q_near[j] -= 2*Math.PI;
        while (q_near[j] < 0)
            q_near[j] += 2*Math.PI;
        //dist += (tmp1-tmp2) * (tmp1-tmp2);
        dist2 += (q[j]-q_near[j]) * (q[j]-q_near[j]);
    }
    dist2 = Math.sqrt(dist2);
    for (i=3; i<q_goal_config.length; i++)
        q_new.push(q_near[i] + 0.5 * (q[i] - q_near[i]) / dist2);
    
    /* var dist = Math.sqrt(dist1+dist2);
    for (i=0; i<q_goal_config.length; i++)
        q_new.push(q_near[i] + 0.5 * (q[i] - q_near[i]) / dist); */

    return q_new;
}

function checkReach(q1, q2)
{
    var i;
    for (i=0; i<3; i++)
        if (Math.abs(q1[i]-q2[i]) > 0.1)
            return false;
    for (i=3; i<q_goal_config.length; i++)
    {
        var tmp1 = q1[i];
        var tmp2 = q2[i];
        while (tmp1 >= 2*Math.PI)
            tmp1 -= 2*Math.PI;
        while (tmp1 < 0)
            tmp1 += 2*Math.PI;
        while (tmp2 >= 2*Math.PI)
            tmp2 -= 2*Math.PI;
        while (tmp2 < 0)
            tmp2 += 2*Math.PI;
        if (Math.abs(tmp1-tmp2) > 0.1)
            return false;
    }
    /* for (i=0; i<q_goal_config.length; i++)
    {
        if (Math.abs(q1[i]-q2[i]) > 0.1)
            return false;
    } */
    return true;
}

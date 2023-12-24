
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

    // STENCIL: see instructor for random time trial code
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform, robot.endeffector.position);

    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0], 2.0)
         + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0], 2.0)
         + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0], 2.0) );
    
    if (kineval.params.trial_ik_random.distance_current < 0.01) //0.01)
    {
        kineval.params.ik_target.position[0][0] = 1.2 * (Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2 * (Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7 * (Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length

    // Compute the current endeffector position and orientation  
    var endeffector_position_world = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    
    // Compute the error term
    var error_position = [];
    for (var i=0; i<3; i++)
        error_position[i] = endeffector_target_world.position[i][0] - endeffector_position_world[i][0];
    
    var error_term = [ [error_position[0]], [error_position[1]], [error_position[2]], [0], [0], [0] ];

    // Compute the Jacobian matrix
    var joint_name = endeffector_joint;
    var joint_chain = [];
    joint_chain.unshift(robot.joints[joint_name]);
    while (robot.joints[joint_name].parent != robot.base)
    {
        var link = robot.joints[joint_name].parent;
        joint_name = robot.links[link].parent;
        joint_chain.unshift(robot.joints[joint_name]);
    }
    //joint_chain.unshift(robot.origin); // 
    
    var jacobian = [];
    /* var endeffector_origin_world = matrix_multiply(robot.joints[endeffector_joint].xform, [[0],[0],[0],[1]]);

    for (var i = 0; i < joint_chain.length; i++)
    {
        var joint_jacobian = [];

        var joint_origin_world = matrix_multiply(joint_chain[i].xform, [ [0],[0],[0],[1] ]);
        var origin_diff = [];
        for (var j=0; j<3; j++)
            origin_diff[j] = endeffector_origin_world[j][0] - joint_origin_world[j][0];
        
        // var z = [ [joint_chain[i].axis[0]], [joint_chain[i].axis[1]], [joint_chain[i].axis[2]], [1] ];
       
        var ax = [ [joint_chain[i].axis[0]], [joint_chain[i].axis[1]], [joint_chain[i].axis[2]], [0] ];
        ax = matrix_multiply(joint_chain[i].xform, ax);
        var z = [ax[0][0], ax[1][0], ax[2][0]];

        var tmp = vector_cross(z, origin_diff);
        joint_jacobian[0] = tmp[0];
        joint_jacobian[1] = tmp[1];
        joint_jacobian[2] = tmp[2];
        joint_jacobian[3] = z[0];
        joint_jacobian[4] = z[1];
        joint_jacobian[5] = z[2];

        jacobian.push(joint_jacobian);
    } */
    for (var i = 0; i < joint_chain.length; i++)
    {
        var joint_jacobian = [];

        //var joint_origin = joint_chain[i].origin.xyz;
        var joint_origin_world = matrix_multiply(joint_chain[i].xform, [ [0],[0],[0],[1] ]);
        //var joint_origin_world = matrix_multiply(joint_chain[i].xform, [ [joint_origin[0]],[joint_origin[1]],[joint_origin[2]], [0] ]);
        var origin_diff = [];
        for (var j=0; j<3; j++)
            origin_diff[j] = endeffector_position_world[j][0] - joint_origin_world[j][0];
        
        var ax = [ [joint_chain[i].axis[0]], [joint_chain[i].axis[1]], [joint_chain[i].axis[2]], [1] ];
        var joint_axis_world = matrix_multiply(joint_chain[i].xform, ax);
    
        var z = [];
        for (var j=0; j<3 ;j++)
            z[j] = joint_axis_world[j] - joint_origin_world[j];

        var tmp = vector_cross(z, origin_diff);
        joint_jacobian[0] = tmp[0];
        joint_jacobian[1] = tmp[1];
        joint_jacobian[2] = tmp[2];
        joint_jacobian[3] = z[0];
        joint_jacobian[4] = z[1];
        joint_jacobian[5] = z[2];

        jacobian.push(joint_jacobian);
    }
    jacobian = matrix_transpose(jacobian);

    // Compute the change in joint configuration
    var dq;
    if (kineval.params.ik_pseudoinverse)
        dq = matrix_multiply(matrix_pseudoinverse(jacobian), error_term);
    else
        dq = matrix_multiply(matrix_transpose(jacobian), error_term);

    // Set the .control field of each joint
    for (var i = 0; i < joint_chain.length; i++)
    {
        joint_chain[i].control = kineval.params.ik_steplength * dq[i][0];
    }   

    // Assign the following variables to test against CI grader
    robot.dx = error_term;
    robot.jacobian = jacobian;
    robot.dq = dq;
}

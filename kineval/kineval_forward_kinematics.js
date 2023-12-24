
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

    // STENCIL: call kineval.buildFKTransforms();
    var s = [generate_identity()];
    // base joint
    s.push(s[s.length-1]);

    var trans= generate_translation_matrix(robot.origin.xyz[0],robot.origin.xyz[1],robot.origin.xyz[2]);
    var rotX = generate_rotation_matrix_X(robot.origin.rpy[0]);
    var rotY = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    var rotZ = generate_rotation_matrix_Z(robot.origin.rpy[2]);
    var rot = matrix_multiply(matrix_multiply(rotZ,rotY),rotX);
    var res = matrix_multiply(trans,rot);
    res = matrix_multiply(s[s.length-1],res);
    s[s.length-1] = res;
    robot.origin.xform = res;
    
    // base movement control
    robot_heading = matrix_multiply(res,[[0],[0],[1],[1]]);
    robot_lateral = matrix_multiply(res,[[1],[0],[0],[1]]);
    
    kineval.buildFKTransforms(s, robot.base);
}

kineval.buildFKTransforms = function buildFKTransforms (s, link) {
    // STENCIL: implement buildFKTransforms, which kicks off
    //   a recursive traversal over links and 
    //   joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // To use the keyboard interface, assign the global variables 
    //   "robot_heading" and "robot_lateral", 
    //   which represent the z-axis (heading) and x-axis (lateral) 
    //   of the robot's base in its own reference frame, 
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form 
    //   as 4x1 homogenous matrices

    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    robot.links[link].xform = s[s.length-1];
    
    if (typeof robot.links[link].children === 'undefined')
        return;
        
    var i;
    for (i=0; i<robot.links[link].children.length; i++)
    {
        s.push(s[s.length-1]);
        var joint_name = robot.links[link].children[i];
        var joint = robot.joints[joint_name];
        var trans= generate_translation_matrix(joint.origin.xyz[0],joint.origin.xyz[1],joint.origin.xyz[2]);
        var rotX = generate_rotation_matrix_X(joint.origin.rpy[0]);
        var rotY = generate_rotation_matrix_Y(joint.origin.rpy[1]);
        var rotZ = generate_rotation_matrix_Z(joint.origin.rpy[2]);
        var rot = matrix_multiply(matrix_multiply(rotZ,rotY),rotX);
        var res = matrix_multiply(trans,rot);

        // 3D axis rotation
        var unit_quater = kineval.quaternionFromAxisAngle(joint.axis, joint.angle);
        var axis_rot = kineval.quaternionToRotationMatrix(unit_quater);
        res = matrix_multiply(res, axis_rot);

        res = matrix_multiply(s[s.length-1],res);
        s[s.length-1] = res;
        joint.xform = res;

        /* if (joint_name === robot.endeffector.frame)
        {
            robot_heading = [[res[0][2]],[res[1][2]],[res[2][2]]];
            robot_lateral = [[res[0][0]],[res[1][0]],[res[2][0]]];
        } */

        buildFKTransforms(s, joint.child);
        s.pop();
    }
    return;
}
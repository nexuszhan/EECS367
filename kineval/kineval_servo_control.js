
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

    // STENCIL: implement FSM to cycle through dance pose setpoints
    var x;
    var index = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];
    var setpoints = kineval.setpoints[index];
    var flag = true;
    for (x in robot.joints)
    {
        kineval.params.setpoint_target[x] = setpoints[x];
        if (Math.abs(kineval.params.setpoint_target[x]-robot.joints[x].angle) > 0.05)
            flag = false;
    }

    if (flag)
    {
        if (kineval.params.dance_pose_index < (kineval.params.dance_sequence_index.length-1))
            kineval.params.dance_pose_index++;
        else
            kineval.params.dance_pose_index = 0;
    }
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

    // STENCIL: implement P servo controller over joints
    var joint_name;
    for (joint_name in robot.joints)
    {
        var joint = robot.joints[joint_name];
        var e = kineval.params.setpoint_target[joint_name] - joint.angle;
        joint.control = joint.servo.p_gain * e;
    }
}



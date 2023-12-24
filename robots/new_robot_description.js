//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "Xing_Tian";
robot.partner_name = "Jin Huang";

// initialize start pose of robot in the world
robot.origin = {xyz: [0, 5, 0], rpy:[0, 0, 0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "link1";  

// specify and create data objects for the links of the robot
robot.links = {"link1": {}, "link2": {}, "link3": {}, "link4": {}, "link5": {}, "link6":{}, "link7":{}, 
               "link8":{}, "link9":{}, "link10":{}, "link11":{}, "link12":{}, "link13":{}, "link14":{}, 
               "link15":{}, "link16":{}, "link17":{}, "link18":{}, "link19":{}, "link20":{}, 
               "link30":{}, "link31":{}, "link32":{}, "link33":{}, "link34":{}, "link35":{}, "link36":{},
               "link37":{}, "link38":{}, "link39":{} };

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

// 右大腿
robot.joints.joint1 = {parent:"link1", child:"link2"};
robot.joints.joint1.origin = {xyz: [-0.7, -1.5, 0], rpy:[0 , 0, 0]};
robot.joints.joint1.axis = [1, 0, 0];

// 左大腿
robot.joints.joint2 = {parent:"link1", child:"link3"};
robot.joints.joint2.origin = {xyz: [0.7, -1.5, 0], rpy:[0, 0, 0]};
robot.joints.joint2.axis = [1, 0, 0];

// 右膝
robot.joints.joint3 = {parent:"link2", child:"link4"};
robot.joints.joint3.origin = {xyz: [0, -1.3, 0], rpy:[0, 0, 0]};
robot.joints.joint3.axis = [1, 0, 0];

// 左膝
robot.joints.joint4 = {parent:"link3", child:"link5"};
robot.joints.joint4.origin = {xyz: [0, -1.3, 0], rpy:[0 , 0, 0]};
robot.joints.joint4.axis = [1, 0, 0];

// 右脚
robot.joints.joint5 = {parent:"link4", child:"link6"};
robot.joints.joint5.origin = {xyz: [0, -1.25, 0], rpy:[0 , 0, 0]};
robot.joints.joint5.axis = [1, 0, 0];

// 左脚
robot.joints.joint6 = {parent:"link5", child:"link7"};
robot.joints.joint6.origin = {xyz: [0, -1.25, 0], rpy:[0 , 0, 0]};
robot.joints.joint6.axis = [1, 0, 0];

// 右肩
robot.joints.joint7 = {parent:"link1", child:"link8"};
robot.joints.joint7.origin = {xyz: [-1.25, 1.25, 0], rpy:[0 , 0, -Math.PI/2.4]};
robot.joints.joint7.axis = [0, 0, 1];

// 左肩
robot.joints.joint8 = {parent:"link1", child:"link9"};
robot.joints.joint8.origin = {xyz: [1.25, 1.25, 0], rpy:[0 , 0, Math.PI/6]};
robot.joints.joint8.axis = [0, 0, 1];

// 右肘
robot.joints.joint9 = {parent:"link8", child:"link10"};
robot.joints.joint9.origin = {xyz: [0, -1.25, 0], rpy:[0 , 0, -Math.PI/2]};
robot.joints.joint9.axis = [0, 0, 1];

// 左肘
robot.joints.joint10 = {parent:"link9", child:"link11"};
robot.joints.joint10.origin = {xyz: [0, -1.25, 0], rpy:[-Math.PI/2 , 0, 0]};
robot.joints.joint10.axis = [1, 0, 0];

// 盾
robot.joints.joint11 = {parent:"link11", child:"link12"};
robot.joints.joint11.origin = {xyz: [0, -1.3, 0], rpy:[Math.PI/2, 0, 0]};
robot.joints.joint11.axis = [1, 0, 0];

robot.joints.joint12 = {parent:"link12", child:"link13"};
robot.joints.joint12.origin = {xyz: [0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint12.axis = [0, 1, 0];

robot.joints.joint13 = {parent:"link12", child:"link14"};
robot.joints.joint13.origin = {xyz: [-0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint13.axis = [0, 1, 0];

robot.joints.joint14 = {parent:"link13", child:"link15"};
robot.joints.joint14.origin = {xyz: [0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint14.axis = [0, 1, 0];

robot.joints.joint15 = {parent:"link14", child:"link16"};
robot.joints.joint15.origin = {xyz: [-0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint15.axis = [0, 1, 0];

robot.joints.joint16 = {parent:"link15", child:"link17"};
robot.joints.joint16.origin = {xyz: [0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint16.axis = [0, 1, 0];

robot.joints.joint17 = {parent:"link16", child:"link18"};
robot.joints.joint17.origin = {xyz: [-0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint17.axis = [0, 1, 0];

robot.joints.joint18 = {parent:"link17", child:"link19"};
robot.joints.joint18.origin = {xyz: [0.2, 0.1, 0], rpy:[0, 0, 0]};
robot.joints.joint18.axis = [0, 1, 0];

robot.joints.joint19 = {parent:"link18", child:"link20"};
robot.joints.joint19.origin = {xyz: [-0.2, 0.2, 0], rpy:[0, 0, 0]};
robot.joints.joint19.axis = [0, 1, 0];

// 装饰(衣领)
robot.joints.joint30 = {parent:"link1", child:"link30"};
robot.joints.joint30.origin = {xyz: [0.75, 1.6, 0], rpy:[0, 0, 0]};
robot.joints.joint30.axis = [1, 0, 0]; 

robot.joints.joint31 = {parent:"link1", child:"link31"};
robot.joints.joint31.origin = {xyz: [-0.75, 1.6, 0], rpy:[0, 0, 0]};
robot.joints.joint31.axis = [1, 0, 0];

// knife handle lower
robot.joints.joint32 = {parent:"link10", child:"link32"};
robot.joints.joint32.origin = {xyz: [0, -1.25, 0], rpy:[ 0, 0, Math.PI/2]};
robot.joints.joint32.axis = [0, 0, 1];

// knife handle upper
robot.joints.joint33 = {parent:"link10", child:"link33"};
robot.joints.joint33.origin = {xyz: [0, -1.25, 0], rpy:[ 0, 0, -Math.PI/2]};
robot.joints.joint33.axis = [0, 0, 1];

// knife cut
robot.joints.joint34 = {parent:"link33", child:"link34"};
robot.joints.joint34.origin = {xyz: [0, -1.25, 0], rpy:[ 0, 0, Math.PI/2]};
robot.joints.joint34.axis = [0, 0, 1];

robot.joints.joint35 = {parent:"link34", child:"link35"};
robot.joints.joint35.origin = {xyz: [0.5, 0.125, 0], rpy:[0, 0, 0]};
robot.joints.joint35.axis = [0, 0, 1];

robot.joints.joint36 = {parent:"link34", child:"link36"};
robot.joints.joint36.origin = {xyz: [-0.5, 0.125, 0], rpy:[0, 0, 0]};
robot.joints.joint36.axis = [0, 0, 1];

robot.joints.joint37 = {parent:"link34", child:"link37"};
robot.joints.joint37.origin = {xyz: [0, -0.6, 0], rpy:[ 0, 0, 0]};
robot.joints.joint37.axis = [0, 0, 1];

robot.joints.joint38 = {parent:"link35", child:"link38"};
robot.joints.joint38.origin = {xyz: [0, -0.45, 0], rpy:[ 0, 0, 0]};
robot.joints.joint38.axis = [0, 0, 1];

robot.joints.joint39 = {parent:"link36", child:"link39"};
robot.joints.joint39.origin = {xyz: [0, -0.45, 0], rpy:[ 0, 0, 0]};
robot.joints.joint39.axis = [0, 0, 1];

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "joint11";
robot.endeffector.position = [[0.5],[0],[0],[1]]; 

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

// 身体
links_geom["link1"] = new THREE.CubeGeometry( 2.0+0.5, 2.5+0.5, 0.5 );
links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );
// 右大腿
links_geom["link2"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link2"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -1.3/2, 0) );
// 左大腿
links_geom["link3"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link3"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -1.3/2, 0) );
// 右小腿
links_geom["link4"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link4"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -1.3/2, 0) );
// 左小腿
links_geom["link5"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link5"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -1.3/2, 0) );
// 右脚
//links_geom["link6"] = new THREE.TetrahedronGeometry( 0.5 );
links_geom["link6"] = new THREE.CubeGeometry( 0.5, 0.25, 0.8 );
links_geom["link6"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25));
// 左脚
links_geom["link7"] = new THREE.CubeGeometry( 0.5, 0.25, 0.8 );
links_geom["link7"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25));
// 右大臂
links_geom["link8"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link8"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));
// 左大臂
links_geom["link9"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link9"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));
// 右小臂
links_geom["link10"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link10"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));
// 左小臂
links_geom["link11"] = new THREE.CylinderGeometry( 0.25, 0.25, 1.3 );
links_geom["link11"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));
// 盾
links_geom["link12"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link12"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link13"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link13"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link14"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link14"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link15"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link15"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link16"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link16"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link17"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link17"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link18"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link18"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link19"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link19"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link20"] = new THREE.CubeGeometry( 0.2, 2, 0.3);
links_geom["link20"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));

// 装饰
links_geom["link30"] = new THREE.CylinderGeometry(0, 0.3, 0.4);
links_geom["link30"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));
links_geom["link31"] = new THREE.CylinderGeometry(0, 0.3, 0.4);
links_geom["link31"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0));

// knife handler 1
links_geom["link32"] = new THREE.CylinderGeometry( 0.1, 0.1, 3 );
links_geom["link32"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

// knife handler 2
links_geom["link33"] = new THREE.CylinderGeometry( 0.1, 0.1, 3 );
links_geom["link33"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

// knife cut
links_geom["link34"] = new THREE.CylinderGeometry( 0.25, 0.25, 1 );
links_geom["link34"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

links_geom["link35"] = new THREE.CylinderGeometry( 0.25, 0.25, 0.75 );
links_geom["link35"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

links_geom["link36"] = new THREE.CylinderGeometry( 0.25, 0.25, 0.75 );
links_geom["link36"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

links_geom["link37"] = new THREE.CylinderGeometry( 0.25, 0, 0.2 );
links_geom["link37"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

links_geom["link38"] = new THREE.CylinderGeometry( 0.25, 0, 0.2 );
links_geom["link38"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));

links_geom["link39"] = new THREE.CylinderGeometry( 0.25, 0, 0.2 );
links_geom["link39"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.5, 0));
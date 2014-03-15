require "rttlib"
require "rttros"

rttlib.color=true

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_ros")
d:import("rtt_roscomm")
-- Start of user code imports
ros=rtt.provides("ros")

ros:import("conman")
ros:import("lwr_fri")
ros:import("velma_hardware")
ros:import("oro_joint_state_publisher")
ros:import("lwr_inertia")
ros:import("controller_common")
ros:import("velma_controller")
-- End of user code

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("rostopic"):connection(topic))
end

-------------------------------------------------------------------------------
-- Hardware interface
-------------------------------------------------------------------------------

d:loadComponent("FRIr", "FRIComponent")
d:setActivity("FRIr", 0.0, 6, rtt.globals.ORO_SCHED_RT)
FRIr = d:getPeer("FRIr")
FRIr:getProperty("fri_port"):set(49948)

d:loadComponent("FRIl", "FRIComponent")
d:setActivity("FRIl", 0.0, 6, rtt.globals.ORO_SCHED_RT)
FRIl = d:getPeer("FRIl")
FRIl:getProperty("fri_port"):set(49938)

d:loadComponent("VT", "VelmaTorso")
d:setActivity("VT", 0.001, 6, rtt.globals.ORO_SCHED_RT)
VT = d:getPeer("VT")

FRIr:configure()
FRIl:configure()
VT:configure()

-------------------------------------------------------------------------------
-- Controller manager
-------------------------------------------------------------------------------
d:loadComponent("scheme", "conman::Scheme")
d:setActivity("scheme", 0.001, 5, rtt.globals.ORO_SCHED_RT)
scheme = d:getPeer("scheme")
scheme:configure()

-------------------------------------------------------------------------------
-- Velma controller
-------------------------------------------------------------------------------

d:loadComponent("Jc", "VectorConcate3")
--d:setActivity("Jc", 0.001, 2, rtt.globals.ORO_SCHED_RT)
Jc = d:getPeer("Jc")

d:connect("VT.JointPosition", "Jc.In0", rtt.Variable("ConnPolicy"))
d:connect("FRIr.JointPosition", "Jc.In1", rtt.Variable("ConnPolicy"))
d:connect("FRIl.JointPosition", "Jc.In2", rtt.Variable("ConnPolicy"))

d:loadComponent("Jvc", "VectorConcate3")
--d:setActivity("Jvc", 0.001, 2, rtt.globals.ORO_SCHED_RT)
Jvc = d:getPeer("Jvc")

d:connect("VT.JointVelocity", "Jvc.In0", rtt.Variable("ConnPolicy"))
d:connect("FRIr.JointVelocity", "Jvc.In1", rtt.Variable("ConnPolicy"))
d:connect("FRIl.JointVelocity", "Jvc.In2", rtt.Variable("ConnPolicy"))

d:loadComponent("Ts", "VectorSplit3")
--d:setActivity("Ts", 0.001, 2, rtt.globals.ORO_SCHED_RT)
Ts = d:getPeer("Ts")
split=Ts:getProperty("outputs")
split:get():resize(3)
split[0]=2;
split[1]=7;
split[2]=7;

d:connect("Ts.Out0", "VT.JointTorqueCommand", rtt.Variable("ConnPolicy"))
d:connect("Ts.Out1", "FRIr.JointTorqueCommand", rtt.Variable("ConnPolicy"))
d:connect("Ts.Out2", "FRIl.JointTorqueCommand", rtt.Variable("ConnPolicy"))

d:loadComponent("Mass", "RobotMassMatrix")
Mass = d:getPeer("Mass")
Mass:loadService("robot")
Mass:configure()

d:connect("Jc.Out", "Mass.JointPosition", rtt.Variable("ConnPolicy"))

d:loadComponent("CImp", "CartesianImpedance")
--d:setActivity("CImp", 0.001, 5, rtt.globals.ORO_SCHED_RT)
CImp = d:getPeer("CImp")
CImp:loadService("robot")
CImp:configure()

d:connect("Jc.Out", "CImp.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("Jvc.Out", "CImp.JointVelocity", rtt.Variable("ConnPolicy"))
d:connect("Mass.MassMatrix", "CImp.MassMatrixInv", rtt.Variable("ConnPolicy"))

d:connect("CImp.JointTorqueCommand", "Ts.In", rtt.Variable("ConnPolicy"))

d:loadComponent("PoseInt", "CartesianInterpolator")
PoseInt = d:getPeer("PoseInt")

d:connect("CImp.CartesianPosition0", "PoseInt.CartesianPosition", rtt.Variable("ConnPolicy"))
d:connect("PoseInt.CartesianPositionCommand", "CImp.CartesianPositionCommand0", rtt.Variable("ConnPolicy"))

d:loadComponent("JntLimit", "JointLimitAvoidance")
JntLimit = d:getPeer("JntLimit")
upper_limit=JntLimit:getProperty("upper_limit")
upper_limit:get():resize(16)
upper_limit[0]=100
upper_limit[1]=100
upper_limit[2]=2.96
upper_limit[3]=2.09
upper_limit[4]=2.96
upper_limit[5]=2.09
upper_limit[6]=2.96
upper_limit[7]=2.09
upper_limit[8]=2.96
upper_limit[9]=2.96
upper_limit[10]=2.09
upper_limit[11]=2.96
upper_limit[12]=2.09
upper_limit[13]=2.96
upper_limit[14]=2.09
upper_limit[15]=2.96

lower_limit=JntLimit:getProperty("lower_limit")
lower_limit:get():resize(16)
lower_limit[0]=-100
lower_limit[1]=-100
lower_limit[2]=-2.96
lower_limit[3]=-2.09
lower_limit[4]=-2.96
lower_limit[5]=-2.09
lower_limit[6]=-2.96
lower_limit[7]=-2.09
lower_limit[8]=-2.96
lower_limit[9]=-2.96
lower_limit[10]=-2.09
lower_limit[11]=-2.96
lower_limit[12]=-2.09
lower_limit[13]=-2.96
lower_limit[14]=-2.09
lower_limit[15]=-2.96

limit_range=JntLimit:getProperty("limit_range")
limit_range:get():resize(16)
limit_range[0]=0.26
limit_range[1]=0.26
limit_range[2]=0.26
limit_range[3]=0.26
limit_range[4]=0.26
limit_range[5]=0.26
limit_range[6]=0.26
limit_range[7]=0.26
limit_range[8]=0.26
limit_range[9]=0.26
limit_range[10]=0.26
limit_range[11]=0.26
limit_range[12]=0.26
limit_range[13]=0.26
limit_range[14]=0.26
limit_range[15]=0.26

max_trq=JntLimit:getProperty("max_trq")
max_trq:get():resize(16)
max_trq[0]=10
max_trq[1]=10
max_trq[2]=100
max_trq[3]=100
max_trq[4]=80
max_trq[5]=50
max_trq[6]=50
max_trq[7]=30
max_trq[8]=30
max_trq[9]=100
max_trq[10]=100
max_trq[11]=80
max_trq[12]=50
max_trq[13]=50
max_trq[14]=30
max_trq[15]=30

d:connect("Jc.Out", "JntLimit.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("Jvc.Out", "JntLimit.JointVelocity", rtt.Variable("ConnPolicy"))
d:connect("Mass.MassMatrix", "JntLimit.MassMatrix", rtt.Variable("ConnPolicy"))
d:connect("JntLimit.JointTorqueCommand", "CImp.NullSpaceTorqueCommand", rtt.Variable("ConnPolicy"))

Jc:configure()
Jvc:configure()
Ts:configure()
PoseInt:configure()
JntLimit:configure()

d:addPeer("scheme", "Jc")
d:addPeer("scheme", "Jvc")
d:addPeer("scheme", "Mass")
d:addPeer("scheme", "CImp")
d:addPeer("scheme", "Ts")
d:addPeer("scheme", "JntLimit")
d:addPeer("scheme", "PoseInt")

scheme:addBlock("Jc")
scheme:addBlock("Jvc")
scheme:addBlock("Mass")
scheme:addBlock("CImp")
scheme:addBlock("Ts")
scheme:addBlock("JntLimit")
scheme:addBlock("PoseInt")
scheme:latchConnections("CImp", "PoseInt", true)

-------------------------------------------------------------------------------
-- ROS Diagnostics
-------------------------------------------------------------------------------

d:loadComponent("JntPub", "JointStatePublisher")
d:setActivity("JntPub", 0.01, 2, rtt.globals.ORO_SCHED_RT)
JntPub = d:getPeer("JntPub")
joints=JntPub:getProperty("joint_names")
joints:get():resize(16)
joints[0]="torso_0_joint"
joints[1]="torso_1_joint"
joints[2]="right_arm_0_joint"
joints[3]="right_arm_1_joint"
joints[4]="right_arm_2_joint"
joints[5]="right_arm_3_joint"
joints[6]="right_arm_4_joint"
joints[7]="right_arm_5_joint"
joints[8]="right_arm_6_joint"
joints[9]="left_arm_0_joint"
joints[10]="left_arm_1_joint"
joints[11]="left_arm_2_joint"
joints[12]="left_arm_3_joint"
joints[13]="left_arm_4_joint"
joints[14]="left_arm_5_joint"
joints[15]="left_arm_6_joint"

d:connect("Jc.Out", "JntPub.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("Jvc.Out", "JntPub.JointVelocity", rtt.Variable("ConnPolicy"))
d:connect("CImp.JointTorqueCommand", "JntPub.JointEffort", rtt.Variable("ConnPolicy"))

JntPub:configure()

-------------------------------------------------------------------------------
-- ROS Command interface

d:loadComponent("CartTrjA", "CartesianTrajectoryAction")
d:setActivity("CartTrjA", 0.01, 1, rtt.globals.ORO_SCHED_RT)
CartTrjA = d:getPeer("CartTrjA")

d:connect("CartTrjA.CartesianTrajectoryCommand", "PoseInt.CartesianTrajectoryCommand", rtt.Variable("ConnPolicy"))

conn2ros(d, "CartTrjA.trajectory", "/trajectory")

CartTrjA:configure()

-------------------------------------------------------------------------------
--conn2ros(d, "LWRDiag.Diagnostics", "/diagnostics")
--conn2ros(d, "FRI.KRL_CMD", "/lwrarm_controller/fri_set_mode")
conn2ros(d, "JntPub.joint_state", "/joint_states")

VT:start()
FRIr:start()
FRIl:start()

scheme:start()
Jc:start()
Jvc:start()
Ts:start()
Mass:start()
--CImp:start()
JntPub:start()
CartTrjA:start()

function start()
   CImp:start()
   JntLimit:start()
   PoseInt:start()
end


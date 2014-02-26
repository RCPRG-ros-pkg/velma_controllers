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
-- ros:import("lwr_fri")
-- ros:import("velma_hardware")
ros:import("oro_joint_state_publisher")
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

--d:loadComponent("FRIr", "FRIComponent")
--d:setActivity("FRIr", 0.0, 6, rtt.globals.ORO_SCHED_RT)
--FRIr = d:getPeer("FRIr")
--FRIr:getProperty("fri_port"):set(49948)

--d:loadComponent("FRIl", "FRIComponent")
--d:setActivity("FRIl", 0.0, 6, rtt.globals.ORO_SCHED_RT)
--FRIl = d:getPeer("FRIl")
--FRIl:getProperty("fri_port"):set(49938)

--d:loadComponent("VT", "VelmaTorso")
--d:setActivity("VT", 0.001, 6, rtt.globals.ORO_SCHED_RT)
--VT = d:getPeer("VT")

--FRIr:configure()
--FRIl:configure()
--VT:configure()

d:loadComponent("VS", "VelmaSim")
d:setActivity("VS", 1, 6, rtt.globals.ORO_SCHED_RT)
VS = d:getPeer("VS")
VS:loadService("robot")
init=VS:getProperty("InitialState")
init:get():resize(16)
init[0]=0.1
init[1]=1.57
init[2]=1.1 
init[3]=1.1 
init[4]=1.1 
init[5]=1.1 
init[6]=1.1 
init[7]=1.1 
init[8]=1.1 
init[9]=1.1 
init[10]=1.1 
init[11]=1.1
init[12]=1.1 
init[13]=1.1 
init[14]=1.1 
init[15]=1.1 
VS:configure()

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

--d:loadComponent("Jc", "VectorConcate3")
--d:setActivity("Jc", 0.001, 2, rtt.globals.ORO_SCHED_RT)
--Jc = d:getPeer("Jc")

--d:connect("VT.JointPosition", "Jc.In0", rtt.Variable("ConnPolicy"))
--d:connect("FRIr.JointPosition", "Jc.In1", rtt.Variable("ConnPolicy"))
--d:connect("FRIl.JointPosition", "Jc.In2", rtt.Variable("ConnPolicy"))

--d:loadComponent("Jvc", "VectorConcate3")
--d:setActivity("Jvc", 0.001, 2, rtt.globals.ORO_SCHED_RT)
--Jvc = d:getPeer("Jvc")

--d:connect("VT.JointVelocity", "Jvc.In0", rtt.Variable("ConnPolicy"))
--d:connect("FRIr.JointVelocity", "Jvc.In1", rtt.Variable("ConnPolicy"))
--d:connect("FRIl.JointVelocity", "Jvc.In2", rtt.Variable("ConnPolicy"))

--d:loadComponent("Ts", "VectorSplit3")
--d:setActivity("Ts", 0.001, 2, rtt.globals.ORO_SCHED_RT)
--Ts = d:getPeer("Ts")
--split=Ts:getProperty("outputs")
--split:get():resize(3)
--split[0]=2;
--split[1]=7;
--split[2]=7;

--d:connect("Ts.Out0", "VT.JointTorqueCommand", rtt.Variable("ConnPolicy"))
--d:connect("Ts.Out1", "FRIr.JointTorqueCommand", rtt.Variable("ConnPolicy"))
--d:connect("Ts.Out2", "FRIl.JointTorqueCommand", rtt.Variable("ConnPolicy"))

d:loadComponent("CImp", "VelmaCartesianImpedance")
--d:setActivity("CImp", 0.001, 5, rtt.globals.ORO_SCHED_RT)
CImp = d:getPeer("CImp")
CImp:loadService("robot")

d:connect("VS.JointPosition", "CImp.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("VS.JointVelocity", "CImp.JointVelocity", rtt.Variable("ConnPolicy"))

d:connect("CImp.JointTorqueCommand", "VS.JointTorqueCommand", rtt.Variable("ConnPolicy"))

--Jc:configure()
--Jvc:configure()
CImp:configure()
--Ts:configure()

--d:addPeer("scheme", "Jc")
--d:addPeer("scheme", "Jvc")
d:addPeer("scheme", "CImp")
--d:addPeer("scheme", "Ts")

--scheme:addBlock("Jc")
--scheme:addBlock("Jvc")
scheme:addBlock("CImp")
--scheme:addBlock("Ts")

-------------------------------------------------------------------------------
-- ROS Diagnostics
-------------------------------------------------------------------------------

d:loadComponent("JntPub", "JointStatePublisher")
d:setActivity("JntPub", 0.01, 2, rtt.globals.ORO_SCHED_RT)
JntPub = d:getPeer("JntPub")
joints=JntPub:getProperty("joint_names")
joints:get():resize(16)
joints[0]="torso_1_joint"
joints[1]="torso_2_joint"
joints[2]="right_arm_1_joint"
joints[3]="right_arm_2_joint"
joints[4]="right_arm_3_joint"
joints[5]="right_arm_4_joint"
joints[6]="right_arm_5_joint"
joints[7]="right_arm_6_joint"
joints[8]="right_arm_7_joint"
joints[9]="left_arm_1_joint"
joints[10]="left_arm_2_joint"
joints[11]="left_arm_3_joint"
joints[12]="left_arm_4_joint"
joints[13]="left_arm_5_joint"
joints[14]="left_arm_6_joint"
joints[15]="left_arm_7_joint"

d:connect("VS.JointPosition", "JntPub.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("VS.JointVelocity", "JntPub.JointVelocity", rtt.Variable("ConnPolicy"))
d:connect("CImp.JointTorqueCommand", "JntPub.JointEffort", rtt.Variable("ConnPolicy"))

JntPub:configure()

--conn2ros(d, "LWRDiag.Diagnostics", "/diagnostics")
--conn2ros(d, "FRI.KRL_CMD", "/lwrarm_controller/fri_set_mode")
conn2ros(d, "JntPub.joint_state", "/joint_states")

--VS:start()
--FRIr:start()
--FRIl:start()

scheme:start()
--Jc:start()
--Jvc:start()
--Ts:start()
--CImp:start()
JntPub:start()


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

d:loadComponent("VS", "VelmaSim")
d:setActivity("VS", 0.001, 6, rtt.globals.ORO_SCHED_RT)
VS = d:getPeer("VS")
VS:loadService("robot")
init=VS:getProperty("InitialState")
init:get():resize(16)
init[0]=0.1
init[1]=-1.57
init[2]=-1.5 
init[3]=1.6 
init[4]=1.57 
init[5]=-1.1
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

d:loadComponent("CImp", "VelmaCartesianImpedance")
CImp = d:getPeer("CImp")
CImp:loadService("robot")

d:loadComponent("PoseInt", "CartesianInterpolator")
PoseInt = d:getPeer("PoseInt")

d:loadComponent("CartTrjA", "CartesianTrajectoryAction")
d:setActivity("CartTrjA", 0.01, 1, rtt.globals.ORO_SCHED_RT)
CartTrjA = d:getPeer("CartTrjA")

d:connect("VS.JointPosition", "CImp.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("VS.JointVelocity", "CImp.JointVelocity", rtt.Variable("ConnPolicy"))

d:connect("CImp.JointTorqueCommand", "VS.JointTorqueCommand", rtt.Variable("ConnPolicy"))

d:connect("CImp.CartesianPosition0", "PoseInt.CartesianPosition", rtt.Variable("ConnPolicy"))
d:connect("PoseInt.CartesianPositionCommand", "CImp.CartesianPositionCommand0", rtt.Variable("ConnPolicy"))

d:connect("CartTrjA.CartesianTrajectoryCommand", "PoseInt.CartesianTrajectoryCommand", rtt.Variable("ConnPolicy"))

conn2ros(d, "CartTrjA.trajectory", "/trajectory")

CImp:configure()
PoseInt:configure()
CartTrjA:configure()

d:addPeer("scheme", "CImp")
d:addPeer("scheme", "PoseInt")
d:addPeer("scheme", "CartTrjA")
scheme:addBlock("CImp")
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

d:connect("VS.JointPosition", "JntPub.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("VS.JointVelocity", "JntPub.JointVelocity", rtt.Variable("ConnPolicy"))
d:connect("CImp.JointTorqueCommand", "JntPub.JointEffort", rtt.Variable("ConnPolicy"))

JntPub:configure()


conn2ros(d, "JntPub.joint_state", "/joint_states")


scheme:start()
VS:start()
JntPub:start()
CartTrjA:start()

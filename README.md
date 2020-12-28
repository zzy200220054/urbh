# UR5+Barrett Hand-ROS
UR5+Barrett Hand/Ubantu16.04/ROS Kinetic
##  基本的安装流程
###  1.下载ur5相关的ros package
```
source /opt/ros/Kinetic/setup.bash
mkdir -p ur_ws/src && cd ur_ws
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```
source devel/setup.bash只在当前终端生效，每次打开其他终端时都要重新source，这样比较麻烦。  
解决方法：gedit ~/.bashrc，打开.bashrc文件，在文件底部添加source ~/path/to/ur5/devel/setup.bash，保存退出即可。

###  2.仿真测试
1.打开终端,启动  
`roslaunch ur_gazebo ur5.launch`  
2.打开新终端  
`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`  
3.再打开一个新终端  
`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`  

###  3.驱动真实的ur5机器人
3.1安装externalcontrol  
要在真正的机器人上使用ur_robot_driver，需要在ur机器人上安装externalcontrol-1.0.urcap，该文件位于Universal_Robots_ROS_Driver/ur_robot_driver的resources文件夹内。注意：要安装此URCap，要求PolyScope的版本不得低于3.7。  
安装步骤：  
1.用U盘将此文件拷贝至机器人示教器的programs文件夹。  
2.在欢迎屏幕上，选择Setup Robot，然后选择URCaps进入URCaps安装屏幕。  
3.单击底部的小加号以打开文件选择器。 在此处，可以看到存储在机器人程序文件夹或插入的USB驱动器中的所有urcap文件。 选择并打开externalcontrol-1.0.urcap文件，然后单击打开。 现在，您的URCaps视图应在活动的URCaps列表中显示External Control，点击右下角重启机器人。  
4.重新启动后，选择为机器人编程，在安装设置部分中找到External Control 。 然后设置外部PC的IP地址，本文设置为192.168.1.101 。请注意，机器人和外部PC必须位于同一网络中，理想情况下，彼此之间应直接连接，以最大程度地减少网络干扰。 自定义端口现在应该保持不变。  
5.要使用新的URCap，请创建一个新程序并将External Control程序节点插入到程序树中。  
6.重新点击命令按钮，则会看到在安装中输入的设置。 检查它们是否正确，然后将程序保存，可以将程序命名为external_control.urp。 现在机器人可以与此驱动程序一起使用了。  

3.2 网络配置
设置机器人静态IP. 设置机器人 ——> 设置网络菜单：  
IP地址: 192.168.1.2  
子网掩码：255.255.255.0  
测试网络连接：  
`ping 192.168.1.2  #IP_OF_THE_ROBOT`  
会看到如下输出：  
64 bytes from 192.168.1.2: icmp_seq=1 ttl=64 time=0.518 ms  
64 bytes from 192.168.1.2: icmp_seq=2 ttl=64 time=0.259 ms  
64 bytes from 192.168.1.2: icmp_seq=3 ttl=64 time=0.282 ms  
如果没有发生任何事情或引发错误，则无法从计算机访问机器人。  

3.3 用ros驱动真实的ur5机器人  
1.网线连接机器人和电脑，启动机器人。  
2.打开电脑终端，启动机器人驱动程序。  
`roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.1.2`  
3.示教器，运行程序 —> 文件 —> 加载程序 —> 选择3.1 节保存的external_control.urp程序，打开—>运行。  
4.新终端启动moveit  
`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true`  
此处报错： [ERROR] : Action client not connected: /follow_joint_trajectory  
解决：找到/ur5_moveit_config/config/controllers.yaml 文件，name: 后添加 scaled_pos_joint_traj_controller  
上述修改方法会导致用gazebo仿真时报错，因此使用gazebo时需要改回源文件中的 - name: ""  
5.新终端启动rviz  
`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`  
然后就可以拖动rviz中的ur5的末端，plan然后execute控制真实的UR5运动。注意观察plan的运行轨迹，慎防撞击。  
至此完成了用ros驱动真实ur5机器人。

###  4.pyHand包下载
用以下版本  
下载地址：https://git.barrett.com/firmware/bhand/tree/pyHand-2.0/
###  5.利用pyHand包中的示例驱动真实的Barrett Hand（Barrett Hand和PC用的是can-usb接口）
1.USB接口连接电脑，启动机械手。  
2.安装Python的can包   
`pip install python-can`  
报错不影响后面的使用，可略过。  
3.运行前先设置  
`sudo ip link set can0 up type can bitrate 1000000`  
4.示例修改  
进入bhand-pyHand-2.0/pyHand/Archive_pyHand_1.0/examples  
选择任意example，打开，修改    
sys.path.append('../../source/pyHand_API')中括号修改为自身电脑对应pyHand_API的路径。  
进入bhand-pyHand-2.0/pyHand/source/pyHand_API  
打开CAN_library_linux.py，修改    
from can.interfaces.interface import Bus为from can.interfaces import Bus  
然后python运行任意example  
至此完成了用pyHand示例驱动真实的Barrett Hand。

###  6.编写python程序驱动真实UR5+Barrett Hand
需要在Kinetic上安装moveit  
`sudo apt-get install ros-kinetic-moveit`  
1.电源及接线确保没问题  
2.打开电脑终端，启动机器人驱动程序。  
`roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.1.2`  
3.示教器，运行程序  
4.新终端启动moveit  
`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true`  
5.运行前设置  
`sudo ip link set can0 up type can bitrate 1000000`  
6.运行python程序  
下列python程序实例将实现控制真实UR5+Barrett Hand模拟完成某处抓取物体移动至另一处放下物体的操作。  
以下是完整代码
```python
# coding=utf-8
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
sys.path.append('../../source/pyHand_API')
import pyHand_api as hand
import time

###
# ur5初始化
print "============ Starting tutorial setup"  #初始化moveit_commander和rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()  #实例化RobotCommander对象，这个接口是机器人总入口
scene = moveit_commander.PlanningSceneInterface()  #实例化PlanningSceneInterface对象，这个接口围绕机器人的世界
group = moveit_commander.MoveGroupCommander("manipulator")  #实例化MoveGroupCommander对象，这个接口应用与一组关节。以规划和执行动作
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)  #创建DisplayTrajectory发布器，可以得到轨迹在Rviz总实现可视化
###
# ur5获得基本信息

print "============ Reference frame: %s" % group.get_planning_frame()  #打印参考系的名称
print "============ Reference frame: %s" % group.get_end_effector_link()  #打印这个组的末端执行器的连接名称
print "============ Robot Groups:"  #获得机器人的所有组
print robot.get_group_names()
print "============ Printing robot state"  #用于调式，打印机器人的状态
print robot.get_current_state()
print "============"

###
# bh初始化

# hand.initialize()
# hand.init_hand()

###
# bh设置常量

MODE = 8
MODE_IDLE= 0
M = 58			# Position move command
MODE_VEL = 4
V = 44
TSTOP = 78

MIN_ENC = 0 		# The closed position for each motor.
MAX_ENC = 105000	# The open position for each motor.

FINGER1 = 11	# Puck ID for F1
FINGER2 = 12	# Puck ID for F2
FINGER3 = 13	# Puck ID for F3
SPREAD  = 14	# Puck ID for SP
HAND_GROUP = 0x405 	# Refers to all motors that respond to group ID 5.

###
# 抓取物体
hand.set_property(FINGER1, V, 70)  #速度设置
hand.set_property(FINGER2, V, 70)
hand.set_property(FINGER3, V, 70)
hand.set_property(HAND_GROUP, MODE, MODE_VEL)
time.sleep(1.5)

hand.set_property(HAND_GROUP, M, MAX_ENC)  #抓取闭合程度
# ... and don't forget to stop the spread quickly.
hand.set_property(SPREAD, MODE, MODE_IDLE)
# Now we wait for the fingers to stop moving so that fingers and spread don't
# run into one another.
time.sleep(1.5)

###
# 移动到某处
joint_positions = [0, 0, -1.5, -0.5, 0.5, -1]
group.set_joint_value_target(joint_positions)
group.go()

###
# 放下物体
hand.set_property(HAND_GROUP, M, MIN_ENC)
hand.set_property(SPREAD, MODE, MODE_IDLE)
time.sleep(1.5)
```
###  7.ROS　Package
将下载的“[Geomagic_Touch_ROS_Drivers](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers)目录下文件”或“[phantomOmniRos](https://github.com/PolarisYxh/phantomOmniRos)中geomagic_touch-master目录下文件”复制 到你的工作空间src目录下  
在工作空间运行  
`catkin_make`  
进行编译  
然后即可通过roslaunch命令打开设备的rviz并且同步位姿


## 出现的一些错误及解决
**1.运行`sudo ./geomagic.sh`时，提示Config directory not set correctly.:　No such file or directory**  

解决：  
检查step.4中的配置目录是否正常创建，以及/etc/enviroment中添加的环境变量是否正确保存  

**2.运行`catkin_make`编译时出现　fatal error: bullet/LinearMath/btMatrix3x3.h:No such file or directory**   

解决：  
缺少必要的依赖，运行`sudo apt-get install libbullet-dev`即可 

**3.运行[phantomOmniRos](https://github.com/PolarisYxh/phantomOmniRos)中launch文件时出现    
Fail to initialize haptic device  
/opt/ros/kinetic/lib/rviz/rviz/symbol lookup error:/opt/ros/kinetic/lib/librviz.so:undefined symbol:_ZNK9QRTreeViewportSizeHintEv**  

解决:  
Ubantu16.04安装相关驱动后出现了Rviz无法正常启动的的问题，尚未有好的结局方案，但可以在终端中用`sudo su`命令进入root用户权限  
再运行`source ~/.bashrc`后，即可使用rviz  

**4运行roslaunch时出现  
HD Error: HD_COMM_ERROR  
Communication Error: Check the device connection and configuration.  
HHD: 0  
Error Code: 302  
Internal Error Code: -28  
Message: Error during main scheduler callback**    

解决：出现该错误一般是Geomagic_Touch_Diagnostic正在运行相关的调试和诊断内容导致的，可以将Geomagic_Touch_Diagnostic程序返回到Mode标签下或直接关闭，之后再运行roslaunch命令即可正常使用  

**5.编译时出现  
error：HLAPI does not name a type**  

解决：尚未找到好的解决方法，但在实践中，卸载并重装cmake**有一定可能**解决该问题，卸载cmake时也会删除ros的部分内容，请**谨慎选择并做好备份**

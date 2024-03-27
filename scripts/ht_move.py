import rospy
from ros_ht_msg.msg import ht_control
from ros_ht_msg.msg import control

# 创建HT控制消息
def create_ht_control_msg(mode: int, x: int = 0, y: int = 0, z: int = 0) -> ht_control:
    control_msg = ht_control()
    control_msg.mode = mode
    control_msg.x = x
    control_msg.y = y
    control_msg.z = z
    return control_msg

def get_time(position: float, speed: float) -> int:
    """
    根据给定的位置和速度计算时间（单位：毫秒）。
    
    参数:
    position: float，需要移动的位置（单位：米）。
    speed: float，移动的速度（单位：米/秒）。
    
    返回值:
    int，计算得到的时间（单位：毫秒）。
    """
    if speed == 0:
        rospy.logerr("速度为0，无法计算时间。")  # 当速度为0时，记录错误信息并返回0
        return 0
    return int((position * 1000) / speed)  # 计算时间并返回整数值

def execute_command_sequence(speed: int, gt1: int, gt2: int):
    """
    执行命令序列，使机器人按照指定速度和次数进行前后和左右移动。

    参数:
    - speed: int, 移动速度，单位为mm/s。
    - gt1: int, 前后移动的次数。
    - gt2: int, 左右移动的次数。

    无返回值。
    """
    pub = rospy.Publisher("/HT_Control", ht_control, queue_size=100) 
    rate = rospy.Rate(1)  

    # 执行前后移动
    for count in range(gt1):
        pub.publish(create_ht_control_msg(mode=1, x=speed))  
        rospy.loginfo(f"第{count+1}次: 向 前/后 行进: mode:FTFD  速度:{speed} mm/s \n")  
        rate.sleep()  

    # 执行左右移动
    for count in range(gt2):
        pub.publish(create_ht_control_msg(mode=1,y=speed))  
        rospy.loginfo(f"第{count+1}次: 向 左/右 行进: mode:FTFD 速度:{speed} mm/s \n")  
        rate.sleep()  

def callback_sub(msg: control):
    """
    根据接收到的控制信息，计算并执行对应的命令序列。
    
    参数:
    - msg: control 类型，包含速度和位置信息的消息对象。
    
    该函数不返回任何值，但会根据消息内容计算出两个目标到达时间，并调用命令执行函数。
    """
    speed      = msg.speed       
    position_x = msg.position_x  
    position_y = msg.position_y  

    # 根据X轴和Y轴的位置信息及速度，计算到达目标的时间
    gt1 = get_time(position_x, speed)
    gt2 = get_time(position_y, speed)

    # 执行速度、第一个目标到达时间和第二个目标到达时间对应的命令序列
    execute_command_sequence(speed, gt1, gt2)

if __name__ == "__main__":
    try:
        rospy.init_node("pub_command2")
        sub = rospy.Subscriber("Control", control, callback_sub, queue_size=100)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
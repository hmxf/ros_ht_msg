import rospy
from ros_ht_msg.msg import ht_control

def HTcontrolfunc(mode=1, x=0, y=0, z=0):
    control = ht_control()
    control.mode = mode
    control.x = x
    control.y = y
    control.z = z
    return control

def dopub(pub, control):
    """
    指令发布
    pub: 发布端
    control: 命令指令
    发布一次控制指令后会休眠 1s
    """
    try:
        rospy.loginfo("指令发送\n")
        pub.publish(control)
    except Exception as e:
        rospy.logerr(f"发布指令失败: {str(e)}")

    du_x = rospy.Duration(1)
    rospy.sleep(du_x)

def move(time, direction, speed, pub, control):
    """
    控制底盘移动
    direction: 移动方向（'forward', 'backward', 'left', 'right', 'rotate'）
    time: 移动时间（单位：秒）
    speed: 相应方向的速度
    pub: 发布实例
    control: 移动指令
    """
    directions_map = {
        "forward": ("前", control.x),
        "backward": ("后", -control.x),
        "left": ("左", control.y),
        "right": ("右", -control.y),
        "rotate": ("旋转", control.z),
    }
    
    for _ in range(time):
        dopub(pub, control)

        if speed != 0:
            direction_text, value = directions_map[direction]
            rospy.loginfo(f"向{direction_text}行进: mode 为 FTFD(四轮四转)，速度 = {abs(value)} mm/s ")

    if direction == "rotate":
        mode = "阿克曼/自转"
        rospy.loginfo(f"原地{direction_text}: mode 为 {mode}，速度 = {value} degree/min")

if __name__ == "__main__":
    
    try:
        rospy.init_node("pub_command1")
        pub = rospy.Publisher("/HT_Control", ht_control, queue_size=100)
        rospy.sleep(rospy.Duration(5))
    except rospy.ROSException as e:
        rospy.logerr(f"初始化节点或创建发布者失败: {str(e)}")

    # 创建指令对象
    control_forwards = HTcontrolfunc(1, 100)  # 前进
    control_left = HTcontrolfunc(1, y=100)     # 左平移
    control_backwards = HTcontrolfunc(1, -100)  # 后退
    control_revolve = HTcontrolfunc(0, 0, 0, 1080)  # 原地旋转 180 度

    # 实验一：前进 20s 后，左平移 2s，后退 20s
    move(20, "forward", control_forwards.x, pub, control_forwards)
    move(2, "left", control_left.y, pub, control_left)
    move(22, "backward", control_backwards.x, pub, control_backwards)

    # 实验二：前进 20s 后，原地旋转，继续前进20s
    move(20, "forward", control_forwards.x, pub, control_forwards)
    move(2, "rotate", control_revolve.z, pub, control_revolve)
    move(20, "backward", control_backwards.x, pub, control_backwards)
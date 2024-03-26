import rospy
from ros_ht_msg.msg import ht_control
from ros_ht_msg.msg import control

# 给话题参数：
def HTcontrolfunc(mode = 1,x = 0,y = 0,z = 0):
    control = ht_control()
    control.mode = mode
    control.x = x
    control.y = y
    control.z = z
    return control


def command_sequence():
    """
    定义的控制方式的命令序列：前进/后退、左右平移、原地旋转
    """

def get_time(p,s):
    return (p*1000)/s

# 接受命令的回调：
def callback_sub(msg):
    Speed = msg.speed         #  单位：mm/s
    Px    = msg.position_x    #  单位：m
    Py    = msg.position_y    #  单位：m

    gt1   = get_time(Px,Speed)
    gt2   = get_time(Py,Speed)
    
    pub =  rospy.Publisher("/HT_Control", ht_control, queue_size=100)
    rate = rospy.Rate(1) # 1Hz

    for count in range(get1):  
        pub.publish(HTcontrolfunc(x=Speed))
        rospy.loginfo("向 前/后 行进: mode:FTFD\n 速度:%d mm/s \n",Speed)
        rate.sleep()  # 按照1Hz频率等待

    for count in range(gt2):  
        pub.publish(HTcontrolfunc(y=Speed))
        rospy.loginfo("向 左/右 行进: mode:FTFD\n 速度:%d mm/s \n",Speed)
        rate.sleep()  # 按照1Hz频率等待


if __name__ == "__main__":
    
    rospy.init_node("pub_command")
    rospy.sleep(rospy.Duration(5))
    sub = rospy.Subscriber("Control",control,callback_sub,queue_size=100)
    rospy.spin()
    
    

    
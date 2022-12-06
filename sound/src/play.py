import os
import rospy
from std_msgs.msg import Int32

def db_playsound(param: Int32):
    rospy.loginfo("cum")
    os.system(f"play -n synth sin {param.data} trim 0 00:01")

play_sub = rospy.Subscriber('/play_sound', Int32, db_playsound)

if __name__ == "__main__":
    rospy.loginfo("Piss")
    rospy.init_node("sound_node")
    rospy.spin()
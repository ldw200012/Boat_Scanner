import rospy

if __name__ == '__main__':
    rospy.init_node('nodes_processtime_handler')

    node1_processtime = rospy.get_param("node1_processtime", 0.00)
    node2_processtime = rospy.get_param("node2_processtime", 0.00)

    node1_max_length = len("[vessel_pointcloud_scanner] ==> [detection.py] node \033[91mRUNNING [0.00]ms\033[0m .....")
    print(" " * node1_max_length, end="\r")
    print(f"[vessel_pointcloud_scanner] ==> [detection.py] node \033[91mRUNNING [{node1_processtime:.2f}]ms\033[0m " + "." * self.dot_cnt, end="\r", flush=True)


    self.dot_cnt = (self.dot_cnt+1) if self.dot_cnt<self.dot_cnt_max else 0


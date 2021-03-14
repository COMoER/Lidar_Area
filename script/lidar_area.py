#!/home/sjtu/anaconda3/envs/yolov5/bin/python
import rospy
import cv2
import numpy as np
import threading
import kdtree

from lidar_ros_area.msg import Grid
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
import ctypes
import inspect

import sys
sys.path.append("/home/sjtu/DATA/Workspace/zy_stock/yolov5_4.0_radar/")
import tool_Locator

area_node_name = "/lidar_ros_area_node/livox_pc"

left_pos = [65.0, 0.0, -80.0]  # AprilTag整体左下角世界坐标
size = 387  # 图案宽度
point1 = [28.5, 152.5]  # 图案左下角于板上位置，整体左下角为坐标原点

from camera import K_0,C_0


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

class Radar:
    __init_flag = False
    __working_flag = False

    __threading = None

    __pub_threading = None

    __msg = None

    __pub = rospy.Publisher("camera_pose",Float32MultiArray,queue_size=10)

    __lock = threading.Lock()

    __pose_init = False

    __grids_size = None
    __grids_center_z = None

    __grids_init = False


    @staticmethod
    def __callback_area(data):
        Radar.__lock.acquire()

        if not Radar.__grids_init:
            Radar.__grids_init = True

        Radar.__now_t = data.header.seq
        Radar.__grids_size = data.size
        Radar.__grids_center_z = data.center_z
        Radar.__lock.release()

    def get_pose(self,frame):
        if not Radar.__pose_init:
            # object_points = np.array(
            #     [[left_pos[0] + point1[0] + size, left_pos[1] + point1[1], left_pos[2]],  # right_down
            #      [left_pos[0] + point1[0], left_pos[1] + point1[1], left_pos[2]],  # left_down
            #      [left_pos[0] + point1[0], left_pos[1] + point1[1] + size, left_pos[2]],  # left_top
            #      [left_pos[0] + point1[0] + size, left_pos[1] + point1[1] + size, left_pos[2]]
            #      # right_top
            #      ], dtype=np.float64)
            object_points = np.array(
                    [[0.46,0,0],  # right_down
                     [0,0,0],  # left_down
                     [0,0.46,0],  # left_top
                     [0.46,0.46,0]
                     # right_top
                     ], dtype=np.float64)
            R, tvec = tool_Locator.locate(frame.copy(), object_points, K_0, C_0, apriltag_family='tag25h9', debug=False)[0]

            T = np.zeros((3,4),np.float32)
            T[0:3, 0:3] = R
            T[0:3, 3] = tvec.reshape(-1)


            Radar.__msg = Float32MultiArray(data = T.reshape(-1).tolist())

            Radar.__pose_init = True

            return R,tvec
        print("Get pose already!")
        return None,None

    def update_pose(self, frame):
        object_points = np.array(
            [[0.46, 0, 0],  # right_down
             [0, 0, 0],  # left_down
             [0, 0.46, 0],  # left_top
             [0.46, 0.46, 0]
             # right_top
             ], dtype=np.float64)
        R, tvec = tool_Locator.locate(frame.copy(), object_points, K_0, C_0, apriltag_family='tag25h9', debug=False)[
            0]

        T = np.zeros((3, 4), np.float32)
        T[0:3, 0:3] = R
        T[0:3, 3] = tvec.reshape(-1)

        d = MultiArrayDimension()
        d.size = 12
        d.stride = 1
        label = input("which label:")
        d.label = label
        Radar.__msg = Float32MultiArray()
        Radar.__msg.layout.dim.append(d)
        Radar.__msg.data = T.reshape(-1).tolist()
        print(T)
        Radar.__pose_init = True


    @staticmethod
    def __begin():
        rospy.init_node('radar_main', anonymous=True)
        rospy.Subscriber(area_node_name, Grid,Radar.__callback_area)
    @staticmethod
    def __main_loop():
        rospy.spin()

    @staticmethod
    def __loop_pub():
        rospy.loginfo("Start Publish Pose.")
        r = rospy.Rate(50)
        while(not rospy.is_shutdown()):
            Radar.__pub.publish(Radar.__msg)
            r.sleep()



    def __init__(self,rows,cols,channels):
        self.rows = rows
        self.cols = cols
        self.channels = channels
        if not Radar.__init_flag:
            Radar.__begin()
            Radar.__init_flag = True
            Radar.__threading=threading.Thread(target = Radar.__main_loop,daemon=True)
            Radar.__pub_threading = threading.Thread(target = Radar.__loop_pub,daemon=True)

            kdtree()

    def start(self):
        if not Radar.__working_flag:
            Radar.__threading.start()
            Radar.__working_flag = True
            Radar.__pub_threading.start()
    def stop(self):
        if Radar.__working_flag:
            stop_thread(Radar.__threading)
            Radar.__working_flag = False
            if Radar.__pose_init:
                stop_thread(Radar.__pub_threading)

    def __del__(self):
        self.stop()

        

if __name__ == '__main__':
    from camera import HT_Camera
    cap = HT_Camera()
    flag,frame = cap.read()
    cv2.namedWindow("frame",cv2.WINDOW_NORMAL)

    print(K_0)
    print(C_0)

    r = Radar(3,3,3)

    cv2.imshow("frame",frame)
    key = cv2.waitKey(80)
    r.update_pose(frame)

    r.start()

    while(flag and key != ord('q')&0xFF):
        cv2.imshow("frame",frame)
        key = cv2.waitKey(80)
        if key == ord('s') & 0xFF:
            r.update_pose(frame)
        flag, frame = cap.read()

    del r









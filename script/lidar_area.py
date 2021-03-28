#!/home/sjtu/anaconda3/envs/yolov5/bin/python
import rospy
import cv2
import numpy as np
import threading
import kdtree
import math

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

K = 3

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

def ceil(x):
    return int(math.ceil(x))

def floor(x):
    return int(math.floor(x))

class Radar:
    __init_flag = False
    __working_flag = False

    __threading = None

    __pub_threading = None

    __msg = None

    __pub = rospy.Publisher("camera_pose",Float32MultiArray,queue_size=10)

    __lock = threading.Lock()

    __pose_init = False

    __grids_init = False

    __grids_size = None

    __grids_center_z = None


    @staticmethod
    def __callback_area(data):
        Radar.__lock.acquire()

        if not Radar.__grids_init:
            Radar.__grids_init = True
            rospy.loginfo("Receive grid message!")

        Radar.__now_t = data.header.seq
        Radar.__grids_size = np.array(data.size,np.int)
        Radar.__grids_center_z = np.float32(data.center_z)
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

            self.rvec = cv2.Rodrigues(R)[0]
            self.tvec = tvec


            Radar.__msg = Float32MultiArray(data = T.reshape(-1).tolist())

            Radar.__pose_init = True

            return R,tvec
        print("Get pose already!")
        return None,None

    # def update_pose(self, frame):
    #     object_points = np.array(
    #         [[0.46, 0, 0],  # right_down
    #          [0, 0, 0],  # left_down
    #          [0, 0.46, 0],  # left_top
    #          [0.46, 0.46, 0]
    #          # right_top
    #          ], dtype=np.float64)
    #     R, tvec = tool_Locator.locate(frame.copy(), object_points, K_0, C_0, apriltag_family='tag25h9', debug=False)[0]
    #
    #     T = np.zeros((3, 4), np.float32)
    #     T[0:3, 0:3] = R
    #     T[0:3, 3] = tvec.reshape(-1)
    #
    #
    #     d = MultiArrayDimension()
    #     d.size = 12
    #     d.stride = 1
    #     label = input("which label:")
    #     d.label = label
    #     Radar.__msg = Float32MultiArray()
    #     Radar.__msg.layout.dim.append(d)
    #     Radar.__msg.data = T.reshape(-1).tolist()
    #     print(T)
    #     Radar.__pose_init = True

    def tree_init(self):
        assert Radar.__pose_init
        if not self.__tree_init:
            rospy.loginfo("Grids in python initing..")
            time = rospy.get_time()
            img_points = cv2.projectPoints(self.grids,self.rvec,self.tvec,K_0,C_0)[0].reshape(-1,2).tolist()
            self.img_points = {}
            for i,p in enumerate(img_points):
                p = tuple(p)
                self.img_points[p] = i
            self.__tree = kdtree.create(img_points,2)

            self.__tree_init = True
            rospy.loginfo("Grids in python init! using %0.5f",rospy.get_time()-time)

    def search_z(self, u, v,img):
        assert self.__tree_init
        while(not Radar.__grids_init):
            rospy.logerr("Grids not received...")
        Radar.__lock.acquire()
        knn = self.__tree.search_knn((u,v),self.k)
        index = [self.img_points[tuple(point.data)] for point,dis in knn]
        size = np.float32(Radar.__grids_size[index])
        dsize = size / np.sum(size)
        center_z = np.float32(Radar.__grids_center_z[index])

        z = np.max(Radar.__grids_center_z)

        for i,p in enumerate(self.img_points.keys()):
            if p[0] >= 0  and p[0] < 3088 and p[1] >= 0 and p[1] < 2064:
                if Radar.__grids_center_z[i] > 0:
                    img[int(p[1]),int(p[0])] = (255,255,255)
                    print(p,':',Radar.__grids_size[i])
                else:
                    img[int(p[1]), int(p[0])] = (255,0,0)

        Radar.__lock.release()
        # return img
        return img,np.sum((center_z * dsize))

    # def search_z_demo(self, u, v):
    #     assert self.__tree_init
    #     time = rospy.get_time()
    #     knn = self.__tree.search_knn((u,v),self.k)
    #     index = [self.img_points[tuple(point.data)] for point,dis in knn]
    #     rospy.loginfo("using %.05f to search"%(rospy.get_time()-time))
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

    def __init__(self,x_min,x_max,y_min,y_max,z_min,z_max,vol_size,k):
        if not Radar.__init_flag:

            rows = ceil((x_max - x_min) / vol_size)

            cols = ceil((y_max - y_min) / vol_size)

            channels = ceil((z_max - z_min) / vol_size)

            x = np.linspace((x_min+vol_size/2),(x_min+vol_size/2+vol_size*(rows-1)),rows,endpoint=True)

            y = np.linspace((y_min + vol_size / 2),(y_min + vol_size / 2 + vol_size * (cols - 1)),cols, endpoint=True)

            z = np.linspace((z_min + vol_size / 2),(z_min + vol_size / 2 + vol_size * (channels - 1)),channels,endpoint = True)

            mesh_x,mesh_y,mesh_z = np.meshgrid(x,y,z)

            self.grids = np.stack((mesh_y.reshape(-1),mesh_x.reshape(-1),mesh_z.reshape(-1)),axis = 1)

            self.__tree_init = False

            self.k = k

            Radar.__begin()
            Radar.__init_flag = True
            Radar.__threading=threading.Thread(target = Radar.__main_loop,daemon=True)
            Radar.__pub_threading = threading.Thread(target = Radar.__loop_pub,daemon=True)


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

        
RECORD = True
if __name__ == '__main__':
    from camera import HT_Camera
    from datetime import datetime

    title = datetime.now().strftime('%Y-%m-%d %H-%M-%S')
    # writer = cv2.VideoWriter(
    #         "/home/sjtu/DATA/Workspace/zy_stock/record_%s.mp4"%title,
    #         cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 30.0, (3088,2064))
    # print(writer)
    # cap = HT_Camera()
    cap = cv2.VideoCapture("/home/sjtu/DATA/Workspace/zy_stock/yolov5_4.0_radar/lidar_area/src/lidar_area/data/record_2021-03-27 23-02-31.mp4")
    flag,frame = cap.read()
    cv2.namedWindow("frame",cv2.WINDOW_NORMAL)


    xmax = rospy.get_param("/lidar_ros_area_node/xmax")
    xmin = rospy.get_param("/lidar_ros_area_node/xmin")
    ymax = rospy.get_param("/lidar_ros_area_node/ymax")
    ymin = rospy.get_param("/lidar_ros_area_node/ymin")
    zmax = rospy.get_param("/lidar_ros_area_node/zmax")
    zmin = rospy.get_param("/lidar_ros_area_node/zmin")
    vol_size = rospy.get_param("/lidar_ros_area_node/vol_size")

    r = Radar(xmin,xmax,ymin,ymax,zmin,zmax,vol_size,K)
    #
    cv2.imshow("frame",frame)
    key = cv2.waitKey(80)
    # # r.update_pose(frame)
    #
    # # r.update_pose(frame)

    r.get_pose(frame)
    #
    r.tree_init()
    #
    r.start()
    #
    rect = cv2.selectROI("frame",frame,False)
    while(flag and key != ord('q')&0xFF):


        cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3)
        u,v = int(rect[0]+rect[2]/2),int(rect[1]+rect[3]/2)
        frame, z = r.search_z(u, v, frame)
        # if key == ord('c') & 0xFF:
        #     cv2.imwrite("/home/sjtu/DATA/Workspace/zy_stock/c.jpg",frame)
        if key == ord('r') & 0xFF:
            rect = cv2.selectROI("frame", frame, False)


        if key == ord('s') & 0xFF:

            print(np.concatenate(
                (cv2.undistortPoints(np.float32([u, v]), K_0, C_0).reshape(-1, 2), np.ones((1, 1), np.float32)),
                axis=1) * z)
        cv2.imshow("frame",frame)
        flag, frame = cap.read()

        # writer.write(frame)
        key = cv2.waitKey(0)
        # writer.write(frame)
    #
    # #perform the delete function to end the threadings
    # writer.release()
    del r
    # writer.release()








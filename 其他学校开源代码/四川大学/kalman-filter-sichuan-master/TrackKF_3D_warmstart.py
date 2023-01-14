import cv2
import numpy as np



class KalmanFilter(object):

    def __init__(self):
        self.error_frame_size = 6 #时间序列错误帧窗口大小
        self.get_target_size = 4   #卡尔曼重新寻找目标缓冲时间
        self.kalman = cv2.KalmanFilter(6, 3) # 4：状态数，包括（x，y，dx，dy）坐标及速度（每次移动的距离）；2：观测量，能看到的是坐标值
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                                  [0, 1, 0, 0, 0, 0],
                                                  [0, 0, 1, 0, 0, 0]], np.float32) # 系统测量矩阵
        self.kalman.transitionMatrix = np.array([[1, 0, 0, 1, 0, 0],
                                                 [0, 1, 0, 0, 1, 0],
                                                 [0, 0, 1, 0, 0, 1],
                                                 [0, 0, 0, 1, 0, 0],
                                                 [0, 0, 0, 0, 1, 0],
                                                 [0, 0, 0, 0, 0, 1]],      np.float32) # 状态转移矩阵
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0, 0, 0],
                                                [0, 1, 0, 0, 0, 0],
                                                [0, 0, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 0, 0],
                                                [0, 0, 0, 0, 1, 0],
                                                [0, 0, 0, 0, 0, 1]], np.float32)*0.03 # 系统过程噪声协方差
        self.current_measurement = np.array((3, 1), np.float32)
        self.last_measurement = np.array((3, 1), np.float32)
        self.current_prediction = np.zeros((3, 1), np.float32)
        self.last_prediction = np.zeros((3, 1), np.float32)
        self.error_frame = 0
        self.lostflag = 1
        self.counter = self.get_target_size

    def track(self,x,y,z):
        self.last_prediction = self.current_prediction # 把当前预测存储为上一次预测
        self.last_measurement = self.current_measurement # 把当前测量存储为上一次测量

        if self.lostflag == 0:               #有目标的情况下丢目标
            if (abs(self.last_measurement[0] - x) > 15 or abs(self.last_measurement[1] - y) > 15 or abs(self.last_measurement[2] - z) > 15): #步兵移动速度3.5，一帧0.04s
                self.error_frame = self.error_frame + 1
            else:
                self.error_frame = 0

            if self.error_frame > 0 and self.error_frame < self.error_frame_size:
                  self.current_measurement = np.array([[np.float32(self.last_prediction[0])],
                                                 [np.float32(self.last_prediction[1])],
                                                 [np.float32(self.last_prediction[2])]])
            elif self.error_frame >= self.error_frame_size:
                  self.current_measurement = np.array([[np.float32(x)], [np.float32(y)], [np.float32(z)]]) # 当前测量
                  self.lostflag = 1
                  self.error_frame = 0
            elif  self.error_frame == 0:
                  self.current_measurement = np.array([[np.float32(x)], [np.float32(y)], [np.float32(z)]])  # 当前测量
                  self.lostflag = 0
        if self.lostflag == 1:
            self.current_measurement = np.array([[np.float32(x)], [np.float32(y)], [np.float32(z)]])



        self.kalman.correct(self.current_measurement) # 用当前测量来校正卡尔曼滤波器
        self.current_prediction = self.kalman.predict() # 计算卡尔曼预测值，作为当前预测


        #lmx, lmy, lmz = self.last_measurement[0], self.last_measurement[1], self.last_measurement[2] # 上一次测量坐标
        cmx, cmy, cmz = self.current_measurement[0], self.current_measurement[1], self.current_measurement[2] # 当前测量坐标
        #lpx, lpy, lpz = self.last_prediction[0], self.last_prediction[1], self.last_prediction[2] # 上一次预测坐标

        cpx, cpy, cpz = self.current_prediction[0], self.current_prediction[1], self.current_prediction[2] # 当前预测坐标


        if self.lostflag == 1:
            self.counter = self.counter - 1
            if(self.counter == 0):
                self.lostflag = 0
                self.counter = self.get_target_size
            return cmx,cmy,cmz

        else:
            return cpx,cpy,cpz

        

if __name__ == '__main__':

     yaw =   np.array([110,112,114,256,258,260,262,264,268,115,116,119,120], np.float)
     pitch = np.array([110,112,114,256,258,260,262,264,268,115,116,119,120], np.float)
     roll =  np.array([110,112,114,256,258,260,262,264,268,115,116,119,120], np.float)
     a = KalmanFilter()

     for i in range(len(yaw)):
        nx,ny,nz = a.track(yaw[i],pitch[i],roll[i])
        print("z:")
        print(int(nz))



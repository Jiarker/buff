#scurm-2021-vision-master
1. 模块功能：

输入：输入一个object在多维坐标系下的坐标

迭代：模块内部通过卡尔曼滤波的五个方程，整合预测值及实际值，修正滤波器，得到在协方差最小情况下的预测坐标（如果当前观测值突变为0，则以上一次的预测值作为本次观测值）

输出：该object在下一时刻的坐标

简而言之就是实现了对单一目标的追踪，并且具有丢目标处理

2. 使用样例：
该模块封装为类

Import...

a = KalmanFilter()

next_x , next_y = a.track(x , y) #x,y为当前检测到的坐标，next_x,next_y为预测值

如需修改目标维数，或去除坐标突变处理，或返回当前实际值等
需要在KalmanFilter类中修改初始化的矩阵部分等

3. 模块参数：
transitionMatrix       #Latter 1: Larger, faster regression

processNoiseCov        #3: Larger, slower regression

measurementNoiseCov    #1: Larger, quicker regression

error_frame_size = 6   #时间序列错误帧窗口大小

get_target_size = 5    #卡尔曼重新寻找目标缓冲时间
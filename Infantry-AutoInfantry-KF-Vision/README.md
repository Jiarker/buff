## 预测模块 Predictor
路径： ./src/Solver/Predictor.cpp
```c++
/**
 *  @brief  初始化状态值
 *  @param  coord   相机坐标系下目标的坐标
 *  @param  gyro_pose   IMU数据
 */
    void initState(cv::Point3f coord, GyroPose gyro_pose);

/**
 *  @brief  根据世界坐标进行预测
 *  @param  coord   相机坐标系下目标的坐标
 *  @param  gyro_pose   IMU数据
 *  @return 世界坐标系
 */
    cv::Point3f predict(cv::Point3f coord, GyroPose gyro_pose);
```

坐标转换函数位于 ./src/Extend/Tools.cpp中  
具体公式：$p'=q*p*q^{-1}$  

P.S 该模块中所用图片处理延迟(dt)，通过陀螺时间戳之间的差值获取
## 弹道补偿模块 Predictor
路径： ./include/Solver/Predictor.h


## 线程模块 ImageProgress
路径： ./src/ImageProgress/ImageProgress.cpp  

线程算法思路：各干各的，有需要就从缓存区取，将自己的产物放入缓存区

读取线程: **ReadIMU2()**

## can设置
1. 查看can口号，一般为“can0”
```
ifconfig –a 
```

2. 根据can口号（以下为can0）设置

```
sudo ip link set can0 type can bitrate 1000000    #设置波特率
sudo ifconfig can0 up    # 使能can口
```

P.S.
```
ip -details link show can0  # 查看can口信息
```

## 日志
1. 侧面装甲板测距存在较大误差，偏大
2. 抖动问题，准备修改通信协议后做时间同步，已修改
3. 反陀螺需要更多测试
4. 测试相机帧率<=285帧(0.0035s)，大于0.0035s时，约为曝光时间
5. 测试妙算相机滞后于IMU 约4ms
6. 相机出现段错误，需要异常处理或者直接“看门狗”，已做异常处理未测试
7. 大符拟合取得阶段性成果
8.  大符识别需要更Robust的处理
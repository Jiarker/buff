import numpy as np
from filterpy.kalman import KalmanFilter, UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import time

if __name__ == "__main__":
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from filterpy.common import Saver


DT = 0.05  # 初始化时的检测间隔时间


def fx_CV_3D(x, dt):
    '''
    CV 模型状态转移方程
    '''
    F = np.array([[1, 0, 0, dt, 0, 0],
                 [0, 1, 0, 0, dt, 0],
                 [0, 0, 1, 0, 0, dt],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]], dtype=float)
    
    return np.dot(F, x)


def fx_CVA_3D(x, dt):
    '''
    CVA 模型状态转移函数
    '''
    F = np.array([[1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, dt, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, dt, 0, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0, dt, 0, 0, 0],
                  [0, 0 ,0, 0, 1, 0, 0, dt, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],], dtype=float)

    return np.dot(F, x)


def fx_CTRV_3D(x, dt):
    '''
    CTRV 状态转移函数
    x: [x, y, z, v, theta, phi, theta_v, phi_v]
    '''
    v_ = x[3]
    theta, phi, theta_v, phi_v = x[4], x[5], x[6], x[7]
    t_sum, t_diff = theta + phi, theta - phi
    d_sum, d_diff = theta_v + phi_v, theta_v - phi_v
    d_z = v_ * np.sin(phi) * dt if abs(phi_v) < 1e-5 else v_ / phi_v * (-np.cos(phi + phi_v * dt) + np.cos(phi))
    F = []
    if abs(d_sum) < 1e-5 and abs(d_diff) < 1e-5:
        F = np.array([
            0.5 * v_ * ( \
                (np.cos(t_sum) + np.cos(t_diff)) * dt \
            ),
            0.5 * v_ * ( \
                (np.sin(t_sum) + np.sin(t_diff)) * dt \
            ),
            d_z,
            0.,
            theta_v,
            phi_v,
            0.,
            0.,
        ])
    elif abs(d_sum) < 1e-5:
        F = np.array([
            0.5 * v_ * ( \
                (np.cos(t_sum) * dt) + \
                (1. / d_diff * (np.sin(t_diff + d_diff * dt) - np.sin(t_diff))) \
            ),
            0.5 * v_ * ( \
                (np.sin(t_sum) * dt) + \
                (1. / d_diff * (-np.cos(t_diff + d_diff * dt) + np.cos(t_diff))) \
            ),
            d_z, 0., theta_v, phi_v, 0., 0.,
        ])
    elif abs(d_diff) < 1e-5:
        F = np.array([
            0.5 * v_ * ( \
                (1. / d_sum * (np.sin(t_sum + d_sum * dt) - np.sin(t_sum))) + \
                (np.cos(t_diff) * dt) \
            ),
            0.5 * v_ * ( \
                (1. / d_sum * (-np.cos(t_sum + d_sum * dt) + np.cos(t_sum))) + \
                (np.sin(t_diff) * dt) \
            ),
            d_z, 0., theta_v, phi_v, 0., 0.,
        ])
    else:
        F = np.array([
            0.5 * v_ * ( \
                (1. / d_sum * (np.sin(t_sum + d_sum * dt) - np.sin(t_sum))) + \
                (1. / d_diff * (np.sin(t_diff + d_diff * dt) - np.sin(t_diff))) \
            ),
            0.5 * v_ * ( \
                (1. / d_sum * (-np.cos(t_sum + d_sum * dt) + np.cos(t_sum))) + \
                (1. / d_diff * (-np.cos(t_diff + d_diff * dt) + np.cos(t_diff))) \
            ),
            d_z, 0., theta_v, phi_v, 0., 0.,
        ])

    return x + F


def hx(x):
    return x[:3].copy()


class track_3D:
    '''
    3D 目标跟踪预测
    '''

    def __init__(self, method="CV"):
        '''3D 目标检测
        使用无损卡尔曼滤波单目标跟踪/预测

        Parameters:
            method: str: 3D 运动模型. "CV"/"CVA"/"CTRV"
        '''

        if not (method in ["CV", "CVA", "CTRV"]):
            raise ValueError("Method should in \"CV\"/\"CVA\"/\"CTRV\"")

        self.method = method

        if method == "CVA":
            points = MerweScaledSigmaPoints(10, alpha=1., beta=2., kappa=-1) # Gaussian process
            self.kf = UnscentedKalmanFilter(dim_x=10, dim_z=3, dt=DT, fx=fx_CVA_3D, hx=hx, points=points)
            self.kf.x = np.ones(10, dtype=float)
            self.kf.P[3:, 3:] *= 1.
            self.kf.P *= 1.
            self.kf.R = np.diag([0.001, 0.001, 0.001])
            self.kf.Q = Q_discrete_white_noise(2, dt=DT, block_size=5)


        elif method == "CTRV":
            points = MerweScaledSigmaPoints(8, alpha=1., beta=2., kappa=-1)
            self.kf = UnscentedKalmanFilter(dim_x=8, dim_z=3, dt=DT, hx=hx, fx=fx_CTRV_3D, points=points)
            self.kf.x = np.ones(8, dtype=float)
            self.kf.P[3:, 3:] *= 2
            self.kf.P *= 1.
            self.kf.R = np.diag([0.001, 0.001, 0.001])
            self.kf.Q = Q_discrete_white_noise(2, dt=DT, var=0.1, block_size=4)

        else:
            points = MerweScaledSigmaPoints(6, alpha=1., beta=2., kappa=-1)
            self.kf = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=DT, hx=hx, fx=fx_CV_3D, points=points)
            self.kf.x = np.ones(6, dtype=float)
            self.kf.P[3:, 3:] *= 1.
            self.kf.P *= 1.
            self.kf.R = np.diag([0.001, 0.001, 0.001])
            self.kf.Q = Q_discrete_white_noise(2, dt=DT, var=0.1, block_size=3)

        self.time_stamp = time.time()   # 预测时间戳

    def predict(self, dt=None):
        '''
        预测运动轨迹
        
        Parameters:
            dt: float
                预测dt(s)时间后的状态
        '''
        if dt is None:
            dt = time.time() - self.time_stamp
        
        self.kf.predict(dt=dt)
        self.time_stamp = time.time()
        return self.kf.x[:3]

    def update(self, z):
        '''
        使用观测值更新滤波器
        
        Parameters:
            z: numpy.array
                3维空间坐标(x, y, z)
        '''
        self.kf.update(z)


if __name__ == "__main__":
    tracker = track_3D(method="CVA")
    
    z = np.linspace(0, 100, 50)
    x = (0.1 * z) ** 2
    y = z
    
    g_true = np.concatenate((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)), axis=1)
    zs = g_true + np.random.normal(size=g_true.shape, scale=2)

    res = []

    for i in range(zs.shape[0]):
        res.append(tracker.predict(dt=DT))
        tracker.update(zs[i, :])

    res = np.array(res)
    print("err: {}".format(np.linalg.norm(res - g_true, axis=1).mean()))

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.plot(x, y, z, c="g")
    ax.scatter(zs[:, 0], zs[:, 1], zs[:, 2], c="r")
    ax.plot(res[:, 0], res[:, 1], res[:, 2], c="b")
    plt.show()
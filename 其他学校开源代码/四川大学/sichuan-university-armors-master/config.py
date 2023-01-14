'''
Author: holakk
Date: 2021-03-25 16:51:53
LastEditors: holakk
LastEditTime: 2021-03-25 16:56:26
Description: file content
'''
from Armors.module import GlobalConfig


class config:
    def __init__(self, team='R'):
        """

        :param team: 我方机器人的阵营。 R代表红、B代表蓝
        """
        self.WIDTH = GlobalConfig.camera_resol_widt
        self.HEIGHT = GlobalConfig.camera_resol_high

        # 形态学操作参数
        self.open = GlobalConfig.armor_detect_config['open']
        self.close = GlobalConfig.armor_detect_config['close']
        self.erode = GlobalConfig.armor_detect_config['erode']
        self.dilate = GlobalConfig.armor_detect_config['dilate']

        # 装甲板筛选参数
        self.area_threshold = GlobalConfig.armor_detect_config['area_threshold']
        self.w_h_ratio = GlobalConfig.armor_detect_config['w_h_ratio']
        self.diff_x = GlobalConfig.armor_detect_config['diff_x']
        self.diff_y = GlobalConfig.armor_detect_config['diff_y']
        self.diff_h = GlobalConfig.armor_detect_config['diff_h']

        if team == 'R':
            self.hmin = GlobalConfig.enemy_blue_threshold['hmin']
            self.hmax = GlobalConfig.enemy_blue_threshold['hmax']
            self.smin = GlobalConfig.enemy_blue_threshold['smin']
            self.smax = GlobalConfig.enemy_blue_threshold['smax']
            self.vmin = GlobalConfig.enemy_blue_threshold['vmin']
            self.vmax = GlobalConfig.enemy_blue_threshold['vmax']
        else:
            self.hmin = GlobalConfig.enemy_red_threshold['hmin']
            self.hmax = GlobalConfig.enemy_red_threshold['hmax']
            self.smin = GlobalConfig.enemy_red_threshold['smin']
            self.smax = GlobalConfig.enemy_red_threshold['smax']
            self.vmin = GlobalConfig.enemy_red_threshold['vmin']
            self.vmax = GlobalConfig.enemy_red_threshold['vmax']

'''
Author: holakk
Date: 2021-03-29 21:57:30
LastEditors: holakk
LastEditTime: 2021-03-29 22:06:48
Description: file content
'''
"""
Author: Eric Feng
Date: 2020-11-18 21:06:09
LastEditTime: 2020-11-28 20:36:31
LastEditors: Eric Feng
Description: Armor detect scipt
FilePath: \Armors\armor_detector.py
"""
import numpy as np
import cv2
# from module.ModuleCamera import Camera
from .utils import *
# import .config

__all__ = ["ArmorDetector"]


def create_trackbars(config):
    """
    Create trackbars to adjust params
    """

    def nothing(x):
        """Nothing
        """
        pass

    cv2.namedWindow("color_adjust", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("hmin", "color_adjust", config.hmin, 255, nothing)
    cv2.createTrackbar("hmax", "color_adjust", config.hmax, 255, nothing)
    cv2.createTrackbar("smin", "color_adjust", config.smin, 255, nothing)
    cv2.createTrackbar("smax", "color_adjust", config.smax, 255, nothing)
    cv2.createTrackbar("vmin", "color_adjust", config.vmin, 255, nothing)
    cv2.createTrackbar("vmax", "color_adjust", config.vmax, 255, nothing)

    cv2.namedWindow("mor_adjust", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("open", "mor_adjust", config.open, 30, nothing)
    cv2.createTrackbar("close", "mor_adjust", config.close, 30, nothing)
    cv2.createTrackbar("erode", "mor_adjust", config.erode, 30, nothing)
    cv2.createTrackbar("dilate", "mor_adjust", config.dilate, 30, nothing)


def key_comp(elem):
    k0 = (elem[0][0] - (1024 / 2)) * (elem[0][0] - (1024 / 2))
    k1 = (elem[0][1] - (1280 / 2)) * (elem[0][1] - (1280 / 2))
    return k0 + k1


class ArmorDetector:
    def __init__(self, config, team = 'R', debug=False):
        '''
        '''

        self.config = config

        self.debug = debug

        self.team = team

        if debug:
            create_trackbars(self.config)

    def convert_hsv(self, frame):
        """
        Convert to HSV image.

        :arg frame: origin frame ready to process
        :returns binary, hsv: binary image. hsv converted image.
        """

        # if self.debug:
        #     hmin = cv2.getTrackbarPos('hmin', 'color_adjust')
        #     hmax = cv2.getTrackbarPos('hmax', 'color_adjust')
        #     smin = cv2.getTrackbarPos('smin', 'color_adjust')
        #     smax = cv2.getTrackbarPos('smax', 'color_adjust')
        #     vmin = cv2.getTrackbarPos('vmin', 'color_adjust')
        #     vmax = cv2.getTrackbarPos('vmax', 'color_adjust')
        # else:
        #     hmin = self.config.hmin
        #     hmax = self.config.hmax
        #     smin = self.config.smin
        #     smax = self.config.smax
        #     vmin = self.config.vmin
        #     vmax = self.config.vmax

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self.team == 'R':
            lower_hsv = np.array([100, 43, 150])
            upper_hsv = np.array([124, 255, 255])
            mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
        else:
            lower_hsv0 = np.array([0, 43, 150])
            upper_hsv0 = np.array([13, 255, 255])
            lower_hsv1 = np.array([156, 43, 150])
            upper_hsv1 = np.array([180, 255, 255])
            mask0 = cv2.inRange(hsv, lowerb=lower_hsv0, upperb=upper_hsv0)
            mask1 = cv2.inRange(hsv, lowerb=lower_hsv1, upperb=upper_hsv1)
            mask = mask0 + mask1

        return mask, hsv

    # Morphology modules
    def _open_morphology(self, binary_frame, size: tuple):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, size)
        dst = cv2.morphologyEx(binary_frame, cv2.MORPH_OPEN, kernel)
        return dst

    def _close_morphology(self, binary_frame, size: tuple):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, size)
        dst = cv2.morphologyEx(binary_frame, cv2.MORPH_CLOSE, kernel)
        return dst

    def _erode_morphology(self, binary_frame, size: tuple):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, size)
        dst = cv2.morphologyEx(binary_frame, cv2.MORPH_ERODE, kernel)
        return dst

    def _dilate_morphology(self, binary_frame, size: tuple):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, size)
        dst = cv2.morphologyEx(binary_frame, cv2.MORPH_DILATE, kernel)
        return dst

    # Morphology modules End.

    def _morphology_change(self, input_frame, debug=False):
        """Preprocess images.
        First convert to hsv color space to filter color.
        Then use morphology changes to process image.

        :arg input_frame: image ready to process
        :arg debug: show processed image to window if debug is True.
        :return dst:
        """

        # Get morphology params from trackbars

        if debug:
            open_size = cv2.getTrackbarPos("open", "mor_adjust")
            close_size = cv2.getTrackbarPos("close", "mor_adjust")
            erode_size = cv2.getTrackbarPos("erode", "mor_adjust")
            dilate_size = cv2.getTrackbarPos("dilate", "mor_adjust")
        else:
            open_size = self.config.open
            close_size = self.config.close
            erode_size = self.config.erode
            dilate_size = self.config.dilate


        # HSV binary convert
        dst, _ = self.convert_hsv(input_frame)

        if debug:
            cv2.imshow("binary", dst)

        # Morphology change
        dst = self._open_morphology(dst, (open_size, open_size))
        dst = self._close_morphology(dst, (close_size, close_size))
        dst = self._erode_morphology(dst, (erode_size, erode_size))
        dst = self._dilate_morphology(dst, (dilate_size, dilate_size))

        if debug:
            cv2.imshow("Morphology", dst)
        return dst

    def _find_contours(self, binary, frame=None, debug=False):
        """Finding contours of binary image.
        Use OpenCV to find contours.
        Then delete useless contours.

        :arg binary: binary image (processed by self._morphology_change())
        :arg frame: src frame used to show debug image.(Only used when debug is True)
        :arg debug: show debug image when debug is True

        :return armor_list: armors detected.
        """

        contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = len(contours)

        # 显示debug轮廓信息
        if debug:
            debug_img = cv2.drawContours(frame, contours, -1, (0,255,0), 5)

        if debug:
            data_list = self._data_insert(contours, count, debug, debug_img)
        else:
            data_list = self._data_insert(contours, count)
        
        return data_list

    # @jit
    def _data_insert(self, contours, count, debug=False, debug_img=None):
        data_list = []

        if count > 0:
            for i, contour in enumerate(contours):
                # dict of contour info
                data_dict = dict()

                area = cv2.contourArea(contour)
                rect = cv2.minAreaRect(contour)
                rect_x, rect_y = rect[0]
                rect_w, rect_h = rect[1]
                z = rect[2]

                if(rect_w < rect_h):
                    rect_w, rect_h = rect_h, rect_w
                    z = float(z) + 90

                # 矩形4个顶点
                coor = cv2.boxPoints(rect)

                if debug:
                    cv2.rectangle(debug_img, (coor[0][0], coor[0][1]), (coor[2][0], coor[2][1]), (0, 0, 200), 2)

                # 通过矩形筛选轮廓数据
                data_dict["area"] = area
                data_dict["rx"], data_dict["ry"] = rect_x, rect_y
                data_dict["rh"], data_dict["rw"] = rect_h, rect_w
                data_dict["z"] = z
                data_dict["coo"] = coor

                data_list.append(data_dict)

        if debug:
            cv2.imshow("Detected image", debug_img)

        return data_list

    # @jit
    def _data_select(self, data_list):
        """Select detected data.

        :arg data_list: list detected by self._find_contours.
        
        :return select_list: light_list selected.
        """

        # 第一次筛选
        # 根据灯条大小、比例、角度
        first_select_list = []

        if len(data_list) > 0:
            for iter in data_list:
                data_rh, data_rw = iter["rh"], iter["rw"]
                data_area, data_angle = iter["area"], iter["z"]

                if float(data_rw) >= self.config.w_h_ratio * float(data_rh) \
                    and data_area >= self.config.area_threshold \
                    and abs(data_angle) > 45. and abs(data_angle) < 135.:
                    first_select_list.append(iter)

        n = len(first_select_list)

        second_select_list = []

        for i in range(n):
            for j in range(i + 1, n):
                data_ryi = float(first_select_list[i].get("ry", 0))
                data_ryj = float(first_select_list[j].get("ry", 0))
                data_rhi = float(first_select_list[i].get("rh", 0))
                data_rhj = float(first_select_list[j].get("rh", 0))
                data_rxi = float(first_select_list[i].get("rx", 0))
                data_rxj = float(first_select_list[j].get("rx", 0))
                data_rwi = float(first_select_list[i].get("rw", 0))
                data_rwj = float(first_select_list[j].get("rw", 0))
                data_zi = float(first_select_list[i].get('z', 0))
                data_zj = float(first_select_list[j].get('z', 0))

                l_w = np.sqrt((data_rxi - data_rxj) * (data_rxi - data_rxj) + (data_ryi - data_ryj) * (data_ryi - data_ryj))
                l_h = (data_rwi + data_rwj) / 2.

                if abs(data_zi - data_zj) <= 45. \
                    and l_w >= 1.8 * l_h \
                    and l_w <= 3.1 * l_h:
                    second_select_list.append((first_select_list[i], first_select_list[j]))
                """
                if (abs(data_ryi - data_ryj) <= self.config.diff_y * (data_rhi + data_rhj)) \
                    and (abs(data_rhi - data_rhj) <= self.config.diff_h * max(data_rhi, data_rhj)) \
                    and (abs(data_rxi - data_rxj) <= self.config.diff_x * (data_rwi + data_rwj)):
                    second_select_list.append((first_select_list[i], first_select_list[j]))
                """

        return second_select_list

    # @jit
    def _armor_process(self, selected_data):
        armors = []

        for rect_i, rect_j in selected_data:
            rxi, ryi = float(rect_i.get("rx", 0)), float(rect_i.get("ry", 0))
            cooi = rect_i.get("coo", 0)
            boxi = COO2Vertices(cooi)


            rxj, ryj = float(rect_j.get("rx", 0)), float(rect_j.get("ry", 0))
            cooj = rect_j.get("coo", 0)
            boxj = COO2Vertices(cooj)

            center = [(rxi + rxj) / 2, (ryi + ryj) / 2]
            arm_l = get_right(boxi)
            arm_r = get_left(boxj)
            armor = [center, arm_l[0], arm_l[1], arm_r[0], arm_r[1]]
            armors.append(armor)

        return armors

    def detect(self, input_frame):
        """
        :param input_frame: img ready to compute
        :return armors: list of armors.
                        eg. [arm0, arm1, ...]. For each armor [0]: Center (x, y); [1-4]: 4 end point
        """
        # Resize image as config
        # input_frame = cv2.resize(input_frame, (self.config.WIDTH, self.config.HEIGHT), interpolation=cv2.INTER_CUBIC)
        input_frame = input_frame.copy()

        binary = self._morphology_change(input_frame, debug=self.debug)

        # 轮廓检测结果
        if self.debug:
            data_list = self._find_contours(binary, input_frame, self.debug)
        else:
            data_list = self._find_contours(binary, debug=self.debug)

        selected_list = self._data_select(data_list)

        armor_list = self._armor_process(selected_list)

        return armor_list


# if __name__ == "__main__":
#
#     config_ = config.config('B')
#
#     detector = ArmorDetector(config_,team='B', debug=True)
#     temp = Camera(in_nums=0)
#     #test_frame = temp.getp()
#     exit_signal = False
#     #cv2.imshow("strat",test_frame )
#     #cv2.waitKey(0)
#     while True:
#         frame = cv2.cvtColor(temp.getp(), cv2.COLOR_RGB2BGR)
#
#         test_frame = frame.copy()
#         start = time.time()
#         armor_list = detector.detect(frame)
#         end = time.time()
#         print("time delay: {}\t\tfps: {}".format(end - start, 1. / (end-start)))
#         print("Armor count: {}".format(len(armor_list)))
#         if len(armor_list) > 0:
#             for armor in armor_list:
#                 for point in armor:
#                     cv2.circle(test_frame, (int(point[0]), int(point[1])), 10, (0, 0, 255), 5)
#
#         cv2.imshow("Detected Img", test_frame)
#         if cv2.waitKey(2) & 0xFF == ord('q'):
#                 cv2.destroyAllWindows()
#                 exit_signal = True
#         else:
#             pass
#
#     exit_signal = True


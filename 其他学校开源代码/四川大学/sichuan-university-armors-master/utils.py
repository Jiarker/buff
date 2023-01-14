import numpy as np


def COO2Vertices(box):
    # 获取四个顶点坐标
    left_point_x = np.min(box[:, 0])
    right_point_x = np.max(box[:, 0])
    top_point_y = np.min(box[:, 1])
    bottom_point_y = np.max(box[:, 1])

    left_point_y = box[:, 1][np.where(box[:, 0] == left_point_x)][0]
    right_point_y = box[:, 1][np.where(box[:, 0] == right_point_x)][0]
    top_point_x = box[:, 0][np.where(box[:, 1] == top_point_y)][0]
    bottom_point_x = box[:, 0][np.where(box[:, 1] == bottom_point_y)][0]
    # 上下左右四个点坐标
    return [[top_point_x, top_point_y], [bottom_point_x, bottom_point_y], [left_point_x, left_point_y],
            [right_point_x, right_point_y]]


def get_left(vertices):
    vertices = np.array(vertices)
    lx = np.min(vertices[[0, 1, 3], 0])
    ly = vertices[:, 1][np.where(vertices[:, 0] == lx)][0]

    vec = [[lx, ly], [vertices[2, 0], vertices[2, 1]]]

    if ly < vertices[2, 1]:
        vec = [[vertices[2, 0], vertices[2, 1]], [lx, ly]]

    return vec

def get_right(vertices):
    vertices = np.array(vertices)
    rx = np.max(vertices[[0, 1, 2], 0])
    ry = vertices[:, 1][np.where(vertices[:, 0] == rx)][0]

    vec = [[rx, ry], [vertices[3, 0], vertices[3, 1]]]

    if ry > vertices[3, 1]:
        vec = [[vertices[3, 0], vertices[3, 1]], [rx, ry]]

    return vec
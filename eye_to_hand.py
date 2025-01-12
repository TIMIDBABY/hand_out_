import os
import cv2
import xlrd2
from math import *
import numpy as np


class Calibration:
    def __init__(self):
        self.K = np.array([[1300.03408145, 0, 937.24675227],
                                  [0, 1299.27454754, 477.09869063],
                                  [0, 0, 1]], dtype=np.float64)
        self.distortion = np.array([[0.01817482, 0.05906624, -0.0000728, 0.00043427, -0.21260624]])
        self.target_x_number = 11    # 设置棋盘格w和h方向的角点数量（=方格数-1）
        self.target_y_number = 8
        self.target_cell_size = 0.025    # 棋盘格格子大小为25mm

    def angle2rotation(self, rx, ry, rz):
        Rx = np.array([[1, 0, 0], [0, cos(rx), -sin(rx)], [0, sin(rx), cos(rx)]])
        Ry = np.array([[cos(ry), 0, sin(ry)], [0, 1, 0], [-sin(ry), 0, cos(ry)]])
        Rz = np.array([[cos(rz), -sin(rz), 0], [sin(rz), cos(rz), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R

    def gripper2base(self, tx, ty, tz, rx, ry, rz):
        R_gripper2base = self.angle2rotation(rx, ry, rz)
        T_gripper2base = np.array([[tx], [ty], [tz]])
        Matrix_gripper2base = np.column_stack([R_gripper2base, T_gripper2base])
        Matrix_gripper2base = np.row_stack((Matrix_gripper2base, np.array([0, 0, 0, 1])))
        return Matrix_gripper2base

    def inverse_transformation_matrix(self, T):
        R = T[:3, :3]
        t = T[:3, 3]
        # 计算旋转矩阵的逆矩阵
        R_inv = R.T
        # 计算平移向量的逆矩阵
        t_inv = -np.dot(R_inv, t).reshape((3, 1))
        return R_inv, t_inv

    def target2camera(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.target_x_number, self.target_y_number), None)

        # 绘制检测到的角点
        cv2.drawChessboardCorners(img, (11, 8), corners, ret)
        # 显示图片和提示信息
        cv2.imshow(f'Image', img)
        cv2.putText(img, "Press ESC for next image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        cv2.imshow(f'Image', img)
        # 等待按键事件
        key = cv2.waitKey(0)

        corner_points = np.zeros((2, corners.shape[0]), dtype=np.float64)
        for i in range(corners.shape[0]):
            corner_points[:, i] = corners[i, 0, :]
        object_points = np.zeros((3, self.target_x_number * self.target_y_number), dtype=np.float64)
        count = 0
        for i in range(self.target_y_number):
            for j in range(self.target_x_number):
                object_points[:2, count] = np.array(
                    [(self.target_x_number - j - 1) * self.target_cell_size,
                     (self.target_y_number - i - 1) * self.target_cell_size])
                count += 1
        retval, rvec, tvec = cv2.solvePnP(object_points.T, corner_points.T, self.K, distCoeffs=self.distortion)
        Matrix_target2camera = np.column_stack(((cv2.Rodrigues(rvec))[0], tvec))
        Matrix_target2camera = np.row_stack((Matrix_target2camera, np.array([0, 0, 0, 1])))
        R_target2camera = Matrix_target2camera[:3, :3]
        T_target2camera = Matrix_target2camera[:3, 3].reshape((3, 1))
        return R_target2camera, T_target2camera

    def process(self, img_path, pose_path):
        image_list = []
        for root, dirs, files in os.walk(img_path):
            if files:
                # 排序文件列表，按文件名的数字顺序进行排序
                files.sort(key=lambda x: int(x.split('.')[0]))
                for file in files:
                    image_name = os.path.join(root, file)
                    image_list.append(image_name)
        R_target2camera_list = []
        T_target2camera_list = []
        for img_path in image_list:
            img = cv2.imread(img_path)
            R_target2camera, T_target2camera = self.target2camera(img)
            R_target2camera_list.append(R_target2camera)
            T_target2camera_list.append(T_target2camera)
        R_base2gripper_list = []
        T_base2gripper_list = []
        data = xlrd2.open_workbook(pose_path)
        table = data.sheets()[0]
        for row in range(table.nrows):
            tx = table.cell_value(row, 0)
            ty = table.cell_value(row, 1)
            tz = table.cell_value(row, 2)
            rx = table.cell_value(row, 3)
            ry = table.cell_value(row, 4)
            rz = table.cell_value(row, 5)
            Matrix_gripper2base = self.gripper2base(tx, ty, tz, rx, ry, rz)
            R_base2gripper, T_base2gripper = self.inverse_transformation_matrix(Matrix_gripper2base)
            R_base2gripper_list.append(R_base2gripper)
            T_base2gripper_list.append(T_base2gripper)

        R_camera2base, T_camera2base = cv2.calibrateHandEye(R_base2gripper_list, T_base2gripper_list,
                                                         R_target2camera_list, T_target2camera_list)
        print("旋转矩阵：")
        print(R_camera2base)
        print("平移向量：")
        print(T_camera2base)
        return R_camera2base, T_camera2base, R_base2gripper_list, T_base2gripper_list, R_target2camera_list, T_target2camera_list

    def check_result(self, R_cb, T_cb, R_bg, T_bg, R_tc, T_tc):
        for i in range(len(R_bg)):
            RT_base2gripper = np.column_stack((R_bg[i], T_bg[i]))
            RT_base2gripper = np.row_stack((RT_base2gripper, np.array([0, 0, 0, 1])))
            print(RT_base2gripper)

            RT_camera2base = np.column_stack((R_cb, T_cb))
            RT_camera2base = np.row_stack((RT_camera2base, np.array([0, 0, 0, 1])))
            print(RT_camera2base)

            RT_target2camera = np.column_stack((R_tc[i], T_tc[i]))
            RT_target2camera = np.row_stack((RT_target2camera, np.array([0, 0, 0, 1])))
            print(RT_target2camera)

            RT_target2gripper = RT_base2gripper @ RT_camera2base @ RT_target2camera
            print("第{}次验证结果为:".format(i))
            print(RT_target2gripper)
            print('')


if __name__ == "__main__":
    image_path = r"F:/code/img"
    pose_path = r"F:/code/pose.xlsx"
    calibrator = Calibration()
    R_cb, T_cb, R_bg, T_bg, R_tc, T_tc = calibrator.process(image_path, pose_path)
    calibrator.check_result(R_cb, T_cb, R_bg, T_bg, R_tc, T_tc)
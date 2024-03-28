#!/usr/bin/env python3 
#codeing =utf-8 

import math
import time
import mujoco_py
from mujoco_py import load_model_from_path, MjSim, MjViewer
import glfw  # 用于检查窗口关闭事件
import os
import rospy
import numpy as np
import cv2

from mujoco_py import GlfwContext
GlfwContext(offscreen=True)

class Mujoco_Model():
    def __init__(self) -> None:
        # 寻找模型路径
        current_path = os.path.dirname(os.path.dirname(__file__))
        current_path = current_path + "/meshes_mujoco"
        model_path = current_path + "/aloha_v1.xml"
        print("The directory is ",model_path)
        # 加载仿真环境
        model = load_model_from_path(model_path)
        self.sim = MjSim(model)
        self.viewer = MjViewer(self.sim)
        self.offscreen_f = mujoco_py.MjRenderContextOffscreen(self.sim, 0, quiet = True)
        self.offscreen_l = mujoco_py.MjRenderContextOffscreen(self.sim, 0, quiet = True)
        self.offscreen_r = mujoco_py.MjRenderContextOffscreen(self.sim, 0, quiet = True)

    def pos_ctrl(self, joint_name, target_angle):
        '''
        仿真环境中设定的position执行器控制
        '''
        joint_id = self.sim.model.get_joint_qpos_addr(joint_name)
        current_angle = self.sim.data.qpos[joint_id]
        # print(joint_name, ": ", current_angle)
        actuator_id = self.sim.model.actuator_name2id(joint_name)
        self.sim.data.ctrl[actuator_id] = target_angle

    def sim_stop(self):
        while not glfw.window_should_close(self.viewer.window):
            self.sim.step()
            self.viewer.render()
    
    def test1(self):
        count = 0
        while not glfw.window_should_close(self.viewer.window):
            if(count > 200 and count < 600):
                self.pos_ctrl("fl_joint1",3.14)
                self.pos_ctrl("fl_joint2",0.958)
                self.pos_ctrl("fl_joint3",0.99)

                self.pos_ctrl("fr_joint1",3.14)
                self.pos_ctrl("fr_joint2",0.958)
                self.pos_ctrl("fr_joint3",0.99)
            if(count > 600 and count < 1000):
                self.pos_ctrl("fl_joint1",1.67)
                self.pos_ctrl("fr_joint1",1.67)
            if(count > 600 and count < 1000):
                self.pos_ctrl("fl_joint1",0)
                self.pos_ctrl("fl_joint2",0)
                self.pos_ctrl("fl_joint3",0)

                self.pos_ctrl("fr_joint1",0)
                self.pos_ctrl("fr_joint2",0)
                self.pos_ctrl("fr_joint3",0)
            if(count > 1000 and count < 1400):
                self.pos_ctrl("fl_joint1",3.14)
                self.pos_ctrl("fl_joint2",0.958)
                self.pos_ctrl("fl_joint3",0.99)

                self.pos_ctrl("fr_joint1",3.14)
                self.pos_ctrl("fr_joint2",0.958)
                self.pos_ctrl("fr_joint3",0.99)
            if(count > 1400 and count < 1800):
                self.pos_ctrl("fl_joint1",-3.14)
                self.pos_ctrl("fl_joint2",0)
                self.pos_ctrl("fl_joint3",0)

                self.pos_ctrl("fr_joint1",-3.14)
                self.pos_ctrl("fr_joint2",0)
                self.pos_ctrl("fr_joint3",0)
                count = 0
            # 获取窗口屏幕图像
            self.offscreen_f.render(width=640, height=480, camera_id=0)
            data_f = self.offscreen_f.read_pixels(width=640, height=480, depth=False)
            self.offscreen_l.render(width=640, height=480, camera_id=1)
            data_l = self.offscreen_l.read_pixels(width=640, height=480, depth=False)
            self.offscreen_r.render(width=640, height=480, camera_id=2)
            data_r = self.offscreen_r.read_pixels(width=640, height=480, depth=False)

            # 从渲染结果中提取相机图像
            image_f = np.flipud(data_f)
            image_l = np.flipud(data_l)
            image_r = np.flipud(data_r)

            # 显示相机图像
            cv2.imshow("Camera Image_f", image_f)
            cv2.imshow("Camera Image_l", image_l)
            cv2.imshow("Camera Image_r", image_r)
            cv2.waitKey(1)
            self.sim.step()
            self.viewer.render()
            time.sleep(0.01)
            count = count + 1    

def main():
    test = Mujoco_Model()
    test.test1()
    
if __name__ == "__main__":  

    main()
    


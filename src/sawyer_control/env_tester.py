#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
from sawyer_control.sawyer_image import ImageSawyerEnv

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='position', img_observation=True, update_hz=20)
def pos():
    return env._end_effector_pose()[:3]

print(pos())
env.set_safety_box(ee_low = np.array([0.405, -0.13, 0.23128]), ee_high = np.array([0.92, 0.156, 0.23128]))

reset_pos = np.array([0.40474366724, 0.02265634, 0.23128])
def reset_env():
    env.thresh = False
    curr = pos()
    env._joint_act(reset_pos - curr)
    env.thresh = True


reset_env()
# for i in range(9415, 10000):
#     print(i)
#     cur = pos()
#     delta = np.array([np.random.uniform(-0.1, 0.1), np.random.uniform(-0.1, 0.1), 0])
#     env._joint_act(delta)
#     cv2.imwrite('new_pushing_images/' + str(i) + '.png', env.get_image_data())
#     if i % 50 == 0:
#         env.thresh = False
#         x = np.random.uniform(0.41, 0.9)
#         goal = np.array([x, 0.02265634, 0.23128])
#         env._joint_act(goal - pos())
#         input()
#         reset_env()




#fixed z = 0.23128
# reset x, y = [0.43167627, 0.01648346
# goals from 0.92

# box low = [0.405. -0.13, 0.23128]
# box_high = [0.92, 0.156, 0.23128]

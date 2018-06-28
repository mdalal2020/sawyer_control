from sawyer_reaching import SawyerXYPushingEnv
from sawyer_control.wrappers import ImageEnv
import numpy as np
from railrl.exploration_strategies.ou_strategy import OUStrategy
import cv2
env = SawyerXYPushingEnv(action_mode='position', reward='norm', pause_on_reset=True)
env = ImageEnv(env)
ou = OUStrategy(env.action_space)
env.reset()
def collect_data(folder_name, log_dir = '/tmp/', N=10000, rollout_len = 50):
    dir = log_dir + folder_name + '/'
    for i in range(N):
        env.step(ou.get_action_from_raw_action(np.zeros(3)))
        img = env._get_image_data()
        cv2.imwrite(dir + str(i) + '.png', img)
        if i % rollout_len == 0:
            env.reset()





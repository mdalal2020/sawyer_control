from collections import deque
import torch
from railrl.envs.wrappers import ProxyEnv
from rllab.envs.base import Env
from PIL import Image
from rllab.spaces.box import Box
import numpy as np
import rospy
from sawyer_control.srv import image

class ImageSawyerEnv(ProxyEnv, Env):
    def __init__(self, wrapped_env, imsize=84, history_length=1, keep_prev=0, **kwargs):
        self.quick_init(locals())
        super().__init__(wrapped_env)
        self.imsize = imsize
        self.image_length = 3 * self.imsize * self.imsize
        # This is torch format rather than PIL image
        self.image_shape = (self.imsize, self.imsize)
        # Flattened past image queue
        self.history_length = keep_prev + 1
        self.history = deque(maxlen=self.history_length)
        self.observation_space = Box(low=0.0,
                                     high=1.0,
                                     shape=(self.image_length * self.history_length)
                                     )
    def step(self, action):
        # image observation get returned as a flattened 1D array
        true_state, reward, done, info = super().step(action)

        observation = self._get_image()
        self.history.append(observation)
        history = self._get_history().flatten()
        full_obs = self._get_obs(history, true_state)
        return full_obs, reward, done, info

    def reset(self):
        true_state = super().reset()
        self.history = deque(maxlen=self.history_length)

        observation = self._get_image()
        self.history.append(observation)
        history = self._get_history().flatten()
        full_obs = self._get_obs(history, true_state)
        return full_obs

    def _get_obs(self, history_flat, true_state):
        # adds extra information from true_state into to the image observation.
        return history_flat

    def _get_history(self):
        observations = list(self.history)
        obs_count = len(observations)
        for _ in range(self.history_length - obs_count):
            dummy = np.zeros(self.image_shape)
            observations.append(dummy)
        #try np.concatenate(observations, axis=0)
        return np.c_[observations]

    def retrieve_images(self):
        # returns images in unflattened PIL format
        images = []
        for image_obs in self.history:
            pil_image = self.torch_to_pil(torch.Tensor(image_obs))
            images.append(pil_image)
        return images

    def _get_observation(self):
        return self._get_image()

    def _get_image(self):
        temp = self.request_image()
        img = np.array(temp)
        image = img.reshape(84, 84, 3)
        return image

    def request_image(self):
        rospy.wait_for_service('images')
        try:
            request = rospy.ServiceProxy('images', image, persistent=True)
            obs = request()
            return (
                    obs.image
            )
        except rospy.ServiceException as e:
            print(e)

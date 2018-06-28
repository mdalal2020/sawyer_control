import cv2
from sawyer_control.srv import image
import numpy as np
import rospy
import time
from collections import OrderedDict
from rllab.spaces.box import Box
class ImageEnv(SawyerEnv):
	def __init__(self,
				 wrapped_env,
				 imsize=84,
				 grayscale=False,
	):
		self.quick_init(locals())
		super().__init__(wrapped_env)
		self.imsize = imsize
		if grayscale:
			self.image_length = self.imsize * self.imsize
		else:
			self.image_length = 3 * self.imsize * self.imsize
		# This is torch format rather than PIL image
		self.image_shape = (self.imsize, self.imsize)
		# Flattened past image queue
		self.transpose = transpose
		self.history_length=1
		self.normalize = normalize
		self._render_local = False

		self.observation_space = Box(low=0.0,
									 high=1.0,
									 shape=(self.image_length * self.history_length,))

	def step(self, action):
		# image observation get returned as a flattened 1D array
		true_state, reward, done, info = super().step(action)

		observation = self._image_observation()
		return observation, reward, done, info

	def reset(self):
		true_state = super().reset()
		observation = self._image_observation()
		return observation

	def get_image(self):
		"""TODO: this should probably consider history"""
		return self._image_observation()

	def _get_obs(self, history_flat, true_state):
		# adds extra information from true_state into to the image observation.
		# Used in ImageWithObsEnv.
		return history_flat
	def _get_image_data(self):
		# returns the image as a torch format np array
		temp = self.request_image()
		# update the get image in server. get an 84 x 84 x 3
		# reshape to 84 x 84 x 3
		temp = np.array(temp)
		temp = temp.reshape(84, 84, 3)
		return temp
	def _image_observation(self):
		image_obs = self._get_image_data()
		if self._render_local:
			cv2.imshow('env', image_obs)
			cv2.waitKey(1)
		if self.grayscale:
			image_obs = Image.fromarray(image_obs).convert('L')
			image_obs = np.array(image_obs)
		if self.normalize:
			image_obs = image_obs / 255.0
		if self.transpose:
			image_obs = image_obs.transpose()
		image_obs = image_obs.flatten()
		return image_obs

	def enable_render(self):
		self._render_local = True

	def _render(self):
		image_obs = self._get_image_data()
		cv2.imshow('env', image_obs)
		cv2.waitKey(1)

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
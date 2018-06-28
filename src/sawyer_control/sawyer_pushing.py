from collections import OrderedDict
import numpy as np
from sawyer_control.sawyer_env_base import SawyerEnv
from rllab.spaces.box import Box
from sawyer_control.serializable import Serializable
from railrl.envs.multitask.multitask_env import MultitaskEnv

JOINT_ANGLES_HIGH = np.array([
	1.70167993,
	1.04700017,
	3.0541791,
	2.61797006,
	3.05900002,
	2.09400001,
	3.05899961
])

JOINT_ANGLES_LOW = np.array([
	-1.70167995,
	-2.14700025,
	-3.0541801,
	-0.04995198,
	-3.05900015,
	-1.5708003,
	-3.05899989
])

JOINT_VEL_HIGH = 2*np.ones(7)
JOINT_VEL_LOW = -2*np.ones(7)

MAX_TORQUES = 0.5 * np.array([8, 7, 6, 5, 4, 3, 2])
JOINT_TORQUE_HIGH = MAX_TORQUES
JOINT_TORQUE_LOW = -1*MAX_TORQUES

JOINT_VALUE_HIGH = {
	'position': JOINT_ANGLES_HIGH,
	'velocity': JOINT_VEL_HIGH,
	'torque': JOINT_TORQUE_HIGH,
}
JOINT_VALUE_LOW = {
	'position': JOINT_ANGLES_LOW,
	'velocity': JOINT_VEL_LOW,
	'torque': JOINT_TORQUE_LOW,
}

END_EFFECTOR_POS_LOW = -1.2 * np.ones(3)
END_EFFECTOR_POS_HIGH = 1.2 *np.ones(3)

END_EFFECTOR_ANGLE_LOW = -1*np.ones(4)
END_EFFECTOR_ANGLE_HIGH = np.ones(4)

END_EFFECTOR_VALUE_LOW = {
	'position': END_EFFECTOR_POS_LOW,
	'angle': END_EFFECTOR_ANGLE_LOW,
}

END_EFFECTOR_VALUE_HIGH = {
	'position': END_EFFECTOR_POS_HIGH,
	'angle': END_EFFECTOR_ANGLE_HIGH,
}

class SawyerPushingEnv(SawyerEnv):
	def __init__(self,
				 desired = None,
				 randomize_goal_on_reset=False,
				 **kwargs
				 ):
		Serializable.quick_init(self, locals())
		super().__init__(**kwargs)
		if desired is None:
			self._randomize_desired_end_effector_pose()
		else:
			self.desired = desired
		self._randomize_goal_on_reset = randomize_goal_on_reset
		self.z = 0.15203016
		self.action_mode = 'position'
		self.ee_safety_box_high = np.array([0.89, 0.17, self.z])
		self.ee_safety_box_low = np.array([0.49, -0.21, self.z])
		self.center = np.array([0.65469509, -0.02415424, self.z])

	# def _act(self, action):
	# 	# import ipdb; ipdb.set_trace()
	# 	act = action.copy().tolist()
	# 	act += [0]
	# 	act = np.array(act)
	# 	return self._joint_act(act)


	# def reset_to_center(self):
	# 	self.thresh = False
	# 	diff = self._end_effector_pose()[:2] - self.center
	# 	self._act(diff)
	# 	self.thresh = True





	def reward(self):
		current = self._end_effector_pose()[:3]
		differences = self.desired - current
		reward = self.reward_function(differences)
		return reward

	def _set_observation_space(self):
		lows = np.hstack((
			JOINT_VALUE_LOW['position'],
			JOINT_VALUE_LOW['velocity'],
			JOINT_VALUE_LOW['torque'],
			END_EFFECTOR_VALUE_LOW['position'],
			END_EFFECTOR_VALUE_LOW['angle'],
			END_EFFECTOR_VALUE_LOW['position'],
		))

		highs = np.hstack((
			JOINT_VALUE_HIGH['position'],
			JOINT_VALUE_HIGH['velocity'],
			JOINT_VALUE_HIGH['torque'],
			END_EFFECTOR_VALUE_HIGH['position'],
			END_EFFECTOR_VALUE_HIGH['angle'],
			END_EFFECTOR_VALUE_HIGH['position'],
		))

		self._observation_space = Box(lows, highs)

	def _randomize_desired_end_effector_pose(self):
		#self.desired = np.random.uniform(self.safety_box_lows, self.safety_box_highs, size=(1, 3))[0]
		high = self.ee_safety_box_high
		low = self.ee_safety_box_low
		dx = np.random.uniform(low[0], high[0])
		dy = np.random.uniform(low[1], high[1])
		dz = np.random.uniform(low[2], high[2])
		self.desired =  np.array([dx, dy, dz])


	def reset(self):
		self.in_reset = True
		self._safe_move_to_neutral()
		self.in_reset = False
		if self._randomize_goal_on_reset:
			self._randomize_desired_end_effector_pose()
		return self._get_observation()

	def _get_statistics_from_paths(self, paths):
		statistics = OrderedDict()
		stat_prefix = 'Test'
		distances_from_target, final_position_distances = self._extract_experiment_info(paths)
		statistics.update(self._update_statistics_with_observation(
			distances_from_target,
			stat_prefix,
			'End Effector Distance from Target'
		))

		statistics.update(self._update_statistics_with_observation(
			final_position_distances,
			stat_prefix,
			'Final End Effector Distance from Target'
		))

		return statistics

	def _extract_experiment_info(self, paths):
		obsSets = [path["next_observations"] for path in paths]
		positions = []
		desired_positions = []
		distances = []
		final_positions = []
		final_desired_positions = []
		for obsSet in obsSets:
			for observation in obsSet:
				pos = np.array(observation[21:24])
				des = np.array(observation[28:31])
				distances.append(np.linalg.norm(pos - des))
				positions.append(pos)
				desired_positions.append(des)
			final_positions.append(positions[-1])
			final_desired_positions.append(desired_positions[-1])

		distances = np.array(distances)
		final_positions = np.array(final_positions)
		final_desired_positions = np.array(final_desired_positions)
		final_position_distances = np.linalg.norm(final_positions - final_desired_positions, axis=1)
		return [distances, final_position_distances]


class SawyerXYZReachingMultitaskEnv(SawyerEnv, MultitaskEnv):
	def __init__(self,
				 desired = None,
				 randomize_goal_on_reset=False,
				 **kwargs
				 ):
		Serializable.quick_init(self, locals())
		SawyerEnv.__init__(self, **kwargs)
		MultitaskEnv.__init__(self, **kwargs)
		# if desired is None:
		#     self._randomize_desired_end_effector_pose()
		# else:
		#     self.desired = desired
		self._randomize_goal_on_reset = randomize_goal_on_reset

		high = self.ee_safety_box_high
		low = self.ee_safety_box_low
		self.goal_space = Box(low, high)

	@property
	def goal_dim(self):
		return 3

	def convert_obs_to_goals(self, obs):
		return obs[:, 21:24] # TODO: check this!

	def sample_goals(self, batch_size):
		return np.vstack([self.goal_space.sample() for i in range(batch_size)])

	def set_goal(self, goal):

		#self.MultitaskEnv.set_goal(goal)
		MultitaskEnv.set_goal(self, goal)
		# For VAE: actually need to move the arm to this position
		self.desired = goal

	def _get_observation(self):
		angles = self._joint_angles()
		_, velocities, torques, _ = self.request_observation()
		velocities = np.array(velocities)
		torques = np.array(torques)
		endpoint_pose = self._end_effector_pose()

		temp = np.hstack((
			angles,
			velocities,
			torques,
			endpoint_pose,
		))
		return temp

	def reward(self):
		current = self._end_effector_pose()[:3]
		differences = self.desired - current
		reward = self.reward_function(differences)
		return reward

	def _set_observation_space(self):
		lows = np.hstack((
			JOINT_VALUE_LOW['position'],
			JOINT_VALUE_LOW['velocity'],
			JOINT_VALUE_LOW['torque'],
			END_EFFECTOR_VALUE_LOW['position'],
			END_EFFECTOR_VALUE_LOW['angle'],
			# END_EFFECTOR_VALUE_LOW['position'],
		))

		highs = np.hstack((
			JOINT_VALUE_HIGH['position'],
			JOINT_VALUE_HIGH['velocity'],
			JOINT_VALUE_HIGH['torque'],
			END_EFFECTOR_VALUE_HIGH['position'],
			END_EFFECTOR_VALUE_HIGH['angle'],
			# END_EFFECTOR_VALUE_HIGH['position'],
		))

		self._observation_space = Box(lows, highs)

	def _randomize_desired_end_effector_pose(self):
		#self.desired = np.random.uniform(self.safety_box_lows, self.safety_box_highs, size=(1, 3))[0]
		assert False # Don't randomize goal inside reset! In MultitaskEnv this is done by set_goal
		high = self.ee_safety_box_high
		low = self.ee_safety_box_low
		dx = np.random.uniform(low[0], high[0])
		dy = np.random.uniform(low[1], high[1])
		dz = np.random.uniform(low[2], high[2])
		self.desired =  np.array([dx, dy, dz])

	def compute_her_reward_np(self, observation, action, next_observation, goal):
		return -np.linalg.norm(next_observation[21:24] - goal)

	def reset(self):
		self.in_reset = True
		self._safe_move_to_neutral()
		self.in_reset = False
		# Don't randomize goal inside reset! In MultitaskEnv this is done by set_goal
		# if self._randomize_goal_on_reset:
			# self._randomize_desired_end_effector_pose()
		return self._get_observation()



	def step(self, action):
		self._act(action)
		observation = self._get_observation()
		#print(observation.shape)
		reward = self.reward() * self.reward_magnitude
		done = False
		info = {}
		info['goal'] = self.desired.copy()
		if self.img_observation:
			observation = self.get_image()
		return observation, reward, done, info

	def _get_statistics_from_paths(self, paths):
		statistics = OrderedDict()
		stat_prefix = 'Test'
		distances_from_target, final_position_distances = self._extract_experiment_info(paths)
		statistics.update(self._update_statistics_with_observation(
			distances_from_target,
			stat_prefix,
			'End Effector Distance from Target'
		))

		statistics.update(self._update_statistics_with_observation(
			final_position_distances,
			stat_prefix,
			'Final End Effector Distance from Target'
		))

		return statistics

	def _extract_experiment_info(self, paths):
		obsSets = [path["next_observations"] for path in paths]
		goalSet = [path['goals'][0] for path in paths]
		positions = []
		desired_positions = []
		distances = []
		final_positions = []
		final_desired_positions = []
		for obsSet, goal in zip(obsSets, goalSet):
			for observation in obsSet:
				pos = np.array(observation[21:24])
				distances.append(np.linalg.norm(pos - goal))
				positions.append(pos)
				desired_positions.append(goal)
			final_positions.append(positions[-1])
			final_desired_positions.append(desired_positions[-1])

		distances = np.array(distances)
		final_positions = np.array(final_positions)
		final_desired_positions = np.array(final_desired_positions)
		final_position_distances = np.linalg.norm(final_positions - final_desired_positions, axis=1)
		return [distances, final_position_distances]


class SawyerXYZReachingImgMultitaskEnv(SawyerEnv, MultitaskEnv):
	def __init__(self,
				 desired = None,
				 randomize_goal_on_reset=False,
				 reprsentation_size = 16,
				 **kwargs
				 ):
		Serializable.quick_init(self, locals())
		SawyerEnv.__init__(self, **kwargs)
		MultitaskEnv.__init__(self, **kwargs)
		# if desired is None:
		#     self._randomize_desired_end_effector_pose()
		# else:
		#     self.desired = desired
		self._randomize_goal_on_reset = randomize_goal_on_reset
		self.desired = np.zeros((3))
		# self.representation_size = reprsentation_size
		#high = self.ee_safety_box_high
		#low = self.ee_safety_box_low
		#self.goal_space = Box(low, high)

		high = self.ee_safety_box_high
		low = self.ee_safety_box_low
		self.goal_space = Box(low, high)
		self.img_observation = True
		self.reset_pos = np.array([0.40474366724, 0.02265634, 0.23128])
		self.set_safety_box(ee_low=np.array([0.405, -0.13, 0.23128]), ee_high=np.array([0.92, 0.156, 0.23128]))

		# TODO: Set
	@property
	def goal_dim(self):
		return 3


	def sample_goals(self, batch_size):
		return np.vstack([self.goal_space.sample() for i in range(batch_size)])

	def set_goal(self, goal):
		MultitaskEnv.set_goal(self, goal)
		# For VAE: actually need to move the arm to this position
		self.thresh = False
		self._joint_act((goal - self._end_effector_pose()[:3]))
		self.desired = goal
		self.thresh = True
		input()

	def reward(self):
		current = self._end_effector_pose()[:3]
		differences = self.desired - current
		reward = self.reward_function(differences)
		return reward

	def _set_observation_space(self):
		# lows = np.hstack((
		#     JOINT_VALUE_LOW['position'],
		#     JOINT_VALUE_LOW['velocity'],
		#     JOINT_VALUE_LOW['torque'],
		#     END_EFFECTOR_VALUE_LOW['position'],
		#     END_EFFECTOR_VALUE_LOW['angle'],
		#     # END_EFFECTOR_VALUE_LOW['position'],
		# ))
		#
		# highs = np.hstack((
		#     JOINT_VALUE_HIGH['position'],
		#     JOINT_VALUE_HIGH['velocity'],
		#     JOINT_VALUE_HIGH['torque'],
		#     END_EFFECTOR_VALUE_HIGH['position'],
		#     END_EFFECTOR_VALUE_HIGH['angle'],
		#     # END_EFFECTOR_VALUE_HIGH['position'],
		# ))

		self._observation_space = Box(np.zeros((21168,)), 1.0*np.ones(21168,))

	def _randomize_desired_end_effector_pose(self):
		#self.desired = np.random.uniform(self.safety_box_lows, self.safety_box_highs, size=(1, 3))[0]
		assert False # Don't randomize goal inside reset! In MultitaskEnv this is done by set_goal
		high = self.ee_safety_box_high
		low = self.ee_safety_box_low
		dx = np.random.uniform(low[0], high[0])
		dy = np.random.uniform(low[1], high[1])
		dz = np.random.uniform(low[2], high[2])
		self.desired =  np.array([dx, dy, dz])

	def compute_her_reward_np(self, observation, action, next_observation, goal):
		return -np.linalg.norm(next_observation[21:24] - goal)

	def reset(self):
		self.in_reset = True
		# self._safe_move_to_neutral()
		self.thresh = False
		curr = self._end_effector_pose()[:3]
		self._joint_act(self.reset_pos - curr)
		self.thresh = True
		self.in_reset = False
		# Don't randomize goal inside reset! In MultitaskEnv this is done by set_goal
		# if self._randomize_goal_on_reset:
			# self._randomize_desired_end_effector_pose()
		input()
		if self.img_observation:
			observation = self.get_image()
		else:
			observation = self._get_observation()
		return observation

	def _get_observation(self):
		angles = self._joint_angles()
		_, velocities, torques, _ = self.request_observation()
		velocities = np.array(velocities)
		torques = np.array(torques)
		endpoint_pose = self._end_effector_pose()

		temp = np.hstack((
			angles,
			velocities,
			torques,
			endpoint_pose,
		))
		return temp



	def step(self, action):
		self._act(action)
		observation = self._get_observation()
		# print(observation.shape)
		reward = self.reward() * self.reward_magnitude
		done = False
		info = {}
		info['cartesian_goal'] = self.desired.copy()
		pos = np.array(observation[21:24])
		info['distance_to_goal'] = np.linalg.norm(pos - self.desired)
		if self.img_observation:
			observation = self.get_image()
		return observation, reward, done, info


	def _get_statistics_from_paths(self, paths):
		statistics = OrderedDict()
		stat_prefix = 'Test'
		#distances_from_target = np.array([path['distance_to_goal'] for path in paths]).flatten()
		distances_from_target = []
		final_position_distances = []
		for path in paths:
			distances_from_target += [p['distance_to_goal'] for p in path['env_infos']]
			final_position_distances += [[p['distance_to_goal'] for p in path['env_infos']][-1]]
		#final_position_distances = np.array([path['distance_to_goal'][-1] for path in paths]).flatten()
		# distances_from_target, final_position_distances = self._extract_experiment_info(paths)
		final_position_distances = np.array(final_position_distances)
		distances_from_target = np.array(distances_from_target)
		statistics.update(self._update_statistics_with_observation(
			distances_from_target,
			stat_prefix,
			'End Effector Distance from Target'
		))

		statistics.update(self._update_statistics_with_observation(
			final_position_distances,
			stat_prefix,
			'Final End Effector Distance from Target'
		))

		return statistics

	# def _extract_experiment_info(self, paths):
	#     distSets = [path['distance_goal'] for path in paths]
	#     goalSet = [path['goals'][0] for path in paths]
	#     positions = []
	#     desired_positions = []
	#     distances = []
	#     final_positions = []
	#     final_desired_positions = []
	#     for distSet, goal in zip(distSets, goalSet):
	#         for dist in distSet:
	#             distances.append(dist)
	#             positions.append(pos)
	#             desired_positions.append(goal)
	#         final_positions.append(positions[-1])
	#         final_desired_positions.append(desired_positions[-1])
	#
	#     distances = np.array(distances)
	#     final_positions = np.array(final_positions)
	#     final_desired_positions = np.array(final_desired_positions)
	#     final_position_distances = np.linalg.norm(final_positions - final_desired_positions, axis=1)
	#     return [distances, final_position_distances]

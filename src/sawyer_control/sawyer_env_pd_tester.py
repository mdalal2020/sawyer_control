from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='position')
total = 0
num_steps = 10
env.reset()
error = 0
num_desireds = 1
num_trials = 5
desireds = [np.random.uniform(env.safety_box_lows, env.safety_box_highs, size=(1, 3))[0] for i in range(num_desireds)]
for desired in desireds:
    dist = 0
    env.desired = desired
    for i in range(num_trials):
        env.reset()
        for j in range(num_steps):
            original = env._end_effector_pose()[:3]
            act = desired - original
            env._act(act)
        current = env._end_effector_pose()[:3]
        dist += np.linalg.norm(desired-current)
    print(dist/num_trials)
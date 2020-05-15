#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty
import gym
from gym import spaces
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
import math
import numpy as np
import tf
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import ApplyBodyWrench
import torch
import torch.nn as nn
from time import sleep
from collections import namedtuple
import torch.optim as optim
from tensorboardX import SummaryWriter
import matplotlib.pyplot as plt
# maybe do some 'wait for service' here
N_ACTION_SPACE = 2
N_OBSERVATION_SPACE = 4
_theta =  0 
_x = 0
vel = 1.0
_x_vel = 0
_theta_vel = 0
HIDDEN_SIZE = 50
BATCH_SIZE = 32
PERCENTILE = 80
wrench = Wrench()
Episode = namedtuple('Episode', field_names=['reward', 'steps'])
EpisodeStep = namedtuple('EpisodeStep', field_names=['observation', 'action'])



rospy.init_node("env")
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)



		

class Net(nn.Module):
    def __init__(self, obs_size, hidden_size, n_actions):
        super(Net, self).__init__()
        self.net = nn.Sequential(
        	# nn.Conv1D(1,hid)
            # nn.Linear(obs_size, hidden_size),
            # nn.ReLU(),
            # nn.Linear(hidden_size, hidden_size*2),
            # nn.ReLU(),
            nn.Linear(obs_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, n_actions)
        )

    def forward(self, x):
        return self.net(x)


class cartpole(gym.Env):
	"""docstring for cartpole"""
	def __init__(self):
		super(cartpole, self).__init__()
		reset_simulation()
		self.angle_threshold = math.pi/8	
		self.pos_threshold = 1
		self.vel_threshold = 1000
		self.theta_vel_thresold = 1000
		high = np.array([self.angle_threshold,self.pos_threshold,self.vel_threshold,self.theta_vel_thresold],dtype= np.float32)
		self.action_space = spaces.Discrete(N_ACTION_SPACE)
		self.observation_space = spaces.Box(-high,high,dtype= np.float32)
		self.state = None 

	def reset(self):
		self.state = (_theta,_x,_x_vel,_theta_vel)
		reset_simulation()
		return self.state

		return 
	def step(self,action):
		self._take_action(action)
		done = bool( _x > self.pos_threshold or _x < -self.pos_threshold or _theta > self.angle_threshold or _theta < -self.angle_threshold)

		if not done:
			reward = 1.0
		else : 
			reward = 0.0

		self.state = (_theta,_x,_x_vel,_theta_vel)

		return np.array(self.state),reward,done,{}

	def _take_action(self,action):
		if(action==0):
			wrench.force.x = 1700
			apply_body_wrench(body_name="link_chassis",wrench = wrench,duration= rospy.Duration(0.001))
		else:
			wrench.force.x = -1700
			apply_body_wrench(body_name="link_chassis",wrench = wrench,duration= rospy.Duration(0.001))

def iterate_batches(env, net, batch_size):
    batch = []
    episode_reward = 0.0
    episode_steps = []
    obs = env.reset()
    sm = nn.Softmax(dim=1)
    while True:
        obs_v = torch.FloatTensor([obs])
        act_probs_v = sm(net(obs_v))
        act_probs = act_probs_v.data.numpy()[0]
        action = np.random.choice(len(act_probs), p=act_probs)
        next_obs, reward, is_done, _ = env.step(action)
        episode_reward += reward
        episode_steps.append(EpisodeStep(observation=obs, action=action))
        if is_done:
            batch.append(Episode(reward=episode_reward, steps=episode_steps))
            episode_reward = 0.0
            episode_steps = []
            next_obs = env.reset()
            sleep(0.15)
            if len(batch) == batch_size:
                yield batch
                batch = []
        obs = next_obs

def filter_batch(batch, percentile):
    rewards = list(map(lambda s: s.reward, batch))
    reward_bound = np.percentile(rewards, percentile)
    reward_mean = float(np.mean(rewards))

    train_obs = []
    train_act = []
    for example in batch:
        if example.reward < reward_bound:
            continue
        train_obs.extend(map(lambda step: step.observation, example.steps))
        train_act.extend(map(lambda step: step.action, example.steps))

    train_obs_v = torch.FloatTensor(train_obs)
    train_act_v = torch.LongTensor(train_act)
    return train_obs_v, train_act_v, reward_bound, reward_mean


def callback(msg):
	global _theta,_x,_x_vel,_theta_vel
	(r, _theta, y) = tf.transformations.euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
	_x = msg.pose[1].position.x
	_x_vel = msg.twist[1].linear.x
	_theta_vel = msg.twist[2].angular.y

	

if __name__ == '__main__':
	sub = rospy.Subscriber("/gazebo/link_states",LinkStates,callback)
	env = cartpole()
	obs_size = env.observation_space.shape[0]
	n_actions = env.action_space.n
	net = Net(obs_size,HIDDEN_SIZE,n_actions)
	objective = nn.CrossEntropyLoss()
	optimizer = optim.Adam(params=net.parameters(), lr=0.01)
	writer = SummaryWriter(comment="-cartpole")
	x_val = []
	y_val =  []
	i= 0 
	for iter_no, batch in enumerate(iterate_batches(env, net, BATCH_SIZE)):
		obs_v, acts_v, reward_b, reward_m = filter_batch(batch, PERCENTILE)
		x_val.append(i)
		y_val.append(reward_m)
		# print(y_val)
		# print(x_val)
		plt.cla()
		plt.plot(x_val,y_val)
		plt.show(block=False)
		plt.pause(0.05)
		i = i+1
		optimizer.zero_grad()
		action_scores_v = net(obs_v)
		loss_v = objective(action_scores_v, acts_v)
		loss_v.backward()
		optimizer.step()
		print("%d: loss=%.3f, reward_mean=%.1f, reward_bound=%.1f" % (
            iter_no, loss_v.item(), reward_m, reward_b))
		writer.add_scalar("loss", loss_v.item(), iter_no)
		writer.add_scalar("reward_bound", reward_b, iter_no)
		writer.add_scalar("reward_mean", reward_m, iter_no)
		# if reward_m > 199:
		# 	print("Solved!")
		# 	break
    
	writer.close()
	
	while not rospy.is_shutdown():
		print(env.step(0))
		sleep(1)
import os
import gym
import time
import torch
import datetime
import numpy as np
import torch.nn as nn
import torch.nn.functional as F

LOG_STD_MAX = 2
LOG_STD_MIN = -20

XY_DIM = 2
DISTANCE_THRESHOLD = 0.5

OBS_CLIP_LOW = np.array([-np.inf] * 6)
OBS_SPACE = gym.spaces.Box(low=OBS_CLIP_LOW, high=-OBS_CLIP_LOW)

ACT_CLIP_LOW = np.array([-0.1, -np.pi / 10.])
ACT_CLIP_HIGH = np.array([0.1, np.pi / 10.])
ACT_SPACE = gym.spaces.Box(low=ACT_CLIP_LOW, high=ACT_CLIP_HIGH, dtype=np.float32)

GOAL_CLIP_LOW = np.array([-4, -4, -np.pi])
GOAL_CLIP_HIGH = np.array([20., 20., np.pi])
GOAL_SPACE = gym.spaces.Box(low=GOAL_CLIP_LOW, high=GOAL_CLIP_HIGH, dtype=np.float32)


def squeeze(obs):
    return [obs[:3], obs[3:]]


def flatten(obs):
    return np.concatenate([obs[0], obs[1]])


def transform(act):
    yaw, delta = act[1], act[0]
    return [delta * np.cos(yaw), delta * np.sin(yaw)]


def reward_for_height(height):
    return - np.abs(height - 0.35) * 2.


def reward_for_energy(act_delta):
    return - np.sqrt(np.sum(act_delta ** 2)) * 0.5


def reward_for_success(distance):
    return 5. if distance < DISTANCE_THRESHOLD else 0.


def reward_for_direction(obs, obs2, goal):
    a, b = obs2 - obs, goal - obs
    cos_theta = np.dot(a, b) / (np.sqrt(a.dot(a)) * np.sqrt(b.dot(b)))
    return cos_theta * 2.


def soft_update(target, source, tau=0.005):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)


class ReplayBuffer:
    def __init__(self, state_dim, action_dim, max_size=int(1e6)):
        self.state_buf = np.zeros((max_size, state_dim))
        self.action_buf = np.zeros((max_size, action_dim))
        self.next_state_buf = np.zeros((max_size, state_dim))
        self.done_buf = np.zeros((max_size, 1))
        self.reward_buf = np.zeros((max_size, 1))
        self.top = 0
        self._size = 0
        self.max_size = max_size

    @property
    def size(self):
        return self._size

    def __len__(self):
        return self._size

    def __getitem__(self, item):
        pass

    def reset(self):
        self.top, self._size = 0, 0

    def add(self, state, action, reward, next_state, done):
        self.state_buf[self.top] = state
        self.action_buf[self.top] = action
        self.reward_buf[self.top] = reward
        self.next_state_buf[self.top] = next_state
        self.done_buf[self.top] = done
        self.top = (self.top + 1) % self.max_size
        self._size = min(self.max_size, self._size + 1)

    def sample(self, batch_size):
        indexes = np.random.randint(0, self.size, size=batch_size)
        # print(self.state_buf[indexes])

        obs = torch.FloatTensor(self.state_buf[indexes]).cuda()
        act = torch.FloatTensor(self.action_buf[indexes]).cuda()
        rew = torch.FloatTensor(self.reward_buf[indexes]).cuda()
        obs2 = torch.FloatTensor(self.next_state_buf[indexes]).cuda()
        done = torch.FloatTensor(self.done_buf[indexes]).cuda()

        data = {
            'obs': obs,
            'act': act,
            'rew': rew,
            'obs2': obs2,
            'done': done
        }

        return data, indexes


class GoalConditionalBuffer(ReplayBuffer):
    def __init__(self, obs_dim, act_dim, goal_dim, max_size=int(1e6)):
        super(GoalConditionalBuffer, self).__init__(obs_dim, act_dim, max_size=max_size)
        self.context_buf = np.zeros((max_size, goal_dim))
        self.next_context_buf = np.zeros((max_size, goal_dim))

    def add(self, obs, act, rew, obs2, done, goal=None, next_goal=None):
        self.context_buf[self.top] = goal
        if next_goal is not None:
            self.next_context_buf[self.top] = next_goal
        super(GoalConditionalBuffer, self).add(obs, act, rew, obs2, done)

    def sample(self, batch_size):
        data, indexes = super(GoalConditionalBuffer, self).sample(batch_size)
        data['goal'] = torch.FloatTensor(self.context_buf[indexes]).cuda()
        data['goal2'] = torch.FloatTensor(self.next_context_buf[indexes]).cuda()

        return data, indexes


class DeterministicActor(nn.Module):
    def __init__(self, obs_dim, act_dim, goal_dim, act_bounds, act_offset):
        super(DeterministicActor, self).__init__()

        self.actor = nn.Sequential(
            nn.Linear(obs_dim + goal_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, act_dim),
            nn.Tanh()
        )

        self.act_bounds = act_bounds
        self.act_offset = act_offset

    def forward(self, obs, goal):
        return self.actor(torch.cat([obs, goal], -1)) * self.act_bounds + self.act_offset


class StochasticActor(nn.Module):
    def __init__(self, obs_dim, act_dim, goal_dim, log_std_min, log_std_max,
                 act_bounds, act_offset, hidden_size=256):
        super(StochasticActor, self).__init__()
        self.val = nn.Sequential(
            nn.Linear(obs_dim + goal_dim, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU()
        )

        self.mu = nn.Linear(hidden_size, act_dim)
        self.log_std = nn.Linear(hidden_size, act_dim)

        self.log_std_min = log_std_min
        self.log_std_max = log_std_max

        self.act_bounds = act_bounds
        self.act_offset = act_offset

    def forward(self, obs, goal, deterministic=False, with_log_pi=True):
        val = self.val(torch.cat([obs, goal], axis=-1))
        mu = self.mu(val)
        log_std = F.relu(self.log_std(val)).clamp(self.log_std_min, self.log_std_max)
        std = torch.exp(log_std)

        dist = torch.distributions.normal.Normal(mu, std)
        if deterministic:
            act = mu
        else:
            act = dist.rsample()

        if with_log_pi:
            log_pi = dist.log_prob(act).sum(axis=-1, keepdim=True)
            log_pi -= (2 * (np.log(2) - act - F.softplus(-2 * act))).sum(axis=-1, keepdim=True)
        else:
            log_pi = None

        act = F.tanh(act) * self.act_bounds + self.act_offset
        return act, log_pi


class TD3Critic(nn.Module):
    def __init__(self, obs_dim, act_dim, goal_dim):
        super(TD3Critic, self).__init__()
        self.q1 = nn.Sequential(
            nn.Linear(obs_dim + act_dim + goal_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1),
        )

        self.q2 = nn.Sequential(
            nn.Linear(obs_dim + act_dim + goal_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1),
        )

    def forward(self, obs, act, goal):
        return self.q1(torch.cat([obs, act, goal], -1)), \
               self.q2(torch.cat([obs, act, goal], -1))

    def Q1(self, obs, act, goal):
        return self.q1(torch.cat([obs, act, goal], -1))


class TD3:
    def __init__(self, obs_space, act_space, goal_space, pi_lr, q_lr, gamma=0.99, tau=0.005):
        self.total_steps = 0
        self.policy_noise = 0.2
        self.act_noise = 0.1
        self.noise_clip = 0.2
        self.update_policy_freq = 2

        self.obs_space = obs_space
        self.act_space = act_space
        self.obs_dim = obs_space.shape[0]
        self.act_dim = act_space.shape[0]
        self.goal_dim = goal_space.shape[0]
        self.act_clip_low = act_space.low
        self.act_clip_high = act_space.high
        act_bounds = (self.act_clip_high - self.act_clip_low) / 2.
        act_offset = (self.act_clip_high + self.act_clip_low) / 2.
        act_bounds = torch.FloatTensor(act_bounds).cuda()
        act_offset = torch.FloatTensor(act_offset).cuda()

        self.pi = DeterministicActor(self.obs_dim, self.act_dim, self.goal_dim, act_bounds, act_offset).cuda()
        self.pi_target = DeterministicActor(self.obs_dim, self.act_dim, self.goal_dim, act_bounds, act_offset).cuda()
        self.pi_target.load_state_dict(self.pi.state_dict())
        self.pi_opt = torch.optim.Adam(self.pi.parameters(), pi_lr)

        self.q = TD3Critic(self.obs_dim, self.act_dim, self.goal_dim).cuda()
        self.q_target = TD3Critic(self.obs_dim, self.act_dim, self.goal_dim).cuda()

        self.q_target.load_state_dict(self.q.state_dict())
        self.q_opt = torch.optim.Adam(self.q.parameters(), lr=q_lr, weight_decay=0.0001)

        self.tau = tau
        self.gamma = gamma
        self.q_norm_reg = 0.0005
        self.act_norm_reg = 0.001
        self.criterion = torch.nn.SmoothL1Loss()

    def get_action(self, obs, goal):
        with torch.no_grad():
            act = self.pi(obs, goal).cpu().data.numpy()
        return act

    def update(self, batch):
        self.total_steps += 1
        obs, act, rew, obs2, goal, goal2, done = batch['obs'], batch['act'], \
                                                 batch['rew'], batch['obs2'], batch['goal'], batch['goal2'], batch[
                                                     'done']

        with torch.no_grad():
            act_clip_low = torch.FloatTensor(self.act_clip_low).cuda()
            act_clip_high = torch.FloatTensor(self.act_clip_high).cuda()
            noise = (
                torch.randn_like(act) * self.policy_noise
            ).clamp(-self.noise_clip, self.noise_clip)
            act2 = self.pi_target(obs2, goal2) + noise
            act2 = torch.min(torch.max(act2, act_clip_low), act_clip_high)

            target_Q1, target_Q2 = self.q_target(obs2, act2, goal2)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = rew + (1 - done) * self.gamma * target_Q

        current_Q1, current_Q2 = self.q(obs, act, goal)
        loss_Q = self.criterion(current_Q1, target_Q) + self.criterion(current_Q2, target_Q)
        q_norm = (torch.norm(current_Q1) + torch.norm(current_Q2))
        loss_Q += q_norm * self.q_norm_reg

        self.q_opt.zero_grad()
        loss_Q.backward()
        self.q_opt.step()

        if self.total_steps % self.update_policy_freq == 0:
            for p in self.q.parameters():
                p.requires_grad = False

            act = self.pi(obs, goal)
            pi_loss = - self.q.Q1(obs, act, goal).mean()
            pi_norm = self.act_norm_reg * torch.norm(act)
            pi_loss += pi_norm

            self.pi_opt.zero_grad()
            pi_loss.backward()
            self.pi_opt.step()

            for p in self.q.parameters():
                p.requires_grad = True

            with torch.no_grad():
                soft_update(self.q_target, self.q, self.tau)
                soft_update(self.pi_target, self.pi, self.tau)
        else:
            pi_loss = None

        return pi_loss, loss_Q

    def save(self, dir_path):
        torch.save(self.pi.state_dict(), dir_path + 'pi.pth')
        torch.save(self.pi_opt.state_dict(), dir_path + 'pi_opt.pth')
        torch.save(self.pi_target.state_dict(), dir_path + 'pi_target.pth')
        torch.save(self.q.state_dict(), dir_path + 'q.pth')
        torch.save(self.q_opt.state_dict(), dir_path + 'q_opt.pth')
        torch.save(self.q_target.state_dict(), dir_path + 'q_target.pth')

    def load(self, dir_path):
        self.pi.load_state_dict(torch.load(dir_path + 'pi.pth'))
        self.pi_opt.load_state_dict(torch.load(dir_path + 'pi_opt.pth'))
        self.pi_target.load_state_dict(torch.load(dir_path + 'pi_target.pth'))
        self.q.load_state_dict(torch.load(dir_path + 'q.pth'))
        self.q_opt.load_state_dict(torch.load(dir_path + 'q_opt.pth'))
        self.q_target.load_state_dict(torch.load(dir_path + 'q_target.pth'))


class SAC:
    def __init__(self, obs_space, act_space, goal_space, pi_lr, q_lr, gamma=0.99, tau=0.005):
        self.tau = tau
        self.alpha = 0.2
        self.gamma = gamma
        self.q_norm_reg = 0.0005
        self.act_norm_reg = 0.001
        self.total_steps = 0
        self.update_policy_freq = 1
        self.criterion = torch.nn.SmoothL1Loss()

        self.obs_space = obs_space
        self.act_space = act_space
        self.obs_dim = obs_space.shape[0]
        self.act_dim = act_space.shape[0]
        self.goal_dim = goal_space.shape[0]
        self.act_clip_low = act_space.low
        self.act_clip_high = act_space.high
        act_bounds = (self.act_clip_high - self.act_clip_low) / 2.
        act_offset = (self.act_clip_high + self.act_clip_low) / 2.
        act_bounds = torch.FloatTensor(act_bounds).cuda()
        act_offset = torch.FloatTensor(act_offset).cuda()

        self.pi = StochasticActor(self.obs_dim, self.act_dim, self.goal_dim, LOG_STD_MIN, LOG_STD_MAX, act_bounds,
                                  act_offset).cuda()
        self.pi_opt = torch.optim.Adam(self.pi.parameters(), pi_lr)

        self.q = TD3Critic(self.obs_dim, self.act_dim, self.goal_dim).cuda()
        self.q_target = TD3Critic(self.obs_dim, self.act_dim, self.goal_dim).cuda()

        self.q_target.load_state_dict(self.q.state_dict())
        self.q_opt = torch.optim.Adam(self.q.parameters(), lr=q_lr, weight_decay=0.0001)

    def get_action(self, obs, goal, deterministic=False):
        with torch.no_grad():
            act, log_pi = self.pi(obs, goal, deterministic)
        return act.cpu().data.numpy()

    def update(self, batch):
        obs, act, goal, rew, done, obs2, goal2 = batch['obs'], batch['act'], \
                                                 batch['goal'], batch['rew'], batch['done'], batch['obs2'], batch[
                                                     'goal2']

        with torch.no_grad():
            act2, log_pi = self.pi(obs2, goal2)

            target_Q1, target_Q2 = self.q_target(obs2, act2, goal2)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = rew + self.gamma * (1. - done) * (target_Q - self.alpha * log_pi)

        current_Q1, current_Q2 = self.q(obs, act, goal)
        # delta = target_Q - current_Q1

        self.q_opt.zero_grad()
        loss_Q = self.criterion(current_Q1, target_Q) + self.criterion(current_Q2, target_Q)
        # q_norm = (torch.norm(current_Q1) + torch.norm(current_Q2)) * 0.5
        # loss_Q += self.q_norm_reg * q_norm
        loss_Q.backward()
        self.q_opt.step()

        # self.total_steps += 1
        # self.update_policy_freq = 1
        if self.total_steps % self.update_policy_freq == 0:
            for p in self.q.parameters():
                p.requires_grad = False

            act, log_pi = self.pi(obs, goal)
            pi_loss_1, pi_loss_2 = self.q(obs, act, goal)
            pi_loss = torch.min(pi_loss_1, pi_loss_2)
            pi_loss = (self.alpha * log_pi - pi_loss).mean()

            self.pi_opt.zero_grad()
            pi_loss.backward()
            self.pi_opt.step()

            for p in self.q.parameters():
                p.requires_grad = True

            with torch.no_grad():
                soft_update(self.q_target, self.q, self.tau)
        else:
            pi_loss = None

        return pi_loss, loss_Q

    def save(self, dir_path):
        torch.save(self.pi.state_dict(), dir_path + 'pi.pth')
        torch.save(self.pi_opt.state_dict(), dir_path + 'pi_opt.pth')
        torch.save(self.q.state_dict(), dir_path + 'q.pth')
        torch.save(self.q_target.state_dict(), dir_path + 'q_target.pth')
        torch.save(self.q_opt.state_dict(), dir_path + 'q_opt.pth')

    def load(self, dir_path):
        self.pi.load_state_dict(torch.load(dir_path + 'pi.pth'))
        self.pi_opt.load_state_dict(torch.load(dir_path + 'pi_opt.pth'))
        self.q.load_state_dict(torch.load(dir_path + 'q.pth'))
        self.q_target.load_state_dict(torch.load(dir_path + 'q_target.pth'))
        self.q_opt.load_state_dict(torch.load(dir_path + 'q_opt.pth'))


class RLAgent2:
    def __init__(self,
                 gamma=0.99,
                 pi_lr=3e-4,
                 q_lr=1e-3,
                 warmup_steps=1000,
                 update_times=300,
                 update_steps=500,
                 max_episode_steps=500,
                 steps_per_epoch=5000,
                 evaluate=False,
                 alg_type='TD3'
                 ):

        self.obs_space = OBS_SPACE
        self.act_space = ACT_SPACE
        self.goal_space = GOAL_SPACE

        self.obs_dim = OBS_SPACE.shape[0]
        self.act_dim = ACT_SPACE.shape[0]
        self.goal_dim = GOAL_SPACE.shape[0]

        # print('obs_dim', self.obs_dim)
        # print('act_dim', self.act_dim)
        # print('goal_dim', self.goal_dim)

        self.act_bounds = (ACT_CLIP_HIGH - ACT_CLIP_LOW) / 2.
        self.act_offset = (ACT_CLIP_HIGH + ACT_CLIP_LOW) / 2.
        self.act_bounds = torch.FloatTensor(self.act_bounds).cuda()
        self.act_offset = torch.FloatTensor(self.act_offset).cuda()

        self.q_lr = q_lr
        self.pi_lr = pi_lr
        self.gamma = gamma
        self.batch_size = 128
        self.hidden_size = 256
        self.buf_size = int(2e5)
        self.warmup_steps = warmup_steps
        # self.update_times = update_times # default update_times is 300
        self.update_times = 150
        self.update_steps = update_steps
        self.max_episode_steps = max_episode_steps
        self.steps_per_epoch = steps_per_epoch
        self.evaluate = evaluate

        if alg_type == 'TD3':
            self.policy = TD3(OBS_SPACE, ACT_SPACE, GOAL_SPACE, self.pi_lr, self.q_lr, self.gamma)
        elif alg_type == 'SAC':
            self.policy = SAC(OBS_SPACE, ACT_SPACE, GOAL_SPACE, self.pi_lr, self.q_lr, self.gamma)
        else:
            raise NotImplementedError

        self.buffer = GoalConditionalBuffer(self.obs_dim, self.act_dim, self.goal_dim, self.buf_size)

        self.ep_steps = 0
        self.total_steps = 0
        self.noise_clip_low = np.array([-0.02, -0.1])
        self.noise_clip_high = np.array([0.02, 0.1])

        self.goal = None
        self.last_obs = None
        self.last_act = None
        self.distance = None
        self.act_delta = None

        self.warmup_steps = 10

        # self.evaluate = True
        self.start_time = time.time()

        print('*' * 30)
        if not self.evaluate:
            current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            log_dir = [
                './log', alg_type + '_pi_lr_' + str(self.pi_lr) + '_q_lr_' + str(self.q_lr),
                current_time
            ]
            self.save_path = os.path.join(*log_dir)
            if not os.path.exists(self.save_path):
                os.makedirs(self.save_path)
            print('Training and save_path is {}'.format(self.save_path))
            self.save(self.save_path)
        else:

            # self.save_path = './log/TD3_pi_lr_0.0003_q_lr_0.001/20210822-155118'
            # self.save_path = './log/TD3_pi_lr_0.0003_q_lr_0.001/20210822-154855'
            self.save_path = './log/TD3_pi_lr_0.0003_q_lr_0.001/20210823-112834'
            self.save_path = './log/TD3_pi_lr_0.0003_q_lr_0.001/20210823-112818'

            print('Evaluating and load_path is {}'.format(self.save_path))
            self.load(self.save_path)
        print('*' * 30)

    def reset(self, current_time=None):
        self.ep_steps = 0

        self.last_obs = None
        self.last_act = None
        self.distance = None
        self.act_delta = None

        self.goal = None

    def sample_goal(self):
        if self.evaluate:
            # goal = np.array([18., 18., 0.])
            # goal = np.array([0., 18., 0.])
            goal = np.array([18., 0., 0.])
        else:
            goal = self.goal_space.sample()
        self.goal = goal
        return self.goal

    def episode_ended(self):
        return self.ep_steps >= self.max_episode_steps

    def compute_distance(self, xy, goal_xy):
        self.distance = np.sqrt(np.sum((xy - goal_xy) ** 2))

    def composite_reward(self, obs, obs2, raw_obs2):
        self.compute_distance(obs2[:XY_DIM], self.goal[:XY_DIM])
        rew = - self.distance
        rew += reward_for_height(obs2[2])
        rew += reward_for_energy(self.act_delta)
        rew += reward_for_success(self.distance)
        rew += reward_for_direction(obs[:XY_DIM], obs2[:XY_DIM], self.goal[:XY_DIM])
        return rew

    def get_action(self, flatten_obs):
        if self.evaluate:
            obs_t = torch.FloatTensor(flatten_obs).cuda()
            goal_t = torch.FloatTensor(self.goal).cuda()
            act = self.policy.get_action(obs_t, goal_t)
        else:
            if np.random.random_sample() > 0.2:
                obs_t = torch.FloatTensor(flatten_obs).cuda()
                goal_t = torch.FloatTensor(self.goal).cuda()
                act = self.policy.get_action(obs_t, goal_t)
                scale = np.ones_like(act).clip(self.noise_clip_low, self.noise_clip_high)
                noise = np.random.normal(0, scale) * self.act_bounds.cpu().data.numpy()
                act = (act + noise).clip(ACT_CLIP_LOW, ACT_CLIP_HIGH)
            else:
                act = self.act_space.sample()
        return act

    def get_com_goal(self, obs):
        flatten_obs = flatten(obs)
        if self.goal is None:
            self.goal = self.sample_goal()
            self.compute_distance(flatten_obs[:XY_DIM], self.goal[:XY_DIM])
            done = True if self.distance < DISTANCE_THRESHOLD else False
        else:
            if np.isnan(flatten_obs).any():
                rew, done, flatten_obs = -100, True, self.last_obs
            else:
                rew = self.composite_reward(self.last_obs, flatten_obs, obs)
                done = True if self.distance < DISTANCE_THRESHOLD else False
            self.buffer.add(self.last_obs, self.last_act, rew, flatten_obs, done, self.goal, self.goal)

        # update counter
        self.ep_steps += 1
        self.total_steps += 1

        # obs will be [nan, nan, nan] when dog falls
        is_nan = np.isnan(flatten_obs).any()

        if self.total_steps < self.warmup_steps or is_nan:
            # warm-up or avoid invalid action
            act = self.act_space.sample()
        else:
            # get action from RL policy
            act = self.get_action(flatten_obs)

        self.last_obs, self.last_act = flatten_obs, act

        if is_nan:
            self.reset()
            done = False

        if self.act_delta is None:
            self.act_delta = act
        else:
            self.act_delta = act - self.last_act

        if self.total_steps > self.warmup_steps and self.total_steps % self.update_steps == 0:
            self.update()
            cur_epoch = self.total_steps // self.update_steps
            print('Current Epoch: {} Total Steps: {} Cost Time: {:.2f}'.format(cur_epoch, self.total_steps,
                                                                               time.time() - self.start_time))
            self.start_time = time.time()
            self.save(self.save_path)

        # for stabilization
        if self.ep_steps < 5:
            return [(0., 0., 0), (0., 0., 0.)], done
        else:
            transform_act = transform(act)
            planned_com = [[flatten_obs[0] + transform_act[0], flatten_obs[1] + transform_act[1], 0],
                           [0, 0, flatten_obs[-1] + act[-1]]]
            if self.evaluate:
                print(flatten_obs, act, planned_com)
            return planned_com, done

    def update(self):
        for _ in range(self.update_times):
            data, idx = self.buffer.sample(self.batch_size)
            self.policy.update(data)

    def save(self, dir_path):
        print('Save the policy in :{}'.format(self.save_path))
        if not os.path.exists(dir_path + '/save'):
            os.mkdir(dir_path + '/save')
        self.policy.save(dir_path + '/save/master_')

    def load(self, dir_path):
        print('Load the policy in :{}'.format(self.save_path))
        self.policy.load(dir_path + '/save/master_')
        

from actor import Actor
from critic import Critic
from frame_stack import FrameStack
from ou_noise import OuNoise
from replay_buffer import ReplayBuffer
from rclpy.node import Node 
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import math

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class DDPGAgent(Node):
    def __init__(self):
        super().__init__('ddpg_agent')
        # 하이퍼파라미터 선언
        self.declare_parameter('actor_lr', 1e-5)
        self.declare_parameter('critic_lr', 5e-5)
        self.declare_parameter('critic_l2_decay', 1e-2) 
        self.declare_parameter('gamma', 0.99)
        self.declare_parameter('tau', 1e-3)
        self.declare_parameter('theta', 0.15)
        self.declare_parameter('sigma', 0.2)
        self.declare_parameter('mu', 0.0)
        self.declare_parameter('batch_size', 64)
        self.declare_parameter('max_buffer_size', 100000)
        self.declare_parameter('warmup_steps', 10000)

        self.actor_lr = self.get_parameter('actor_lr').value
        self.critic_lr = self.get_parameter('critic_lr').value
        self.critic_l2_decay = self.get_parameter('critic_l2_decay').value
        self.gamma = self.get_parameter('gamma').value
        self.tau = self.get_parameter('tau').value
        self.theta = self.get_parameter('theta').value
        self.sigma = self.get_parameter('sigma').value
        self.mu = self.get_parameter('mu').value
        self.batch_size = self.get_parameter('batch_size').value
        self.max_buffer_size = self.get_parameter('max_buffer_size').value
        self.warmup_steps = self.get_parameter('warmup_steps').value
        
        # 네트워크 초기화 (Actor/Critic __init__에서 이미 reset_parameters 호출됨)
        self.Actor = Actor().to(device)
        self.Actor_target = Actor().to(device)
        self.Actor_target.load_state_dict(self.Actor.state_dict())

        self.Critic = Critic(2).to(device)
        self.Critic_target = Critic(2).to(device)
        self.Critic_target.load_state_dict(self.Critic.state_dict())
        
        # 옵티마이저 설정
        self.actor_optimizer = torch.optim.Adam(self.Actor.parameters(), lr=self.actor_lr)
        self.critic_optimizer = torch.optim.Adam(
            self.Critic.parameters(), lr=self.critic_lr, weight_decay=self.critic_l2_decay
        )

        self.OU = OuNoise(2, self.mu, self.theta, self.sigma, 1)
        self.replay_buffer = ReplayBuffer(self.max_buffer_size, self.batch_size, (4, 64, 64, 3), (4, 128), (2,))
        self.train_step = 0

    def _sanitize_tensor(self, t, name="tensor"):
        """NaN/Inf를 0으로 치환하고, 이상 발견 시 로그 출력"""
        bad = torch.isnan(t) | torch.isinf(t)
        if bad.any():
            cnt = bad.sum().item()
            print(f"[SANITIZE] {name}: {cnt}개 NaN/Inf 감지 → 0으로 치환")
            t = torch.where(bad, torch.zeros_like(t), t)
        return t

    def get_action(self, state):
        # 이미지 전처리: (4, 64, 64, 3) -> (1, 12, 64, 64)
        state_img = torch.FloatTensor(state['image']).permute(0, 3, 1, 2).reshape(12, 64, 64).unsqueeze(0).to(device)
        # 센서: [0,1] → [-1,1] (train_model과 동일한 범위)
        state_sensor = torch.FloatTensor(state['sensors']).flatten().unsqueeze(0).to(device) * 2.0 - 1.0

        self.Actor.eval()
        with torch.no_grad():
            action = self.Actor(state_img, state_sensor)
        self.Actor.train()
        
        if torch.isnan(action).any():
            return np.array([0.0, 0.0], dtype=np.float32)
        
        raw_action = action.cpu().numpy()[0]
        action_with_noise = raw_action + self.OU.sample()
        
        steering = np.clip(action_with_noise[0], -0.6, 0.6)
        throttle = np.clip(action_with_noise[1], -1.0, 1.0)
        return np.array([steering, throttle], dtype=np.float32)

    def soft_update_target(self):
        for target_param, online_param in zip(self.Actor_target.parameters(), self.Actor.parameters()):
            target_param.data.copy_(self.tau * online_param.data + (1 - self.tau) * target_param.data)
        for target_param, online_param in zip(self.Critic_target.parameters(), self.Critic.parameters()):
            target_param.data.copy_(self.tau * online_param.data + (1 - self.tau) * target_param.data)

    def train_model(self):
        if self.replay_buffer.cnt < self.warmup_steps:
            return None, None, None
        
        self.train_step += 1
        (state_img, state_sensor, action, reward, next_state_img, next_state_sensors, done) = self.replay_buffer.sample()

        # 텐서 변환 + NaN/Inf 방어
        # 센서: [0,1] → [-1,1] 변환 (get_action과 동일한 범위, 이미지 [-1,1]과 통일)
        state_img = self._sanitize_tensor(
            torch.FloatTensor(state_img).permute(0, 1, 4, 2, 3).reshape(-1, 12, 64, 64).to(device),
            "state_img")
        state_sensor = self._sanitize_tensor(
            torch.FloatTensor(state_sensor).reshape(-1, 512).to(device) * 2.0 - 1.0,
            "state_sensor")
        action = self._sanitize_tensor(
            torch.FloatTensor(action).to(device),
            "action")
        
        reward = self._sanitize_tensor(torch.FloatTensor(reward).to(device).view(-1, 1), "reward")
        reward = torch.clamp(reward, -2.0, 2.0)
        done = self._sanitize_tensor(torch.FloatTensor(done).to(device).view(-1, 1), "done")

        next_state_img = self._sanitize_tensor(
            torch.FloatTensor(next_state_img).permute(0, 1, 4, 2, 3).reshape(-1, 12, 64, 64).to(device),
            "next_state_img")
        next_state_sensors = self._sanitize_tensor(
            torch.FloatTensor(next_state_sensors).reshape(-1, 512).to(device) * 2.0 - 1.0,
            "next_state_sensors")

        # 타겟 Q값 계산
        with torch.no_grad():
            next_actions = self.Actor_target(next_state_img, next_state_sensors)
            next_q = self.Critic_target(next_state_img, next_state_sensors, next_actions)
            target_q = reward + (1 - done) * self.gamma * next_q.view(-1, 1)
            target_q = torch.clamp(target_q, -50.0, 50.0)
            
        # 현재 Q값 계산
        q = self.Critic(state_img, state_sensor, action).view(-1, 1)

        # Critic 업데이트
        critic_loss = nn.SmoothL1Loss()(q, target_q)
        self.critic_optimizer.zero_grad()
        critic_loss.backward()

        # 표준 gradient clipping (clip_grad_norm_ 만으로 충분)
        torch.nn.utils.clip_grad_norm_(self.Critic.parameters(), max_norm=1.0)

        # NaN gradient 안전장치: NaN이면 즉시 종료 (원인 분석 필요)
        for name, param in self.Critic.named_parameters():
            if param.grad is not None and (torch.isnan(param.grad).any() or torch.isinf(param.grad).any()):
                print(f"CRITICAL: Critic gradient NaN/Inf at {name}, step {self.train_step}! Exiting...")
                import sys; sys.exit(1)
        
        self.critic_optimizer.step()

        # Actor 업데이트
        for p in self.Critic.parameters(): p.requires_grad = False
        scaled_action = self.Actor(state_img, state_sensor)
        actor_loss = -self.Critic(state_img, state_sensor, scaled_action).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        for p in self.Critic.parameters(): p.requires_grad = True

        torch.nn.utils.clip_grad_norm_(self.Actor.parameters(), max_norm=1.0)

        for name, param in self.Actor.named_parameters():
            if param.grad is not None and (torch.isnan(param.grad).any() or torch.isinf(param.grad).any()):
                print(f"CRITICAL: Actor gradient NaN/Inf at {name}, step {self.train_step}! Exiting...")
                import sys; sys.exit(1)

        self.actor_optimizer.step()

        self.soft_update_target()

        # 주기적 로깅
        if self.train_step <= 10 or self.train_step % 1000 == 0:
            print(f"[Step {self.train_step}] actor_loss={actor_loss.item():.4f}, "
                  f"critic_loss={critic_loss.item():.4f}, Q={q.mean().item():.4f}")

        return actor_loss.item(), critic_loss.item(), q.mean().item()

    def save_model(self, path='./ddpg_model'):
        import os
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory): os.makedirs(directory, exist_ok=True)
        torch.save({'Actor': self.Actor.state_dict(), 'Critic': self.Critic.state_dict()}, path + '_' + time.strftime('%Y%m%d_%H%M%S') + '.pth')

    def record_check_point(self, path='./ddpg_model', episode=0):
        import os
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory): os.makedirs(directory, exist_ok=True)
        torch.save({'Actor': self.Actor.state_dict(), 'Critic': self.Critic.state_dict()}, f"{path}_ep_{episode}.pth")

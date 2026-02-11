# Critic 신경망: Actor와 동일한 CNN+MLP 구조에 action을 2번째 FC에서 합류
import torch
import torch.nn as nn
import torch.nn.functional as F
import math

class Critic(torch.nn.Module):
    def __init__(self, action_size):
        super(Critic, self).__init__()

        # 카메라 이미지 CNN (Conv → ReLU, 정규화 없음)
        self.conv1 = nn.Conv2d(12, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 32, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=3, stride=1)

        # 센서 값 MLP (Linear → LayerNorm → ReLU)
        self.mlp_low1 = nn.Linear(512, 400)
        self.ln_low1 = nn.LayerNorm(400)
        self.mlp_low2 = nn.Linear(400, 300)
        self.ln_low2 = nn.LayerNorm(300)

        # 통합 FC (이미지 800 + 센서 300 = 1100)
        # 논문 권장: action은 두 번째 hidden layer에서 concat
        self.fc1 = nn.Linear(1100, 200)
        self.ln_fc1 = nn.LayerNorm(200)
        self.fc2 = nn.Linear(200 + action_size, 200)  # action을 두 번째 층에서 합류
        self.ln_fc2 = nn.LayerNorm(200)
        self.value_output = nn.Linear(200, 1)
        
        self.reset_parameters()

    def reset_parameters(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                fan_in = m.weight.data.size(1)
                lim = 1.0 / math.sqrt(fan_in)
                nn.init.uniform_(m.weight, -lim, lim)
                if m.bias is not None:
                    nn.init.uniform_(m.bias, -lim, lim)
            elif isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_in', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
        
        # 출력층은 작은 범위로 초기화 (DDPG 논문 규정)
        nn.init.uniform_(self.value_output.weight, -3e-3, 3e-3)
        nn.init.uniform_(self.value_output.bias, -3e-3, 3e-3)

    def forward(self, img, sensor, action):
        # CNN 경로: Conv → ReLU (정규화 없음)
        x = F.relu(self.conv1(img))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = torch.flatten(x, 1)  # (batch, 800)

        # 센서 MLP 경로: Linear → LayerNorm → ReLU
        x_s = F.relu(self.ln_low1(self.mlp_low1(sensor)))
        x_s = F.relu(self.ln_low2(self.mlp_low2(x_s)))

        # 통합 (state features만 먼저)
        combined = torch.cat([x, x_s], dim=1)  # (batch, 1100)

        # FC1: state features만으로 먼저 처리 (논문 권장)
        z = F.relu(self.ln_fc1(self.fc1(combined)))

        # FC2: action을 두 번째 hidden layer에서 concat (DDPG 논문 권장)
        z = torch.cat([z, action], dim=1)  # (batch, 200 + action_size)
        z = F.relu(self.ln_fc2(self.fc2(z)))

        # Q-value 출력 (발산 방지를 위해 clamp)
        q = self.value_output(z)
        return torch.clamp(q, -50.0, 50.0)

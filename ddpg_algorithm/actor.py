import torch
import torch.nn as nn
import torch.nn.functional as F
import math

# 액터 신경망 구현
# CNN(이미지) + MLP(센서) → FC → Action

class Actor(torch.nn.Module):
    def __init__(self):
        super(Actor, self).__init__()
        # 입력: (12, 64, 64) -> conv1 -> (32, 15, 15)
        # conv2 -> (32, 7, 7) -> conv3 -> (32, 5, 5)
        # Flatten: 32 * 5 * 5 = 800

        # 카메라 이미지 CNN (Conv → ReLU, 정규화 없음)
        # RL에서 CNN에 BatchNorm/GroupNorm/LayerNorm을 넣으면
        # 저분산 입력 시 1/std 폭발로 NaN gradient가 발생함.
        # 이미지가 이미 [-1, 1]로 정규화되어 있으므로 불필요.
        self.conv1 = nn.Conv2d(12, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 32, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=3, stride=1)

        # 센서 값 MLP (Linear → LayerNorm → ReLU)
        # FC 경로에서는 LayerNorm이 안정적으로 동작함 (RL 표준 관례)
        self.mlp_low1 = nn.Linear(512, 400)
        self.ln_low1 = nn.LayerNorm(400)
        self.mlp_low2 = nn.Linear(400, 300)
        self.ln_low2 = nn.LayerNorm(300)
        
        # 통합 FC (이미지 800 + 센서 300 = 1100)
        self.fc1 = nn.Linear(1100, 200)
        self.ln_fc1 = nn.LayerNorm(200)
        self.fc2 = nn.Linear(200, 200)
        self.ln_fc2 = nn.LayerNorm(200)
        self.action_output = nn.Linear(200, 2)
        
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
        nn.init.uniform_(self.action_output.weight, -3e-3, 3e-3)
        nn.init.uniform_(self.action_output.bias, -3e-3, 3e-3)

    def forward(self, img, sensor):
        # CNN 경로: Conv → ReLU (정규화 없음)
        x = F.relu(self.conv1(img))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = torch.flatten(x, 1)  # (batch, 800)

        # 센서 MLP 경로: Linear → LayerNorm → ReLU
        x_s = F.relu(self.ln_low1(self.mlp_low1(sensor)))
        x_s = F.relu(self.ln_low2(self.mlp_low2(x_s)))

        combined = torch.cat([x, x_s], dim=1)  # (batch, 1100)

        # FC 경로: Linear → LayerNorm → ReLU
        z = F.relu(self.ln_fc1(self.fc1(combined)))
        z = F.relu(self.ln_fc2(self.fc2(z)))
        raw_action = torch.tanh(self.action_output(z))
        
        # Actor가 직접 환경 제약에 맞는 스케일의 Action을 출력
        # Steering: [-1, 1] -> [-0.6, 0.6]
        # Throttle: [-1, 1] -> [-1.0, 1.0]
        scale_tensor = torch.tensor([0.6, 1.0], device=raw_action.device)
        return raw_action * scale_tensor

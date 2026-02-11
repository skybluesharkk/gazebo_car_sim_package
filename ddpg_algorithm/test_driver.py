import rclpy
import torch
import numpy as np
import cv2
import time
import os
import argparse

from env_node import EnvNode
from ddpg_agent import DDPGAgent
from frame_stack import FrameStack

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def main():
    parser = argparse.ArgumentParser(description='Test Driver and Video Recorder')
    parser.add_argument('--model', type=str, required=True, help='Path to the model checkpoint (.pth)')
    parser.add_argument('--output', type=str, default='driving_video.mp4', help='Output video filename')
    parser.add_argument('--max_steps', type=int, default=1000, help='Maximum steps to record')
    args = parser.parse_args()

    rclpy.init()
    
    env = EnvNode()
    agent = DDPGAgent()
    frame_stack = FrameStack(4)
    
    # 모델 로드
    print(f"Loading model from: {args.model}")
    if os.path.exists(args.model):
        checkpoint = torch.load(args.model, map_location=device)
        agent.Actor.load_state_dict(checkpoint['Actor'])
        agent.Actor.eval()
        print("Model loaded successfully!")
    else:
        print(f"Error: Model file {args.model} not found.")
        return

    # 비디오 설정
    print("Waiting for sensors...")
    env.reset()
    obs = env.get_observation()
    while obs['image'] is None:
        rclpy.spin_once(env)
        obs = env.get_observation()
    
    height, width, channels = 64, 64, 3
    scale = 4
    video_w = width * scale
    video_h = height * scale
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(args.output, fourcc, 10.0, (video_w, video_h))
    
    print(f"Recording started: {args.output}")
    
    state = frame_stack.reset(obs)
    
    steps = 0
    total_reward = 0
    
    try:
        while steps < args.max_steps:
            # 전처리 (train/get_action과 동일)
            state_img = torch.FloatTensor(state['image']).permute(0, 3, 1, 2).reshape(1, 12, 64, 64).to(device)
            state_val = torch.FloatTensor(state['sensors']).flatten().unsqueeze(0).to(device) * 2.0 - 1.0  # [0,1] → [-1,1]
            
            with torch.no_grad():
                action_tensor = agent.Actor(state_img, state_val)
                action = action_tensor.cpu().numpy()[0]
                
            next_obs_raw, reward, done = env.step(action)
            total_reward += reward
            
            # 프레임 기록
            if next_obs_raw['image'] is not None:
                # 이미지 범위: [-1, 1] → [0, 255]
                frame = ((next_obs_raw['image'] * 0.5 + 0.5) * 255).clip(0, 255).astype(np.uint8)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame_large = cv2.resize(frame, (video_w, video_h), interpolation=cv2.INTER_NEAREST)
                out.write(frame_large)
            
            next_state = frame_stack.step(next_obs_raw)
            state = next_state
            
            steps += 1
            if steps % 100 == 0:
                print(f"Step {steps}/{args.max_steps}, Reward: {total_reward:.2f}")
                
            if done:
                print("Episode finished (Done).")
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        out.release()
        env.destroy_node()
        agent.destroy_node()
        rclpy.shutdown()
        print(f"Video saved to {args.output}")
        print(f"Total Reward: {total_reward:.2f}")

if __name__ == '__main__':
    main()

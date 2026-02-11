import rclpy
import torch
import numpy as np
import random
import argparse
from torch.utils.tensorboard import SummaryWriter

from env_node import EnvNode
from ddpg_agent import DDPGAgent
from frame_stack import FrameStack


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0)
    args, unknown = parser.parse_known_args()  
    seed = args.seed

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    rclpy.init()

    writer = SummaryWriter(f"runs/ddpg_gazebo_ros2/seed_{seed}")

    env = EnvNode()
    agent = DDPGAgent()
    frame_stack = FrameStack(4)

    print(f"Start training (seed={seed})")

    for i_episode in range(1, env.MAX_EPISODE + 1):

        env.reset()
        obs_raw = env.get_observation()
        
        # 입력 데이터 검증
        for key in ["sensors", "image"]:
            if obs_raw[key] is not None and (np.isnan(obs_raw[key]).any() or np.isinf(obs_raw[key]).any()):
                obs_raw[key] = np.nan_to_num(obs_raw[key], nan=0.0, posinf=1.0, neginf=-1.0)
                   
        state = frame_stack.reset(obs_raw)

        total_reward = 0.0
        episode_actor_loss = []
        episode_critic_loss = []
        episode_q_val = []

        for t in range(env.MAX_STEP):

            action = agent.get_action(state)

            next_obs_raw, reward, done = env.step(action)

            # NaN 체크
            for key in ["sensors", "image"]:
                if next_obs_raw[key] is not None and (np.isnan(next_obs_raw[key]).any() or np.isinf(next_obs_raw[key]).any()):
                    next_obs_raw[key] = np.nan_to_num(next_obs_raw[key], nan=0.0, posinf=1.0, neginf=-1.0)

            next_state = frame_stack.step(next_obs_raw)

            agent.replay_buffer.push(
                state,
                action,
                reward,
                next_state,
                float(done)
            )

            total_reward += reward

            # 학습 (시작 시점은 ddpg_agent.py의 warmup_steps에서 통합 관리)
            actor_loss, critic_loss, q_val = agent.train_model()
            if actor_loss is not None:
                episode_actor_loss.append(actor_loss)
            if critic_loss is not None:
                episode_critic_loss.append(critic_loss)
            if q_val is not None:
                episode_q_val.append(q_val)

            if done:
                env.publish_zero_action()
                rclpy.spin_once(env, timeout_sec=0.05)
                break

            state = next_state

        # 로깅
        writer.add_scalar("Episode/Reward", total_reward, i_episode)
        writer.add_scalar("Episode/Steps", t + 1, i_episode)
        # warmup 구간(데이터 수집 중)에는 loss/Q를 기록하지 않음
        if episode_actor_loss:
            writer.add_scalar("Episode/Actor_Loss", np.mean(episode_actor_loss), i_episode)
        if episode_critic_loss:
            writer.add_scalar("Episode/Critic_Loss", np.mean(episode_critic_loss), i_episode)
        if episode_q_val:
            writer.add_scalar("Episode/Avg_Q_Value", np.mean(episode_q_val), i_episode)

        print(f"[Episode {i_episode}] reward={total_reward:.2f}, steps={t+1}")

        if i_episode % 50 == 0:
            agent.record_check_point(
                path=f"models/ddpg_seed_{seed}",
                episode=i_episode
            )

    print("Training complete")

    agent.save_model(path=f"models/ddpg_final_seed_{seed}")
    writer.close()

    env.destroy_node()
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

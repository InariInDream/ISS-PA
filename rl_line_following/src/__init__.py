from gymnasium.envs.registration import register

register(
    id='LineFollowingEnv-v0',
    entry_point='rl_environment:LineFollowingEnv',
)

# rcs_sb3

Stable-Baselines3 integration for RCS Gymnasium environments.

The package keeps SB3-specific code behind a small algorithm interface so another
RL backend can be added later without changing the environment creation code.

```python
from rcs_sb3 import SB3PPO, SB3PPOConfig

trainer = SB3PPO(SB3PPOConfig(total_timesteps=100_000))
model = trainer.build(env)
trainer.learn()
```

For runtime control, load or build the algorithm and insert it into the RCS
wrapper stack:

```python
ppo = SB3PPO.load("ppo_rcs_model")
env = ppo.as_wrapper(env)

obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step()
```

`StableBaselines3PolicyWrapper` stores the latest observation returned by the
wrapped RCS stack. On each `step()`, it asks the SB3 model for an action, converts
that action back into the RCS action dict, and passes it down to the robot layer.

By default, `SB3PPO.build()` prepares environments with `StableBaselines3Wrapper`,
which flattens RCS dict actions and optionally flattens dict observations for
`MlpPolicy`. Use `policy="MultiInputPolicy"` and `flatten_observations=False`
when you want SB3 to consume dictionary observations directly.

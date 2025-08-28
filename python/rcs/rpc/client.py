import gymnasium as gym
import rpyc
from rpyc.utils.classic import obtain

class RcsClient(gym.Env):
    def __init__(self, host='localhost', port=50051):
        super().__init__()
        self.conn = rpyc.connect(host, port)
        self.server = self.conn.root
        # Optionally, fetch spaces from server if needed
        # self.observation_space = ...
        # self.action_space = ...

    def step(self, action):
        return self.server.step(action)

    def reset(self, **kwargs):
        return self.server.reset(**kwargs)

    def get_obs(self):
        return self.server.get_obs()

    @property
    def unwrapped(self):
        return self.server.unwrapped()
    
    @property
    def action_space(self):
        return obtain(self.server.action_space())

    def close(self):
        self.conn.close()

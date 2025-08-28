
# import wrapper
from gymnasium import Wrapper
import rpyc
from rpyc.utils.server import ThreadedServer
rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True

@rpyc.service
class RcsServer(Wrapper, rpyc.Service):
    def __init__(self, env, host='localhost', port=50051):
        super().__init__(env)
        self.host = host
        self.port = port

    @rpyc.exposed
    def step(self, action):
        """Perform a step in the environment using the Wrapper base class."""
        return super().step(action)

    @rpyc.exposed
    def reset(self, **kwargs):
        """Reset the environment using the Wrapper base class."""
        return super().reset(**kwargs)

    @rpyc.exposed
    def get_obs(self):
        """Get the current observation using the Wrapper base class if available."""
        if hasattr(super(), 'get_obs'):
            return super().get_obs()
        elif hasattr(self.env, 'get_obs'):
            return self.env.get_obs()
        else:
            raise NotImplementedError("The environment does not have a get_obs method.")

    @rpyc.exposed
    def unwrapped(self):
        """Return the unwrapped environment using the Wrapper base class."""
        return super().unwrapped
    
    @rpyc.exposed
    def action_space(self):
        """Return the action space using the Wrapper base class."""
        return super().action_space

    def start(self):
        import time
        print(f"Starting RcsServer RPC (looped OneShotServer) on {self.host}:{self.port}")
        t = ThreadedServer(self, port=self.port)
        t.start()
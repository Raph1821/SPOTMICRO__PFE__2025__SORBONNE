#!/usr/bin/env python3
"""Abstract base class for environment randomizer.

Adapted from spot_mini_mini for RL_app
"""

import abc


class EnvRandomizerBase(object):
    """Abstract base class for environment randomizer.

    An EnvRandomizer is called in environment.reset(). It will
    randomize physical parameters of the objects in the simulation.
    The physical parameters will be fixed for that episode and be
    randomized again in the next environment.reset().
    
    This is CRITICAL for sim-to-real transfer - without domain randomization,
    policies trained in simulation will fail on real hardware due to:
    - Model uncertainties (mass, inertia, friction)
    - Actuator variations (motor damping, voltage)
    - Environmental differences (ground friction, disturbances)
    """

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def randomize_env(self, env):
        """Randomize the simulated objects in the environment.

        Args:
            env: The SpotMicroEnv environment to be randomized.
        """
        pass

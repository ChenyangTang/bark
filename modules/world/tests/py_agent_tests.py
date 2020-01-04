# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import numpy as np
import matplotlib as mpl
if os.environ.get('DISPLAY', '') == '':
    print('no display found. Using non-interactive Agg backend')
    mpl.use('Agg')
from bark import *
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer


# TODO: verify parameter class
class AgentTests(unittest.TestCase):
    def test_write_params_agent(self):
        params = ParameterServer()
        behavior = BehaviorConstantVelocity(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel(params)
        shape = Polygon2d([1.25, 1, 0], [
            Point2d(0, 0),
            Point2d(0, 2),
            Point2d(4, 2),
            Point2d(4, 0),
            Point2d(0, 0)
        ])
        init_state = np.zeros(4)
        agent = Agent(init_state, behavior, dynamic, execution, shape,
                      params.AddChild("agent"))
        params.save("written_agents_param_test.json")
        print(agent)

    def test_draw_agents(self):
        params = ParameterServer()
        behavior = BehaviorConstantVelocity(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel(params)
        shape = Polygon2d([1.25, 1, 0], [
            Point2d(0, 0),
            Point2d(0, 2),
            Point2d(4, 2),
            Point2d(4, 0),
            Point2d(0, 0)
        ])
        shape2 = CarLimousine()

        init_state = [0, 3, 2, 1]
        init_state2 = [0, 0, 5, 4]

        agent = Agent(init_state, behavior, dynamic, execution, shape,
                      params.AddChild("agent"))
        agent2 = Agent(init_state2, behavior, dynamic, execution, shape2,
                       params.AddChild("agent"))
        print(agent, agent2)



if __name__ == '__main__':
    unittest.main()

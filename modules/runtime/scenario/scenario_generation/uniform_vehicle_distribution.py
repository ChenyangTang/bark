# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.model_json_conversion \
  import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, \
  GoalDefinitionStateLimits, GoalDefinitionSequential
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser

import numpy as np
import math
import copy

class UniformVehicleDistribution(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(UniformVehicleDistribution, self).__init__(params,
                                                      num_scenarios,
                                                      random_seed)
    self.initialize_params(params)

  def initialize_params(self, params):
    params_temp = \
      self._params["Scenario"]["Generation"]["UniformVehicleDistribution"]
    self._map_file_name = params_temp["MapFilename",
      "Path to the open drive map", 
      "modules/runtime/tests/data/city_highway_straight.xodr",    ]
    self._goal_end = params_temp["GoalEnd",
      "The center of the agent's goal region polygon",
      [5128, 5200] ]
    self._goal_start = params_temp["GoalStart",
      "The coordinates of the start of the goal",
      [] ]
    self._sources = params_temp["Sources",
      "A list of points around which other vehicles spawn. \
        Points should be on different lanes. Lanes must be near these points \
      (<0.5m) Provide a list of lists with x,y-coordinates",
     [[5000.626, 5006.8305]]]
    self._sinks = params_temp["Sinks",
      "A list of points defining end of other vehicles routes.\
        Points should be on different lanes and match the order of the\
        source points. Lanes must be near these points (<0.5m) \
        Provide a list of lists with x,y-coordinates",
        [[ 5111.626, 5193.1725]] ]  
    assert len(self._sinks) == len(self._sources)      
    self._goal_number = params_temp["GoalNumber",
      "The number of sequential goals for other agents",
      3 ]   
    self._goal_distance = params_temp["GoalDistance",
      "The distance of sequential goals for other agents",
      [0, 30] ]
    self._goal_state_limits = params_temp["GoalStateLimits",
      "x,y and theta limits around center line of lane between start and end applied to both lateral sides \
       (only valid if start and end goal of ego are given)",
       [0.1, 0, 0.08]]
    self._vehicle_distance_range = params_temp["VehicleDistanceRange",
      "Distance range between vehicles in meter given as tuple from which" + \
      "distances are sampled uniformly",
      (10, 20)]
    self._other_velocity_range = params_temp["OtherVehicleVelocityRange",
      "Lower and upper bound of velocity in km/h given as tuple from which" + \
      " velocities are sampled uniformly",
      (20,30)]
    self._ego_velocity_range = params_temp["EgoVehicleVelocityRange",
      "Lower and upper bound of velocity in km/h given as tuple from which" + \
      " velocities are sampled uniformly",
      (20,30)]
    json_converter = ModelJsonConversion()
    self._agent_params = params_temp["VehicleModel",
      "How to model the other agents",
      json_converter.agent_to_json(self.default_agent_model())]
    if not isinstance(self._agent_params, dict):
        self._agent_params = self._agent_params.convert_to_dict()
    np.random.seed(self._random_seed)


  def create_scenarios(self, params, num_scenarios, random_seed):
    """ 
        see baseclass
    """
    scenario_list = []
    for scenario_idx in range(0, num_scenarios):
      scenario = self.create_single_scenario()     
      scenario_list.append(scenario)
    return scenario_list

  def sequential_goal(self, do_lane_change, goal_start, goal_end):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.convert_to_dict())
    world = scenario.get_world_state()
    goal_list = []
    goal_start = copy.deepcopy(goal_start)
    goal_end = copy.deepcopy(goal_end)

    if do_lane_change == 1:
      # need to change in the future
      goal_start[0]  += 4.
      goal_end[0] += 4.

    connecting_center_line, s_start, s_end, _, lane_id_end = \
        self.center_line_between_source_and_sink(world.map,
                                                 goal_start,
                                                 goal_end)

    goal_center_line = get_line_from_s_interval(connecting_center_line, s_start, s_end)
    # build polygon representing state limits
    lims = self._goal_state_limits
    goal_limits_left = goal_center_line.translate(Point2d(-lims[0], -lims[1]))
    goal_limits_right = goal_center_line.translate(Point2d(lims[0], lims[1]))
    goal_limits_right.reverse()
    goal_limits_left.append_linestring(goal_limits_right)
    polygon = Polygon2d([0,0,0], goal_limits_left)

    # build sequential goals
    num = self._goal_number[0]
    for i in range(num):
      goal_definition = GoalDefinitionStateLimits(polygon, (1.57-0.08, 1.57+0.08))
      goal_list.append(goal_definition)
    return GoalDefinitionSequential(goal_list)

  def create_agents_on_line(self, world, sinks):
    agent_list = []

    # create all agents
    for idx, source in enumerate(self._sources):
      connecting_center_line, s_start, s_end, _, lane_id_end = \
        self.center_line_between_source_and_sink(world.map,
                                                 source,
                                                 sinks[idx])
      goal_center_line = get_line_from_s_interval(connecting_center_line, s_start, s_end)
      goal_polygon = Polygon2d([0, 0, 0],
                               [Point2d(-1.5,0),
                                Point2d(-1.5,8),
                                Point2d(1.5,8),
                                Point2d(1.5,0)])
      goal_polygon = goal_polygon.translate(Point2d(sinks[idx][0],
                                                    sinks[idx][1]))
      goal_definition = GoalDefinitionPolygon(goal_polygon)
      agent_list.extend(
        self.place_agents_along_linestring(world,
                                           connecting_center_line,
                                           s_start,
                                           s_end,
                                           self._agent_params,
                                           goal_definition))
    return agent_list

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.convert_to_dict())
    world = scenario.get_world_state()

    # creat all agents on both lanes
    agent_list = self.create_agents_on_line(world, self._sinks)
    description=self._params.convert_to_dict()
    description["ScenarioGenerator"] = "UniformVehicleDistribution"

    # choose which vehicle will be the ego agent
    num_agents = len(agent_list)
    ego_agent = agent_list[math.floor(num_agents/4)]

    # TODO: all agents
    # create sequential goal for all agents
    agent_num = len(agent_list)
    do_lane_change = np.random.randint(0, 2)
    
    for agent in agent_list:
      if agent_list.index(agent) < (agent_num//2):
        if agent == ego_agent: local_para = do_lane_change
        else: local_para = 0
      else: local_para = 1
      agent.goal_definition = \
      self.sequential_goal(local_para,
                           self._goal_start,
                           self._goal_end)
    ego_agent.goal_definition.lane_change = do_lane_change

    # only one agent is ego in the middle of all other agents
    scenario._agent_list = agent_list
    scenario._eval_agent_ids = [ego_agent.id]
    return scenario

  def place_agents_along_linestring(self,
                                    world,
                                    linestring,
                                    s_start,
                                    s_end,
                                    agent_params,
                                    goal_definition):
    s = s_start
    if s_end < s_start:
      linestring.reverse()
      s = s_end
      s_end = s_start
    agent_list = []
    while s < s_end:
      # set agent state on linestring with random velocity
      xy_point =  get_point_at_s(linestring, s)
      angle = get_tangent_angle_at_s(linestring, s)
      
      velocity = self.sample_velocity_uniform(self._other_velocity_range)
      agent_state = np.array([0, xy_point.x(), xy_point.y(), angle, velocity ])

      agent_params = self._agent_params.copy()
      agent_params["state"] = agent_state
      agent_params["goal_definition"] = goal_definition
      agent_params["map_interface"] = world.map

      converter = ModelJsonConversion()
      bark_agent = converter.agent_from_json(agent_params, self._params)
      agent_list.append(bark_agent)

      # move forward on linestring based on vehicle size and max/min distance
      s += bark_agent.shape.front_dist + bark_agent.shape.rear_dist + \
                  self.sample_distance_uniform(self._vehicle_distance_range)
    return agent_list


  def sample_velocity_uniform(self, velocity_range):
    return np.random.uniform(velocity_range[0], velocity_range[1])

  def sample_distance_uniform(self, distance_range):
    return np.random.uniform(distance_range[0], distance_range[1])

  def sample_srange_uniform(self, srange):
    return np.random.uniform(srange[0], srange[1])

  def center_line_between_source_and_sink(self, map_interface, source, sink):
    lane_source = map_interface.find_nearest_lanes(Point2d(source[0],
                                                           source[1]),
                                                   1)[0]
    lane_sink = map_interface.find_nearest_lanes(Point2d(sink[0],
                                                         sink[1]),
                                                 1)[0]
    driving_corridor = \
      map_interface.compute_driving_corridor_from_start_to_goal(
        lane_source.lane_id,
        lane_sink.lane_id)

    _, s_start, _ = get_nearest_point_and_s(driving_corridor.center,
                                            Point2d(source[0],source[1]))
    _, s_end, _ = get_nearest_point_and_s(driving_corridor.center,
                                          Point2d(sink[0],sink[1]))
    return driving_corridor.center, \
           s_start, \
           s_end, \
           lane_source.lane_id, \
           lane_sink.lane_id


  def default_agent_model(self):
    param_server = ParameterServer()
    behavior_model = BehaviorConstantVelocity(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)
    map_interface = MapInterface()

    agent_2d_shape = CarLimousine()
    init_state = np.array([0, 0, 0, 0, 0])

    agent_default = Agent(init_state,
                          behavior_model,
                          dynamic_model,
                          execution_model,
                          agent_2d_shape,
                          param_server)

    return agent_default

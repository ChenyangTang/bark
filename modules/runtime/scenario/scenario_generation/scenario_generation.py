# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import pickle
import os
from modules.runtime.commons.parameters import ParameterServer

class ScenarioGeneration:
  def __init__(self, params=None, num_scenarios=None, random_seed=None):
    self._params = params
    self._random_seed = random_seed
    self._current_scenario_idx = 0

    if params is None:
        self._params = ParameterServer()
    else:
        self._params = params
    self.initialize_params(params=self._params)

    self._scenario_list = self.create_scenarios(params, num_scenarios, random_seed)

  def initialize_params(self, params):
    """Initialize params or default params necessary for scenario creation
    
    Arguments:
        params {[modules.runtime.commons.parameters.ParameterServer]} --
              [a parameter server instance to init params from]

    """
    pass

  @property
  def params(self):
      return self._params

  def get_next_scenario(self):
    if self._current_scenario_idx >= self.num_scenarios:
      self._current_scenario_idx = 0
      #print("Resetting scenario index to zero")
    scenario = self.get_scenario(self._current_scenario_idx)
    scenario_idx = self._current_scenario_idx
    self._current_scenario_idx += 1
    return scenario, scenario_idx

  def get_num_scenarios(self):
    return len(self._scenario_list)

  def get_scenario(self, idx):
    return self._scenario_list[idx].copy()

  def __iter__(self):
    self._current_iter_idx=0
    return self

  def __next__(self):
    if self._current_iter_idx < self.get_num_scenarios():
        scenario = self.get_scenario(self._current_iter_idx)
        idx = self._current_iter_idx
        self._current_iter_idx += 1
        return scenario, idx
    else:
        raise StopIteration

  def create_scenarios(self, params, num_scenarios, random_seed):
    """ Creates a list of scenario class instances which should be
        deterministically reproducible given the random seed,
        the params and the number of scenarios
    
    Arguments:
        params {[bark.common.ParameterServer]} -- [provides additional parameters]
        num_scenarios {[int]} -- [how many scenarios should be created]
        random_seed {[unsigned int]} -- [seed used to make scenario generation
                                         based on random factors reproducible]
    
    Returns:
        scenario_list {[a list of instances of type scenario class]} -- 
          [each scenario in this list defines one initial world condition]
    """
    return None

  @property
  def num_scenarios(self):
    return len(self._scenario_list)

  def dump_scenario_list(self, filename):
    with open(filename, "wb") as file:
      # print("SAVE PATH:", os.path.abspath(filename))
      pickle.dump(self._scenario_list, file)

  def load_scenario_list(self, filename):
    with open(filename, "rb") as file:
      self._scenario_list = pickle.load(file)

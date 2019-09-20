# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from load.benchmark_database import BenchmarkDatabase
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import time

                                                
db = BenchmarkDatabase(database_root="external/benchmark_database_release")
scenario_generation = db.get_scenario_generator(scenario_set_id=0)

for _ in range(0, 5): # run 5 scenarios in a row, repeating after 3
    scenario, idx = scenario_generation.get_next_scenario()
    world_state = scenario.get_world_state()
    print("Running scenario {} of {}".format(idx, scenario_generation.num_scenarios))
    for _ in range(0, 10): # run each scenario for 3 steps
        world_state.step(sim_step_time)
        viewer.drawWorld(world_state)
        viewer.show(block=False)
        time.sleep(sim_step_time/sim_real_time_factor)

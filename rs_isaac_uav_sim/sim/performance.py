# Copyright 2026 ROBOTSIX
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Optional inclusive phase timings and RTF from the actual simulation clock."""

import math
from time import perf_counter


class SimulationProfiler:
    """Measure completed windows without CUDA synchronization or ROS clocks."""

    def __init__(self, simulation_time, physics_dt, *, interval=5.0,
                 clock=perf_counter, report=print):
        """Start a timing window after simulator and transport initialization."""
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError('interval must be finite and positive')
        if not math.isfinite(physics_dt) or physics_dt <= 0:
            raise ValueError('physics_dt must be finite and positive')
        self.clock = clock
        self.report = report
        self.interval = interval
        self.physics_dt = physics_dt
        self.wall = clock()
        self.simulation_time = simulation_time
        self.last_simulation_time = simulation_time
        self.timings = {}
        self.updates = 0

    def record(self, label, elapsed):
        """Accumulate inclusive seconds for a completed phase."""
        total, count, maximum = self.timings.get(label, (0.0, 0, 0.0))
        self.timings[label] = (total + elapsed, count + 1, max(maximum, elapsed))

    def call(self, label, function, *args, **kwargs):
        """Time a call, preserving its return value and exceptions."""
        start = self.clock()
        try:
            return function(*args, **kwargs)
        finally:
            self.record(label, self.clock() - start)

    def sample(self, simulation_time):
        """Report actual sim-time advancement per wall second every interval."""
        self.updates += 1
        now = self.clock()
        elapsed = now - self.wall
        reset = simulation_time < self.last_simulation_time
        self.last_simulation_time = simulation_time
        if reset:
            self.wall, self.simulation_time = now, simulation_time
            self.timings.clear()
            self.updates = 0
            return
        if elapsed < self.interval:
            return
        sim_elapsed = simulation_time - self.simulation_time
        if elapsed > 0:
            self.report(
                f'SIM_PERF rtf={sim_elapsed / elapsed:.3f} '
                f'physics_hz={sim_elapsed / self.physics_dt / elapsed:.1f} '
                f'loop_hz={self.updates / elapsed:.1f}', flush=True)
            self.report('SIM_PROFILE inclusive wall times; nested phases overlap; '
                        'CUDA phases may include waits for preceding work:', flush=True)
            for label, (total, count, maximum) in sorted(
                    self.timings.items(), key=lambda item: item[1][0], reverse=True):
                self.report(
                    f'SIM_PROFILE {label} total_ms={total * 1000:.1f} calls={count} '
                    f'avg_ms={total * 1000 / count:.3f} max_ms={maximum * 1000:.3f}',
                    flush=True)
        self.wall, self.simulation_time = now, simulation_time
        self.updates = 0
        self.timings.clear()

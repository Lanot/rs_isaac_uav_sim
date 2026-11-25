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

"""Opt-in preallocated IO for the existing Isaac Sim 6.2 PhysX rigid-body view."""

import numpy as np


class DirectTensorIO:
    """Bypass RigidPrim indexing/scatter helpers without changing body order."""

    def __init__(self, frame_view, torch, device, count):
        """Reuse the wrapper's initialized tensor view and allocate fixed buffers."""
        # Isaac Sim 6.2 exposes this handle internally, not as a stable public
        # wrapper property. Reusing it preserves exactly the original body order.
        self.view = getattr(frame_view, '_physics_view', None)
        if self.view is None or self.view.count != count:
            raise RuntimeError('Direct tensors require an initialized 6.2 RigidPrim '
                               'with exactly the configured drone count')
        self.torch = torch
        self.device = torch.device(device)
        self.count = count
        pinned = self.device.type == 'cuda'
        self.indices = torch.arange(count, dtype=torch.int32, device=self.device)
        self.state_device = torch.empty((count, 13), dtype=torch.float32, device=self.device)
        self.state_host = torch.empty((count, 13), dtype=torch.float32,
                                      device='cpu', pin_memory=pinned)
        self.wrench_device = torch.empty((2, count, 3), dtype=torch.float32, device=self.device)
        self.wrench_host = torch.empty((2, count, 3), dtype=torch.float32,
                                       device='cpu', pin_memory=pinned)
        self.forces = self.wrench_device[0]
        self.torques = self.wrench_device[1]
        self.host_forces = self.wrench_host[0].numpy()
        self.host_torques = self.wrench_host[1].numpy()

    def read_state(self):
        """Return an owned CPU snapshot with the wrapper's wxyz convention."""
        # Native PhysX transforms use xyz + xyzw, unlike the wrapper's wxyz.
        transforms = self.view.get_transforms()
        velocities = self.view.get_velocities()
        self.torch.cat((transforms, velocities), dim=1, out=self.state_device)
        # Block before reading the host buffer; no delayed/stale sensor state.
        self.state_host.copy_(self.state_device, non_blocking=False)
        snapshot = self.state_host.numpy().copy()
        return snapshot[:, :3], snapshot[:, [6, 3, 4, 5]], snapshot[:, 7:]

    def apply_wrenches(self, forces, torques):
        """Submit the same global wrenches with one packed host/device copy."""
        np.copyto(self.host_forces, forces, casting='unsafe')
        np.copyto(self.host_torques, torques, casting='unsafe')
        # Block so the pinned staging buffer can safely be reused next step.
        self.wrench_device.copy_(self.wrench_host, non_blocking=False)
        self.view.apply_forces_and_torques_at_position(
            force_data=self.forces, torque_data=self.torques, position_data=None,
            indices=self.indices, is_global=True)

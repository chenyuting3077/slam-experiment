-- Copied verbatim from spot_cartographer's installed configuration_files
-- (/root/colcon_ws/install/spot_cartographer/share/spot_cartographer/configuration_files/assets_writer_pcd.lua
-- in the slam-lab-jazzy docker image). tracking_frame = "imu_link". Used as-is for
-- dataset2_2026_09_21, dataset3_vanjee_full, and dataset3_livox_full/cartographer_xsens.
-- See assets_writer_pcd_livox.lua for the only diff (tracking_frame = "livox_frame")
-- used by the pure-Livox cartographer run.
-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "transform.lua"

options = {
  tracking_frame = "imu_link",
  pipeline = {
    {
      action = "fixed_ratio_sampler",
      sampling_ratio = 0.2
    },
    {
      action = "min_max_range_filter",
      min_range = 0.2,
      max_range = 30.,
    },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = 0.05,
      miss_per_hit_limit = 1.5
    },
    {
      action = "write_pcd",
      filename = "map.pcd"
    },
  }
}

return options

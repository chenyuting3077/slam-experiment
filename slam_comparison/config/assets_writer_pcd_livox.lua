-- assets_writer_pcd.lua from compal_amr_allen/spot_cartographer, only tracking_frame changed to livox_frame (the Livox run).
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
  tracking_frame = "livox_frame",
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

-- Point cloud export config for the Campus dataset (single VLP-16), based on
-- assets_writer_backpack_3d.lua but simplified to just write a PCD (no
-- X-ray images, no dual-laser coloring since we only have one lidar).
options = {
  tracking_frame = "base_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.5,
      max_range = 100.,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = 0.05,
    },
    {
      action = "write_pcd",
      filename = "cartographer_campus.pcd",
    },
  }
}
return options

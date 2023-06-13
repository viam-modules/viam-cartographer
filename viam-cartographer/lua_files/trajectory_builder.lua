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

-- Copied from https://github.com/cartographer-project/cartographer/tree/ef00de2317dcf7895b09f18cc4d87f8b533a019b/configuration_files
-- This must be kept in sync with the corresponding lua files in the
-- google carbtographer project, when updating the google cartographer
-- git submodule
include "trajectory_builder_2d.lua"
include "trajectory_builder_3d.lua"

TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,
--  pure_localization_trimmer = {
--    max_submaps_to_keep = 3,
--  },
  collate_fixed_frame = true,
  collate_landmarks = false,
}

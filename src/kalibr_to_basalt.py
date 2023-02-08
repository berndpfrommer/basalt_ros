#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import yaml
import argparse
import numpy as np
import json
import transforms3d


def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(e)


def make_intrinsics(k):
    intr = k['intrinsics']
    y = {"fx": intr[0],
         "fy": intr[1],
         "cx": intr[2],
         "cy": intr[3]}
    distortion_model = k['distortion_model']
    if (distortion_model != 'equidistant'):
        raise Exception('unimplemented distortion model: '
                        + distortion_model)
    dc = k['distortion_coeffs']
    y["k1"] = dc[0]
    y["k2"] = dc[1]
    y["k3"] = dc[2]
    y["k4"] = dc[3]
    return y


def make_tf(k):
    y = {}
    T = np.array(k["T_cam_imu"])
    q = transforms3d.quaternions.mat2quat(T[0:3, 0:3])
    p = T[0:3, 3]
    y["px"] = p[0]
    y["py"] = p[1]
    y["pz"] = p[2]
    y["qw"] = q[0]
    y["qx"] = q[1]
    y["qy"] = q[2]
    y["qz"] = q[3]
    return y


def parse_camera(k):
    y = {}
    y["intrinsics"] = make_intrinsics(k)
    y["resolution"] = k['resolution']
    y["T_imu_cam"] = make_tf(k)
    y["camera_type"] = "kb4"
    y["timeshift_cam_imu"] = k['timeshift_cam_imu']
    return y


def kalibr_to_basalt(kalibr_imu, chain):
    basalt = {"T_imu_cam": [],
              "intrinsics": [],
              "resolution": [],
              "vignette": []}

    sum_offset = 0
    for k, v in chain.items():
        kalibr = parse_camera(v)
        basalt["T_imu_cam"].append(kalibr["T_imu_cam"])
        basalt["intrinsics"].append(
            {"camera_type": kalibr["camera_type"],
             "intrinsics": kalibr["intrinsics"]})
        basalt["resolution"].append(kalibr["resolution"])
        ts = kalibr["timeshift_cam_imu"]
        sum_offset += float(ts)
    sum_offset = sum_offset / len(chain)

    imu = kalibr_imu["imu0"]
    # from basalt code comments:
    # [b_x, b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6]
    s = 0.0  # realsense example has s = 0.02
    basalt["calib_accel_bias"] = [0, 0, 0, s, 0, 0, s, 0, s]
    # from basalt code comments:
    # calibrates rotation, axis scaling and misalignment and has 12 parameters
    # [b_x, b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6, s_7, s_8, s_9]
    # assume all zeros
    basalt["calib_gyro_bias"] = [0] * 12
    basalt["imu_update_rate"] = imu["update_rate"]
    basalt["accel_noise_std"] = [imu["accelerometer_noise_density"]] * 3
    basalt["gyro_noise_std"] = [imu["gyroscope_noise_density"]] * 3
    basalt["accel_bias_std"] = [imu["accelerometer_random_walk"]] * 3
    basalt["gyro_bias_std"] = [imu["gyroscope_random_walk"]] * 3
    # basalt["cam_time_offset_ns"] = int(sum_offset * 1e9)
    basalt["cam_time_offset_ns"] = int(0)

    return {"value0": basalt}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--camchain_imu", required=True,
                        help="imu camchain input file from kalibr output")
    parser.add_argument("-i", "--imu", required=True,
                        help="imu input file from kalibr output")
    parser.add_argument("-o", "--output", required=True,
                        help="output file in basalt format")
    args = parser.parse_args()

    imu_dict = read_yaml(args.imu)
    chain_dict = read_yaml(args.camchain_imu)
    basalt_dict = kalibr_to_basalt(imu_dict, chain_dict)
    json_object = json.dumps(basalt_dict, indent=4)
    with open(args.output, "w") as outfile:
        outfile.write(json_object)


if __name__ == "__main__":
    main()

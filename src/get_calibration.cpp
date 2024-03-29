// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <basalt/device/rs_t265.h>
#include <getopt.h>

#include <iostream>
#include <string>

// needed for writing calibration via cereal
#include <basalt/serialization/headers_serialization.h>

static void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "get_calibration -o output_file_name [-a "
               "discrete_time_accel_noise_cov]"
            << std::endl;
}

int main(int argc, char ** argv)
{
  std::string calib_file_name;
  double discrete_time_accel_noise_cov(-1.0);
  int opt;

  while ((opt = getopt(argc, argv, "o:h?a:")) != -1) {
    switch (opt) {
      case 'o':
        calib_file_name = optarg;
        break;
      case 'a':
        discrete_time_accel_noise_cov = std::stod(optarg);
        break;
      case 'h':
      case '?':
      default:
        usage();
        return (-1);
    }
  }
  if (calib_file_name.empty()) {
    usage();
    return (-1);
  }

  const int skip_frames = 1;
  const bool manual_exposure = false;
  const int webp_quality = 90;
  const float exposure = 5.0;
  basalt::RsT265Device::Ptr t265_device;

  t265_device.reset(new basalt::RsT265Device(
    manual_exposure, skip_frames, webp_quality, exposure));
  t265_device->start();

  auto calib = t265_device->exportCalibration();
  if (calib) {
    if (discrete_time_accel_noise_cov > 0) {
      const double ans =
        std::sqrt(discrete_time_accel_noise_cov / calib->imu_update_rate);
      std::cout
        << "WARNING: OVERWRITING accel_noise_cov: "
        << calib->dicrete_time_accel_noise_std().array().square().transpose()
        << " with: " << discrete_time_accel_noise_cov << std::endl;
      calib->accel_noise_std = Eigen::Vector3d(ans, ans, ans);
    }
    std::ofstream os(calib_file_name);
    cereal::JSONOutputArchive archive(os);
    archive(*calib);
  } else {
    std::cerr << "NO CALIBRATION WRITTEN!" << std::endl;
  }

  return (0);
}

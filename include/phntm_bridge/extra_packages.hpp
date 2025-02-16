#pragma once

bool installExtraPackages();

const auto CONFIG_PATH = "/ros2_ws/phntm_bridge_params.yaml";
const auto CHECKED_PACKAGES_FILE_PATH = "/ros2_ws/phntm_checked_packages.yaml";
const auto IGNORE_PACKAGES_CHECK_FILE = "/ros2_ws/phntm_no_packages_check.lock";
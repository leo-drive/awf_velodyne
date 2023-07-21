#pragma once

#include "rclcpp/rclcpp.hpp"

namespace velodyne_pointcloud
{

class InvalidPointChecker
{
public:
  InvalidPointChecker(
    const bool & _is_remove_active, const std::vector<long> & _invalid_rings,
    const std::vector<long> & _invalid_angles_start, const std::vector<long> & _invalid_angles_end);

  /**
   * @brief Checks if the point is invalid.
   * @param ring The ring number of the point.
   * @param azimuth The azimuth angle of the point.
   * @return True if the point is invalid, false otherwise.
   */
  [[nodiscard]] bool is_invalid(const uint16_t & ring, const uint16_t & azimuth) const;

private:
  const bool is_remove_active_;
  const std::vector<long> invalid_rings_;
  const std::vector<long> invalid_angles_start_;
  const std::vector<long> invalid_angles_end_;
};

}  // namespace velodyne_pointcloud

#include "velodyne_pointcloud/InvalidPointChecker.h"

namespace velodyne_pointcloud
{
InvalidPointChecker::InvalidPointChecker(
  const bool & _is_remove_active, const std::vector<long> & _invalid_rings,
  const std::vector<long> & _invalid_angles_start, const std::vector<long> & _invalid_angles_end)
: is_remove_active_(_is_remove_active),
  invalid_rings_(_invalid_rings),
  invalid_angles_start_(_invalid_angles_start),
  invalid_angles_end_(_invalid_angles_end)
{
}
bool InvalidPointChecker::is_invalid(const uint16_t & ring, const uint16_t & azimuth) const
{
  if (!is_remove_active_) {
    return false;
  }

  const auto iter = std::find(invalid_rings_.begin(), invalid_rings_.end(), ring);
  if (iter != invalid_rings_.end()) {
    const auto index = iter - invalid_rings_.begin();
    if (azimuth >= invalid_angles_start_[index] && azimuth <= invalid_angles_end_[index]) {
      return true;
    }
  }

  return false;
}

}  // namespace velodyne_pointcloud
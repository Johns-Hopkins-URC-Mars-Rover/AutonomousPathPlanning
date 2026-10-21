#pragma once

#include <cstddef>
#include <map>
#include <utility>
#include <vector>

namespace rover_imu_fusion
{

// Holds items until their timestamp is older than a cutoff, then releases them
// in timestamp order. Items with equal stamps keep their arrival order.
// When full, the oldest item is dropped.
template<typename T>
class ReorderBuffer
{
public:
  explicit ReorderBuffer(std::size_t max_size = 1000)
  : max_size_(max_size) {}

  void push(double stamp, T item)
  {
    items_.emplace(stamp, std::move(item));
    while (items_.size() > max_size_) {
      items_.erase(items_.begin());
      ++dropped_;
    }
  }

  // Removes and returns every item with stamp <= cutoff, oldest first.
  std::vector<T> popUntil(double cutoff)
  {
    std::vector<T> out;
    auto end = items_.upper_bound(cutoff);
    for (auto it = items_.begin(); it != end; ++it) {
      out.push_back(std::move(it->second));
    }
    items_.erase(items_.begin(), end);
    return out;
  }

  std::size_t size() const {return items_.size();}
  std::size_t dropped() const {return dropped_;}

private:
  std::size_t max_size_;
  std::size_t dropped_{0};
  std::multimap<double, T> items_;
};

}  // namespace rover_imu_fusion

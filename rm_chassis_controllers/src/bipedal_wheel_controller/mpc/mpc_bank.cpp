#include "bipedal_wheel_controller/mpc/mpc_bank.h"
#include "bipedal_wheel_controller/dynamics/model10.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace rm_chassis_controllers
{
bool mpc_bank::initialize(const PhysicalModel& model, const mpc_backend::Params& params,
                          const std::vector<double>& leg_lengths, double hysteresis)
{
  entries_.clear();
  leg_lengths_.clear();
  selected_ = false;
  if (!model.geometry_domain.allFinite() || model.geometry_domain[0] >= model.geometry_domain[1] ||
      leg_lengths.size() < 2 || !std::isfinite(hysteresis) || hysteresis < 0. ||
      leg_lengths.front() != model.geometry_domain[0] || leg_lengths.back() != model.geometry_domain[1])
    return false;
  for (std::size_t i = 0; i < leg_lengths.size(); ++i)
    if (!std::isfinite(leg_lengths[i]) ||
        (i > 0 && (leg_lengths[i] <= leg_lengths[i - 1] || hysteresis >= .5 * (leg_lengths[i] - leg_lengths[i - 1]))))
      return false;

  try
  {
    std::vector<std::unique_ptr<Entry>> entries;
    entries.reserve(leg_lengths.size() * leg_lengths.size());
    for (double left : leg_lengths)
      for (double right : leg_lengths)
      {
        std::unique_ptr<Entry> entry(new Entry);
        entry->lengths << left, right;
        const auto linear_model = linearModel(model, entry->lengths);
        entry->equilibrium = linear_model.equilibrium;
        entry->feedforward = linear_model.uff;
        if (!entry->backend.initialize(linear_model, params))
          return false;
        entries.push_back(std::move(entry));
      }
    leg_lengths_ = leg_lengths;
    entries_ = std::move(entries);
    hysteresis_ = hysteresis;
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

std::size_t mpc_bank::selectAxis(double length, std::size_t previous) const
{
  const auto count = leg_lengths_.size();
  if (selected_)
  {
    const double lower =
        previous == 0 ? leg_lengths_.front() : .5 * (leg_lengths_[previous - 1] + leg_lengths_[previous]) - hysteresis_;
    const double upper = previous + 1 == count ?
                             leg_lengths_.back() :
                             .5 * (leg_lengths_[previous] + leg_lengths_[previous + 1]) + hysteresis_;
    if (length >= lower && length <= upper)
      return previous;
  }
  const auto upper = std::lower_bound(leg_lengths_.begin(), leg_lengths_.end(), length);
  if (upper == leg_lengths_.begin())
    return 0;
  if (upper == leg_lengths_.end())
    return count - 1;
  const auto index = static_cast<std::size_t>(upper - leg_lengths_.begin());
  return length - leg_lengths_[index - 1] <= leg_lengths_[index] - length ? index - 1 : index;
}

mpc_bank::Entry* mpc_bank::select(const Eigen::Vector2d& lengths)
{
  if (entries_.empty() || !lengths.allFinite() || (lengths.array() < leg_lengths_.front()).any() ||
      (lengths.array() > leg_lengths_.back()).any())
  {
    reset();
    return nullptr;
  }
  const auto count = leg_lengths_.size();
  const auto left = selectAxis(lengths[0], active_index_ / count);
  const auto right = selectAxis(lengths[1], active_index_ % count);
  const auto index = left * count + right;
  if (!selected_ || index != active_index_)
    entries_[index]->backend.reset();
  active_index_ = index;
  selected_ = true;
  return entries_[index].get();
}

void mpc_bank::reset()
{
  if (selected_)
    entries_[active_index_]->backend.reset();
  selected_ = false;
}

const mpc_bank::Entry* mpc_bank::active() const
{
  return selected_ ? entries_[active_index_].get() : nullptr;
}

std::size_t mpc_bank::size() const
{
  return entries_.size();
}
}  // namespace rm_chassis_controllers

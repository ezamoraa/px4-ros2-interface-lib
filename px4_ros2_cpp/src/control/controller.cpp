#include <px4_ros2/control/controller.hpp>

#include <cassert>
#include <cfloat>

namespace px4_ros2
{

bool ControllerBase::doRegister()
{
  if (_registered) {return false;} // support call from multiple modes
  assert(_setpoint_types.empty());

  for (auto * setpoint : _new_setpoint_types) {
    _setpoint_types.push_back(setpoint->getSharedPtr());
    setpoint->doRegister();
  }
  _new_setpoint_types.clear();
  _new_contexts.clear();
  setControllerUpdateRateFromSetpointTypes();
  _registered = true;
  return true;
}

void ControllerBase::doUnregister()
{
  if (!_registered) {return;}
  setActive(false);
  for (auto & setpoint : _setpoint_types) {
    setpoint->doUnregister();
  }
  _registered = false;
}

void ControllerBase::setActive(bool active)
{
  if (active) {
    _last_controller_update = node().get_clock()->now();
    onActivate();
    for (auto & setpoint : _setpoint_types) {
      setpoint->setShouldActivateCallback(
        [this, setpoint]() {
          for (auto & setpoint_type : _setpoint_types) {
            if (setpoint_type.get() == setpoint.get()) {
              if (!setpoint_type->active()) {
                setpoint_type->setActive(true);
                setConfigurationFromSetpointType(*setpoint);  // use active setpoint configuration
                RCLCPP_DEBUG(node().get_logger(), "Changing setpoint type");
              }
            } else {
              setpoint_type->setActive(false);
            }
          }
        });
    }
  } else {
    for (auto & setpoint : _setpoint_types) {
      setpoint->setActive(false);
    }
    onDeactivate();
  }
  _active = active;
  updateControllerUpdateTimer();
}

void ControllerBase::updateControllerUpdateTimer()
{
  const bool activate = _active && _controller_update_rate_hz > FLT_EPSILON;

  if (activate) {
    if (!_controller_update_timer) {
      _controller_update_timer = node().create_wall_timer(
        std::chrono::milliseconds(
          static_cast<int64_t>(1000.f /
          _controller_update_rate_hz)), [this]() {
          const auto now = node().get_clock()->now();
          const float dt_s = (now - _last_controller_update).seconds();
          _last_controller_update = now;
          updateController(dt_s);
        });
    }
  } else {
    if (_controller_update_timer) {
      _controller_update_timer.reset();
    }
  }
}

void ControllerBase::setControllerUpdateRate(float rate_hz)
{
  _controller_update_timer.reset();
  _controller_update_rate_hz = rate_hz;
  updateControllerUpdateTimer();
}

void ControllerBase::setControllerUpdateRateFromSetpointTypes()
{
  // Set update rate based on setpoint types
  float max_update_rate = -1.f;
  for (const auto & setpoint_type : _setpoint_types) {
    if (setpoint_type->desiredUpdateRateHz() > max_update_rate) {
      max_update_rate = setpoint_type->desiredUpdateRateHz();
    }
  }
  if (max_update_rate > 0.f) {
    setControllerUpdateRate(max_update_rate);
  }
}

void ControllerBase::setConfigurationFromSetpointType(SetpointBase & setpoint_type)
{
  _configuration = setpoint_type.getConfiguration();
}

void ControllerBase::addSetpointType(SetpointBase * setpoint)
{
  assert(!_registered); // enforce initialization before registration (i.e. in constructor)
  _new_setpoint_types.push_back(setpoint);
}

void ControllerBase::setRequirement(const RequirementFlags & requirement_flags)
{
  assert(!_registered); // enforce initialization before registration (i.e. in constructor)
  modeRequirements() |= requirement_flags;
  for (auto * context : _new_contexts) {
    context->setRequirement(modeRequirements());  // make sure all contexts are updated
  }
}

void ControllerBase::onSetpointTypeAdded(Context * context)
{
  // The controller was added to a context
  assert(!_registered); // enforce initialization before registration (i.e. in constructor)
  context->setRequirement(modeRequirements());
  _new_contexts.push_back(context);
}

} // namespace px4_ros2
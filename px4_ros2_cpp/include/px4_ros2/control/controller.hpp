#pragma once

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{

class ControllerBase : public Context, public SetpointBase
{
public:
  explicit ControllerBase(rclcpp::Node & node)
  : Context(node), SetpointBase()
  {}

  explicit ControllerBase(rclcpp::Node & node, Context & context)
  : Context(node), SetpointBase(context)
  {}

  ~ControllerBase() override = default;

  bool doRegister() override;
  void doUnregister() override;

  virtual void onActivate() {};
  virtual void onDeactivate() {};

  Configuration getConfiguration() override {return _configuration;}

  void setControllerUpdateRate(float rate_hz);

  virtual void updateController([[maybe_unused]] float dt_s) {}

  void setActive(bool active) override;

  void setRequirement(const RequirementFlags & requirement_flags) override;

  RequirementFlags & modeRequirements() {return _mode_requirements;}

  void onSetpointTypeAdded(Context * context) override;

private:
  void addSetpointType(SetpointBase * setpoint) override;

  void updateControllerUpdateTimer();

  void setConfigurationFromSetpointType(SetpointBase & setpoint_type);
  void setControllerUpdateRateFromSetpointTypes();

  Configuration _configuration{};

  float _controller_update_rate_hz{0.f};
  rclcpp::TimerBase::SharedPtr _controller_update_timer;
  rclcpp::Time _last_controller_update{};

  std::vector<std::shared_ptr<SetpointBase>> _setpoint_types;
  std::vector<SetpointBase *> _new_setpoint_types; ///< This stores new setpoints during initialization, until registration
  std::vector<Context *> _new_contexts;  ///< This stores contexts during initialization, until registration
  RequirementFlags _mode_requirements;
  bool _registered;
};

/** @}*/
} /* namespace px4_ros2 */

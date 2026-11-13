#!/usr/bin/env python3
"""Copy pinned Autoware's simulator into an isolated source overlay and instrument its step.

This never edits the pinned source checkout. The output must be a new directory
under a diagnostic overlay and is intended only for simulation diagnostics.
"""
import argparse
from pathlib import Path
import shutil
import sys


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{path}: expected one patch anchor, found {count}: {old!r}')
    path.write_text(text.replace(old, new, 1))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--autoware-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True,
                        help='new isolated source directory in a diagnostic overlay')
    args = parser.parse_args()
    source = args.autoware_root / 'src/core/autoware_simple_planning_simulator/autoware_simple_planning_simulator'
    if not (source / 'package.xml').is_file():
        parser.error(f'not an Autoware simple-planning-simulator package: {source}')
    if args.output.exists():
        parser.error(f'output already exists; refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, args.output, symlinks=True,
                    ignore=shutil.ignore_patterns('build', 'install', 'log', '__pycache__'))

    header = args.output / 'include/autoware/simple_planning_simulator/simple_planning_simulator_core.hpp'
    cpp = args.output / 'src/simple_planning_simulator/simple_planning_simulator_core.cpp'
    package = args.output / 'package.xml'
    replace_once(header, '#include "sensor_msgs/msg/imu.hpp"',
                 '#include "sensor_msgs/msg/imu.hpp"\n#include "std_msgs/msg/float64_multi_array.hpp"')
    replace_once(header, 'using sensor_msgs::msg::Imu;',
                 'using sensor_msgs::msg::Imu;\nusing std_msgs::msg::Float64MultiArray;')
    replace_once(cpp, '#include <chrono>', '#include <chrono>\n#include <cmath>')
    replace_once(header,
                 '  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;',
                 '  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;\n'
                 '  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_steering_step_;')
    # Place diagnostic state near the existing model pointer without changing model behavior.
    replace_once(header,
                 '  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer',
                 '  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer\n'
                 '  bool enable_steering_step_diagnostics_ = false;\n'
                 '  double steering_rate_limit_ = 0.0;')
    replace_once(cpp,
                 '  enable_pub_steer_ = declare_parameter("enable_pub_steer", true);',
                 '  enable_pub_steer_ = declare_parameter("enable_pub_steer", true);\n'
                 '  enable_steering_step_diagnostics_ =\n'
                 '    declare_parameter("enable_steering_step_diagnostics", false);')
    replace_once(cpp,
                 '  if (enable_pub_steer_) {\n'
                 '    pub_steer_ = create_publisher<SteeringReport>("output/steering", QoS{1});\n'
                 '  }',
                 '  if (enable_pub_steer_) {\n'
                 '    pub_steer_ = create_publisher<SteeringReport>("output/steering", QoS{1});\n'
                 '  }\n'
                 '  if (enable_steering_step_diagnostics_) {\n'
                 '    pub_steering_step_ = create_publisher<Float64MultiArray>(\n'
                 '      "output/steering_integration_step", QoS{10});\n'
                 '  }')
    replace_once(cpp, '  const double steer_rate_lim = declare_parameter("steer_rate_lim", 5.0);',
                 '  const double steer_rate_lim = declare_parameter("steer_rate_lim", 5.0);\n'
                 '  steering_rate_limit_ = steer_rate_lim;')
    replace_once(cpp,
                 '    const double dt = delta_time_.get_dt(get_clock()->now());\n\n'
                 '    if (current_control_mode_.mode == ControlModeReport::AUTONOMOUS) {',
                 '    const auto integration_time = get_clock()->now();\n'
                 '    const double dt = delta_time_.get_dt(integration_time);\n'
                 '    const double previous_steering = vehicle_model_ptr_->getSteer();\n'
                 '    double requested_steering = std::numeric_limits<double>::quiet_NaN();\n'
                 '    if (current_control_mode_.mode == ControlModeReport::AUTONOMOUS) {\n'
                 '      if (const auto * control = std::get_if<Control>(&current_input_command_)) {\n'
                 '        requested_steering = control->lateral.steering_tire_angle;\n'
                 '      }\n'
                 '    } else {\n'
                 '      requested_steering = current_manual_ackermann_cmd_.lateral.steering_tire_angle;\n'
                 '    }\n\n'
                 '    if (current_control_mode_.mode == ControlModeReport::AUTONOMOUS) {')
    replace_once(cpp,
                 '    if (simulate_motion_) {\n'
                 '      vehicle_model_ptr_->update(dt);\n'
                 '    }\n'
                 '  }',
                 '    if (simulate_motion_) {\n'
                 '      vehicle_model_ptr_->update(dt);\n'
                 '    }\n'
                 '    if (pub_steering_step_) {\n'
                 '      const double applied = vehicle_model_ptr_->getSteer();\n'
                 '      const double increment = applied - previous_steering;\n'
                 '      const bool valid_dt = std::isfinite(dt) && dt > 0.0;\n'
                 '      const double rate = valid_dt ? increment / dt : 0.0;\n'
                 '      std_msgs::msg::Float64MultiArray diagnostic;\n'
                 '      diagnostic.data = {dt, previous_steering, requested_steering, applied,\n'
                 '        increment, rate, steering_rate_limit_,\n'
                 '        static_cast<double>(integration_time.seconds()),\n'
                 '        valid_dt ? 1.0 : 0.0};\n'
                 '      pub_steering_step_->publish(diagnostic);\n'
                 '    }\n'
                 '  }')
    replace_once(package, '  <depend>sensor_msgs</depend>',
                 '  <depend>sensor_msgs</depend>\n  <depend>std_msgs</depend>')
    print(f'Prepared instrumented source copy: {args.output}')
    print('Diagnostic array order: dt, previous, requested, applied, increment, rate, limit, ROS seconds, valid_dt')
    print('Pinned source was read only; instrumentation is opt-in via enable_steering_step_diagnostics.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2)

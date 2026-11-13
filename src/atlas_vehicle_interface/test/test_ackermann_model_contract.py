from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def ros_params(relative_path):
    document = yaml.safe_load((ROOT / relative_path).read_text())
    return document["/**"]["ros__parameters"]


def test_shared_ackermann_geometry_matches_nav2_simulator():
    vehicle = ros_params("atlas_vehicle_interface/config/ackermann_vehicle.param.yaml")
    nav2 = yaml.safe_load((ROOT / "atlas_nav2_demo/config/ackermann_simulation.yaml").read_text())
    simulator = nav2["ackermann_simulator"]["ros__parameters"]
    adapter = nav2["ackermann_adapter"]["ros__parameters"]

    assert vehicle["wheel_base"] == simulator["wheelbase_m"] == adapter["wheelbase_m"] == 1.20
    assert vehicle["max_steer_angle"] == simulator["steering_limit_rad"] == adapter["steering_limit_rad"] == 0.50


def test_shared_ackermann_geometry_matches_autoware_simulation():
    vehicle = ros_params("atlas_vehicle_interface/config/ackermann_vehicle.param.yaml")
    historical_vehicle = ros_params("atlas_closed_loop_demo/config/test_vehicle.param.yaml")
    simulator = ros_params("atlas_closed_loop_demo/config/simulator.param.yaml")

    assert historical_vehicle["wheel_base"] == vehicle["wheel_base"]
    assert historical_vehicle["max_steer_angle"] == vehicle["max_steer_angle"]
    assert simulator["wheel_base"] == vehicle["wheel_base"]
    assert simulator["max_steer_angle"] == vehicle["max_steer_angle"]
    assert simulator["vehicle_model_type"] == "DELAY_STEER_ACC_GEARED"


def test_autoware_actuator_limits_do_not_exceed_ackermann_model():
    vehicle = ros_params("atlas_vehicle_interface/config/ackermann_vehicle.param.yaml")
    simulator = ros_params("atlas_closed_loop_demo/config/simulator.param.yaml")
    gate = ros_params("atlas_closed_loop_demo/config/gate.param.yaml")
    planner = ros_params("atlas_closed_loop_demo/config/planning_overrides.param.yaml")
    nav2 = yaml.safe_load((ROOT / "atlas_nav2_demo/config/ackermann_simulation.yaml").read_text())
    model = nav2["ackermann_simulator"]["ros__parameters"]

    assert simulator["steer_lim"] == vehicle["max_steer_angle"] == model["steering_limit_rad"]
    assert simulator["steer_rate_lim"] < model["steering_rate_limit_radps"]
    assert simulator["vel_lim"] <= model["speed_limit_mps"]
    assert simulator["vel_rate_lim"] <= model["acceleration_limit_mps2"]
    assert planner["max_velocity"] <= model["speed_limit_mps"]
    assert planner["max_vel"] <= model["speed_limit_mps"]
    for profile_name in ("nominal", "on_transition"):
        profile = gate[profile_name]
        assert max(profile["steer_cmd_lim"]) <= vehicle["max_steer_angle"]
        assert max(profile["steer_rate_lim_for_steer_cmd"]) <= model["steering_rate_limit_radps"]
        assert profile["vel_lim"] <= model["speed_limit_mps"]


def test_installed_humble_control_contract_is_ackermann_native():
    from autoware_control_msgs.msg import Control

    command = Control()
    assert hasattr(command.lateral, "steering_tire_angle")
    assert hasattr(command.lateral, "steering_tire_rotation_rate")
    assert hasattr(command.longitudinal, "velocity")
    assert hasattr(command.longitudinal, "acceleration")

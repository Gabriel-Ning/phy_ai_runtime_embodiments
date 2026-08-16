from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


SHARE = Path(get_package_share_directory("piper_description"))


def _render(filename, **mappings):
    document = xacro.process_file(
        SHARE / "urdf" / filename,
        mappings={name: str(value).lower() for name, value in mappings.items()},
    )
    return ET.fromstring(document.toxml())


def _components(root):
    return {element.attrib["name"]: element for element in root.findall("ros2_control")}


def _plugin(component):
    return component.findtext("./hardware/plugin")


def _parameters(component):
    return {
        parameter.attrib["name"]: parameter.text
        for parameter in component.findall("./hardware/param")
    }


def _joints(component):
    return [joint.attrib["name"] for joint in component.findall("joint")]


def test_arm_component_keeps_strict_six_joint_contract():
    components = _components(_render("piper.urdf.xacro"))
    assert set(components) == {"PiperHardware"}
    arm = components["PiperHardware"]
    assert _plugin(arm) == "piper_hardware_interface/PiperHardwareInterface"
    assert _joints(arm) == [f"joint{index}" for index in range(1, 7)]
    assert "init_can" not in _parameters(arm)


def test_single_gripper_is_an_independent_real_component():
    root = _render(
        "piper_with_gripper.urdf.xacro",
        prefix="left_",
        can_interface="piper0",
    )
    components = _components(root)
    assert set(components) == {"PiperHardware", "PiperGripperHardware"}
    assert _joints(components["PiperHardware"]) == [
        f"left_joint{index}" for index in range(1, 7)
    ]

    gripper = components["PiperGripperHardware"]
    assert _plugin(gripper) == "piper_hardware_interface/PiperGripperInterface"
    assert _joints(gripper) == ["left_gripper_joint1"]
    assert _parameters(gripper) == {
        "can_interface": "piper0",
        "home_on_activate": "true",
    }

    physical_joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert (
        physical_joints["left_gripper_joint1"].find("limit").attrib["upper"] == "0.04"
    )
    assert physical_joints["left_gripper_joint2"].find("mimic").attrib == {
        "joint": "left_gripper_joint1",
        "multiplier": "-1",
        "offset": "0",
    }


def test_fake_arm_and_gripper_remain_separate_components():
    components = _components(
        _render("piper_with_gripper.urdf.xacro", use_fake_hardware="true")
    )
    assert len(components) == 2
    assert {_plugin(component) for component in components.values()} == {
        "mock_components/GenericSystem"
    }


def test_dual_arm_defaults_to_table_and_piper_grippers():
    root = _render(
        "piper_bimanual_manipulation.urdf.xacro",
        use_fake_hardware="false",
        left_can_interface="piper0",
        right_can_interface="piper1",
    )
    components = _components(root)
    assert set(components) == {
        "left_piper_hardware",
        "left_piper_gripper_hardware",
        "right_piper_hardware",
        "right_piper_gripper_hardware",
    }
    for side, can_interface in (("left", "piper0"), ("right", "piper1")):
        gripper = components[f"{side}_piper_gripper_hardware"]
        assert _plugin(gripper) == "piper_hardware_interface/PiperGripperInterface"
        assert _parameters(gripper)["can_interface"] == can_interface

    assert all(
        "init_can" not in _parameters(component) for component in components.values()
    )

    links = {link.attrib["name"] for link in root.findall("link")}
    assert {"table_base_link", "table_table_link"}.issubset(links)

    physical_joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert physical_joints["world_to_left_mount"].find("origin").attrib == {
        "rpy": "0 0 -1.57079632679",
        "xyz": "-0.38 0.32 0.71",
    }
    assert physical_joints["world_to_right_mount"].find("origin").attrib == {
        "rpy": "0 0 -1.57079632679",
        "xyz": "0.38 0.32 0.71",
    }
    for prefix in ("left_", "right_"):
        assert physical_joints[f"{prefix}gripper_tcp_joint"].find("origin").attrib == {
            "rpy": "0 0 0",
            "xyz": "0 0 0.1358",
        }


def test_dual_arm_can_disable_grippers():
    root = _render(
        "piper_bimanual_manipulation.urdf.xacro",
        enable_left_gripper="false",
        enable_right_gripper="false",
    )
    assert set(_components(root)) == {
        "left_piper_hardware",
        "right_piper_hardware",
    }
    links = {link.attrib["name"] for link in root.findall("link")}
    assert {"table_base_link", "table_table_link"}.issubset(links)


def test_dual_arm_workcell_and_hardware_tuning_are_configurable():
    root = _render(
        "piper_bimanual_manipulation.urdf.xacro",
        connected_to="cell_base",
        enable_table="false",
        use_fake_hardware="false",
        left_can_interface="piper0",
        right_can_interface="piper1",
        left_mit_kd_effort_damping="0.1",
        right_mit_kd_effort_damping="0.2",
        left_gripper_home_on_activate="false",
        right_gripper_home_on_activate="false",
    )
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    components = _components(root)

    assert "cell_base" in links
    assert {"table_base_link", "table_table_link"}.isdisjoint(links)
    assert joints["world_to_left_mount"].find("parent").attrib["link"] == "cell_base"
    assert joints["world_to_right_mount"].find("parent").attrib["link"] == "cell_base"
    assert _parameters(components["left_piper_hardware"])[
        "mit_kd_effort_damping"
    ] == "0.1"
    assert _parameters(components["right_piper_hardware"])[
        "mit_kd_effort_damping"
    ] == "0.2"
    assert _parameters(components["left_piper_gripper_hardware"])[
        "home_on_activate"
    ] == "false"
    assert _parameters(components["right_piper_gripper_hardware"])[
        "home_on_activate"
    ] == "false"

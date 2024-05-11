import yaml
import numpy as np


class SetupNotFoundError(Exception):
    pass


def load_setup(path: str):
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            f.close()
        return data
    except FileNotFoundError:
        raise SetupNotFoundError


def load_parameters(path: str):
    setup_dict = load_setup(path).get("movement_core")

    diagram_variables = setup_dict.get("diagram_variables", None)

    if diagram_variables is None:
        raise RuntimeError(
            "KinematicsSolver initialization has failed! "
            "'diagram_variables' key not found in setup dict!")

    m1 = np.array(diagram_variables.get("m1"))
    a1 = np.array(diagram_variables.get("a1"))
    a2 = np.array(diagram_variables.get("a2"))
    a3 = np.array(diagram_variables.get("a3"))
    soft_ang1 = diagram_variables.get("software_angle1")
    soft_ang2 = diagram_variables.get("software_angle2")
    soft_ang3 = diagram_variables.get("software_angle3")

    if any(vector is None for vector in [m1, a1, a2, a3, soft_ang1, soft_ang2, soft_ang3]):
        raise RuntimeError(
            "Kinematics initialization has failed! 'diagram_variables' are corrupted! ")

    return m1, a1, a2, a3, soft_ang1, soft_ang2, soft_ang3

from pathlib import Path

from apf_optuna.geometry import ObstacleCourse


COURSE_PATH = (
    Path(__file__).resolve().parents[2] / 'drone_bringup' / 'worlds' / 'optuna_course.sdf'
)


def test_course_parser_finds_repeated_collision_boxes():
    course = ObstacleCourse.from_sdf(COURSE_PATH)

    assert len(course.boxes) == 52


def test_clearance_detects_obstacle_and_gate_opening():
    course = ObstacleCourse.from_sdf(COURSE_PATH)

    assert course.clearance(100.0, 0.0, 15.0, 1.1, 0.3) < 0.0
    assert course.clearance(380.0, 0.0, 15.0, 1.1, 0.3) > 4.0
    assert course.clearance(380.0, 10.0, 15.0, 1.1, 0.3) < 0.0
    assert course.clearance(380.0, 13.5, 15.0, 1.1, 0.3) > 1.0


def test_vertical_margin_does_not_treat_drone_as_a_sphere():
    course = ObstacleCourse.from_sdf(COURSE_PATH)

    assert course.clearance(280.0, 0.0, 18.4, 1.1, 0.3) > 0.0

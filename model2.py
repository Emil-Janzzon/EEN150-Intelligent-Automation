from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse
from model.model import Model, from_goal_to_goal

g = predicates.guards.from_str
a = predicates.actions.from_str


def the_model() -> Model:

    initial_state = State(
        # control variables
        r1_robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        r1_robot_command = 'move_j',
        r1_robot_velocity = 2.0,
        r1_robot_acceleration = 0.5,
        r1_robot_goal_frame = 'unknown',   # where to go with the tool tcp
        r1_robot_tcp_frame = 'r1_svt_tcp', # the tool tcp to use
        r1_gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        r1_gripper_command = 'none', # pick_red, pick_green, pick_blue, drop_red, drop_green, drop_blue

        r2_robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        r2_robot_command = 'move_j',
        r2_robot_velocity = 2.0,
        r2_robot_acceleration = 0.5,
        r2_robot_goal_frame = 'unknown',   # where to go with the tool tcp
        r2_robot_tcp_frame = 'r2_svt_tcp', # the tool tcp to use
        r2_gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        r2_gripper_command = 'none', # pick_red, pick_green, pick_blue, drop_red, drop_green, drop_blue

        goal_as_string = "",
        replan = False,

        # measured variables
        r1_robot_state = "initial",  # "exec", "done", "failed" 
        r1_robot_pose = "unknown",
        r2_robot_state = "initial",  # "exec", "done", "failed" 
        r2_robot_pose = "unknown",
        replanned = False,

        #estimated
        green_cube_at = "pose_1", # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
        red_cube_at = "pose_2",  # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
        blue_cube_at = "pose_3",  # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
    )

    ops = {}
    
  # Move to above operation for both robots

    for r in ["r1", "r2"]:
        for pos in ["pose_1", "pose_2", "pose_3", f"{r}_buffer"]:
            not_chosen_robot = "r2" if r == "r1" else "r1"
            ops[f"{r}_op_move_to_{pos}"] = Operation(
                name=f"{r}_op_move_to_{pos}",
                precondition=Transition("pre", g(f"!{r}_robot_run && {r}_robot_state == initial && {r}_robot_pose == above_{pos} && {not_chosen_robot}_robot_pose != above_{pos} && {not_chosen_robot}_robot_pose != {pos}"), 
                a(f"{r}_robot_command = move_j, {r}_robot_run, {r}_robot_goal_frame = {pos}")),
                postcondition=Transition("post", g(f"{r}_robot_state == done"), 
                a(f"!{r}_robot_run, {r}_robot_pose <- {pos}")),
                effects=(),
                to_run=Transition.default()
            )

    for r in ["r1", "r2"]:
        for pos in ["pose_1", "pose_2", "pose_3", f"{r}_buffer"]:
            not_chosen_robot = "r2" if r == "r1" else "r1"
            ops[f"{r}_op_move_to_above_{pos}"] = Operation(
                name=f"{r}_op_move_to_above_{pos}",
                precondition=Transition("pre", g(f"!{r}_robot_run && {r}_robot_state == initial && {r}_robot_pose != above_{pos} && {not_chosen_robot}_robot_pose != above_{pos} && {not_chosen_robot}_robot_pose != {pos}"), 
                a(f"{r}_robot_command = move_j, {r}_robot_run, {r}_robot_goal_frame = above_{pos}")),
                postcondition=Transition("post", g(f"{r}_robot_state == done"), 
                a(f"!{r}_robot_run, {r}_robot_pose <- above_{pos}")),
                effects=(),
                to_run=Transition.default()
            )

    # Pick up operation for both robots
    for r in ["r1", "r2"]:
        for colour in ["blue", "red", "green"]:
            for pos in ["pose_1", "pose_2", "pose_3", f"{r}_buffer"]:
                not_chosen_robot = "r2" if r == "r1" else "r1"
                ops[f"{r}_op_pick_up_{colour}_from_{pos}"] = Operation(
                    name=f"{r}_op_pick_up_{colour}_from_{pos}",
                    precondition=Transition("pre", 
                        g(f"{r}_robot_pose == {pos} && {colour}_cube_at == {pos} && !{r}_gripper_run && {colour}_cube_at != {r}_gripper && !{r}_robot_run && {not_chosen_robot}_robot_pose != {pos} && {not_chosen_robot}_robot_pose != above_{pos}"), 
                        a(f"{r}_gripper_run, {r}_gripper_command = pick_{colour}")),
                    postcondition=Transition("post", g(f"{r}_gripper_command == done"), 
                        a(f"!{r}_gripper_run, {colour}_cube_at <- {r}_gripper")),
                    effects=(),
                    to_run=Transition.default()
                )

    # Place at operation for both robots
    for r in ["r1", "r2"]:
        for colour in ["blue", "red", "green"]:
            for pos in ["pose_1", "pose_2", "pose_3", f"{r}_buffer"]:
                not_chosen_robot = "r2" if r == "r1" else "r1"
                ops[f"{r}_op_place_{colour}_at_{pos}"] = Operation(
                    name=f"{r}_op_place_{colour}_at_{pos}",
                    precondition=Transition("pre", 
                        g(f"{r}_robot_pose == {pos} && {colour}_cube_at == {r}_gripper && {colour}_cube_at != {pos} && {r}_gripper_run && {not_chosen_robot}_robot_pose != {pos} && {not_chosen_robot}_robot_pose != above_{pos}"), 
                        a(f"{r}_gripper_run, {r}_gripper_command = place_{colour}")),
                    postcondition=Transition("post", g(f"{r}_gripper_command == done"), 
                        a(f"!{r}_gripper_run, {colour}_cube_at <- {pos}")),
                    effects=(),
                    to_run=Transition.default()
                )

        # Add transitions for free operations
    transitions = []

    return Model(
        initial_state,
        ops,
        transitions
    )

def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()

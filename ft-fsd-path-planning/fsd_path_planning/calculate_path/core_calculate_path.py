#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Path calculation class using Delaunay Triangulation.

Description: Last step in Pathing pipeline
Project: fsd_path_planning
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, cast

import numpy as np
from scipy.spatial import Delaunay

from fsd_path_planning.calculate_path.path_calculator_helpers import (
    PathCalculatorHelpers,
)
from fsd_path_planning.calculate_path.path_parameterization import PathParameterizer
from fsd_path_planning.types import BoolArray, FloatArray, IntArray
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.math_utils import (
    angle_from_2d_vector,
    circle_fit,
    normalize_last_axis,
    rotate,
    trace_distance_to_next,
    unit_2d_vector_from_angle,
    vec_angle_between,
)
from fsd_path_planning.utils.spline_fit import SplineEvaluator, SplineFitterFactory

SplineEvalByType = List[SplineEvaluator]


@dataclass
class PathCalculationInput:
    """Dataclass holding calculation variables."""
    left_cones: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    right_cones: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    left_to_right_matches: IntArray = field(default_factory=lambda: np.zeros(0, dtype=int))
    right_to_left_matches: IntArray = field(default_factory=lambda: np.zeros(0, dtype=int))
    position_global: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    direction_global: FloatArray = field(default_factory=lambda: np.array([1, 0]))
    global_path: Optional[FloatArray] = field(default=None)


@dataclass
class PathCalculationScalarValues:
    """Class holding scalar values of a path calculator."""
    maximal_distance_for_valid_path: float
    mpc_path_length: float = 30
    mpc_prediction_horizon: int = 40


class CalculatePath:
    """
    Class that takes all path calculation responsibilities after the cones have been matched.
    """

    def __init__(
        self,
        smoothing: float,
        predict_every: float,
        maximal_distance_for_valid_path: float,
        max_deg: int,
        mpc_path_length: float,
        mpc_prediction_horizon: int,
    ):
        self.input = PathCalculationInput()
        self.scalars = PathCalculationScalarValues(
            maximal_distance_for_valid_path=maximal_distance_for_valid_path,
            mpc_path_length=mpc_path_length,
            mpc_prediction_horizon=mpc_prediction_horizon,
        )
        self.path_calculator_helpers = PathCalculatorHelpers()
        self.spline_fitter_factory = SplineFitterFactory(
            smoothing, predict_every, max_deg
        )

        path_parameterizer = PathParameterizer(
            prediction_horizon=self.scalars.mpc_prediction_horizon
        )

        self.previous_paths = [
            path_parameterizer.parameterize_path(
                self.calculate_initial_path(), None, None, False
            )
        ]
        self.mpc_paths = []
        self.path_is_trivial_list = []
        self.path_updates = []

    def calculate_initial_path(self) -> FloatArray:
        initial_path = self.spline_fitter_factory.fit(
            self.path_calculator_helpers.calculate_almost_straight_path()
        ).predict(der=0)
        return initial_path

    def set_new_input(self, new_input: PathCalculationInput) -> None:
        self.input = new_input

    def calculate_trivial_path(self) -> FloatArray:
        origin_path = self.path_calculator_helpers.calculate_almost_straight_path()[1:]
        yaw = angle_from_2d_vector(self.input.direction_global)
        path_rotated: FloatArray = rotate(origin_path, yaw)  # type: ignore
        final_trivial_path: FloatArray = path_rotated + self.input.position_global
        return final_trivial_path

    def number_of_matches_on_one_side(self, side: ConeTypes) -> int:
        assert side in (ConeTypes.LEFT, ConeTypes.RIGHT)
        matches_of_side = (
            self.input.left_to_right_matches
            if side == ConeTypes.LEFT
            else self.input.right_to_left_matches
        )
        return_value: int = np.sum(matches_of_side != -1)
        return return_value

    def side_score(self, side: ConeTypes) -> tuple:
        matches_of_side = (
            self.input.left_to_right_matches
            if side == ConeTypes.LEFT
            else self.input.right_to_left_matches
        )
        matches_of_side_filtered = matches_of_side[matches_of_side != -1]
        n_matches = len(matches_of_side_filtered)
        n_indices_sum = matches_of_side_filtered.sum()
        return n_matches, n_indices_sum

    def select_side_to_use(self) -> Tuple[FloatArray, IntArray, FloatArray]:
        side_to_pick = max([ConeTypes.LEFT, ConeTypes.RIGHT], key=self.side_score)
        side_to_use, matches_to_other_side, other_side_cones = (
            (self.input.left_cones, self.input.left_to_right_matches, self.input.right_cones)
            if side_to_pick == ConeTypes.LEFT
            else (self.input.right_cones, self.input.right_to_left_matches, self.input.left_cones)
        )
        return side_to_use, matches_to_other_side, other_side_cones

    def calculate_centerline_via_delaunay(self) -> Tuple[FloatArray, Optional[Delaunay]]:
        """
        Calculates dense path points using Delaunay Triangulation.
        Applies distance and angle/index constraints to filter out bad edges.
        """
        left = self.input.left_cones
        right = self.input.right_cones
        
        if len(left) < 2 or len(right) < 2:
            self.last_tri = None
            self.last_midpoints = None
            self.valid_edges = []
            return np.zeros((0, 2)), None

        points = np.vstack([left, right])
        num_left = len(left)
        
        # 1. Perform Triangulation
        tri = Delaunay(points)
        midpoints = []
        scores = []
        valid_edges_pts = []  # Store the actual lines to draw them later

        # --- CONSTRAINTS ---
        MAX_EDGE_DISTANCE = 7.0  # Max track width + margin (filters hairpin jumps)
        MAX_INDEX_DIFF = 6       # Max offset in sorted arrays (filters bad angles/diagonals)

        # 2. Extract midpoints from valid cross-track edges
        for simplex in tri.simplices:
            for i in range(3):
                p1, p2 = simplex[i], simplex[(i + 1) % 3]
                
                is_p1_left = p1 < num_left
                is_p2_left = p2 < num_left
                
                # Check if it's a cross-track edge (one left, one right)
                if is_p1_left != is_p2_left:
                    idx_l = p1 if is_p1_left else p2
                    idx_r = (p2 if not is_p2_left else p1) - num_left
                    
                    # CONSTRAINT 1: Filter out edges that are too long
                    dist = np.linalg.norm(points[p1] - points[p2])
                    if dist > MAX_EDGE_DISTANCE:
                        continue
                        
                    # CONSTRAINT 2: Filter out long diagonals using index offset
                    if abs(idx_l - idx_r) > MAX_INDEX_DIFF:
                        continue
                        
                    midpoints.append((points[p1] + points[p2]) / 2)
                    scores.append(idx_l + idx_r)
                    valid_edges_pts.append([points[p1], points[p2]])

        if not midpoints:
            self.last_tri = tri
            self.last_midpoints = None
            self.valid_edges = []
            return np.zeros((0, 2)), tri
            
        midpoints = np.array(midpoints)
        scores = np.array(scores)
        
        # Remove identical midpoints safely
        _, unique_indices = np.unique(np.round(midpoints, 4), axis=0, return_index=True)
        unique_midpoints = midpoints[unique_indices]
        unique_scores = scores[unique_indices]
        
        # 3. Sort sequentially using the topological score
        sorted_indices = np.argsort(unique_scores)
        centerline = unique_midpoints[sorted_indices]
        
        # Save variables for the visualization script
        self.last_tri = tri
        self.last_midpoints = centerline
        self.valid_edges = valid_edges_pts
        
        return centerline, tri

    def fit_matches_as_spline(self, center_along_match_connection: FloatArray) -> FloatArray:
        try:
            path_update = self.spline_fitter_factory.fit(
                center_along_match_connection
            ).predict(der=0)
        except ValueError:
            path_update = self.spline_fitter_factory.fit(
                self.previous_paths[-1][:, 1:3]
            ).predict(der=0)
        return path_update

    def overwrite_path_if_it_is_too_far_away(self, path_update: FloatArray) -> FloatArray:
        min_distance_to_path = np.linalg.norm(
            self.input.position_global - path_update, axis=-1
        ).min()
        if min_distance_to_path > self.scalars.maximal_distance_for_valid_path:
            path_update = self.previous_paths[-1][:, 1:3]
        return path_update

    def refit_path_for_mpc_with_safety_factor(self, final_path: FloatArray) -> FloatArray:
        path_length_fixed = self.spline_fitter_factory.fit(final_path).predict(
            der=0, max_u=self.scalars.mpc_path_length * 1.5
        )
        return path_length_fixed

    def extend_path(self, path_update: FloatArray) -> FloatArray:
        car_to_path = path_update - self.input.position_global
        mask_path_is_in_front_of_car = (
            np.dot(car_to_path, self.input.direction_global) > 0
        )
        for i, value in enumerate(mask_path_is_in_front_of_car.copy()):
            if value:
                mask_path_is_in_front_of_car[i:] = True
                break

        mask_path_is_in_front_of_car[-20:] = True

        if not mask_path_is_in_front_of_car.any():
            return path_update

        path_infront_of_car = path_update[mask_path_is_in_front_of_car]
        cum_path_length = trace_distance_to_next(path_infront_of_car).cumsum()
        path_length = cum_path_length[-1]

        if path_length > self.scalars.mpc_path_length:
            return path_update

        relevant_path = path_infront_of_car[-20:]
        center_x, center_y, radius = circle_fit(relevant_path)
        center = np.array([center_x, center_y])
        radius_to_use = min(max(radius, 10), 100)

        if radius_to_use < 80:
            relevant_path_centered = relevant_path - center
            three_points = relevant_path_centered[
                [0, int(len(relevant_path_centered) / 2), -1]
            ]
            homogeneous_points = np.column_stack((np.ones(3), three_points))
            orientation = np.linalg.det(homogeneous_points)
            orientation_sign = np.sign(orientation)

            start_angle = float(angle_from_2d_vector(three_points[0]))
            end_angle = start_angle + orientation_sign * np.pi
            new_points_angles = np.linspace(start_angle, end_angle)
            new_points_raw = (
                unit_2d_vector_from_angle(new_points_angles) * radius_to_use
            )
            new_points = new_points_raw - new_points_raw[0] + path_update[-1]
        else:
            second_last_point = path_update[-2]
            last_point = path_update[-1]
            direction = last_point - second_last_point
            direction = direction / np.linalg.norm(direction)
            new_points = last_point + direction * np.arange(30)[:, None]

        new_points = new_points[1:]
        return np.row_stack((path_update, new_points))

    def create_path_for_mpc_from_path_update(self, path_update: FloatArray) -> FloatArray:
        path_connected_to_car = self.connect_path_to_car(path_update)
        path_with_enough_length = self.extend_path(path_connected_to_car)
        path_with_no_path_behind_car = self.remove_path_behind_car(
            path_with_enough_length
        )
        path_length_fixed = self.refit_path_for_mpc_with_safety_factor(
            path_with_no_path_behind_car
        )
        path_with_length_for_mpc = self.remove_path_not_in_prediction_horizon(
            path_length_fixed
        )
        return path_with_length_for_mpc

    def do_all_mpc_parameter_calculations(self, path_update: FloatArray) -> FloatArray:
        path_with_length_for_mpc = self.create_path_for_mpc_from_path_update(
            path_update
        )
        path_parameterizer = PathParameterizer(
            prediction_horizon=self.scalars.mpc_prediction_horizon
        )
        path_parameterized = path_parameterizer.parameterize_path(
            path_with_length_for_mpc,
            self.input.position_global,
            self.input.direction_global,
            path_is_closed=False,
        )
        return path_parameterized

    def cost_mpc_path_start(self, path_length_fixed: FloatArray) -> FloatArray:
        distance_cost: FloatArray = np.linalg.norm(
            self.input.position_global - path_length_fixed, axis=1
        )
        return distance_cost

    def connect_path_to_car(self, path_update: FloatArray) -> FloatArray:
        distance_to_first_point = np.linalg.norm(
            self.input.position_global - path_update[0]
        )
        car_to_first_point = path_update[0] - self.input.position_global
        angle_to_first_point = vec_angle_between(
            car_to_first_point, self.input.direction_global
        )

        if distance_to_first_point < 0.5 or angle_to_first_point > np.pi / 2:
            return path_update

        new_point = (
            self.input.position_global
            + normalize_last_axis(car_to_first_point[None])[0] * 0.2
        )
        path_update = np.row_stack((new_point, path_update))
        return path_update

    def remove_path_behind_car(self, path_length_fixed: FloatArray) -> FloatArray:
        idx_start_mpc_path = int(self.cost_mpc_path_start(path_length_fixed).argmin())
        path_length_fixed_forward: FloatArray = path_length_fixed[idx_start_mpc_path:]
        return path_length_fixed_forward

    def remove_path_not_in_prediction_horizon(
        self, path_length_fixed_forward: FloatArray
    ) -> FloatArray:
        distances = trace_distance_to_next(path_length_fixed_forward)
        cum_dist = np.cumsum(distances)
        mask_cum_distance_over_mcp_path_length: BoolArray = (
            cum_dist > self.scalars.mpc_path_length
        )
        if len(mask_cum_distance_over_mcp_path_length) <= 1:
            return self.previous_paths[-1]

        first_point_over_distance = cast(
            int, mask_cum_distance_over_mcp_path_length.argmax()
        )

        if (
            first_point_over_distance == 0
            and not mask_cum_distance_over_mcp_path_length[0]
        ):
            first_point_over_distance = len(cum_dist)
        path_with_length_for_mpc: FloatArray = path_length_fixed_forward[
            :first_point_over_distance
        ]
        return path_with_length_for_mpc

    def store_paths(
        self,
        path_update: FloatArray,
        path_with_length_for_mpc: FloatArray,
        path_is_trivial: bool,
    ) -> None:
        self.path_updates = self.path_updates[-10:] + [path_update]
        self.mpc_paths = self.mpc_paths[-10:] + [path_with_length_for_mpc]
        self.path_is_trivial_list = self.path_is_trivial_list[-10:] + [path_is_trivial]

    def run_path_calculation(self) -> Tuple[FloatArray, FloatArray]:
        """Calculate path."""
        if self.input.global_path is not None:
            distance = np.linalg.norm(
                self.input.position_global - self.input.global_path, axis=1
            )
            idx_closest_point_to_path = distance.argmin()
            roll_value = -idx_closest_point_to_path + len(self.input.global_path) // 3
            path_rolled = np.roll(self.input.global_path, roll_value, axis=0)
            distance_rolled = np.roll(distance, roll_value)
            mask_distance = distance_rolled < 30
            center_along_match_connection = path_rolled[mask_distance]

        elif len(self.input.left_cones) < 3 and len(self.input.right_cones) < 3:
            if len(self.previous_paths) > 0:
                center_along_match_connection = self.previous_paths[-1][:, 1:3]
            else:
                center_along_match_connection = self.calculate_trivial_path()
        else:
            # 1. Use Delaunay approach to generate dense waypoints
            center_along_match_connection, _ = self.calculate_centerline_via_delaunay()
            
            # 2. Safety Fallback: Use the original matching if Delaunay fails
            if len(center_along_match_connection) < 2:
                side_to_use, matches_to_other_side, other_side_cones = self.select_side_to_use()
                match_on_other_side = other_side_cones[matches_to_other_side]
                
                center_along_match_connection = self.calculate_centerline_points_of_matches(
                    side_to_use, matches_to_other_side, match_on_other_side
                )

        path_update_too_far_away = self.fit_matches_as_spline(
            center_along_match_connection
        )

        path_update = self.overwrite_path_if_it_is_too_far_away(
            path_update_too_far_away
        )

        try:
            path_parameterization = self.do_all_mpc_parameter_calculations(path_update)
        except ValueError:
            path_parameterization = self.do_all_mpc_parameter_calculations(
                self.previous_paths[-1][:, 1:3]
            )

        self.store_paths(path_update, path_parameterization, False)
        self.previous_paths = self.previous_paths[-10:] + [path_parameterization]

        return path_parameterization, center_along_match_connection
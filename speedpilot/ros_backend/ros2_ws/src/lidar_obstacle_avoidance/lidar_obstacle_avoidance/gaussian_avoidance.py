import math
from dataclasses import dataclass

import numpy as np


@dataclass
class AvoidancePlan:
    x_path: np.ndarray
    y_path: np.ndarray
    steering_angles: np.ndarray
    steering_angle: float
    next_y: float
    reference_y: float
    is_avoiding: bool


def gaussian(x, amplitude, mu, sigma):
    return amplitude * np.exp(-((x - mu) ** 2) / (2 * sigma ** 2))


def obstacle_to_local_xy(angle_deg, distance):
    angle_rad = math.radians(angle_deg)

    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)

    return x, y


class GaussianAvoidanceController:
    def __init__(
        self,
        safety_distance=0.8,
        max_steering_deg=35,
        path_length=8.0,
        points=300,
        return_tolerance=0.03,
        lookahead_factor=15,
        replan_tolerance=0.2,
    ):
        self.safety_distance = safety_distance
        self.max_steering_deg = max_steering_deg
        self.path_length = path_length
        self.points = points
        self.return_tolerance = return_tolerance
        self.lookahead_factor = lookahead_factor
        self.replan_tolerance = replan_tolerance

        self.is_avoiding = False
        self.reference_y = 0.0

        self.last_obstacles = []
        self.active_x_path = None
        self.active_y_path = None
        self.active_steering_angles = None

    def reset(self, current_y=0.0):
        self.is_avoiding = False
        self.reference_y = current_y

        self.last_obstacles = []
        self.active_x_path = None
        self.active_y_path = None
        self.active_steering_angles = None

    def update(self, current_y, visible_obstacles, speed):
        if visible_obstacles and not self.is_avoiding:
            self.reference_y = current_y
            self.is_avoiding = True
            self._replan(current_y, visible_obstacles)

        elif self.is_avoiding:
            if visible_obstacles:
                if (
                    self._obstacles_changed(visible_obstacles)
                    or self._path_collides_with_visible_obstacles(visible_obstacles)
                    or self._too_close_to_visible_obstacle(visible_obstacles)
                ):
                    self._replan(current_y, visible_obstacles)

            elif self.active_x_path is None or self.active_x_path[-1] <= 0:
                self.reset(current_y=self.reference_y)

        if self.is_avoiding:
            self.active_x_path = self.active_x_path - speed

            lookahead_x = speed * self.lookahead_factor
            lookahead_x = max(lookahead_x, 0.1)

            target_y = np.interp(
                lookahead_x,
                self.active_x_path,
                self.active_y_path,
            )

            dy = target_y - current_y
            dx = lookahead_x

            steering_angle = math.degrees(math.atan2(dy, dx))
            steering_angle = float(
                np.clip(
                    steering_angle,
                    -self.max_steering_deg,
                    self.max_steering_deg,
                )
            )

            next_y = current_y + math.tan(
                math.radians(steering_angle)
            ) * speed

            if not visible_obstacles:
                distance_to_reference = abs(next_y - self.reference_y)

                if (
                    distance_to_reference <= self.return_tolerance
                    or self.active_x_path[-1] <= 0
                ):
                    next_y = self.reference_y
                    self.reset(current_y=self.reference_y)
                    steering_angle = 0.0

            print("target_y =", target_y)
            print("dy =", dy)
            print("steering =", steering_angle)

            return AvoidancePlan(
                x_path=self.active_x_path
                if self.active_x_path is not None
                else np.linspace(0, self.path_length, self.points),
                y_path=self.active_y_path
                if self.active_y_path is not None
                else np.full(self.points, self.reference_y),
                steering_angles=self.active_steering_angles
                if self.active_steering_angles is not None
                else np.zeros(self.points),
                steering_angle=steering_angle,
                next_y=next_y,
                reference_y=self.reference_y,
                is_avoiding=self.is_avoiding,
            )

        x_path = np.linspace(0, self.path_length, self.points)
        y_path = np.full_like(x_path, current_y)
        steering_angles = np.zeros_like(x_path)

        return AvoidancePlan(
            x_path=x_path,
            y_path=y_path,
            steering_angles=steering_angles,
            steering_angle=0.0,
            next_y=current_y,
            reference_y=self.reference_y,
            is_avoiding=False,
        )
    
    def _too_close_to_visible_obstacle(self, visible_obstacles):
        for angle_deg, distance, width_deg in visible_obstacles:
            obs_x, obs_y_local = obstacle_to_local_xy(angle_deg, distance)

            if obs_x <= 0:
                continue

            obstacle_width = distance * math.tan(
                math.radians(width_deg / 2)
            )

            min_distance = obstacle_width + self.safety_distance

            distance_to_obstacle = math.sqrt(
                obs_x ** 2 + obs_y_local ** 2
            )

            if distance_to_obstacle <= min_distance:
                return True

        return False

    def _replan(self, current_y, visible_obstacles):
        self.last_obstacles = visible_obstacles.copy()

        (
            self.active_x_path,
            self.active_y_path,
            self.active_steering_angles,
        ) = self.plan_path(
            current_y=current_y,
            visible_obstacles=visible_obstacles,
        )

    def plan_path(self, current_y, visible_obstacles):
        x_path = np.linspace(0, self.path_length, self.points)

        current_offset = current_y - self.reference_y

        return_strength = 2.5
        y_relative = current_offset * np.exp(-x_path / return_strength)

        for angle_deg, distance, width_deg in visible_obstacles:
            obs_x, obs_y_local = obstacle_to_local_xy(
                angle_deg,
                distance,
            )

            if obs_x <= 0:
                continue

            obs_y_global = current_y + obs_y_local
            obs_y_relative = obs_y_global - self.reference_y

            obstacle_width = distance * math.tan(
                math.radians(width_deg / 2)
            )

            clearance = obstacle_width + self.safety_distance

            target_y_relative = self._choose_avoidance_side(
                obs_y_relative=obs_y_relative,
                clearance=clearance,
                current_offset=current_offset,
            )

            sigma = max(0.7, obstacle_width + 0.8)

            y_relative += gaussian(
                x_path,
                amplitude=target_y_relative,
                mu=obs_x,
                sigma=sigma,
            )

        y_global = self.reference_y + y_relative

        dy_dx = np.gradient(y_global, x_path)

        steering_angles = np.degrees(np.arctan(dy_dx))
        steering_angles = np.clip(
            steering_angles,
            -self.max_steering_deg,
            self.max_steering_deg,
        )

        return x_path, y_global, steering_angles

    def _choose_avoidance_side(
        self,
        obs_y_relative,
        clearance,
        current_offset,
    ):
        center_threshold = 0.2

        if obs_y_relative > center_threshold:
            return obs_y_relative - clearance

        if obs_y_relative < -center_threshold:
            return obs_y_relative + clearance

        if abs(current_offset) > center_threshold:
            return math.copysign(clearance, current_offset)

        return -clearance

    def _path_collides_with_visible_obstacles(self, visible_obstacles):
        if self.active_x_path is None or self.active_y_path is None:
            return True

        for angle_deg, distance, width_deg in visible_obstacles:
            obs_x, obs_y_local = obstacle_to_local_xy(
                angle_deg,
                distance,
            )

            if obs_x <= 0:
                continue

            obstacle_width = distance * math.tan(
                math.radians(width_deg / 2)
            )

            path_y_at_obstacle = np.interp(
                obs_x,
                self.active_x_path,
                self.active_y_path,
            )

            distance_to_path = abs(path_y_at_obstacle - obs_y_local)

            min_distance = (
                obstacle_width
                + self.safety_distance
                + self.replan_tolerance
            )

            if distance_to_path <= min_distance:
                return True

        return False

    def _obstacles_changed(self, visible_obstacles):
        old_obstacles = self._normalize_obstacles(self.last_obstacles)
        new_obstacles = self._normalize_obstacles(visible_obstacles)

        if len(old_obstacles) != len(new_obstacles):
            return True

        for obstacle in new_obstacles:
            if obstacle not in old_obstacles:
                return True

        return False

    def _normalize_obstacles(self, obstacles):
        normalized = []

        for angle_deg, distance, width_deg in obstacles:
            normalized.append(
                (
                    round(angle_deg, 0),
                    round(distance, 1),
                    round(width_deg, 0),
                )
            )

        return normalized
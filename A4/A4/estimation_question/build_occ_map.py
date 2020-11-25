#!/usr/bin/env python
from math import atan, sqrt, cos, sin, pi, atan2
from math import pi, log, exp
import numpy as np
import yaml
import cv2


class OccupancyGridMap:
    def __init__(
        self,
        num_rows,
        num_cols,
        meters_per_cell,
        grid_origin_in_map_frame,
        init_log_odds,
    ):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.meters_per_cell = meters_per_cell
        self.log_odds_ratio_occupancy_grid_map = init_log_odds * np.ones(
            (num_rows, num_cols), dtype="float64"
        )
        self.seq = 0

        self.resolution = meters_per_cell
        self.width = num_rows
        self.height = num_cols

        self.origin_x = grid_origin_in_map_frame[0]
        self.origin_y = grid_origin_in_map_frame[1]
        self.origin_z = grid_origin_in_map_frame[2]

    def update_log_odds_ratio_in_grid_coords(self, row, col, delta_log_odds):
        assert row >= 0 and row < self.num_rows
        assert col >= 0 and col < self.num_cols
        self.log_odds_ratio_occupancy_grid_map[row][col] += delta_log_odds

    def cartesian_to_grid_coords(self, x, y):
        gx = (x - self.origin_x) / self.resolution
        gy = (y - self.origin_y) / self.resolution
        row = min(max(int(gy), 0), self.num_rows)
        col = min(max(int(gx), 0), self.num_cols)
        return (row, col)

    def log_odds_ratio_to_belief(self, lor):
        return 1.0 / (1 + np.exp(lor))

    def save_map_as_image(self, filename):

        img = np.zeros((self.height, self.width, 3), np.uint8)
        for y in range(0, self.height):
            for x in range(0, self.width):
                # threshold the pixel
                img[y, x] = int(
                    self.log_odds_ratio_to_belief(
                        self.log_odds_ratio_occupancy_grid_map[y][x]
                    )
                    * 255
                )

        cv2.imwrite(filename, img)


class HuskyMapper:
    def __init__(self, num_rows, num_cols, meters_per_cell, num_points_per_scan):

        og_origin_in_map_frame = np.array([-20, -10, 0])
        self.init_log_odds_ratio = 0  # log(0.5/0.5)
        self.ogm = OccupancyGridMap(
            num_rows,
            num_cols,
            meters_per_cell,
            og_origin_in_map_frame,
            self.init_log_odds_ratio,
        )

        self.num_points_per_scan = num_points_per_scan
        self.max_laser_range = 8.0
        self.min_laser_range = 0.1
        self.max_laser_angle = 2.3496449940734436
        self.min_laser_angle = -2.3561899662017822
        self.angles_in_baselaser_frame = [
            (self.max_laser_angle - self.min_laser_angle)
            * float(i)
            / self.num_points_per_scan
            + self.min_laser_angle
            for i in range(self.num_points_per_scan)
        ]
        self.angles_in_baselaser_frame = self.angles_in_baselaser_frame[::-1]
        # This is because the z-axis of husky_1/base_laser is pointing downwards, while for husky_1/base_link and the map frame
        # the z-axis points upwards

        self.baselaser_x_in_map = None
        self.baselaser_y_in_map = None
        self.yaw_map_baselaser = None

        self.lidar_count = 0

    def from_laser_to_map_coordinates(self, points_in_baselaser_frame):
        #
        # TODO: Complete this function
        # The robot's odometry is with respect to the map frame, but the points measured from
        # the laser are given with respect to the laser. This function needs to
        # convert the measured points in the laser scan from the laser to the map frame.
        #

        # This line is a place-holder which is incorrect and should be replaced. It does
        # demonstrate the correct data structure
        points_in_map_frame = points_in_baselaser_frame

        return points_in_map_frame

    def is_in_field_of_view(self, robot_row, robot_col, laser_theta, row, col):
        # Returns true iff the cell (row, col) in the grid is in the field of view of the 2D laser of the
        # robot located at cell (robot_row, robot_col) and having yaw robot_theta in the map frame.
        # Useful things to know:
        # 1) self.ogm.meters_per_cell converts cell distances to metric distances
        # 2) atan2(y,x) gives the angle of the vector (x,y)
        # 3) atan2(sin(theta_1 - theta_2), cos(theta_1 - theta_2)) gives the angle difference between theta_1 and theta_2 in [-pi, pi]
        # 4) self.max_laser_range and self.max_laser_angle specify some of the limits of the laser sensor
        #
        # TODO: fill logic in here
        #

        m = self.ogm.meters_per_cell
        range_diff = sqrt(
            ((row) * m - robot_row * m) ** 2
            + (col * m - (robot_col) * m) ** 2
        )
        angle = atan2(((row - robot_row) * m), (col - robot_col) * m)

        angle_diff = abs(atan2(sin((angle - laser_theta) * m),
                               cos(angle - laser_theta) * m))

        if self.max_laser_range < range_diff or self.max_laser_angle < angle_diff:
            return False

        # This line is wrong. Instead of always returning true, you need to
        # compute some geometry and only return true when the cell is seen.
        return True

    def inverse_measurement_model(
        self,
        row,
        col,
        robot_row,
        robot_col,
        robot_theta_in_map,
        beam_ranges,
        beam_angles,
    ):
        alpha = 0.1
        beta = 10 * pi / 180.0
        p_occupied = 0.999

        #
        # TODO: Find the range r and angle diff_angle of the beam (robot_row, robot_col) ------> (row, col)
        # r should be in meters and diff_angle should be in [-pi, pi]. Useful things to know are same as above.
        #

        m = self.ogm.meters_per_cell

        r = sqrt(
            ((row) * m - robot_row * m) ** 2
            + (col * m - (robot_col) * m) ** 2
        )  # MUST CHANGE THIS
        angle = atan2(((row - robot_row) * m), (col - robot_col) * m)
        diff_angle = abs(atan2(sin((angle - robot_theta_in_map) * m),
                               cos(angle - robot_theta_in_map) * m))  # MUST CHANGE THIS

        closest_beam_angle, closest_beam_idx = min(
            (val, idx)
            for (idx, val) in enumerate([abs(diff_angle - ba) for ba in beam_angles])
        )
        r_cb = beam_ranges[closest_beam_idx]
        theta_cb = beam_angles[closest_beam_idx]

        if (
            r > min(self.max_laser_range, r_cb + alpha / 2.0)
            or abs(diff_angle - theta_cb) > beta / 2.0
        ):
            return self.init_log_odds_ratio

        if r_cb < self.max_laser_range and abs(r - r_cb) < alpha / 2.0:
            return log(p_occupied / (1 - p_occupied))

        if r <= r_cb:
            return log((1 - p_occupied) / p_occupied)

        return 0.0

    def process_odometry(self, x, y, theta):

        self.baselaser_x_in_map = x
        self.baselaser_y_in_map = y
        self.yaw_map_baselaser = theta

    def process_laser_scan(self, ranges_in_baselaser_frame):

        # TODO: Complete the math to compute where the point is in the lasers's frame (x,y) given the
        #       range sensed at the angle that corresponds to each beam
        #
        # Things to know:
        # 1) self.angles_in_baselaser_frame is a list of angles of the same size as the
        #    scan. They correspond so each entry "i" of the ranges and "i" of the angles
        #    matches. Now you have a r, theta to convert to x, y : polar -> euclidean.
        points_xyz_in_baselaser_frame = [np.array([0, 0, 0])] * len(
            ranges_in_baselaser_frame
        )  # CHANGE THIS
        points_xyz_in_map_frame = self.from_laser_to_map_coordinates(
            points_xyz_in_baselaser_frame
        )

        # END OF YOUR CHANGES

        # From here on is the main loop in occupancy grid mapping.
        # If you setup the points correctly, they will now be used to update the map. Nothing
        # more should be changed in this function.
        baselaser_row, baselaser_col = self.ogm.cartesian_to_grid_coords(
            self.baselaser_x_in_map, self.baselaser_y_in_map
        )

        max_laser_range_in_cells = (
            int(self.max_laser_range / self.ogm.meters_per_cell) + 1
        )
        for delta_row in range(-max_laser_range_in_cells, max_laser_range_in_cells):
            for delta_col in range(-max_laser_range_in_cells, max_laser_range_in_cells):
                row = baselaser_row + delta_row
                col = baselaser_col + delta_col

                if (
                    row < 0
                    or row >= self.ogm.num_rows
                    or col < 0
                    or col >= self.ogm.num_cols
                ):
                    continue

                if self.is_in_field_of_view(
                    baselaser_row, baselaser_col, self.yaw_map_baselaser, row, col
                ):
                    delta_log_odds = (
                        self.inverse_measurement_model(
                            row,
                            col,
                            baselaser_row,
                            baselaser_col,
                            self.yaw_map_baselaser,
                            ranges_in_baselaser_frame,
                            self.angles_in_baselaser_frame,
                        )
                        - self.init_log_odds_ratio
                    )

                    self.ogm.update_log_odds_ratio_in_grid_coords(
                        row, col, delta_log_odds
                    )

        # Call image saving every few iterations
        if self.lidar_count % 25 == 0:
            self.ogm.save_map_as_image(
                "debug_image" + str(self.lidar_count) + ".png")

        if self.lidar_count % 5 == 0:
            print("Completed computing laser scan: ", self.lidar_count)

        self.lidar_count = self.lidar_count + 1

    def write_final_result(self):
        self.ogm.save_map_as_image("final_map.png")


if __name__ == "__main__":

    num_rows = 250
    num_cols = 250
    meters_per_cell = 0.2
    hm = HuskyMapper(num_rows, num_cols, meters_per_cell, 720)

    with open(r"husky_data.yaml") as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        # husky_data = yaml.load(file,Loader=yaml.FullLoader)
        husky_data = yaml.load(file)

    for item in husky_data:
        hm.process_odometry(item[1], item[2], item[3])
        hm.process_laser_scan(item[0])

    hm.write_final_result()

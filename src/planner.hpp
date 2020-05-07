#pragma once

#include <cmath>
#include <vector>

#include "helpers.h"
#include "json.hpp"
#include "spline.h"

enum LaneInfo {
    LeftLane,
    CenterLane,
    RightLane,
};

enum Behavior {
    LaneChangeLeft,
    LaneChangeRight,
    SlowDown,
    GoAhead
};

struct Vehicle {
    double   x;
    double   y;
    double   s;
    double   d;
    double   yaw_deg;
    double   yaw_rad;
    double   speed;
    LaneInfo lane;
};

struct CarExistencePrediction {
    bool exist_left;
    bool exist_right;
    bool exist_ahead;
};

struct Point {
    double x;
    double y;
};

LaneInfo estimateLane(double d, double lane_width) {
    if (d < lane_width) {
        return LeftLane;
    } else if (d < lane_width * 2) {
        return CenterLane;
    } else {
        return RightLane;
    }
}

CarExistencePrediction predictCarExistence(std::vector<Vehicle>& vehicles,
                                           const Vehicle&        ego_vehicle,
                                           double                max_dist) {
    CarExistencePrediction prediction = {false, false, false};
    for (auto&& vehicle : vehicles) {
        // Estimate car s position after executing previous trajectory.
        //check_car_s += ((double)prev_size * 0.02 * check_speed);
        if (vehicle.lane == ego_vehicle.lane) {
            // Same lane
            prediction.exist_ahead |= vehicle.s > ego_vehicle.s && vehicle.s < ego_vehicle.s + max_dist;
        } else if (vehicle.lane + 1 == ego_vehicle.lane) {
            // Left lane
            prediction.exist_left |= vehicle.s > ego_vehicle.s - max_dist && vehicle.s < ego_vehicle.s + max_dist;
        } else if (vehicle.lane == ego_vehicle.lane + 1) {
            // Car right
            prediction.exist_right |= vehicle.s > ego_vehicle.s - max_dist && vehicle.s < ego_vehicle.s + max_dist;
        }
    }
    return prediction;
}

Behavior decideBehavior(const CarExistencePrediction& prediction,
                        const Vehicle&                ego_vehicle) {
    Behavior behavior = GoAhead;
    if (prediction.exist_ahead) {
        if (!prediction.exist_left && ego_vehicle.lane != LeftLane) {
            // if there is no car left and there is a left lane
            behavior = LaneChangeLeft;
        } else if (!prediction.exist_right && ego_vehicle.lane != RightLane) {
            // if there is no car right and there is a right lane
            behavior = LaneChangeRight;
        } else {
            behavior = SlowDown;
        }
    } else {
        if (ego_vehicle.lane != CenterLane) {
            // if ego-vehicle is not on the center lane, back to the center
            if (!prediction.exist_left && ego_vehicle.lane == RightLane) {
                behavior = LaneChangeLeft;
            }
            if (!prediction.exist_right && ego_vehicle.lane == LeftLane) {
                behavior = LaneChangeRight;
            }
        }
    }
    return behavior;
}

tk::spline createSpline(Vehicle&              ego_vehicle,
                        const nlohmann::json& previous_path_x,
                        const nlohmann::json& previous_path_y,
                        const nlohmann::json& map_waypoints_s,
                        const nlohmann::json& map_waypoints_x,
                        const nlohmann::json& map_waypoints_y,
                        double                lane_width,
                        double                lane_center,
                        double                next_distance) {
    vector<double> points_x;
    vector<double> points_y;
    std::size_t    prev_size = previous_path_x.size();

    // Set up previous points
    if (prev_size > 1) {
        points_x.push_back(previous_path_x[prev_size - 2]);
        points_y.push_back(previous_path_y[prev_size - 2]);
        points_x.push_back(previous_path_x[prev_size - 1]);
        points_y.push_back(previous_path_y[prev_size - 1]);

        ego_vehicle.x       = points_x[1];
        ego_vehicle.y       = points_y[1];
        ego_vehicle.yaw_rad = std::atan2(points_y[1] - points_y[0], points_x[1] - points_x[0]);
    } else if (prev_size > 0) {
        points_x.push_back(previous_path_x[prev_size - 1]);
        points_y.push_back(previous_path_y[prev_size - 1]);
    } else {
        // Alternatively, use current position
        points_x.push_back(ego_vehicle.x);
        points_y.push_back(ego_vehicle.y);
    }

    // Set up distant points
    std::vector<double> next_wp0 = getXY(ego_vehicle.s + next_distance,
                                         lane_width * ego_vehicle.lane + lane_center,
                                         map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);
    std::vector<double> next_wp1 = getXY(ego_vehicle.s + 2.0 * next_distance,
                                         lane_width * ego_vehicle.lane + lane_center,
                                         map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);
    std::vector<double> next_wp2 = getXY(ego_vehicle.s + 3.0 * next_distance,
                                         lane_width * ego_vehicle.lane + lane_center,
                                         map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);
    points_x.push_back(next_wp0[0]);
    points_x.push_back(next_wp1[0]);
    points_x.push_back(next_wp2[0]);
    points_y.push_back(next_wp0[1]);
    points_y.push_back(next_wp1[1]);
    points_y.push_back(next_wp2[1]);

    // Convert coordinates to local car coordinates
    for (std::size_t i = 0; i < points_x.size(); i++) {
        double shift_x = points_x[i] - ego_vehicle.x;
        double shift_y = points_y[i] - ego_vehicle.y;
        points_x[i]    = shift_x * cos(-ego_vehicle.yaw_rad) - shift_y * sin(-ego_vehicle.yaw_rad);
        points_y[i]    = shift_x * sin(-ego_vehicle.yaw_rad) + shift_y * cos(-ego_vehicle.yaw_rad);
    }

    // Create the spline.
    tk::spline spline;
    spline.set_points(points_x, points_y);
    return std::move(spline);
}
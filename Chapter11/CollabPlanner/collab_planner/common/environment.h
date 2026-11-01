//
//

#pragma once

#include <array>
#include <iostream>
#include <map>
#include <mutex>

#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/vehicle_param.h"

using common::math::Vec2d;

typedef std::array<Vec2d, 8> CharacterCars;
typedef std::map<std::string, CharacterCars> CharacterCarsMap;
typedef std::array<common::math::Pose, 8> Poses;

class Environment {
   public:
    static Environment& Instance() {
        static Environment inst;
        return inst;
    }

    void UpdateVehicle(int index, const common::math::Pose& new_pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        vehicles_pos[index] = new_pose;
    }

    common::math::Pose GetVehicle(int index) {
        std::lock_guard<std::mutex> lock(mutex_);
        return vehicles_pos[index];
    }

   private:
    Environment() {    // std::cout << "Environment construct\n";
    }

    ~Environment() {    // std::cout << "Environment destroy\n";
    }

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    Poses vehicles_pos;
    mutable std::mutex mutex_;

   public:
    double x_min = 0, y_min = 0;
    double x_max = 3.40, y_max = 2.35;

    const int vehicle_num = 8;

    common::math::Pose start, goal;
    VehicleParam vehicle_param;

    std::array<double, 4> xy_bound() const { return {x_min, x_max, y_min, y_max}; }

    // bool CheckPoseCollision(const common::math::Pose& pose, double radius_buffer) {
    //     common::math::AABox2d initial_box({-vehicle.radius - radius_buffer, -vehicle.radius - radius_buffer},
    //                                         {vehicle.radius + radius_buffer, vehicle.radius + radius_buffer});

    //     double xr, yr, xf, yf;

    //     auto f_box = initial_box, r_box = initial_box;
    //     f_box.Shift({xf, yf});
    //     r_box.Shift({xr, yr});

    //     if (CheckCollision(common::math::Box2d(f_box)) || CheckCollision(common::math::Box2d(r_box))) {
    //         return true;
    //     }
    //     return false;
    // }

    // bool CheckCollision(const common::math::Box2d& box) {
    //     if (points.CheckInBox(box)) {
    //         return true;
    //     }

    //     return std::any_of(obstacles.begin(), obstacles.end(), [&](common::Obstacle& ob) {
    //         if (ob.is_moving()) {
    //         return false;
    //         }
    //         return ob.GetPolygon().HasOverlap(box);
    //     });
    // }

    // bool CheckObstacleCollision() {
    //     // Iterate over all obstacles
    //     for (auto it1 = obstacles.begin(); it1 != obstacles.end(); ++it1) {
    //         // Get the current obstacle polygon
    //         const auto& polygon1 = it1->GetPolygon();

    //         // Compare the current obstacle with the remaining obstacles
    //         for (auto it2 = it1 + 1; it2 != obstacles.end(); ++it2) {
    //             // Skip comparison with itself
    //             if (it1 != it2) {
    //                 // Get the other obstacle polygon
    //                 const auto& polygon2 = it2->GetPolygon();

    //                 // Test whether the two polygons overlap
    //                 if (polygon1.HasOverlap(polygon2)) {
    //                     return true;  // Collision found
    //                 }
    //             }
    //         }
    //     }

    //     return false;  // No collision found
    // }

    // bool CheckVehicleCollision(const std::vector<Vehicle>& vehicles) {
    //     // Iterate over all vehicles
    //     for (auto it1 = vehicles.begin(); it1 != vehicles.end(); ++it1) {
    //         // Get the current vehicle polygon
    //         const auto& polygon1 = it1->GetPolygon();

    //         // Compare the current vehicle with the other vehicles
    //         for (auto it2 = it1 + 1; it2 != vehicles.end(); ++it2) {
    //             // Get the other vehicle polygon
    //             const auto& polygon2 = it2->GetPolygon();

    //             // Test whether the two polygons overlap
    //             if (polygon1.HasOverlap(polygon2)) {
    //                 return true;  // Collision found
    //             }
    //         }
    //     }

    //     return false;  // No collision found
    // }
};

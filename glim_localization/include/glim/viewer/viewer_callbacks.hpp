#pragma once

#include <glim/util/callback_slot.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim
{
    struct ViewerCallbacks {
    /**
     * @brief Image input callback
     * @param stamp  Timestamp
     * @param image  Image
     */
        static CallbackSlot<void(int)> user_event;
        static CallbackSlot<void()> on_load_map;
        static CallbackSlot<void(const Eigen::Vector3d& pos)> request_relocalize;
    };
} // namespace glim

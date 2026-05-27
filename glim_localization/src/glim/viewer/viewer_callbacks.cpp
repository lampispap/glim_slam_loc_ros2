#include <glim/viewer/viewer_callbacks.hpp>

namespace glim {

    CallbackSlot<void(int)> ViewerCallbacks::user_event;
    CallbackSlot<void()> ViewerCallbacks::on_load_map;
    CallbackSlot<void(const Eigen::Vector3d& pos)> ViewerCallbacks::request_relocalize;

}
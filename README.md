## Installation

```bash
mkdir gtsam_points/build && cd gtsam_points/build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
````

For ROS2 
```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```
#!/bin/bash
echo "Ubuntu:" $(grep PRETTY_NAME /etc/os-release | cut -d= -f2)

echo -n "PCL: "; (pkg-config --modversion pcl_common 2>/dev/null \
    || pcl_version 2>/dev/null || apt list --installed 2>/dev/null | grep libpcl-dev | awk '{print $2}' \
    || echo "Not found")
echo -n "Eigen: "; (pkg-config --modversion eigen3 2>/dev/null \
    || grep "define EIGEN_WORLD_VERSION" /usr/include/eigen3/Eigen/src/Core/util/Macros.h -A 2 | tail -n2)
echo -n "OpenCV: "; (pkg-config --modversion opencv4 2>/dev/null \
    || python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null \
    || echo "Not found")
echo -n "GTSAM: "; (pkg-config --modversion gtsam 2>/dev/null \
    || dpkg -s ros-humble-gtsam 2>/dev/null | grep Version | awk '{print $2}' \
    || echo "Not found")


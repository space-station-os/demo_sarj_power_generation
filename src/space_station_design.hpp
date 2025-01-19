
#include <Eigen/Dense>


namespace SpaceStationDesign
{
    // Rotation axis of SARJ at SSBF frame
    const Eigen::Vector3d SARJ_ROTATION_AXIS = Eigen::Vector3d(0.0, 1.0, 0.0).normalized();
    const Eigen::Vector3d SAP_BASE_NORMAL_VEC = Eigen::Vector3d(0.0, 0.0, -1.0).normalized();

} // namespace SpaceStationDesig

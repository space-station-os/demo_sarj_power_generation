
#include <Eigen/Dense>


class FrameTransformer
{
private:
    Eigen::Matrix3d local_frame_rot_mat;
    Eigen::Matrix3d inv_local_frame_rot_mat;
    Eigen::Vector3d local_frame_ori_vec;

public:

    FrameTransformer() {
        this->local_frame_rot_mat = Eigen::Matrix3d::Identity();
        this->inv_local_frame_rot_mat = Eigen::Matrix3d::Identity();
        this->local_frame_ori_vec = Eigen::Vector3d::Zero();
    }

    FrameTransformer(const Eigen::Matrix3d& local_frame_rot_mat, const Eigen::Vector3d& local_frame_ori_vec) {
        this->local_frame_rot_mat = local_frame_rot_mat;
        this->local_frame_ori_vec = local_frame_ori_vec;
        this->inv_local_frame_rot_mat = local_frame_rot_mat.inverse();
    }

    Eigen::Vector3d get_local_pos(const Eigen::Vector3d& global_pos_vec) const {
        return this->inv_local_frame_rot_mat * (global_pos_vec - this->local_frame_ori_vec);
    }

    Eigen::Vector3d get_global_pos(const Eigen::Vector3d& local_pos_vec) const {
        return this->local_frame_rot_mat * local_pos_vec + this->local_frame_ori_vec;
    }

    void update_rot_mat(const Eigen::Matrix3d& local_frame_rot_mat) {
        this->local_frame_rot_mat = local_frame_rot_mat;
        this->inv_local_frame_rot_mat = local_frame_rot_mat.inverse();
    }

    void update_ori_vec(const Eigen::Vector3d& local_frame_ori_vec) {
        this->local_frame_ori_vec = local_frame_ori_vec;
    }

    const Eigen::Matrix3d& get_local_frame_rot_mat() const {
        return this->local_frame_rot_mat;
    }

    const Eigen::Matrix3d& get_inv_local_frame_rot_mat() const {
        return this->inv_local_frame_rot_mat;
    }

    const Eigen::Vector3d& get_local_frame_ori_vec() const {
        return this->local_frame_ori_vec;
    }
};


#include <Eigen/Dense>


class FrameTransformer
{
private:
    Eigen::Matrix3d local_frame_basis_mat;
    Eigen::Matrix3d inv_local_frame_basis_mat;
    Eigen::Vector3d local_frame_origin_vec;

public:

    FrameTransformer() {
        this->local_frame_basis_mat = Eigen::Matrix3d::Identity();
        this->inv_local_frame_basis_mat = Eigen::Matrix3d::Identity();
        this->local_frame_origin_vec = Eigen::Vector3d::Zero();
    }

    FrameTransformer(const Eigen::Matrix3d& local_frame_basis_mat, const Eigen::Vector3d& local_frame_origin_vec) {
        this->local_frame_basis_mat = local_frame_basis_mat;
        this->local_frame_origin_vec = local_frame_origin_vec;
        this->inv_local_frame_basis_mat = local_frame_basis_mat.inverse();
    }

    Eigen::Vector3d get_local_pos(const Eigen::Vector3d& global_pos_vec) const {
        return this->inv_local_frame_basis_mat * (global_pos_vec - this->local_frame_origin_vec);
    }

    Eigen::Vector3d get_global_pos(const Eigen::Vector3d& local_pos_vec) const {
        return this->local_frame_basis_mat * local_pos_vec + this->local_frame_origin_vec;
    }

    void update_basis_mat(const Eigen::Matrix3d& local_frame_basis_mat) {
        this->local_frame_basis_mat = local_frame_basis_mat;
        this->inv_local_frame_basis_mat = local_frame_basis_mat.inverse();
    }

    void update_origin_vec(const Eigen::Vector3d& local_frame_origin_vec) {
        this->local_frame_origin_vec = local_frame_origin_vec;
    }

    const Eigen::Matrix3d& get_local_frame_basis_mat() const {
        return this->local_frame_basis_mat;
    }

    const Eigen::Matrix3d& get_inv_local_frame_basis_mat() const {
        return this->inv_local_frame_basis_mat;
    }

    const Eigen::Vector3d& get_local_frame_origin_vec() const {
        return this->local_frame_origin_vec;
    }
};

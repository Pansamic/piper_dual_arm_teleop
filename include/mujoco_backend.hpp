/**
 * @file mujoco_backend.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief MuJoCo physics engine backend.
 * @version 0.1
 * @date 2025-08-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <mutex>
#include <array>
#include <vector>
#include <thread>
#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <modelxml.h>

template<typename T, std::size_t NumDof>
class MujocoBackend
{
public:
    explicit MujocoBackend()
    {
        mjSpec* s = mj_parseXMLString(piper_dual_arm_position_full_xml, nullptr, nullptr, 0);
        if ( s == nullptr )
        {
            throw std::runtime_error("MuJoCo failed to parse XML");
        }
        this->mujoco_model_ = mj_compile(s, nullptr);
        if ( this->mujoco_model_ == nullptr )
        {
            throw std::runtime_error("MuJoCo failed to compile model from spec");
        }
        this->mujoco_data_ = mj_makeData(this->mujoco_model_);
        if ( this->mujoco_data_ == nullptr )
        {
            throw std::runtime_error("MuJoCo failed to make data");
        }
    }
    ~MujocoBackend() = default;
    void start()
    {
        if (this->mujoco_engine_thread_.joinable())
        {
            return;
        }

        this->running_ = true;
        this->mujoco_engine_thread_ = std::thread([this]() {
            while (this->running_)
            {
                this->update();
                // Add a small delay to prevent excessive CPU usage
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }
    void stop()
    {
        this->running_ = false;
        if (this->mujoco_engine_thread_.joinable())
        {
            this->mujoco_engine_thread_.join();
        }
        if ( this->mujoco_data_ != nullptr )
            mj_deleteData(this->mujoco_data_);
        if ( this->mujoco_model_ != nullptr )
            mj_deleteModel(this->mujoco_model_);
    }
    mjModel* getMujocoModel()
    {
        return this->mujoco_model_;
    }
    mjData* getMujocoData()
    {
        return this->mujoco_data_;
    }
    
    void setLeftArmJointPosition(Eigen::Vector<T, NumDof> joint_pos)
    {
        std::lock_guard<std::mutex>(this->mutex_);
        for ( int i=0 ; i<NumDof ; ++i )
        {
            this->mujoco_data_->ctrl[i] = joint_pos(i);
        }
    }

    void setLeftArmJointPosition(std::array<T, NumDof> joint_pos)
    {
        std::lock_guard<std::mutex>(this->mutex_);
        for ( int i=0 ; i<NumDof ; ++i )
        {
            this->mujoco_data_->ctrl[i] = joint_pos[i];
        }
    }

    void setLeftArmJointPosition(std::vector<T> joint_pos)
    {
        if ( joint_pos.size() != NumDof )
        {
            return ;
        }
        std::lock_guard<std::mutex>(this->mutex_);
        for ( int i=0 ; i<NumDof ; ++i )
        {
            this->mujoco_data_->ctrl[i] = joint_pos[i];
        }
    }

    void setRightArmJointPosition(Eigen::Vector<T, NumDof> joint_pos)
    {
        std::lock_guard<std::mutex>(this->mutex_);
        for ( int i=0 ; i<NumDof ; ++i )
        {
            this->mujoco_data_->ctrl[NumDof + 2 + i] = joint_pos(i); // 2 for two finger joints.
        }
    }

    void setRightArmJointPosition(std::array<T, NumDof> joint_pos)
    {
        std::lock_guard<std::mutex>(this->mutex_);
        for ( int i=0 ; i<NumDof ; ++i )
        {
            this->mujoco_data_->ctrl[NumDof + 2 + i] = joint_pos[i]; // 2 for two finger joints.
        }
    }

    void setRightArmJointPosition(std::vector<T> joint_pos)
    {
        if ( joint_pos.size() != NumDof )
        {
            return ;
        }
        std::lock_guard<std::mutex>(this->mutex_);
        for ( int i=0 ; i<NumDof ; ++i )
        {
            this->mujoco_data_->ctrl[NumDof + 2 + i] = joint_pos[i]; // 2 for two finger joints.
        }
    }

    Eigen::Vector<T, NumDof> getLeftArmJointPosition() const
    {
        std::lock_guard<std::mutex>(this->mutex_);
        Eigen::Vector<T, NumDof> joint_pos;
        for ( std::size_t i=0 ; i<NumDof ; ++i )
        {
            joint_pos(i) = this->mujoco_data_->qpos[i];
        }
        return joint_pos;
    }

    Eigen::Vector<T, NumDof> getRightArmJointPosition() const
    {
        std::lock_guard<std::mutex>(this->mutex_);
        Eigen::Vector<T, NumDof> joint_pos;
        for ( std::size_t i=0 ; i<NumDof ; ++i )
        {
            joint_pos(i) = this->mujoco_data_->qpos[NumDof + 2 + i];
        }
        return joint_pos;
    }

    Eigen::Vector<T, 3> getLeftHandMocapPosition() const
    {
        std::lock_guard<std::mutex>(this->mutex_);
        Eigen::Vector<T, 3> pos;
        pos(0) = this->mujoco_data_->mocap_pos[0];
        pos(1) = this->mujoco_data_->mocap_pos[1];
        pos(2) = this->mujoco_data_->mocap_pos[2];
        return pos;
    }
    Eigen::Vector<T, 3> getRightHandMocapPosition() const
    {
        std::lock_guard<std::mutex>(this->mutex_);
        Eigen::Vector<T, 3> pos;
        pos(0) = this->mujoco_data_->mocap_pos[3];
        pos(1) = this->mujoco_data_->mocap_pos[4];
        pos(2) = this->mujoco_data_->mocap_pos[5];
        return pos;
    }
    Eigen::Quaternion<T> getLeftHandMocapOrientation() const
    {
        std::lock_guard<std::mutex>(this->mutex_);
        Eigen::Quaternion<T> quat;
        quat.w() = this->mujoco_data_->mocap_quat[0];
        quat.x() = this->mujoco_data_->mocap_quat[1];
        quat.y() = this->mujoco_data_->mocap_quat[2];
        quat.z() = this->mujoco_data_->mocap_quat[3];
        return quat;
    }
    Eigen::Quaternion<T> getRightHandMocapOrientation() const
    {
        std::lock_guard<std::mutex>(this->mutex_);
        Eigen::Quaternion<T> quat;
        quat.w() = this->mujoco_data_->mocap_quat[4];
        quat.x() = this->mujoco_data_->mocap_quat[5];
        quat.y() = this->mujoco_data_->mocap_quat[6];
        quat.z() = this->mujoco_data_->mocap_quat[7];
        return quat;
    }
private:
    mutable std::mutex mutex_;
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    bool running_ = false;
    std::thread mujoco_engine_thread_;
    void update()
    {
        std::lock_guard<std::mutex>(this->mutex_);
        mj_step(this->mujoco_model_, this->mujoco_data_);
    }
};
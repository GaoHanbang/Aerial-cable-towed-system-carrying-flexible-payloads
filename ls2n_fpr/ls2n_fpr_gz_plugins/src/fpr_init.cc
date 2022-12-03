#include <thread>
#include "fpr_init.h"

namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FprAssembly)

    FprAssembly::FprAssembly() : ModelPlugin()
        {}

    void FprAssembly::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            world = _model->GetWorld();
            fpr = _model;
            world->SetPaused(true);
            std::thread check_thread = std::thread(&FprAssembly::check_and_assemble, this);
            check_thread.detach();
        }

    void FprAssembly::check_and_assemble()
        {
            physics::ModelPtr d1 = nullptr;
            physics::ModelPtr d2 = nullptr;
            physics::ModelPtr d3 = nullptr;
            while (d1 == nullptr || d2 == nullptr || d3 == nullptr || fpr == nullptr) {
                d1 = world->ModelByName("crazy2fly_1");
                d2 = world->ModelByName("crazy2fly_2");
                d3 = world->ModelByName("crazy2fly_3");
            }
            ignition::math::Pose3d pose = d1->WorldPose();
            float joint_offset = 0.06682;
            float drone_0_x_pos = 0.8645123727775691 + joint_offset;
            pose.Pos().X(drone_0_x_pos);
            pose.Pos().Y(0.0);
            pose.Pos().Z(0.06);
            pose.Rot() = ignition::math::Quaterniond::EulerToQuaternion(0,0,-2.356194490192345);
            d1->SetWorldPose(pose);
            pose.Pos().X(-0.49999999999*drone_0_x_pos);
            pose.Pos().Y(0.86602540378*drone_0_x_pos);
            pose.Pos().Z(0.06);
            pose.Rot() = pose.Rot() = ignition::math::Quaterniond::EulerToQuaternion(0,0,-0.2617993877991495);
            d2->SetWorldPose(pose);
            pose.Pos().X(-0.49999999999*drone_0_x_pos);
            pose.Pos().Y(-0.86602540378*drone_0_x_pos);
            pose.Pos().Z(0.06);
            pose.Rot() = pose.Rot() = ignition::math::Quaterniond::EulerToQuaternion(0,0,1.832595714594045);
            d3->SetWorldPose(pose);
            pose.Pos().X(0);
            pose.Pos().Y(0);
            pose.Pos().Z(0.06);
            pose.Rot() = ignition::math::Quaterniond ::Identity;
            fpr->SetWorldPose(pose);

            ignition::math::Pose3d pose_joint;
            pose_joint.Pos().X(0.70710678*joint_offset);
            pose_joint.Pos().Y(-0.70710678*joint_offset);

            physics::JointPtr j1;
            physics::LinkPtr l2 = d1->GetLink("base_link");
            physics::LinkPtr l1 = fpr->GetLink("leg1");
            j1 = world->Physics()->CreateJoint("ball", fpr);
            j1->Attach(l1, l2);
            j1->Load(l1, l2, pose_joint);
            j1->SetModel(d1);
            j1->Init();

            physics::JointPtr j2;
            l2 = d2->GetLink("base_link");
            l1 = fpr->GetLink("leg2");
            j2 = world->Physics()->CreateJoint("ball", fpr);
            j2->Attach(l1, l2);
            j2->Load(l1, l2, pose_joint);
            j2->SetModel(d2);
            j2->Init();

            physics::JointPtr j3;
            l2 = d3->GetLink("base_link");
            l1 = fpr->GetLink("leg3");
            j3 = world->Physics()->CreateJoint("ball", fpr);
            j3->Attach(l1, l2);
            j3->Load(l1, l2, pose_joint);
            j3->SetModel(d3);
            j3->Init();

            world->SetPaused(false);
        }
}
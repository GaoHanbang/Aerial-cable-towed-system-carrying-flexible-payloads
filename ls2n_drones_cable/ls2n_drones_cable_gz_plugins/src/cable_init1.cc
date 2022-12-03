#include <thread>
#include "cable_init.h"

namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(CableAssembly)

    CableAssembly::CableAssembly() : ModelPlugin()
        {}

    void CableAssembly::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            world = _model->GetWorld();
            cable = _model;
            if (_sdf->HasElement("cable_length"))
                cableLength = _sdf->Get<float>("cable_length");
            else
                cableLength = 1;
            world->SetPaused(true);
            std::thread check_thread = std::thread(&CableAssembly::check_and_assemble, this);
            check_thread.detach();
        }

    void CableAssembly::check_and_assemble()
        {
            physics::ModelPtr d1 = nullptr;
            physics::ModelPtr d2 = nullptr;
            while (d1 == nullptr || d2 == nullptr) {
                d1 = world->ModelByName("crazy2fly_1");
                d2 = world->ModelByName("crazy2fly_2");
            }
            physics::LinkPtr la = cable->GetLink("attach_cable_1_right");
            physics::LinkPtr lb = cable->GetLink("attach_cable_2_right");
            ignition::math::Pose3d pose = d1->WorldPose();
            pose.Pos().X(-0.505);
            pose.Pos().Y(0.0);
            pose.Pos().Z(0.06);
            pose.Rot() = ignition::math::Quaterniond::Identity;
            d1->SetWorldPose(pose);
            pose.Pos().Z(0.01);
            la->SetWorldPose(pose);
            pose.Pos().X(1.505);
            pose.Pos().Y(0.0);
            pose.Pos().Z(0.06);
            pose.Rot() = pose.Rot() = ignition::math::Quaterniond::Identity;
            d2->SetWorldPose(pose);
            pose.Pos().Z(0.01);
            lb->SetWorldPose(pose);
                        ignition::math::Pose3d pose_joint;

            physics::JointPtr j1;
            physics::LinkPtr l2 = d1->GetLink("base_link");

            j1 = world->Physics()->CreateJoint("ball", d2);
            j1->Attach(l2, la);
            j1->Load(l2, la, pose_joint);
            j1->SetModel(cable);
            j1->Init();

            physics::JointPtr j2;
            l2 = d2->GetLink("base_link");
            j2 = world->Physics()->CreateJoint("ball", d1);
            j2->Attach(l2, lb);
            j2->Load(l2, lb, pose_joint);
            j2->SetModel(cable);
            j2->Init();
            world->SetPaused(false);
        }
}

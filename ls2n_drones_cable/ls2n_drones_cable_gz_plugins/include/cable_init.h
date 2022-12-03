#ifndef CABLE_INIT_HH
#define CABLE_INIT_HH

#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
namespace gazebo {

    class CableAssembly : public ModelPlugin {
    public:
        CableAssembly();
        ~CableAssembly() override = default;

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    private:
        void check_and_assemble();

        physics::PhysicsEnginePtr physics;
        physics::WorldPtr world;
        physics::ModelPtr cable;
        float cableLength;
    };
}
#endif
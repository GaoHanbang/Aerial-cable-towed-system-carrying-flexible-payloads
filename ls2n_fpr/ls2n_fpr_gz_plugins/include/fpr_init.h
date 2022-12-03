#ifndef FPR_INIT_HH
#define FPR_INIT_HH

#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
namespace gazebo {

    class FprAssembly : public ModelPlugin {
    public:
        FprAssembly();
        ~FprAssembly() override = default;

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    private:
        void check_and_assemble();

        physics::PhysicsEnginePtr physics;
        physics::WorldPtr world;
        physics::ModelPtr fpr;
    };
}
#endif
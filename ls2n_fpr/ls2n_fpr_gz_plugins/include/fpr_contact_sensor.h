#ifndef _FPR_CONTACT_SENSOR_HH_
#define _FPR_CONTACT_SENSOR_HH_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo
{
    class FprContactSensorPrivate;

    /// A plugin that publishes the contact force sensor measurements for the Flying Parallel Robot (FPR).
    class FprContactSensor : public SensorPlugin
    {
    public:
        /// Constructor 
        FprContactSensor();

        /// Destructor
        virtual ~FprContactSensor();

        /// Load the sensor plugin
        /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        std::unique_ptr<FprContactSensorPrivate> impl_;
    };
}
#endif
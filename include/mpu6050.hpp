#pragma once

#include <vislib.hpp>
#include <../MPU6050/src/MPU6050.h>

namespace vislib_mpu6050 {
    namespace {
        namespace vs = vislib;
        namespace gy = vs::gyro;
    };
    
    class Gyroscope;

};

class vislib_mpu6050::Gyroscope : public vislib::gyro::UltimateGyroController<double, size_t, double>, public MPU6050 {
public:
    inline vislib::util::Result<vislib::util::Vector<double>> getAcceleration() const noexcept override {
        vs::util::Vector<double> acceleration(vs::util::Array<double>({double(), double(), double()}));

        int16_t ax, ay, az;

        MPU6050::getAcceleration(&ax, &ay, &az);

        acceleration[0] = ax / 16384.;
        acceleration[1] = ay / 16384.;
        acceleration[2] = az / 16384.;

        return acceleration;

    }

    inline vislib::util::Result<vislib::util::Vector<double>> getAngularSpeed() const noexcept override {
        vs::util::Vector<double> speed(vs::util::Array<double>({double(), double(), double()}));

        int16_t gx, gy, gz;

        MPU6050::getRotation(&gx, &gy, &gz);

        speed[0] = gx / 131.;
        speed[1] = gy / 131.;
        speed[2] = gz / 131.;

        return speed;

    }

    inline vislib::util::Result<double> getYaw() const noexcept override {
        return this->yawConfig.integrator.getIntegral();
    }

    inline vislib::util::Result<double> getPitch() const noexcept override {
        return this->pitchConfig.integrator.getIntegral();
    }

    inline vislib::util::Result<double> getRoll() const noexcept override {
        return this->rollConfig.integrator.getIntegral();
    }
};

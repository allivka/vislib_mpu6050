#pragma once

#include <vislib.hpp>
#include <../MPU6050/src/MPU6050.h>

namespace vislib_mpu6050 {
    class Gyroscope;
};

class vislib_mpu6050::Gyroscope : public vislib::gyro::UltimateGyroController<double, size_t>, public MPU6050 {
public:

    virtual inline vislib::util::Result<vislib::util::Vector<double>> getAcceleration() const noexcept override {
        vislib::util::Vector<double> acceleration(vislib::util::Array<double>({double(), double(), double()}));

        int16_t ax, ay, az;

        static_cast<MPU6050>(*this).getAcceleration(&ax, &ay, &az);

        acceleration[0] = ax / 16384.;
        acceleration[1] = ay / 16384.;
        acceleration[2] = az / 16384.;

        return acceleration;

    }

    virtual inline vislib::util::Result<vislib::util::Vector<double>> getAngularSpeed() const noexcept override {
        vislib::util::Vector<double> speed(vislib::util::Array<double>({double(), double(), double()}));

        int16_t gx, gy, gz;

        static_cast<MPU6050>(*this).getRotation(&gx, &gy, &gz);

        speed[0] = gx / 131.;
        speed[1] = gy / 131.;
        speed[2] = gz / 131.;

        return speed;

    }

    virtual ~Gyroscope() = default;
    
};

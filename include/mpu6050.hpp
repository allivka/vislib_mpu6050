#pragma once

#include <vislib.hpp>
#include <MPU6050.h>

namespace vislib_mpu6050 {
    class GyroscopeDMP;
    class GyroscopeCalculator;
};

class vislib_mpu6050::GyroscopeDMP : public vislib::gyro::UltimateGyroGetter<double, size_t>, public MPU6050 {
    
};

class vislib_mpu6050::GyroscopeCalculator : public vislib::gyro::UltimateGyroCalculator<double, size_t>, public MPU6050 {
protected:
    double gyroDivider = 131.0;
    double accelerationDivider = 16384.0;
public:

    virtual inline vislib::util::Result<vislib::util::Vector<double>> getAcceleration() const noexcept final {
        vislib::util::Vector<double> acceleration(vislib::util::Array<double>({double(), double(), double()}));

        int16_t ax, ay, az;

        static_cast<MPU6050>(*this).getAcceleration(&ax, &ay, &az);

        acceleration[0] = ax / accelerationDivider;
        acceleration[1] = ay / accelerationDivider;
        acceleration[2] = az / accelerationDivider;

        return acceleration;

    }

    virtual inline vislib::util::Result<vislib::util::Vector<double>> getAngularSpeed() const noexcept final {
        vislib::util::Vector<double> speed(vislib::util::Array<double>({double(), double(), double()}));

        int16_t gx, gy, gz;

        static_cast<MPU6050>(*this).getRotation(&gx, &gy, &gz);
        
        speed[0] = gx / gyroDivider;
        speed[1] = gy / gyroDivider;
        speed[2] = gz / gyroDivider;        
        
        return speed;

    }
    
    virtual vislib::util::Error calibrate() noexcept override {
        
        
        this->setXGyroOffset(0);
        this->setYGyroOffset(0);
        this->setZGyroOffset(0);
        
        this->setXAccelOffset(0);
        this->setYAccelOffset(0);
        this->setZAccelOffset(0);
        
        this->CalibrateAccel();
        this->CalibrateGyro();
        
        yawConfig.offset = 0;
        yawConfig.integrator.setIntegral(0);
        
        pitchConfig.offset = 0;
        pitchConfig.integrator.setIntegral(0);
        
        rollConfig.offset = 0;
        rollConfig.integrator.setIntegral(0);
        
        return {};
    }
    
    virtual inline vislib::util::Error setGyroDivider(double divider) noexcept {
        if (divider == 0) return {vislib::util::ErrorCode::invalidArgument, "Cannot set gyro divider to zero"};
        gyroDivider = divider;
        return {};
    }
    
    virtual inline vislib::util::Error setAccelerationDivider(double divider) noexcept {
        if (divider == 0) return {vislib::util::ErrorCode::invalidArgument, "Cannot set acceleration divider to zero"};
        accelerationDivider = divider;
        return {};
    }
    
};

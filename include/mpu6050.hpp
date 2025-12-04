#pragma once

#include <vislib.hpp>
#include <MPU6050_6Axis_MotionApps20.h>

namespace vislib_mpu6050 {
    class GyroscopeBase;
    template <size_t InterruptRegistrySize = 1> class GyroscopeDMP;
    class GyroscopeCalculator;
};

class vislib_mpu6050::GyroscopeBase : public vislib::gyro::UltimateGyroGetter<double, size_t>, public MPU6050 {

public:

    virtual ~GyroscopeBase() = default;

    virtual inline vislib::util::Error init() noexcept {
        initialize();
        if (!testConnection()) return {vislib::util::ErrorCode::initFailed, "Failed mpu6050 connection test"};
        calibrate();

        return {};
    }

    virtual vislib::util::Error calibrate() noexcept override {

        this->setXGyroOffset(0);
        this->setYGyroOffset(0);
        this->setZGyroOffset(0);

        this->setXAccelOffset(0);
        this->setYAccelOffset(0);
        this->setZAccelOffset(0);
        
        this->CalibrateAccel(6);
        this->CalibrateGyro(6);
        
        return {};
    }
};

class vislib_mpu6050::GyroscopeCalculator : public vislib_mpu6050::GyroscopeBase, public vislib::gyro::UltimateGyroCalculator<double, size_t> {
protected:
    double gyroDivider = 131.0;
    double accelerationDivider = 16384.0;
public:

    virtual vislib::util::Error calibrate() noexcept override {

        static_cast<vislib_mpu6050::GyroscopeBase*>(this)->calibrate();

        yawConfig.offset = 0;
        yawConfig.integrator.setIntegral(0);

        pitchConfig.offset = 0;
        pitchConfig.integrator.setIntegral(0);

        rollConfig.offset = 0;
        rollConfig.integrator.setIntegral(0);

        return {};
    }

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

    virtual ~GyroscopeCalculator() = default;

};

 class vislib_mpu6050::GyroscopeDMP : public vislib_mpu6050::GyroscopeBase {
protected:

    volatile bool dmpReady = false;

public:

     virtual vislib::util::Error initDMP(uint16_t interruptPin) noexcept {
        uint8_t status = dmpInitialize();

        if (status != 0) return {vislib::util::ErrorCode::initFailed,
            "DMP initialization failed, status code = " + vislib::util::to_string(static_cast<size_t>(status))
        };

        pinMode(interruptPin, INPUT);
        calibrate();
        setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(interruptPin), [this] {this->dmpReady = true;}, RISING);
        dmpReady = true;


    }

};

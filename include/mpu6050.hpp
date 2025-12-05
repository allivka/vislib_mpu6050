#pragma once

#include <vislib_arduino.hpp>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "../../vislib_arduino/include/vislib_arduino.hpp"

namespace vislib::binds::mpu6050 {
    class GyroscopeBase;
    class GyroscopeDMP;
    class GyroscopeCalculator;
};

class vislib::binds::mpu6050::GyroscopeBase : public gyro::UltimateGyroGetter<double, size_t, double, nullptr_t>, public MPU6050 {
protected:
    double gyroDivider = 131.0;
    double accelerationDivider = 16384.0;
public:

    virtual ~GyroscopeBase() = default;

    virtual util::Error init() noexcept {
        initialize();
        if (!testConnection()) return {util::ErrorCode::initFailed, "Failed mpu6050 connection test"};
        calibrate();

        return {};
    }

    util::Error calibrate() noexcept override {

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

    util::Result<util::Vector<double>> getAcceleration() const noexcept override {
        util::Vector<double> acceleration(vislib::util::Array<double>({double(), double(), double()}));

        int16_t ax, ay, az;

        static_cast<MPU6050>(*this).getAcceleration(&ax, &ay, &az);

        acceleration[0] = ax / accelerationDivider;
        acceleration[1] = ay / accelerationDivider;
        acceleration[2] = az / accelerationDivider;

        return acceleration;

    }

    util::Result<util::Vector<double>> getAngularSpeed() const noexcept override {
        util::Vector<double> speed(vislib::util::Array<double>({double(), double(), double()}));

        int16_t gx, gy, gz;

        static_cast<MPU6050>(*this).getRotation(&gx, &gy, &gz);

        speed[0] = gx / gyroDivider;
        speed[1] = gy / gyroDivider;
        speed[2] = gz / gyroDivider;

        return speed;

    }

    virtual util::Error setGyroDivider(double divider) noexcept {
        if (divider == 0) return {util::ErrorCode::invalidArgument, "Cannot set gyro divider to zero"};
        gyroDivider = divider;
        return {};
    }

    virtual util::Error setAccelerationDivider(double divider) noexcept {
        if (divider == 0) return {util::ErrorCode::invalidArgument, "Cannot set acceleration divider to zero"};
        accelerationDivider = divider;
        return {};
    }
};

class vislib::binds::mpu6050::GyroscopeCalculator : public GyroscopeBase, public gyro::UltimateGyroCalculator<double, size_t> {
public:

    util::Error calibrate() noexcept override {

        static_cast<GyroscopeBase*>(this)->calibrate();

        yawConfig.offset = 0;
        yawConfig.integrator.setIntegral(0);

        pitchConfig.offset = 0;
        pitchConfig.integrator.setIntegral(0);

        rollConfig.offset = 0;
        rollConfig.integrator.setIntegral(0);

        return {};
    }

    virtual ~GyroscopeCalculator() = default;

};

class vislib::binds::mpu6050::GyroscopeDMP : public GyroscopeBase {
protected:

    volatile bool dmpReady = false;

    static arduino::CallbackTable table;

    static void interruptRouter() noexcept {
        if (!table.isInitialized()) return;
        table.manualProcess();
    }

    static uint8_t fifoBuffer[64];

    VectorInt16 aa;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

public:

    static util::Error initInterruptTable(const util::Array<arduino::port_t>& ports) noexcept {

        return table.InitCallbackTable(ports,
            arduino::InterruptPortInitializer,
            [] -> util::Error {return {};},
            arduino::InterruptPortChecker);
    }

    virtual util::Error initDMP(uint16_t interruptPin) noexcept {

        if (!table.isInitialized())
            return {util::ErrorCode::invalidConfiguration, "Interrupt table for gyroscope DMP driver wasn't initialized"};

        uint8_t status = dmpInitialize();

        if (status != 0) return {util::ErrorCode::initFailed,
            "DMP initialization failed, status code = " + util::to_string(static_cast<size_t>(status))
        };

        pinMode(interruptPin, INPUT);
        calibrate();
        setDMPEnabled(true);

        auto e = table.setCallback(interruptPin, [this] {this->dmpReady = true;});

        if (e) return e;

        attachInterrupt(digitalPinToInterrupt(interruptPin), interruptRouter, RISING);
        dmpGetFIFOPacketSize();
        dmpReady = true;

        return {};

    }

    util::Error update(nullptr_t) override {
        if (!dmpReady) return {};

        if (!dmpGetCurrentFIFOPacket(fifoBuffer)) return {};

        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(ypr, &q, &gravity);

        ypr[0] = ypr[0] * 180/M_PI;
        ypr[1] = ypr[1] * 180/M_PI;
        ypr[2] = ypr[2] * 180/M_PI;

        /* ----------- Linear Accel (body frame) ----------- */
        dmpGetAccel(&aa, fifoBuffer);
        dmpGetLinearAccel(&aaReal, &aa, &gravity);

        /* ----------- Linear Accel (world frame) ----------- */
        dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        return {};
    }

    util::Result<double> getYaw() const noexcept override {
        return ypr[0];
    }

    util::Result<double> getPitch() const noexcept override {
        return ypr[1];
    }

    util::Result<double> getRoll() const noexcept override {
        return ypr[2];
    }

    util::Result<util::Vector<double>> getAcceleration() const noexcept override {
        return util::Vector<double>(util::Array<double>({
            static_cast<double>(aaWorld.x),
            static_cast<double>(aaWorld.y),
            static_cast<double>(aaWorld.z)}));
    }

    util::Result<util::Vector<double>> getRealAcceleration() const noexcept {
        return util::Vector<double>(util::Array<double>({
            static_cast<double>(aaReal.x),
            static_cast<double>(aaReal.y),
            static_cast<double>(aaReal.z)}));
    }
};

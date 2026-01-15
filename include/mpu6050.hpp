#pragma once

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <vislib_arduino.hpp>

namespace vislib::binds::mpu6050 {

constexpr size_t DMPFIFOBufferSize = 64;
constexpr double defaultGyroDivider = 131.0;
constexpr double defaultAccelerationDivider = 16384.0;



class GyroscopeBase : public gyro::UltimateGyroGetter<double, size_t, double, nullptr_t>, public MPU6050 {
protected:
    double gyroDivider = defaultGyroDivider;
    double accelerationDivider = defaultAccelerationDivider;
public:

    virtual ~GyroscopeBase() = default;

    virtual core::Error init() noexcept {
        initialize();
        if (!testConnection()) return {core::ErrorCode::initFailed, "Failed mpu6050 connection test"};
        auto e = calibrate();
        if (e) return e;

        return {};
    }

    core::Error calibrate() noexcept override {

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

    core::Result<core::Vector<double>> getAcceleration() const noexcept override {
        core::Vector<double> acceleration(vislib::core::Array<double>({double(), double(), double()}));

        int16_t ax, ay, az;

        static_cast<MPU6050>(*this).getAcceleration(&ax, &ay, &az);

        acceleration[0] = ax / accelerationDivider;
        acceleration[1] = ay / accelerationDivider;
        acceleration[2] = az / accelerationDivider;

        return acceleration;

    }

    core::Result<core::Vector<double>> getAngularSpeed() const noexcept override {
        core::Vector<double> speed(vislib::core::Array<double>({double(), double(), double()}));

        int16_t gx, gy, gz;

        static_cast<MPU6050>(*this).getRotation(&gx, &gy, &gz);

        speed[0] = gx / gyroDivider;
        speed[1] = gy / gyroDivider;
        speed[2] = gz / gyroDivider;

        return speed;

    }

    virtual core::Error setGyroDivider(double divider) noexcept {
        if (divider == 0) return {core::ErrorCode::invalidArgument, "Cannot set gyro divider to zero"};
        gyroDivider = divider;
        return {};
    }

    virtual core::Error setAccelerationDivider(double divider) noexcept {
        if (divider == 0) return {core::ErrorCode::invalidArgument, "Cannot set acceleration divider to zero"};
        accelerationDivider = divider;
        return {};
    }
};

class GyroscopeCalculator : public GyroscopeBase, public gyro::UltimateGyroCalculator<double, size_t> {
public:

    core::Error calibrate() noexcept override {

        static_cast<GyroscopeBase*>(this)->calibrate();

        yawConfig.offset = 0;
        yawConfig.integrator.setIntegral(0);

        pitchConfig.offset = 0;
        pitchConfig.integrator.setIntegral(0);

        rollConfig.offset = 0;
        rollConfig.integrator.setIntegral(0);

        return {};
    }

    ~GyroscopeCalculator() override = default;

};

template <int64_t ID> class DMPInterruptHandler {
public:
    static volatile uint8_t queueLength;
    static void handle() {
        if(queueLength < 255) queueLength++;
        
    }

};

template <int64_t ID> volatile uint8_t DMPInterruptHandler<ID>::queueLength = 0;

template <int64_t GyroID> class GyroscopeDMP : public GyroscopeBase {
protected:
    
    arduino::port_t interruptPin{};
    
    uint16_t packetSize = 0;
    
    uint8_t fifoBuffer[DMPFIFOBufferSize];
    
    VectorInt16 aa;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    
    Quaternion q;
    VectorFloat gravity;
    float ypr[3] = {0, 0, 0};

public:

    core::Error initDMP(arduino::port_t interruptPin) noexcept {
        static_assert(GyroID >= 0, "GyroID must be unique and non-negative");
        uint8_t status = dmpInitialize();

        if (status != 0) return {core::ErrorCode::initFailed,
            "DMP initialization failed, status code = " + core::to_string(static_cast<size_t>(status))
        };

        calibrate();

        static_cast<MPU6050*>(this)->setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(interruptPin), DMPInterruptHandler<GyroID>::handle, RISING);
        packetSize = dmpGetFIFOPacketSize();
        
        this->interruptPin = interruptPin;

        return {};

    }

    virtual ~GyroscopeDMP() {
        detachInterrupt(digitalPinToInterrupt(interruptPin));
        DMPInterruptHandler<GyroID>::queueLength = 0;
    }


    core::Error update(nullptr_t) override {
        static_assert(GyroID >= 0, "GyroID must be unique and non-negative");
        uint8_t intStatus = getIntStatus();

        if (DMPInterruptHandler<GyroID>::queueLength == 0) return {};
        
        DMPInterruptHandler<GyroID>::queueLength--;

        uint16_t size = getFIFOCount();

        if (size < packetSize)
            return {};

        if (size >= 1024) {
            resetFIFO();
            return {};
        }
        
        getFIFOBytes(fifoBuffer, packetSize);

        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(ypr, &q, &gravity);

        ypr[0] = core::rad2Deg(ypr[0]);
        ypr[1] = core::rad2Deg(ypr[1]);
        ypr[2] = core::rad2Deg(ypr[2]);

        dmpGetAccel(&aa, fifoBuffer);
        dmpGetLinearAccel(&aaReal, &aa, &gravity);

        dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        
        return {};
    }

    core::Result<double> getYaw() const noexcept override {
        return ypr[0];
    }

    core::Result<double> getPitch() const noexcept override {
        return ypr[1];
    }

    core::Result<double> getRoll() const noexcept override {
        return ypr[2];
    }

    core::Result<core::Vector<double>> getAcceleration() const noexcept override {
        return core::Vector<double>(core::Array<double>({
            static_cast<double>(aaWorld.x),
            static_cast<double>(aaWorld.y),
            static_cast<double>(aaWorld.z)}));
    }

    core::Result<core::Vector<double>> getRealAcceleration() const noexcept {
        return core::Vector<double>(core::Array<double>({
            static_cast<double>(aaReal.x),
            static_cast<double>(aaReal.y),
            static_cast<double>(aaReal.z)}));
    }
};


}

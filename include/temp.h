#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

static const int INTERRUPT_PIN = 2;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

/* --- Buffers / state --- */
uint8_t fifoBuffer[64];
uint16_t packetSize = 0;
bool dmpReady = false;

/* --- DMP outputs --- */
Quaternion q;
VectorFloat gravity;
float ypr[3];

VectorInt16 aa;       // raw accel
VectorInt16 aaReal;   // linear accel (gravity removed)
VectorInt16 aaWorld;  // linear accel in world frame

void setup() {
    Serial.begin(115200);
    while (!Serial);

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (true);
    }

    Serial.println("Initializing DMP...");
    uint8_t status = mpu.dmpInitialize();

    // OPTIONAL: offsets — in practice you apply your own calibrated values
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (status != 0) {
        Serial.print("DMP init failed: ");
        Serial.println(status);
        while (true);
    }

    // Optional auto-calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;

    Serial.println("DMP ready");
}

void loop() {
    if (!dmpReady) return;

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        return; // no packet yet
    }

    /* ----------- Orientation (YPR) ----------- */
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print("ypr(deg)\t");
    Serial.print(ypr[0] * 180/M_PI); Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI); Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);

    /* ----------- Linear Accel (body frame) ----------- */
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    Serial.print("a_real\t");
    Serial.print(aaReal.x); Serial.print("\t");
    Serial.print(aaReal.y); Serial.print("\t");
    Serial.println(aaReal.z);

    /* ----------- Linear Accel (world frame) ----------- */
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    Serial.print("a_world\t");
    Serial.print(aaWorld.x); Serial.print("\t");
    Serial.print(aaWorld.y); Serial.print("\t");
    Serial.println(aaWorld.z);
}

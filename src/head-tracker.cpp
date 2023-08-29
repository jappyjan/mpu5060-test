#include "head-tracker.hpp"

#define HEAD_TRACKER_FEEDBACK_PIN 4
#define HEAD_TRACKER_MAX_ANGLE_OFFSET 20

void HeadTracker::giveFeedback() {
    digitalWrite(HEAD_TRACKER_FEEDBACK_PIN, HIGH);
    delay(100);
    digitalWrite(HEAD_TRACKER_FEEDBACK_PIN, LOW);
}

void HeadTracker::begin() {
    pinMode(HEAD_TRACKER_FEEDBACK_PIN, OUTPUT);
    digitalWrite(HEAD_TRACKER_FEEDBACK_PIN, LOW);

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    Serial.begin(115200);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(81);
    mpu.setYAccelOffset(1495);
    mpu.setZAccelOffset(1297);
    mpu.setXGyroOffset (52);
    mpu.setYGyroOffset (60);
    mpu.setZGyroOffset (72);

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    this->setOrigin();
}

void HeadTracker::setOrigin() {
    this->updateOrientation();
    originDegree[0] = this->currentDegree[0];
    originDegree[1] = this->currentDegree[1];
    originDegree[2] = this->currentDegree[2];

    // give three feedbacks to indicate that the origin is set
    for (int i = 0; i < 3; i++) {
        giveFeedback();
        delay(400);
    }
}

void HeadTracker::updateOrientation() {
    // read a packet from FIFO
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        return;
    }

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    float ypr[3];
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    currentDegree[0] = ypr[0] * 180/M_PI;
    currentDegree[1] = ypr[1] * 180/M_PI;
    currentDegree[2] = ypr[2] * 180/M_PI;
}

void HeadTracker::tick() {
    this->updateOrientation();

    bool didMoveTooMuch = false;
    for (int i = 0; i < 3; i++) {
        double offset = abs(currentDegree[i] - originDegree[i]);
        if (offset > HEAD_TRACKER_MAX_ANGLE_OFFSET) {
            didMoveTooMuch = true;
            break;
        }
    }

    if (didMoveTooMuch) {
        giveFeedback();
    }
}
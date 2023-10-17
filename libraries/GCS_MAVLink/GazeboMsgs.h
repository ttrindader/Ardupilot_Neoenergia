#pragma once

struct GazeboData {
    float imuLinearAccelerationXYZ[3];
    float imuAngularVelocityRPY[3];
    float magneticFieldXYZ[3];
    float latitude;
    float longitude;
    float altitude;
    float velocityENU[3];    
};

class GazeboMsgs {
public:
    static GazeboData data;

    GazeboData getSensorValues();
};

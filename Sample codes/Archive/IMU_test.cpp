#include <IMUutilities.h>

bias bias_pitch;

// float bias_pitch = 0;
// float bias_pitch_rate = 0;

void setup() {
    Serial.begin(11250);
    while (!Serial) delay(10);
    initializeIMU();
    bias_pitch = calculate_bias();
}

void loop() {
    float pitch = readIMU()-bias_pitch.angle;
    float pitch_rate = readgyro()-bias_pitch.velocity;
    Serial.println("pitch_angle = "+String(pitch) + " pitch_velocity = "+String(pitch_rate)+" Angle_Bias = "+String(bias_pitch.angle)+" Velocity_bias = "+String(bias_pitch.velocity));
    delay(100);
}
#include <IMUutilities.h>

float bias_pitch = 0;
float bias_pitch_rate = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    bias_pitch, bias_pitch_rate = initializeIMU();
}

void loop() {
    float pitch = readIMU()-bias_pitch;
    float pitch_rate = readgyro()-bias_pitch_rate;
    Serial.println("pitch_angle = "+String(pitch) + " pitch_velocity = "+String(pitch_rate)+" Angle_Bias = "+String(bias_pitch)+" Velocity_bias = "+String(bias_pitch_rate));
    delay(100);
}
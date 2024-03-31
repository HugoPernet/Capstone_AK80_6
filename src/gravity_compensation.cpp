// Author @HugoPernet

#include <MotorConfig.cpp>

//weight
float weight = 100;

// Set values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 200.0f;
float kd_in = 1.0f;
float t_in = 0.0f;
// measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

void setup() {
    Serial.begin(900); 
    while (!Serial) delay(10);
    Serial.println("Serial connection established..");

    //CAN PinOut
    pinMode(PIN_CAN_STANDBY, OUTPUT); 
    digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY 
    pinMode(PIN_CAN_BOOSTEN, OUTPUT); 
    digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
    
    // start the CAN bus at 1Mbaud 
    if (!CAN.begin(1000000)) {
        Serial.println("Starting CAN failed!");
        while (1) delay(10); 
        }
    delay(1000);
    Serial.println("CAN strated...");

    //enable Motor
    Serial.println("Entering Motor Mode...");
    EnterMotorMode();
    delay(1000);
    SetZero();
    delay(1000);
    Motor_Out reply = unpack_reply();
    Serial.println(String(reply.Motor_Out_p));
    delay(5000);
}

float dir = -1;
void loop() {
    Motor_Out reply = unpack_reply();
    p_out = reply.Motor_Out_p;
    v_out = reply.Motor_Out_v;
    t_out = reply.Motor_Out_t;
    Serial.println("P_out:"+String(p_out)+ " torque:"+String(t_out)+" V out"+ String(v_out));
    //p_in = p_out;
    t_in = weight*sin(p_out + 0.06) + 0.0;
    t_in = constrain(t_in, T_MIN, T_MAX);
    pack_cmd(p_in,v_in,kp_in,kd_in,t_in);
    delay(100);
}
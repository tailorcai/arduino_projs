#pragma once


// 增量式pid
class MotorPID {
  public:
    float rpm_desired; // = 4*600;
    float kp, ki;
    int pid_out;
    long us_last;
    float integral;

    MotorPID():kp(0.05),ki(0),us_last(micros()),pid_out(128) {}

    void set_rpm(float rpm) {
      rpm_desired = rpm;
    }

    bool run(int encoder_delta) {
      if( rpm_desired == 0) {
        pid_out = 0;
        return true;
      }
      
      long now = micros();
      if( now - us_last < 50*1000) // 50ms at least
        return false;

      float dt = float(now - us_last) / 1e6; // seconds
      
      // encoder.clearCount();
      float error = rpm_desired * 11.0/60.0 - encoder_delta/dt;   //  pulse per seconds
      // pid_encoder = error;// pid_encoder * 0.7 + encoder_last * 0.3;
      integral += error *dt;

      float pid_delta = kp * error + ki * integral;
      pid_out = int( pid_out + pid_delta );

      // regurate 
      pid_out = min(255,max(0,pid_out));

      // Serial.println( String(error) + "," + String(pid_delta)+ "," + String(integral)+ "," + String(pid_out));

      // ledcWrite( PIN_PWM, pid_out );    // update pwm
      us_last = now;
      return true;
    }
};
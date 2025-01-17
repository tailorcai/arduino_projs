#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <Keypad.h>
#include <AgileStateMachine.h>
#include "tennis.h"
// #include <WiFi.h> 
#include <ESP32Encoder.h>

// WifiManager manager;
StateMachine fsm;
LiquidCrystal_I2C lcd(0x27, 16, 2);
ESP32Encoder ball_feed_encoder;

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {35, 36, 37, 38}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {39, 40, 41, 42}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

const int PIN_SDA = 2;
const int PIN_SCL = 1;
const int PIN_PWM1 = 18;  // 16 corresponds to GPIO16
const int PIN_PWM2 = 8;  // 16 corresponds to GPIO16
const int PIN_PWM_FEED = 3; // GPIO3
const int PIN_ENCODER_A = 17, PIN_ENCODER_B = 16; // 

int pwm_percent(int percent) {
  return round(255*max(0,min(100,percent))/100.0);
}

MotorPID feeder_pid;

class Level {
public:
  int v;
  int levels;
  Level(int alevels, int init=0): levels(alevels),v(init) {}

  void inc() {
    v ++;
    if( v == levels)
      v = 0;
  }

  int map(int vmin, int vmax) {
    return v*(vmax-vmin)/(levels-1)+vmin;
  }

};

class MachineRun {
  private:
    bool dirty;
  public:
    Level ball_speed; // %
    Level feed_speed; // %
    Level ball_spin;
    // bool on; //

    MachineRun():ball_speed(5),feed_speed(5),ball_spin(5,2),dirty(false) {
    }

    void init() {
      dirty = true;
    }

    void set_dirty(bool v = true) {
      dirty = v;
    }

    void get_wheel_speed_percent(int& top, int& bottom) {
      float v = get_ball_speed_percent();
      // bottom = top;
      float spin = get_spin_percent()/100.0;
      Serial.println("spin v=" + String(spin));
      if( spin >=0 ) {
        v = min((float)100,  v * (1 + spin));
        bottom = (float) v * (1-spin);
        top = v;
      }
      else {
        bottom = (float) v*(1 + spin);
        top = v;
      }
    }    

    int get_ball_speed_percent() {
        return ball_speed.map(50,100);
    }
    int get_feed_interval() { // seconds per ball
        return feed_speed.map(10,2);    // from 10 to 2
    }
    int get_spin_percent() {
        return ball_spin.map(-30,30);
    }

    bool is_dirty() { return dirty; }
};

MachineRun machine_run;

OneButton btnA( 47, false, false);    // OnOff
OneButton btnB( 21, false, false);    // ball speed
OneButton btnC( 48, false, false);    // feed speed
OneButton btnD( 45, false, false);    // spin level

bool A_pressed = false;
void rc_OnOff_click() {
  A_pressed = true;
}

void rc_Speed_click() {
  machine_run.ball_speed.inc();
  machine_run.set_dirty();
}

void rc_Feed_click() {
  machine_run.feed_speed.inc();
  machine_run.set_dirty();
}

void rc_Spin_click() {
  machine_run.ball_spin.inc();
  machine_run.set_dirty();
}

void OnEnterStartWheel() {
  // start wheel
  // machine_state.on = true;
  A_pressed = false;
  machine_run.set_dirty(true);

  ledcWrite(PIN_PWM1, 255 );//pwm_percent(machine_run.get_ball_speed_percent()));
  ledcWrite(PIN_PWM2, 255 );//pwm_percent(machine_run.get_ball_speed_percent()));

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print( "Starting " );
  lcd.setCursor(1,1);
  lcd.print( String( machine_run.get_ball_speed_percent()) + " 0 0");    
}

void onRunning() {
  // PID should always be running
  // 3 balls per round
  // reduction rate: 1:600
  // 60 / (interval*3) * 600 is the motor rpm
  feeder_pid.set_rpm( 60.0*600/ ( machine_run.get_feed_interval() * 3) );  
  int cnt = ball_feed_encoder.getCount();
  if( feeder_pid.run(cnt) ) {
    ball_feed_encoder.clearCount();
    ledcWrite(PIN_PWM_FEED, feeder_pid.pid_out );
  }

  static long tm = millis();

  if( millis() - tm > 500 || machine_run.is_dirty()) {
    int top, bottom;
    machine_run.get_wheel_speed_percent(top,bottom);
    ledcWrite(PIN_PWM1, pwm_percent(top));
    ledcWrite(PIN_PWM2, pwm_percent(bottom));

    lcd.clear();
    lcd.setCursor(1, 0);
    String s = "On ["+ String(top ) + "," + String(bottom) + "]" ;
    Serial.println(s);
    lcd.print( s );
    lcd.setCursor(1,1);
    s = String( machine_run.get_ball_speed_percent()) + "% " + String(machine_run.get_feed_interval()) + "s " + String(machine_run.ball_spin.v);
    lcd.print( s );    
    Serial.println(s);
    tm = millis();
  }
  machine_run.set_dirty( false ); 
}

void stopAllMotor() {
  ledcWrite(PIN_PWM1, 0);
  ledcWrite(PIN_PWM2, 0);
  ledcWrite(PIN_PWM_FEED, 0);
}

void onIdleRunning() {
  static long tm = millis();
  if( millis() - tm > 500 || machine_run.is_dirty()) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print( "Off" );
    lcd.setCursor(1,1);
    lcd.print( String( machine_run.get_ball_speed_percent()) + " " + String(machine_run.get_feed_interval()));    
  }
  machine_run.set_dirty( false );
}

void onEnterIdle() {
  stopAllMotor();
  // reset button press state
  A_pressed=false;
}

void OnEnterWheelReady() {
  // reset button press state
  A_pressed=false;

  // 电机恢复正常转速
  int top, bottom;
  machine_run.get_wheel_speed_percent(top,bottom);
  ledcWrite(PIN_PWM1, pwm_percent(top));
  ledcWrite(PIN_PWM2, pwm_percent(bottom));
}

// setting PWM properties
const int freq = 2000;
const int resolution = 8;
 
void setup(){
  // 
  pinMode(PIN_PWM1,OUTPUT);
  pinMode(PIN_PWM2,OUTPUT);
  pinMode(PIN_PWM_FEED, OUTPUT);

  // configure LED PWM
  ledcAttach(PIN_PWM1, freq, resolution);
  ledcAttach(PIN_PWM2, freq, resolution);
  ledcAttach(PIN_PWM_FEED, freq, resolution);

  stopAllMotor();

  // 
  Serial.begin(115200);
  Serial.println("Starting");

  Wire.begin(PIN_SDA, PIN_SCL);

  // WIFI
  // manager.setupAP(); // Launch AP mode first, then fail over to connecting to a station
  // while (manager.getState() != Connected) {
	//  	manager.loop();
	//  	delay(10);
	// }

  ball_feed_encoder.attachSingleEdge ( PIN_ENCODER_A, PIN_ENCODER_B );
  ball_feed_encoder.setCount ( 0 );

  // init lcd
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Starting...");

  btnA.attachPress( rc_OnOff_click );
  btnB.attachPress( rc_Speed_click );
  btnC.attachPress( rc_Feed_click );
  btnD.attachPress( rc_Spin_click );

  machine_run.init();

  State* stIdleState = fsm.addState("Idle State",onEnterIdle, nullptr, onIdleRunning);

  State* stStartWheelState = fsm.addState("Start Wheel State", OnEnterStartWheel, nullptr /* exit */, nullptr);
  State* stWheelReadyState = fsm.addState("Wheel Ready State", OnEnterWheelReady, nullptr /* exit */, nullptr);
  State* stRunningState = fsm.addState("Running State", nullptr, stopAllMotor/* exit */, onRunning);

  stIdleState->addTransition(stStartWheelState, A_pressed);

  stStartWheelState->addTransition(stWheelReadyState, 5000);     // 5 seconds and everything is fine,  to start 
  stStartWheelState->addTransition(stIdleState, A_pressed);     // 5 seconds and everything is fine,  to start 

  stWheelReadyState->addTransition( stRunningState, 2000);      // 2s waiting for motor speed
  stWheelReadyState->addTransition(stIdleState, A_pressed);
  //stStartWheelState->addTransition(stHaltState, f_error);    // in case of error, go to error halt
  
  stRunningState->addTransition(stIdleState, A_pressed);
  //stRunningState->addTransition(stHaltState, f_error);    // in case of error, go to error halt

  fsm.setInitialState(stIdleState);
  fsm.start();
}
 
void loop(){
  // Serial.println( btnC.state());

  char key = keypad.getKey();
  switch(key) {
    case 'A': rc_OnOff_click(); break;
    case 'B': rc_Speed_click(); break;
    case 'C': rc_Feed_click();break;
    case 'D': rc_Spin_click();break;
  }

  btnA.tick();
  btnB.tick();
  btnC.tick();
  btnD.tick();

  fsm.execute();

	// manager.loop();
	// if (manager.getState() == Connected)  // only update if WiFi is up
	// 	updateDashboard();                  // update the dashboard values

  delay(1);
}
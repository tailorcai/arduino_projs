#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <Keypad.h>
#include <AgileStateMachine.h>

#include <WiFi.h> 

WifiManager manager;
StateMachine fsm;
LiquidCrystal_I2C lcd(0x27, 16, 2);

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

int pwm_percent(int percent) {
  return round(255*max(0,min(100,percent))/100.0);
}

class Level {
public:
  int v;
  int levels;
  Level(int alevels): levels(alevels),v(0) {}

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
    Level spin;
    // bool on; //

    MachineRun():ball_speed(5),feed_speed(5),spin(3),dirty(false) {}

    void init() {
      dirty = true;
    }

    void set_dirty(bool v = true) {
      dirty = v;
    }

    int get_ball_speed_percent() {
        return ball_speed.map(80,100);
    }
    int get_feed_percent() {
        return feed_speed.map(60,100);
    }
    int get_spin_percent() {
        return feed_speed.map(-30,30);
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
  machine_run.spin.inc();
  machine_run.set_dirty();
}


void OnEnterStartWheel() {
  // start wheel
  // machine_state.on = true;
  A_pressed = false;
  machine_run.set_dirty(true);
}

void onStartRunning() {
  if( machine_run.is_dirty()) {
    ledcWrite(PIN_PWM1, pwm_percent(machine_run.get_ball_speed_percent()));
    ledcWrite(PIN_PWM2, pwm_percent(machine_run.get_ball_speed_percent()));

    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print( "Starting " );
    lcd.setCursor(1,1);
    lcd.print( String( machine_run.get_ball_speed_percent()) + " " + String(0));    
  }
  machine_run.set_dirty(false);
}

void onRunning() {
  if( machine_run.is_dirty()) {
    ledcWrite(PIN_PWM1, pwm_percent(machine_run.get_ball_speed_percent()));
    ledcWrite(PIN_PWM2, pwm_percent(machine_run.get_ball_speed_percent()));

    ledcWrite(PIN_PWM_FEED, pwm_percent(machine_run.get_feed_percent()));

    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print( "On" );
    lcd.setCursor(1,1);
    lcd.print( String( machine_run.get_ball_speed_percent()) + " " + String(machine_run.get_feed_percent()));    
  }
  machine_run.set_dirty( false ); 
}

void stopAllMotor() {
  ledcWrite(PIN_PWM1, 0);
  ledcWrite(PIN_PWM2, 0);
  ledcWrite(PIN_PWM_FEED, 0);
}

void onIdleRunning() {
  if( machine_run.is_dirty()) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print( "Off" );
    lcd.setCursor(1,1);
    lcd.print( String( machine_run.get_ball_speed_percent()) + " " + String(machine_run.get_feed_percent()));    
  }
  machine_run.set_dirty( false );
}

void onEnterIdle() {
  machine_run.set_dirty();  // force update display
  A_pressed=false;
}

void onEnterRunning() {
  machine_run.set_dirty(); // force update display
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
  manager.setupAP(); // Launch AP mode first, then fail over to connecting to a station
  // while (manager.getState() != Connected) {
	//  	manager.loop();
	//  	delay(10);
	// }

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

  State* stStartWheelState = fsm.addState("Start Wheel State", OnEnterStartWheel, stopAllMotor /* exit */, onStartRunning);
  State* stRunningState = fsm.addState("Running State", onEnterRunning, stopAllMotor/* exit */, onRunning);

  stIdleState->addTransition(stStartWheelState, A_pressed);

  stStartWheelState->addTransition(stRunningState, 5000);     // 5 seconds and everything is fine,  to start 
  stStartWheelState->addTransition(stIdleState, A_pressed);     // 5 seconds and everything is fine,  to start 
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

	manager.loop();
	if (manager.getState() == Connected)  // only update if WiFi is up
		updateDashboard();                  // update the dashboard values

  delay(1);
}
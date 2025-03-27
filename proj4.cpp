void setup() {
    // put your setup code here, to run once:
  
  }
  
  void loop() {
    const unsigned long current_time= millis()
    // put your main code here, to run repeatedly:
    if (current_time- last_fsm_time > fsm_interval) {
   fsm_task();
   last_fsm_time = current_time;
   }
  }
   //function to generate direction and PWM signal for each of the motor inputs
    float bound(float value, float min_value, float max_value){
      if (value >= min_value && value <= max_value) {value = value};
      else if (value > max_value){value= max_value}
      else {value= min value}
    }
    // that sets the thrust magnitude and direction for motor motor
   void set_motor(int motor, float val){
  if (val >= -64 && val<= 64) {val = val}
  else if {val > 64} {val = 64}
  else {value = -64}
   }
  void set_motors(float val[3]){
    const float negative_gain[3] = {1.0, 1.0, 1.0};
    if (val[3] < 0) {val[3] = val[3]*negative_gain[3]}
  }
   analogWrite(pin, duty)
  fsm_step() {unsigned long timing= 60 {
    switch
  }
  typedef enum {
   STATE_START
   STATE_L25
   STATE_R25
   STATE_B25
   STATE_neg25
   STATE_L0
   STATE_R0
   STATE_B0
   } State; }
   typedef enum {
   pressed
   unpressed
   counterthreshold
   } event; 
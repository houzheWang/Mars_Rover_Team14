/*
 * Program written by Yue Zhu (yue.zhu18@imperial.ac.uk) in July 2020.
 * pin6 is PWM output at 62.5kHz.
 * duty-cycle saturation is set as 2% - 98%
 * Control frequency is set as 1.25kHz. 
*/

#include <Wire.h>
#include <INA219_WE.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

float closed_loop; // Duty Cycles
float vpd,vb,vb_pd,vref,iL,dutyref,current_mA; // Measurement Variables
float Pin, Pin_pre = 0, Vout, Vout_pre = 0, duty_cycle = 0.5; // Varaiables for MPPT, initialising Pin_pre, Vout_pre and duty_cycle with 0
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration
float oc=0; //internal signals
float Ts=0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float pwm_out;
float V_in, V_out, current_measure, deltaV_in, deltacurrent_measure, current_measure_pre, V_in_pre, open_loop; //incremental conductance mppt

float current_limit = 3.0;
boolean Boost_mode = 0;
boolean CL_mode = 0;

String dataString;


unsigned int loopTrigger;
unsigned int com_count=0;   // a variables to count the interrupts. Used for program debugging.

void setup() {

  //Basic pin setups
  
  noInterrupts(); //disable all interrupts
  pinMode(13, OUTPUT);  //Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); //Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  
  pinMode(6, OUTPUT);
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  Serial.begin(9600);
}

 void loop() {
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt
    
    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    if (Boost_mode){
      if (CL_mode) { //Closed Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
      }else{ // Open Loop Boost
                //From sampling we get,
                //vpd， is the scaled output voltage Va
                //vb， is the input voltage 
                //iL， is the inductor current 
                //then we can calculate the input power from vpd and iL
         V_in = vb_pd*2;   
         V_out = (vpd/330)*890;   
         deltaV_in = V_in - V_in_pre;
         deltacurrent_measure = current_measure - current_measure_pre;
         if (deltaV_in!=0){
            if ((deltacurrent_measure / deltaV_in) > (-current_measure / V_in)){
            open_loop = open_loop + 0.01;
            }
            else if ((deltacurrent_measure / deltaV_in) < (-current_measure / V_in)){
            open_loop = open_loop - 0.01;
            }
         } 
              else if(deltaV_in==0){
            if(deltacurrent_measure > 0){
            open_loop = open_loop + 0.01;
          }
            else if(deltacurrent_measure < 0){
            open_loop = open_loop - 0.01;
          }
        }

//        if (V_out > 8){
//          open_loop = open_loop+0.01;
//        }
//        else{
//          open_loop = open_loop-0.01;
//        }
        V_in_pre = V_in;
        current_measure_pre = current_measure;
        open_loop = saturation(open_loop, 0.99, 0.01); //duty_cycle saturation
        analogWrite(6, (int)(255 - open_loop * 255));
        
      }
      
    }else{      
      if (CL_mode) { // Closed Loop Buck
          pwm_modulate(0); // This disables the Buck as we are not using this mode
      }else{// Open Loop Buck
          pwm_modulate(0); // This disables the Buck as we are not using this mode
      } 
    }
    // closed loop control path
//dataString = "Duty_cycle = " + String(open_loop) + ", Vin = " + String(V_in) + ", Vout = " + String(V_out) + ", Power = " + String(current_measure * vb); //+ "," + String(current_measure); 
        dataString =  "Vin = " + String(V_in) + ",Power = " + String(current_measure * vb); //+ "," + String(current_measure); 
        Serial.println(dataString);
        delay(5);
    digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
  }
}


// Timer A CMP1 interrupt. Every 800us the program enters this interrupt. 
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
}

// This subroutine processes all of the analogue samples, creating the required values for the main loop

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensorValue0 = analogRead(A0); //sample Vb
  sensorValue1 = analogRead(A1);
  sensorValue2 = analogRead(A2); //sample Vref
  sensorValue3 = analogRead(A3); //sample Vpd
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vb_pd = sensorValue1*(4.096/1023.0);
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  if (Boost_mode == 1){
    current_measure = -current_mA/1000.0;
    dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    current_measure = current_mA/1000.0;
    dutyref = sensorValue2 * (1.0 / 1023.0);
  }
  
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturation function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}


/*end of the program.*/

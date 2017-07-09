/*
 AIDA Compressor Sketch
 	
 This sketch is a "Compressor" as described in...
 
 created February 2016
 by Massimo Pennazio
 
 This example code is in the public domain.
 
 */

#include <Arduino.h>
#include <pins_arduino.h>
#include <Wire.h>
#include "AidaFW.h"
#include "AidaDSPStompbox.h"

#define EVER (;;)

// DEFINES USER INTERFACE
#define PREGAIN_MAX 12.0f // dB
#define PREGAIN_MIN 0.0f // dB
#define PRECISION_1 0.1f

#define THR_MAX 6.0f // dB
#define THR_MIN -96.0f // dB
#define PRECISION_2 0.1f

#define RATIO_MAX 16.0f
#define RATIO_MIN 1.0f
#define PRECISION_3 0.25f

#define ATTACK_MAX 500.0f // ms
#define ATTACK_MIN 1.0f // ms
#define PRECISION_4 0.5f

#define HOLD_MAX ATTACK_MAX
#define HOLD_MIN 1.0f // ms
#define PRECISION_5 0.5f

#define DECAY_MAX 2000.0f // ms
#define DECAY_MIN 1.0f // ms
#define PRECISION_6 1.0f

#define BRIGHT_MAX 18.0f // dB
#define BRIGHT_MIN -18.0f  // dB
#define PRECISION_7 0.1f

#define MASTER_VOLUME_MAX 6.00
#define MASTER_VOLUME_MIN -6.00
#define PRECISION_9 0.1f

#define POSTGAIN_MAX 24.0f // dB
#define POSTGAIN_MIN -30.0f // dB
#define PRECISION_10 0.1f

#define F_MIN 500.0f // Hz
#define F_MAX 12000.0f // Hz
#define PRECISION_11 50.0f

#define POT_THR 4 // Threshold for filtering noise on pots (adcs)

#define ON 1
#define OFF 0
#define DEBOUNCETIME  50


#define N_PRESET  4  // Not used
enum preset_t{
  maxfunkyelectricguitar, 
  andreaelectricbass, 
  andreaerocklectricguitar, 
  andreaacousticguitar
};

// FUNCTION PROTOTYPES
void spettacolino();
void clearAndHome(void);
void setBypass(uint8_t);
void setBrightPrePost(uint8_t);
void debounceSwitch1(void);
void debounceSwitch2(void);
void setNotchEq(float);
void switchPreset(void);
void print_menu_putty(void);

// GLOBAL VARIABLES
// ENCODER
int32_t OldPulses = 0;

// UI
uint8_t func_counter = 0;
uint8_t old_func_counter = 0;
uint8_t bypass = OFF;

uint32_t timec=0, prevtimec=0;

// Values in param pulses are startup values for
// DSP Blocks

int32_t param1_pulses = 0; // Pre Gain
int32_t param2_pulses = 0; // Threshold
int32_t param3_pulses = 0; // Ratio
int32_t param4_pulses = 0; // Attack
int32_t param5_pulses = 0; // Hold
int32_t param6_pulses = 0; // Decay
int32_t param7_pulses = 0; // Bright
int32_t param8_pulses = 0; // PrePost Bright
int32_t param9_pulses = 0; // Master Volume
int32_t param10_pulses = 0; // Post Gain
int32_t param11_pulses = 0; // Parametric Eq Frequency

int32_t tmpencoderpulses = 0;
uint8_t restore = 1;  // If 1 startup values are written to DSP

float param1_value = 0.00; 
float param2_value = 0.00; 
float param3_value = 0.00; 
float param4_value = 0.00; 
float param5_value = 0.00; 
float param6_value = 0.00;
float param7_value = 0.00;
uint8_t param8_value = 0;
float param9_value = 0.00;
float param10_value = 0.00;
float param11_value = 0.00;

preset_t preset;
int8_t countpreset = 0;
equalizer_t bright_eq;
equalizer_t param_eq;
compressor_t compressor1;

char inByte = 0x00;

uint16_t pot1 = 0;
uint16_t oldpot1 = 0;
uint16_t pot2 = 0;
uint16_t oldpot2 = 0;

// Pot Filter 1
uint16_t adcvalue1 = 0;
uint32_t sum1 = 0;
uint32_t out1 = 0;
// Pot Filter 2
uint16_t adcvalue2 = 0;
uint32_t sum2 = 0;
uint32_t out2 = 0;

// Push Encoder
uint8_t push_e_count = 0;
uint8_t push_e_function = 0;

// Push 1
uint8_t push_1_lock = 0;

// Push 2
uint8_t push_2_lock = 0;

void setup()
{
  // put your setup code here, to run once:
  // I/O

  // open the USBSerial port
  Serial.begin(115200);
  clearAndHome();
  Serial.println(F("Aida DSP Stombox")); // Welcome message
  Serial.print(F("0x"));
  Serial.println((DEVICE_ADDR_7bit<<1)&~0x01, HEX);

  // DSP board
  InitAida();	// Initialize DSP board
  digitalWrite(RESET, HIGH); // Wake up DSP
  delay(100);  // Start-up delay for DSP
  program_download();    // Here we load program, parameters and hardware configuration to DSP
  delay(20);
  check_program(); // !!!Debug!!!
  delay(5);
  check_param(); // !!!Debug!!!
  delay(5);
  check_config(); // !!!Debug!!!
  delay(2);
  spettacolino();
  
  /*******************************************************************
  *                        VALUE AT STARTUP                          *
  *******************************************************************/
  preset = andreaerocklectricguitar; // Value at startup
  countpreset = (uint8_t)preset;
  
  // Apply preset 
  switchPreset();

  // Initial bypass status read from FOOTSW
  if(digitalRead(FOOTSW) == LOW)
    bypass = ON;
  else
    bypass = OFF;
  setBypass(bypass);
   
  MuteOff();  // Mute DAC Off
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  if(Serial.available()>0) // Serial parser to speed up operations
  {
    inByte = Serial.read();
    switch(inByte)
    {
      case 'q':
        func_counter = 0; 
        break;
      case 'w':
        func_counter = 1; 
        break;
      case 'e':
        func_counter = 2; 
        break;
      case 'r':
        func_counter = 3; 
        break;
      case 't':
        func_counter = 4; 
        break;
      case 'y':
        func_counter = 5; 
        break;
      case 'u':
        func_counter = 6; 
        break;
      case 'i':
        func_counter = 7; 
        break;
      case 'o':
        func_counter = 8; 
        break;
      case 'p': 
        func_counter = 9; 
        break;
      case 'a':
        func_counter = 10; 
        break;
      case 'z':
        countpreset++;
        if(countpreset == N_PRESET)
          countpreset = N_PRESET-1;
        preset = (preset_t)countpreset;
        break;
      case 'x':
        countpreset--;
        if(countpreset < 0)
          countpreset = 0;
        preset = (preset_t)countpreset;
        break;
      case '+':
        tmpencoderpulses = getPulses();
        tmpencoderpulses++;
        setPulses(tmpencoderpulses);
        break;
      case '-':
        tmpencoderpulses = getPulses();
        tmpencoderpulses--;
        setPulses(tmpencoderpulses);
        break;
      default:
        // Do nothing
        break;
    }
  }

  adcvalue1 = analogRead(POT2);
  sum1 = ((((64)-1) * sum1)+((uint32_t)adcvalue1*(64)))/(64);
  out1 = sum1/64;
  pot1 = out1;
  if(!isinrange(pot1, oldpot1, POT_THR))
  {
    func_counter = (uint8_t)selectorwithpot(pot1, 4);
    if(func_counter>10)
      func_counter = 10;
    
    oldpot1 = pot1;
  }
  
  adcvalue2 = analogRead(POT1);
  sum2 = ((((64)-1) * sum2)+((uint32_t)adcvalue2*(64)))/(64);
  out2 = sum2/64;
  pot2 = out2;
  if(!isinrange(pot2, oldpot2, POT_THR))
  {
    func_counter=8;
    param9_value = processpot(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX, pot2); // Master Volume
    MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, param9_value/20.0)); // Set Master Volume
    oldpot2 = pot2;
  }

  debounceSwitch1();
  debounceSwitch2();
  
  timec = millis();
  if(timec-prevtimec >= 250)  // Here we manage control interface every 250ms
  {  
    switchPreset();
    
    if(old_func_counter != func_counter)
    {
      restore = 1;
      old_func_counter = func_counter;
    }
    switch(func_counter)
    {
    case 0: // Pre Gain
      if(restore)
      {
        restore = 0;
        setPulses(param1_pulses);
      }
      param1_pulses = getPulses();
      set_regulation_precision2(PRECISION_1);
      param1_value = processencoder2(PREGAIN_MIN, PREGAIN_MAX); // Pre Gain
      gainCell(DEVICE_ADDR_7bit, PreGainAddr, pow(10, param1_value/20.0));
      break;
    case 1: // Threshold
      if(restore)
      {
        restore = 0;
        setPulses(param2_pulses);
      }
      param2_pulses = getPulses();
      set_regulation_precision2(PRECISION_2);
      param2_value = processencoder2(THR_MIN, THR_MAX); // Threshold
      compressor1.threshold = param2_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 2: // Ratio
      if(restore)
      {
        restore = 0;
        setPulses(param3_pulses);
      }
      param3_pulses = getPulses();
      set_regulation_precision2(PRECISION_3);
      param3_value = processencoder2(RATIO_MIN, RATIO_MAX); // Ratio
      compressor1.ratio = param3_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 3: // Attack
      if(restore)
      {
        restore = 0;
        setPulses(param4_pulses);
      }
      param4_pulses = getPulses();
      set_regulation_precision2(PRECISION_4);
      param4_value = processencoder2(ATTACK_MIN, ATTACK_MAX); // Attack
      compressor1.attack = param4_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 4: // Hold
      if(restore)
      {
        restore = 0;
        setPulses(param5_pulses);
      }
      param5_pulses = getPulses();
      set_regulation_precision2(PRECISION_5);
      param5_value = processencoder2(HOLD_MIN, HOLD_MAX); // Hold
      compressor1.hold = param5_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 5: // Decay
      if(restore)
      {
        restore = 0;
        setPulses(param6_pulses);
      }
      param6_pulses = getPulses();
      set_regulation_precision2(PRECISION_6);
      param6_value = processencoder2(DECAY_MIN, DECAY_MAX); // Decay
      compressor1.decay = param6_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;  
    case 6: // Bright
      if(restore)
      {
        restore = 0;
        setPulses(param7_pulses);
      }
      param7_pulses = getPulses();
      set_regulation_precision2(PRECISION_7);
      param7_value = processencoder2(BRIGHT_MIN, BRIGHT_MAX); // Bright
      bright_eq.boost = param7_value;
      SetBrightPrePost(param8_value);
      break;
    case 7: // Bright Pre / Post
      if(restore)
      {
        restore = 0;
        setPulses(param8_pulses);
      }
      param8_pulses = getPulses();
      param8_value = selectorwithencoder(param8_pulses, 1); // Bright Pre Post
      SetBrightPrePost(param8_value);
      break;
    case 8: // Master Volume
//      if(restore)
//      {
//        restore = 0;
//        setPulses(param8_pulses);
//      }
//      param8_pulses = getPulses();
//      set_regulation_precision2(PRECISION_9);
//      param9_value = processencoder2(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX); // Master Volume
//      MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, param9_value/20.0)); // Set Master Volume
      break;
    case 9: // Post Gain
      if(restore)
      {
        restore = 0;
        setPulses(param10_pulses);
      }
      param10_pulses = getPulses();
      set_regulation_precision2(PRECISION_10);
      param10_value = processencoder2(POSTGAIN_MIN, POSTGAIN_MAX); // Post Gain
      compressor1.postgain = param10_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 10: // Parametric Eq Frequency
      if(restore)
      {
        restore = 0;
        setPulses(param11_pulses);
      }
      param11_pulses = getPulses();
      set_regulation_precision2(PRECISION_11);
      param11_value = processencoder2(F_MIN, F_MAX); // Parametric Eq Frequency
      setNotchEq(param11_value);
      break;
    } // End switch func_counter

    // Display information for user
    
    print_menu_putty();

    prevtimec = timec;
  } // End if 1000ms tick
} // End void loop

void spettacolino()
{
  byte i;
  byte statusc = 0x00;

  for(i=0;i<6;i++)
  {
    statusc ^= 1;
    digitalWrite(STATUS_LED, statusc);
    delay(250);
  }
  digitalWrite(STATUS_LED, LOW);
}

void clearAndHome(void)
{
  Serial.write(0x1b); // ESC
  Serial.print(F("[2J")); // clear screen
  Serial.write(0x1b); // ESC
  Serial.print(F("[H")); // cursor to home
}

void print_menu_putty(void)
{
  clearAndHome();    // !!!Warning use with real terminal emulation program
  Serial.println(F("********************************"));
  Serial.println(F("*    User control interface    *"));
  switch(preset)
  {
    case maxfunkyelectricguitar:
      Serial.println(F("*    FUNKY GUIT                *"));
      break;
    case andreaelectricbass:
      Serial.println(F("*    ELEC BASS                 *"));
      break;
    case andreaerocklectricguitar:
      Serial.println(F("*    ROCK GUIT                 *"));
      break;
    case andreaacousticguitar:
      Serial.println(F("*    ACUST GUIT                *"));
      break;  
  }
  Serial.println(F("********************************"));
  Serial.write('\n');
  Serial.print(F("Encoder pulses: "));
  Serial.println(getPulses(), DEC);
  Serial.write('\n');
  
  // Print menu
  Serial.print(F("Effect status: "));
  if(bypass)
    Serial.println(F("bypass"));
  else
    Serial.println(F("on"));
  Serial.write('\n');  
  if(func_counter==0)
    Serial.print(F("    "));
  Serial.print(F("PreGain: "));
  Serial.print(param1_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==1)
    Serial.print(F("    "));
  Serial.print(F("Thr: "));
  Serial.print(param2_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==2)
    Serial.print(F("    "));
  Serial.print(F("Ratio: "));
  Serial.println(param3_value, 2);
  if(func_counter==3)
    Serial.print(F("    "));
  Serial.print(F("Attack: "));
  Serial.print(param4_value, 1);
  Serial.println(F(" ms"));
  if(func_counter==4)
    Serial.print(F("    "));
  Serial.print(F("Hold: "));
  Serial.print(param5_value, 1);
  Serial.println(F(" ms"));
  if(func_counter==5)
    Serial.print(F("    "));
  Serial.print(F("Decay: "));
  Serial.print(param6_value, 1);
  Serial.println(F(" ms"));
  if(func_counter==6)
    Serial.print(F("    "));
  Serial.print(F("Bright: "));
  Serial.print(param7_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==7)
    Serial.print(F("    "));
  Serial.print(F("Pos: "));
  if(param8_value==1)
    Serial.println(F(" Pre"));
  else if(param8_value==2)
    Serial.println(F(" Post"));
  if(func_counter==8)
    Serial.print(F("    "));
  Serial.print(F("Vol: "));
  Serial.print(param9_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==9)
    Serial.print(F("    "));
  Serial.print(F("Makeup: "));
  Serial.print(param10_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==10)
    Serial.print(F("    "));
  Serial.print(F("Notch: "));
  Serial.print(param11_value, 1);
  Serial.println(F(" Hz"));
  
  Serial.write('\n');
  Serial.print(F("Active item: "));
  Serial.println(func_counter, DEC);
}

void setBypass(uint8_t value)
{
  if(value == ON)
  {
    muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 2); // Bypass
    digitalWrite(STATUS_LED, LOW);
  }
  else
  {
    muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 1); // FX
    digitalWrite(STATUS_LED, HIGH);
  }
}

void SetBrightPrePost(uint8_t value)
{
  bright_eq.S = 1.0;
  bright_eq.gain = 0.0; 
  switch(preset)
  {
    case maxfunkyelectricguitar:
      bright_eq.f0 = 2000.0; // Guitar
      break;
    case andreaelectricbass:
      bright_eq.f0 = 700.0; // Bass
      break;
    case andreaerocklectricguitar:
      bright_eq.f0 = 2000.0; // Guitar
      break;
    case andreaacousticguitar:
      bright_eq.f0 = 850.0; // Acoustic Guitar
      break;  
  }
  bright_eq.boost = param7_value;
  bright_eq.type = HighShelf;
  bright_eq.phase = false;
  
  if(value==1) // Pre
  {
    bright_eq.onoff = ON;
    EQ2ndOrd(DEVICE_ADDR_7bit, PreBrightAddr, &bright_eq);
    delayMicroseconds(100);
    bright_eq.onoff = OFF;
    EQ2ndOrd(DEVICE_ADDR_7bit, PostBrightAddr, &bright_eq);
    delayMicroseconds(100);
  }
  else if(value==2) // Post
  {
    bright_eq.onoff = OFF;
    EQ2ndOrd(DEVICE_ADDR_7bit, PreBrightAddr, &bright_eq);
    delayMicroseconds(100);
    bright_eq.onoff = ON;
    EQ2ndOrd(DEVICE_ADDR_7bit, PostBrightAddr, &bright_eq);
    //delayMicroseconds(100);
  }
}

void debounceSwitch1(void)
{
  static int reading1 = HIGH;
  static int buttonState1; // the current reading from the input pin
  static int lastButtonState1 = LOW; // the previous reading from the input pin
  static unsigned long lastDebounceTime1 = 0; // the last time the output pin was toggled
  
  reading1 = digitalRead(FOOTSW);

  // If the switch changed, due to noise or pressing:
  if (reading1!= lastButtonState1) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
    lastButtonState1 = reading1;
  }
  
  if ((millis() - lastDebounceTime1) > DEBOUNCETIME) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading1 != buttonState1) {
      buttonState1 = reading1;

      if (buttonState1 == LOW) // Switch
        bypass = ON;
      else
        bypass = OFF;
      setBypass(bypass); // Using FOOTSW and STATUS_LED
    }
  }
}

void debounceSwitch2(void)
{
  static int reading2 = HIGH;
  static int buttonState2; // the current reading from the input pin
  static int lastButtonState2 = LOW; // the previous reading from the input pin
  static unsigned long lastDebounceTime2 = 0; // the last time the output pin was toggled
  
  reading2 = digitalRead(RANDOMSW);

  // If the switch changed, due to noise or pressing:
  if (reading2!= lastButtonState2) {
    // reset the debouncing timer
    lastDebounceTime2 = millis();
  }
  
  if ((millis() - lastDebounceTime2) > DEBOUNCETIME) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading2 != buttonState2) {
      buttonState2 = reading2;

      //if (buttonState2 == LOW) // Switch
        // function
      //else
        // function
      // variable ^= 1; // Button management
    }
  }
  lastButtonState2 = reading2;
}

void MuteOff(void)
{
  AIDA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_7bit, CoreRegisterR4Addr, CoreRegisterR4Size, CoreRegisterR4Data);    // Mute DAC Off
}

void MuteOn(void)
{
  AIDA_WRITE_REGISTER_BLOCK( DEVICE_ADDR_7bit, CoreRegisterR0Addr, CoreRegisterR0Size, CoreRegisterR0Data ); // Mute DAC On
}

/*void push1_isr(void)
{
  
}*/

/*void push2_isr(void)
{

}*/

void check_program(void) 
{
  uint8_t value_wr = 0;
  uint8_t buff_r[5];
  uint8_t value_r;
  uint16_t addr = ProgramDataAddr;
  uint16_t i, j, errors;
  
  Serial.println(F("Program checking..."));
  
  errors = 0;
  for(i=0;i<ProgramDataSize;i+=5) // Program address 1024 to 2047
  {
    memset(buff_r, 0, 5);
    AIDA_READ_REGISTER(DEVICE_ADDR_7bit, addr, 5, buff_r); 
    for(j=0;j<5;j++)
    {
      #ifdef __AVR__
      //value_wr = pgm_read_byte_far(&ProgramDataData[i+j]);
      value_wr = pgm_read_byte_near(&ProgramDataData[i+j]);
      #else
      value_wr = ProgramDataData[i+j];
      #endif
      value_r = buff_r[j];
      if(value_wr != value_r)
      {
        errors++;
        break;
      }
    }
    if(errors)
      break;
    addr++;
    delayMicroseconds(100);
  }

  if(errors)
  {
    //Serial.print(F("i: "));
    //Serial.println(i, DEC);
    //Serial.print(F("j: "));
    //Serial.println(j, DEC);
    Serial.print(errors, DEC);
    Serial.println(F(" errors during Program download")); 
    Serial.print(F("Address: "));
    Serial.println(addr, DEC);
    Serial.print(F("Written = "));
    Serial.print(F("0x"));
    Serial.println(value_wr, HEX);
    Serial.print(F("Readed = "));
    Serial.print(F("0x"));
    Serial.println(value_r, HEX);
    while(1);
  }
  else
  {
    Serial.println(F("Program OK"));
  }
}

void check_param(void)
{
  uint8_t value_wr = 0;
  uint8_t buff_r[4];
  uint8_t value_r;
  uint16_t addr = regParamAddr;
  uint16_t i, j, errors;
  
  Serial.println(F("Parameter checking..."));
  
  errors = 0;
  for(i=0;i<regParamSize;i+=4) // 0 to 1023
  {
    memset(buff_r, 0, 4);
    AIDA_READ_REGISTER(DEVICE_ADDR_7bit, addr, 4, buff_r); 
    for(j=0;j<4;j++)
    {
      #ifdef __AVR__
      //value_wr = pgm_read_byte_far(&regParamData[i+j]);
      value_wr = pgm_read_byte_near(&regParamData[i+j]);
      #else
      value_wr = regParamData[i+j];
      #endif
      value_r = buff_r[j];
      if(j==0)
        value_wr&=0x0F;
      if(value_wr != value_r)
      {
        errors++;
        break;
      }
    }
    if(errors)
      break;
    addr++;
    delayMicroseconds(100);
  }
  
  if(errors)
  {
    Serial.print(errors, DEC);
    Serial.println(F(" errors during Reg Param download")); 
    Serial.print(F("Address: "));
    Serial.println(addr, DEC);
    Serial.print(F("Written = "));
    Serial.print(F("0x"));
    Serial.println(value_wr, HEX);
    Serial.print(F("Readed = "));
    Serial.print(F("0x"));
    Serial.println(value_r, HEX);
    while(1);
  }
  else
  {
    Serial.println(F("Reg Param OK"));
  }
}

void check_config(void)
{
  uint8_t value_wr = 0;
  uint8_t buff_r[HWConFigurationSize];
  uint8_t value_r;
  uint16_t addr = HWConFigurationAddr;
  uint16_t i, errors;
  
  Serial.println(F("HW Config checking..."));
  
  errors = 0;
  memset(buff_r, 0, HWConFigurationSize);
  AIDA_READ_REGISTER(DEVICE_ADDR_7bit, addr, HWConFigurationSize, buff_r); // Read all parameters in one block   
  
  for(i=0;i<HWConFigurationSize;i++) //  2076 to 2087 
  {
    #ifdef __AVR__
    //value_wr = pgm_read_byte_far(&HWConFigurationData[i]);
    value_wr = pgm_read_byte_near(&HWConFigurationData[i]);
    #else
    value_wr = HWConFigurationData[i];
    #endif
    value_r = buff_r[i];
    if(value_wr != value_r)
    {
      errors++;
      break;
    }
  }
  
  if(errors)
  {
    Serial.print(errors, DEC);
    Serial.println(F(" errors during HW config download")); 
    Serial.print(F("Address: "));
    Serial.println(addr, DEC);
    Serial.print(F("Written = "));
    Serial.print(F("0x"));
    Serial.println(value_wr, HEX);
    Serial.print(F("Readed = "));
    Serial.print(F("0x"));
    Serial.println(value_r, HEX);
    while(1);
  }
  else
  {
    Serial.println(F("HW Config OK"));
  }
}

void switchPreset(void)
{
  static preset_t oldpreset = maxfunkyelectricguitar;

  if(oldpreset != preset)
  {
    switch(preset)
    {
      case maxfunkyelectricguitar:
        param1_pulses = 0.0      /PRECISION_1; // Pre Gain
        param2_pulses = -30.0    /PRECISION_2; // Threshold
        param3_pulses = 4.0      /PRECISION_3; // Ratio
        param4_pulses = 53.0     /PRECISION_4; // Attack
        param5_pulses = 53.0     /PRECISION_5; // Hold
        param6_pulses = 500.0    /PRECISION_6; // Decay
        param7_pulses = 1.5      /PRECISION_7; // Bright
        param8_pulses = 0; // PrePost Bright
        param9_pulses = 1.0      /PRECISION_9; // Master Volume 05/07
        //param9_pulses = 0.0      /PRECISION_9; // Master Volume
        param10_pulses = 6.0     /PRECISION_10; // Post Gain
        param11_pulses = 500.0  /PRECISION_11; // Parametric Eq Frequency
        break;
      case andreaelectricbass:
        param1_pulses = 1.5      /PRECISION_1; // Pre Gain
        param2_pulses = -29.0    /PRECISION_2; // Threshold
        param3_pulses = 2.9      /PRECISION_3; // Ratio
        param4_pulses = 21.8     /PRECISION_4; // Attack
        param5_pulses = 53.0     /PRECISION_5; // Hold
        param6_pulses = 500.0    /PRECISION_6; // Decay
        param7_pulses = 1.5      /PRECISION_7; // Bright
        param8_pulses = 0; // PrePost Bright
        param9_pulses = 1.0      /PRECISION_9; // Master Volume 05/07
        //param9_pulses = 0.0      /PRECISION_9; // Master Volume
        param10_pulses = 9.0     /PRECISION_10; // Post Gain
        param11_pulses = 1000.0  /PRECISION_11; // Parametric Eq Frequency
        break;
      case andreaerocklectricguitar:
        param1_pulses = 1.5      /PRECISION_1; // Pre Gain
        param2_pulses = -33.0    /PRECISION_2; // Threshold
        param3_pulses = 3.0      /PRECISION_3; // Ratio
        param4_pulses = 53.0     /PRECISION_4; // Attack
        param5_pulses = 53.0     /PRECISION_5; // Hold
        param6_pulses = 209.0    /PRECISION_6; // Decay
        param7_pulses = 1.5      /PRECISION_7; // Bright
        param8_pulses = 0; // PrePost Bright
        param9_pulses = 1.0      /PRECISION_9; // Master Volume 05/07
        //param9_pulses = 0.0      /PRECISION_9; // Master Volume
        param10_pulses = 6.5     /PRECISION_10; // Post Gain
        param11_pulses = 1000.0  /PRECISION_11; // Parametric Eq Frequency
        break;
      case andreaacousticguitar:
        param1_pulses = 3.0      /PRECISION_1; // Pre Gain
        param2_pulses = -40.0    /PRECISION_2; // Threshold
        param3_pulses = 2.2      /PRECISION_3; // Ratio
        param4_pulses = 95.0     /PRECISION_4; // Attack
        param5_pulses = 53.0     /PRECISION_5; // Hold
        param6_pulses = 417.0    /PRECISION_6; // Decay
        param7_pulses = 0.0      /PRECISION_7; // Bright
        param8_pulses = 0; // PrePost Bright
        param9_pulses = 1.0      /PRECISION_9; // Master Volume 05/07
        //param9_pulses = 0.0      /PRECISION_9; // Master Volume
        param10_pulses = 8.8     /PRECISION_10; // Post Gain
        param11_pulses = 1000.0  /PRECISION_11; // Parametric Eq Frequency
        break;
    }
    
    setPulses(param1_pulses);
    set_regulation_precision2(PRECISION_1);
    param1_value = processencoder2(PREGAIN_MIN, PREGAIN_MAX); // Pre Gain
    
    setPulses(param2_pulses);
    set_regulation_precision2(PRECISION_2);
    param2_value = processencoder2(THR_MIN, THR_MAX); // Threshold
    
    setPulses(param3_pulses);
    set_regulation_precision2(PRECISION_3);
    param3_value = processencoder2(RATIO_MIN, RATIO_MAX); // Ratio
    
    setPulses(param4_pulses);
    set_regulation_precision2(PRECISION_4);
    param4_value = processencoder2(ATTACK_MIN, ATTACK_MAX); // Attack
    
    setPulses(param5_pulses);
    set_regulation_precision2(PRECISION_5);
    param5_value = processencoder2(HOLD_MIN, HOLD_MAX); // Hold
    
    setPulses(param6_pulses);
    set_regulation_precision2(PRECISION_6);
    param6_value = processencoder2(DECAY_MIN, DECAY_MAX); // Decay
    
    setPulses(param7_pulses);
    set_regulation_precision2(PRECISION_7);
    param7_value = processencoder2(BRIGHT_MIN, BRIGHT_MAX); // Bright
    
    param8_value = selectorwithencoder(param8_pulses, 1); // Bright Pre Post
    
    setPulses(param9_pulses);
    set_regulation_precision2(PRECISION_9);
    param9_value = processencoder2(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX); // Master Volume
    
    setPulses(param10_pulses);
    set_regulation_precision2(PRECISION_10);
    param10_value = processencoder2(POSTGAIN_MIN, POSTGAIN_MAX); // Post Gain
    
    setPulses(param11_pulses);
    set_regulation_precision2(PRECISION_11);
    param11_value = processencoder2(F_MIN, F_MAX); // Parametric Eq Frequency
    
    // Update values into DSP registers
    
    // KGain
    gainCell(DEVICE_ADDR_7bit, KGainAddr, 2.83);
    delayMicroseconds(100);
    
    // Pre Gain
    gainCell(DEVICE_ADDR_7bit, PreGainAddr, pow(10, param1_value/20.0));
    delayMicroseconds(100);
    
    // Bright Filter Pre & Post
    SetBrightPrePost(param8_value);
    delayMicroseconds(100);
    
    // Param Eq 
    setNotchEq(param11_value);
    delayMicroseconds(100);
    
    compressor1.threshold = param2_value;
    compressor1.ratio = param3_value;
    compressor1.attack = param4_value;
    compressor1.hold = param5_value;
    compressor1.decay = param6_value;
    compressor1.postgain = param10_value;
    CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
    delayMicroseconds(100);
    
    // Master Volume
    MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, param9_value/20.0)); // Set Master Volume 
    delayMicroseconds(100); 
    
    setPulses(param1_pulses);
    func_counter = 0;

    oldpreset = preset;
  }
}

void setNotchEq(float f)
{
  param_eq.Q = 1.41;
  param_eq.gain = 0.0;
  param_eq.f0 = f;
  param_eq.boost = -6.00;
  param_eq.type = Parametric;
  param_eq.phase = false;
  if(preset==andreaacousticguitar)
    param_eq.onoff = ON;
  else
    param_eq.onoff = OFF;
  EQ2ndOrd(DEVICE_ADDR_7bit, PreParametricAddr, &param_eq);
}

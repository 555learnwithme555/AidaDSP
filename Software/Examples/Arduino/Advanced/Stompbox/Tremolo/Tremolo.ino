/*
 AIDA Tremolo Sketch
 
 created November 2014
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
#define VOLMAX       15.00f
#define VOLMIN      -80.00f
#define FREQMIN       0.1f // Hz
#define FREQMAX      35.0f // Hz
#define BPMMIN        5.0f
#define BPMMAX      260.0f
#define COLORMIN    -15.00f // dB
#define COLORMAX     15.00f // dB
#define DEPTHMIN      0.0f
#define DEPTHMAX      1.0f
#define POT_THR       4

#define ON 1
#define OFF 0
#define DEBOUNCETIME  50

// FUNCTION PROTOTYPES
void spettacolino();
void clearAndHome(void);
void setBypass(void);
void setMix(uint8_t);
void setMode(void);
void setFrequency(void);
void setLfo(void);
void setColor(float);
void setVolume(float);
void setDepth(void);
void print_menu_putty(void);

// GLOBAL VARIABLES
// ENCODER
int32_t OldPulses = 0;

// UI
uint8_t bypass = OFF;
uint8_t func_counter = 0;
uint8_t old_func_counter = 0;
uint8_t mode = 1;
uint8_t lfotype = 1;
uint8_t restore = 1;
uint8_t mix = 0;
uint16_t readbackcount = 0;
int32_t freqpulses = 198;
int32_t bpmpulses = 213;
int32_t lfotypepulses = 0;
int32_t modepulses = 0;
int32_t colorpulses = 0;
int32_t volumepulses = 0;
int32_t mixpulses = 96;
int32_t depthpulses = 0;
uint32_t timec=0, prevtimec=0;

float volumedB = 0.00;
float frequency = 0.00;
float bpm = 0.00;
float colorvalue = 0.00;
float depthvalue = 0.00;
equalizer_t color;

uint16_t pot1 = 0;
uint16_t oldpot1 = 0;
uint16_t pot2 = 0;
uint16_t oldpot2 = 0;
uint16_t pot3 = 0;
uint16_t oldpot3 = 0;
uint16_t pot4 = 0;
uint16_t oldpot4 = 0;

// Push Encoder
uint8_t push_e_count = 0;
uint8_t push_e_function = 0;

// Push 1
uint8_t push_1_lock = 0;

// Push 2
uint8_t push_2_lock = 0;

equalizer_t equ;
// Pot Filter 1
uint16_t adcvalue1 = 0;
uint32_t sum1 = 0;
uint32_t out1 = 0;
// Pot Filter 2
uint16_t adcvalue2 = 0;
uint32_t sum2 = 0;
uint32_t out2 = 0;
// Pop Filter 3
uint16_t adcvalue3 = 0;
uint32_t sum3 = 0;
uint32_t out3 = 0;
// Pop Filter 4
uint16_t adcvalue4 = 0;
uint32_t sum4 = 0;
uint32_t out4 = 0;

void setup()
{
  // put your setup code here, to run once:
  // I/O

  // open the USBSerial port
  SerialUSB.begin(115200);
  //while(!SerialUSB);
  //clearAndHome();
  SerialUSB.println(F("Aida DSP Stompbox")); // Welcome message
  SerialUSB.print(F("0x"));
  SerialUSB.println((DEVICE_ADDR_7bit<<1)&~0x01, HEX);

  // DSP board
  InitAida();	// Initialize DSP board
  spettacolino();
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
  //spettacolino();  
  AIDA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_7bit, CoreRegisterR4Addr, CoreRegisterR4Size, CoreRegisterR4Data);    // Mute DAC Off
  
  // Set parameters to operate in normal mode 
  mode = 1;
  gainCell(DEVICE_ADDR_7bit, DepthAddr, 0.5);
  delayMicroseconds(100);
  dc_source(DEVICE_ADDR_7bit, BiasAddr, 0.5);
  delayMicroseconds(100);
  muxnoiseless(DEVICE_ADDR_7bit, AbsAddr, 2); // Abs Off
  delayMicroseconds(100);
 
  // Initialize LPF filters 
  equ.onoff = ON;
  equ.type = Lowpass;
  equ.gain = 0.00;
  equ.f0 = 10.0; // 10Hz LPF on saw and square waveforms
  EQ1stOrd(DEVICE_ADDR_7bit, LPF1Addr, &equ);
  delayMicroseconds(100);
  EQ1stOrd(DEVICE_ADDR_7bit, LPF2Addr, &equ);  
  
  // Bypass status init = disable
  bypass = OFF;
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  adcvalue1 = analogRead(POT1);
  sum1 = ((((64)-1) * sum1)+((uint32_t)adcvalue1*(64)))/(64);
  out1 = sum1/64;
  pot1 = out1;
  if(!isinrange(pot1, oldpot1, POT_THR))
  {
    func_counter=0;
    oldpot1 = pot1;
    //bpm = processpot(BPMMIN, BPMMAX, pot1);
    //frequency = bpm / 60.0; // Bpm to frequency conversion
    frequency = processpot(FREQMIN, FREQMAX, pot1);
    setFrequency();
  }
  
//  adcvalue2 = analogRead(POT2);
//  sum2 = ((((64)-1) * sum2)+((uint32_t)adcvalue2*(64)))/(64);
//  out2 = sum2/64;
//  pot2 = out2;
//  if(!isinrange(pot2, oldpot2, POT_THR))
//  {
//    func_counter=5;
//    oldpot2 = pot2;
//    mix = (uint8_t)processpot(0.0, 100.0, pot2);
//    setMix(mix);
//  }
  
//  adcvalue3 = analogRead(POT3);
//  sum3 = ((((64)-1) * sum3)+((uint32_t)adcvalue3*(64)))/(64);
//  out3 = sum3/64;
//  pot3 = out3;
//  if(!isinrange(pot3, oldpot3, POT_THR))
//  {
//    func_counter=3;
//    oldpot3 = pot3;
//    colorvalue = processpot(COLORMIN, COLORMAX, pot3);
//    setColor(colorvalue);
//  }

  adcvalue4 = analogRead(POT2);
  sum4 = ((((64)-1) * sum4)+((uint32_t)adcvalue4*(64)))/(64);
  out4 = sum4/64;
  pot4 = out4;
  if(!isinrange(pot4, oldpot4, POT_THR))
  {
    func_counter=4;
    oldpot4 = pot4;
    volumedB = processpot(VOLMIN, VOLMAX, pot4);
    setVolume(volumedB);
  }
  
  if(digitalRead(FOOTSW)==LOW)
  {
    delay(DEBOUNCETIME); // debounce
    if(digitalRead(FOOTSW)==LOW)
    {
      if(push_1_lock != 1)
      {
        push_1_lock = 1;
        //bypass ^= 1;
        bypass = ON;
      }
    }
  }
  else
  {
    push_1_lock = 0;
    bypass = OFF;
  }

  if(digitalRead(RANDOMSW)==LOW)
  {
    delay(DEBOUNCETIME); // debounce
    if(digitalRead(RANDOMSW)==LOW)
    {
      if(push_2_lock != 1)
      {
        push_2_lock = 1;
        // Do something
      }
    }
  }
  else
  {
    push_2_lock = 0;
    // Do something
  }
  
  timec = millis();
  if(timec-prevtimec >= 250)  // Here we manage control interface every 250ms
  { 
    clearAndHome();    // !!!Warning use with real terminal emulation program
    SerialUSB.println(F("********************************"));
    SerialUSB.println(F("*    User control interface    *"));
    SerialUSB.println(F("*     AIDA Tremolo Sketch      *"));
    SerialUSB.println(F("********************************"));
    SerialUSB.write('\n');
    
    setMode();
    setBypass(); // Using FOOTSW and STATUS_LED
    
    if(old_func_counter != func_counter)
    {
      restore = 1;
      old_func_counter = func_counter;
    }
    switch(func_counter)
    {
    case 0: // Frequency
      /*if(restore)
      {
        restore = 0;
        //setPulses(freqpulses);
        setPulses(bpmpulses);
      }
      set_regulation_precision(ON); // Fine regulation
      //freqpulses = getPulses();
      bpmpulses = getPulses();
      //frequency = processencoder(FREQMIN, FREQMAX, freqpulses);
      bpm = processencoder(BPMMIN, BPMMAX, bpmpulses);
      frequency = bpm / 60.0;
      setFrequency();
      */
      break;
    case 1: // LFO type
      if(restore)
      {
        restore = 0;
        setPulses(lfotypepulses);
      }
      lfotypepulses = getPulses();
      lfotype = selectorwithencoder(lfotypepulses, 2); 
      if(mode==2 || mode==4)
      {
        if(lfotype>2)
          lfotype=2; // Use only Tri, Sin with harmonic mode
      }
      setLfo();
      break;
    case 2: // Mode
      if(restore)
      {
        restore = 0;
        setPulses(modepulses);
      }
      modepulses = getPulses();
      mode = selectorwithencoder(modepulses, 2); 
      setMode();
      break;
    case 3: // Color
      if(restore)
      {
        restore = 0;
        setPulses(colorpulses);
      }
      set_regulation_precision(OFF); // Rough regulation
      colorpulses = getPulses();
      colorvalue = processencoder(COLORMIN, COLORMAX, colorpulses);
      setColor(colorvalue);
      break;
    case 4: // Volume
      /*if(restore)
      {
        restore = 0;
        setPulses(volumepulses);
      }
      set_regulation_precision(OFF); // Rough regulation
      volumepulses = getPulses();
      volumedB = processencoder(VOLMIN, VOLMAX, volumepulses);
      setVolume(volumedB);*/
      break;  
    case 5: // Mix
      if(restore)
      {
        restore = 0;
        setPulses(mixpulses);
      }
      set_regulation_precision(OFF); // Rough regulation
      mixpulses = getPulses();
      mix = (uint8_t)processencoder(0, 100, mixpulses);
      setMix(mix);
      break;
    case 6: // Depth
      if(restore)
      {
        restore = 0;
        setPulses(depthpulses);
      } 
      //set_regulation_precision(OFF); // Rough regulation
      depthpulses = getPulses();
      depthvalue = processencoder(DEPTHMIN, DEPTHMAX, depthpulses);
      //setDepth(); // !!!Debug!!!
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
  SerialUSB.write(0x1b); // ESC
  SerialUSB.print(F("[2J")); // clear screen
  SerialUSB.write(0x1b); // ESC
  SerialUSB.print(F("[H")); // cursor to home
}

void check_program(void) 
{
  uint8_t value_wr = 0;
  uint8_t buff_r[5];
  uint8_t value_r;
  uint16_t addr = ProgramDataAddr;
  uint16_t i, j, errors;
  
  SerialUSB.println(F("Program checking..."));
  
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
    SerialUSB.print(errors, DEC);
    SerialUSB.println(F(" errors during Program download")); 
    SerialUSB.print(F("Address: "));
    SerialUSB.println(addr, DEC);
    SerialUSB.print(F("Written = "));
    SerialUSB.print(F("0x"));
    SerialUSB.println(value_wr, HEX);
    SerialUSB.print(F("Readed = "));
    SerialUSB.print(F("0x"));
    SerialUSB.println(value_r, HEX);
    while(1);
  }
  else
  {
    SerialUSB.println(F("Program OK"));
  }
}

void check_param(void)
{
  uint8_t value_wr = 0;
  uint8_t buff_r[4];
  uint8_t value_r;
  uint16_t addr = regParamAddr;
  uint16_t i, j, errors;
  
  SerialUSB.println(F("Parameter checking..."));
  
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
    SerialUSB.print(errors, DEC);
    SerialUSB.println(F(" errors during Reg Param download")); 
    SerialUSB.print(F("Address: "));
    SerialUSB.println(addr, DEC);
    SerialUSB.print(F("Written = "));
    SerialUSB.print(F("0x"));
    SerialUSB.println(value_wr, HEX);
    SerialUSB.print(F("Readed = "));
    SerialUSB.print(F("0x"));
    SerialUSB.println(value_r, HEX);
    while(1);
  }
  else
  {
    SerialUSB.println(F("Reg Param OK"));
  }
}

void check_config(void)
{
  uint8_t value_wr = 0;
  uint8_t buff_r[HWConFigurationSize];
  uint8_t value_r;
  uint16_t addr = HWConFigurationAddr;
  uint16_t i, errors;
  
  SerialUSB.println(F("HW Config checking..."));
  
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
    SerialUSB.print(errors, DEC);
    SerialUSB.println(F(" errors during HW config download")); 
    SerialUSB.print(F("Address: "));
    SerialUSB.println(addr, DEC);
    SerialUSB.print(F("Written = "));
    SerialUSB.print(F("0x"));
    SerialUSB.println(value_wr, HEX);
    SerialUSB.print(F("Readed = "));
    SerialUSB.print(F("0x"));
    SerialUSB.println(value_r, HEX);
    while(1);
  }
  else
  {
    SerialUSB.println(F("HW Config OK"));
  }
}

void setMix(uint8_t percent)
{
  static uint8_t oldpercent = 0;
  float value = 0.00;
  
  if(oldpercent != percent)
  {
    if(percent>100)
      percent = 100;
    value = percent/100.00;
  
    // MIX
    AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, MixAddr, false, value);  // Dry	 
    AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, MixAddr+1, true, 1.00-value);   // Wet
  }
}

void setFrequency(void)
{
  static float oldfrequency = 0.00;
  
  if(frequency != oldfrequency) // Freq change, update all oscillators
  { 
    triangle_source(DEVICE_ADDR_7bit, Triangle1Addr, frequency*2.00);
    delayMicroseconds(10);
    sine_source(DEVICE_ADDR_7bit, Tone1Addr, frequency);
    delayMicroseconds(10);
    sawtooth_source(DEVICE_ADDR_7bit, Sawtooth1Addr, frequency);
    delayMicroseconds(10);
    square_source(DEVICE_ADDR_7bit, Square1Addr, frequency);
    delayMicroseconds(10);
    
    oldfrequency = frequency;
  }
  /* 
  switch(lfotype)
  {
  case 1:  
    triangle_source(DEVICE_ADDR_7bit, Triangle1, frequency);
    break;
  case 2:
    sine_source(DEVICE_ADDR_7bit, Triangle1, frequency);
    break;
  case 3:
    sawtooth_source(DEVICE_ADDR_7bit, Triangle1, frequency);
    break;
  case 4:
    square_source(DEVICE_ADDR_7bit, Triangle1, frequency);
    break;
  }
  */
}

void setLfo(void)
{
  static uint8_t lfotypeold = 0.00;
  
  if(lfotypeold != lfotype)
  {
    muxnoiseless(DEVICE_ADDR_7bit, LfoSelectorAddr, lfotype);
    lfotypeold = lfotype;
  }
}

void setMode(void)
{
  static uint8_t oldmode = 0;
  
  if(oldmode != mode)
  {
    switch(mode)
    {
    case 1:  
      muxnoiseless(DEVICE_ADDR_7bit, HarmonicAddr, 2); // NO Harmonic
      muxnoiseless(DEVICE_ADDR_7bit, OptoAddr, 2); // NO Opto
      gainCell(DEVICE_ADDR_7bit, DepthAddr, 0.5);
      delayMicroseconds(100);
      dc_source(DEVICE_ADDR_7bit, BiasAddr, 0.5);
      delayMicroseconds(100);
      muxnoiseless(DEVICE_ADDR_7bit, AbsAddr, 2); // Abs Off
      delayMicroseconds(100);
      break;
    case 2:
      muxnoiseless(DEVICE_ADDR_7bit, HarmonicAddr, 1); // SI Harmonic
      muxnoiseless(DEVICE_ADDR_7bit, OptoAddr, 2); // NO Opto
      gainCell(DEVICE_ADDR_7bit, DepthAddr, 1.0);
      delayMicroseconds(100);
      dc_source(DEVICE_ADDR_7bit, BiasAddr, 0.0);
      delayMicroseconds(100);
      muxnoiseless(DEVICE_ADDR_7bit, AbsAddr, 1); // Abs On
      delayMicroseconds(100);
      break;
    case 3:
      muxnoiseless(DEVICE_ADDR_7bit, HarmonicAddr, 2); // NO Harmonic
      muxnoiseless(DEVICE_ADDR_7bit, OptoAddr, 1); // SI Opto
      gainCell(DEVICE_ADDR_7bit, DepthAddr, 0.5);
      delayMicroseconds(100);
      dc_source(DEVICE_ADDR_7bit, BiasAddr, 0.5);
      delayMicroseconds(100);
      muxnoiseless(DEVICE_ADDR_7bit, AbsAddr, 2); // Abs Off
      delayMicroseconds(100);
      break;
    case 4:
      muxnoiseless(DEVICE_ADDR_7bit, HarmonicAddr, 1); // SI Harmonic
      muxnoiseless(DEVICE_ADDR_7bit, OptoAddr, 1); // SI Opto
      gainCell(DEVICE_ADDR_7bit, DepthAddr, 1.0);
      delayMicroseconds(100);
      dc_source(DEVICE_ADDR_7bit, BiasAddr, 0.0);
      delayMicroseconds(100);
      muxnoiseless(DEVICE_ADDR_7bit, AbsAddr, 1); // Abs On
      delayMicroseconds(100);
      break;
    }
    oldmode = mode;
  }
}

void setDepth(void)
{ 
  static float olddepthvalue = 0.00;
  float temp = 0.00;
  
  if(olddepthvalue != depthvalue)
  {
    if(mode==2 || mode==4) // Harmonic mode, scale lfo
    {
      gainCell(DEVICE_ADDR_7bit, DepthAddr, 1.0-depthvalue);
    }
    else // Normal mode, scale lfo and bias together
    {
      temp = (1.0-depthvalue)*0.5;
      gainCell(DEVICE_ADDR_7bit, DepthAddr, temp);
      delayMicroseconds(100);
      dc_source(DEVICE_ADDR_7bit, BiasAddr, 0.5+(depthvalue*0.25));
    }
    olddepthvalue = depthvalue;
  }
}

void print_menu_putty(void)
{
  // Print menu
  SerialUSB.print(F("Effect status: "));
  if(bypass)
    SerialUSB.println(F("bypass"));
  else
    SerialUSB.println(F("on"));
  SerialUSB.write('\n');  
  if(func_counter==0)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Freq. "));
  SerialUSB.print(frequency, 1);
  SerialUSB.println(F(" Hz"));
  if(func_counter==1)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Lfo type: "));
  if(lfotype==1)
    SerialUSB.println(F("triangular"));
  if(lfotype==2)
    SerialUSB.println(F("sine"));
  if(lfotype==3)
    SerialUSB.println(F("sawtooth"));
  if(lfotype==4)
    SerialUSB.println(F("square"));
  //SerialUSB.println(lfotype, DEC);
  if(func_counter==2)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Mode: "));
  if(mode==1)
    SerialUSB.println(F("normal"));
  if(mode==2)
    SerialUSB.println(F("harmonic"));
  if(mode==3)
    SerialUSB.println(F("opto"));
  if(mode==4)
    SerialUSB.println(F("opto + harmonic"));
  //SerialUSB.println(mode, DEC);
  if(func_counter==3)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Color: "));
  SerialUSB.print(colorvalue, 1);
  SerialUSB.println(F(" dB"));
  if(func_counter==4)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Volume: "));
  SerialUSB.print(volumedB, 1);
  SerialUSB.println(F(" dB"));
  if(func_counter==5)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Mix: "));
  SerialUSB.print(mix, DEC);
  SerialUSB.println(F(" %"));
  if(func_counter==6)
    SerialUSB.print(F("    "));
  SerialUSB.print(F("Depth: "));
  SerialUSB.println(depthvalue, 2);
  
  SerialUSB.write('\n');
  SerialUSB.print(F("Active item: "));
  SerialUSB.println(func_counter, DEC);
}

void setBypass(void)
{
  static uint8_t oldbypass = ON;
  
  if(oldbypass != bypass)
  {
    if(bypass == ON)
    {
      muxnoiseless(DEVICE_ADDR_7bit, BypassSelectorAddr, 2); // Bypass 
      digitalWrite(STATUS_LED, LOW);
    }
    else
    {
      muxnoiseless(DEVICE_ADDR_7bit, BypassSelectorAddr, 1); // Fx
      digitalWrite(STATUS_LED, HIGH);
    }
    oldbypass = bypass;
  }
}

void setColor(float boost)
{
  static float oldboost = 0.00;
  
  if(boost != oldboost)
  {
    color.S = 0.70;
    color.f0 = 1200.00;
    color.boost = boost;
    color.type = HighShelf;
    EQ2ndOrd(DEVICE_ADDR_7bit, ColorAddr, &color);
    
    oldboost = boost;
  }
}

void setVolume(float boostdb)
{
  static float oldboostdb = 0.00;
  float boostlinear = 0.00;
  
  if(boostdb != oldboostdb)
  {
    boostlinear = pow(10, boostdb/20.0);
    MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolAddr, boostlinear);
    
    oldboostdb = boostdb;
  }
}

/*void push1_isr(void)
{
  
}*/

/*void push2_isr(void)
{

}*/


#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <ArduinoSTL.h>
#include <ArduinoQueue.h>
#include <vector>
#include <initializer_list>
#include <SoftwareSerial.h>

#include <MassTransitRoutesData.h>

#pragma once

typedef String Line;

typedef enum State
{
  S0_IDLE,
  S1_WAIT,
  S2_BLINKING_ON,
  S3_BLINKING_OFF,
  S4_LEFT,
  S5_EXTENDED_LEFT,
  S6_LEFT_FORWARD,
  S7_FORWARD,
  S8_EXTENDED_FORWARD,
  S9_FORWARD_RIGHT,
  S10_RIGHT,
  S11_EXTENDED_RIGHT,
  S12_LEFT_RIGHT,
  S13_INHIBIT,
  S14_INHIBIT_ALL_MODE,
  S15_LEFT_FORWARD_RIGHT
};

enum Mode
{
  individual,
  left_forward_and_right,
  forward_right_and_left,
  left_right_and_forward,
  universal
};

Line receiver_make_line(uint16_t num, 
                        Terminuses_Kharkiv provenance, 
                        Terminuses_Kharkiv destination, 
                        std::initializer_list<Deviations_Kharkiv> deviations = {});
                        
void timed_automaton_run(Line line);
void do_at_leaving_hyperstate(Line line);
boolean is_present_in_set(Line line, std::vector<Line> &lines_set);
void check_old_line_departure();
boolean is_current_line_present();
void read_transit_line();
void handle_detected_line();

//---------------------------------// L  M  R
const uint8_t LEFT_LED_PIN = 24;   // _______
const uint8_t MIDDLE_LED_PIN = 23; // |o o o|
const uint8_t RIGHT_LED_PIN = 22;  //  ‾|o|‾
const uint8_t BOTTOM_LED_PIN = 25; //    ‾
const uint8_t AUX_LED_PIN = 27;    //    B

// Time constants to define duration of each phase (staying inside state)
const uint16_t t_blink_half_period = 500;
const uint16_t blinking_duration = 4000;
const uint16_t t_forward_phase = 12000;
const uint16_t t_extended_forward_phase = 7000;
const uint16_t t_turn_phase = 18000;
const uint16_t t_extended_turn_phase = 9000;
const uint16_t t_inhibit = 30000;
const uint16_t t_inhibit_all_mode = 35000;
const uint16_t t_all_directions_phase = 25000;

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <Queue.h>
#include <vector>
#include <initializer_list>
#include <stdio.h>
#include <SoftwareSerial.h>

#pragma once

typedef String Line;

typedef enum State {
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

enum Mode {
  individual,
  left_forward_and_right,
  forward_right_and_left,
  left_right_and_forward,
  universal
};

enum class Terminuses
{
    Rte_de_Lyon,                        // 1, 61, d1
    Pte_de_Loyat,                       // 1, 14, d1
    P_Curie,                            // 2
    Pole_La_Bastide,                    // 2, 6
    Montjovis,                          // 4, 39
    Pole_St_Lazare,                     // 4, 15, 24, 62, d4
    La_Cornue,                          // 5, d5
    Les_Courrieres,                     // 5, 16
    J_Gagnant,                          // 5
    Mal_Juin,                           // 6, d4
    Mal_Joffre,                         // 8, 22, d8
    Lycee_Dautry,                       // 8
    Le_Palais_Vert_Vallon,              // 8
    Cite_R_Dautry,                      // 8, d5
    L_P_St_Exupery,                     // 8
    Le_Palais_Beauregard,               // 8
    Le_Palais_Puy_Neige,                // 8
    L_Serpollet,                        // 10, 21, d10
    Ch_Le_Gendre,                       // 10, 63, 21, d10
    Isle_Les_Champs,                    // 12, 63
    Panazol_Manderesse,                 // 12, 61
    Le_Theil,                           // 14 
    College_Ronsard,                    // 14
    Lycee_Renoir,                       // 14
    Boisseuil_Z_A_La_Plaine,            // 15
    Pl_W_Churchill, // 16 17 18 20 24 25 26 31 32 34 35 36 37 38 41 44 46 EX1
    Verneuil_Pennevayre,                // 16
    Beaune,                             // 18
    Bonnac_Le_Masbatin,                 // 18
    Pole_Fougeras,                      // 18, 20, 29, 30, 65
    Fontgeaudrant,                      // 24
    Mas_Blanc,                          // 25
    Peyrilac_Baneche,                   // 26
    Limoges_Ciel,                       // 28
    Verneuil_Les_Vaseix,                // 28
    Rilhac_Rancon_Bramaud,              // 29
    Rilhac_Rancon_Cassepierre_Ecole,    // 30
    Eyjeaux_Bourg,                      // 31
    Feytiat_Mas_Gauthier,               // 32
    St_Just_Grateloube,                 // 34
    Feytiat_Plein_Bois,                 // 35
    Condat_Versanas,                    // 36
    Couzeix_La_Croix_d_Anglard,         // 37
    Couzeix_Anglard,                    // 38
    Chaptelat_Le_Theillol,              // 39
    Mas_Gigou,                          // 41
    Solignac_Bourg,                     // 44
    St_Just_Fontanguly,                 // 46
    Panazol_Mairie,                     // 61
    Feytiat_Pl_de_l_Europe,             // 62
    Z_I_Nord_3,                         // 65
    Veyrac_Bourg,                       // EX1
    Puy_Ponchet,                        // 22, d8
    DEPOT
};

enum class Lines 
{
  d1 = 66, d4, d5, d8, d10, EX1
};

enum class Deviations
{
    L_P_J_Monnet,
    ENSIL,
    P_Morand,
    J_Montalat,
    Vieux_Crezin,
    Villagory,
    Le_Sablard,
    Les_Chenes_Verts,
    Coyol,
    Ocealim,
    L_Bleriot,
    Couzeix_Anglard
};

Line receiver_make_line(uint16_t num, Terminuses provenance, Terminuses destination, std::initializer_list<Deviations> deviations = {});
void timed_automaton_run(Line line);
void do_at_leaving_hyperstate(Line line);
boolean is_present_in_set(Line line, std::vector<Line> &lines_set);
void check_old_line_departure();
boolean is_current_line_present();
void read_transit_line();
void handle_detected_line();

//-----------------------------------// L  M  R
const uint8_t LEFT_LED_PIN = 6;      // _______
const uint8_t MIDDLE_LED_PIN = 5;    // |o o o|
const uint8_t RIGHT_LED_PIN = 4;     //  ‾|o|‾
const uint8_t BOTTOM_LED_PIN = 7;    //    ‾
const uint8_t AUX_LED_PIN = 8;       //    B

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

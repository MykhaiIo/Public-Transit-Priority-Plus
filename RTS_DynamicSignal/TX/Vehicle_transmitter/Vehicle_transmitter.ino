#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoSTL.h>
#include <vector>
#include <stdio.h>
#include <initializer_list>

#define rx 2
#define tx 3

typedef String Line;

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

SoftwareSerial BTserial(rx, tx);

Line transmitter_make_line(uint16_t num, Terminuses provenance, Terminuses destination, uint32_t vehicle_num, std::initializer_list<Deviations> deviations = {});

Line transit1 = transmitter_make_line(4, Terminuses::Pole_St_Lazare, Terminuses::Montjovis, 911);
Line transit2 = transmitter_make_line(24, Terminuses::Fontgeaudrant, Terminuses::Pl_W_Churchill, 454);
Line transit3 = transmitter_make_line(36, Terminuses::Condat_Versanas, Terminuses::Pl_W_Churchill, 257);
Line transit4 = transmitter_make_line(2, Terminuses::P_Curie, Terminuses::Pole_La_Bastide, 123);
Line transit5 = transmitter_make_line(44, Terminuses::Solignac_Bourg, Terminuses::Pl_W_Churchill, 455);
Line transit_ = transmitter_make_line(35, Terminuses::Feytiat_Plein_Bois, Terminuses::Pl_W_Churchill, 233, {Deviations::Villagory});

Line route_code = transit3; // test route code to transmit

boolean allowed_to_response = 0;

void setup()
{
  pinMode(rx, INPUT);
  pinMode(tx, OUTPUT);
  Serial.begin(9600);
  // start communication with the HC-05 using 9600
  BTserial.begin(9600);
  Serial.println("BTserial started at 9600");
}

Line transmitter_make_line(const uint16_t num,
                           const Terminuses provenance,
                           const Terminuses destination,
                           const uint32_t vehicle_num,
                           std::initializer_list<Deviations> deviations = {}) {
  // only involved transit signal controllers will handle deviations transmitted
  // after certain sequence of defined values, others will ignore it
  // Vehicle number won't be analized during line-to-give-priority definition
  Line line = "{{" + String(num, DEC) + 
              "<<" + String((uint8_t)provenance, HEX) + 
              ">>" + String((uint8_t)destination, HEX) + 
              "^^";
  const Deviations *p_deviations = deviations.begin();
  while (p_deviations != deviations.end())
  {
    line += String((uint8_t)*p_deviations, HEX) + '-';
    ++p_deviations;
  }
  return line + 
         "##" + String(vehicle_num, HEX) + '}';
}

void loop()
{ 
  // Request template: ++{{NNN<<PP>>DD^^DV- ... DV-##VEHNUM}?
  static String request_data = "";
  String request = "", response = "";
  allowed_to_response = 0;
  
  while (BTserial.available() > 0) {
  char buff = BTserial.read();
  request_data.concat(buff);
    if (buff == '?') {
      request = request_data;
      request_data = "";
      allowed_to_response = 1;
    }

    Serial.println(request);

    if (route_code.substring(
                route_code.indexOf("{{"), 
                route_code.indexOf('}') + 1
                            ) == // if requested code matches to currently transmitted by vehicle
        request.substring(
              request.indexOf("++") + 2, 
              request.indexOf('?')
              )
        ) {
      response = "--" + 
                 route_code.substring(
                          route_code.indexOf("{{"), 
                          route_code.indexOf('}') + 1
                                     ) + 
                  "!;";
    }
  }

  if (allowed_to_response) {
    BTserial.print(response);
    Serial.println(response);

  } else {
    BTserial.print(route_code);
  }
  delay(100);
}

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <ArduinoSTL.h>
#include <vector>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <initializer_list>

#include <MassTransitRoutesData.h>

#define rx 3
#define tx 2

typedef String Line;

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
    {'7','8','9','🔼'},
    {'4','5','6','🔽'},
    {'1','2','3','◀'},
    {'✔','0','↩','▶'}
};

byte rowPins[ROWS] = {7, 6, 5, 4}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {8, 9, 10, 11}; //connect to the column pinouts of the keypad

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/* std::vector<Terminus> Terminuses_ {};
std::vector<Deviation> Deviations_ {};
std::map<uint16_t,Line> Lines_ {}; */

SoftwareSerial BTserial(rx, tx);

Line transmitter_make_line(uint16_t num, Terminuses_Kharkiv provenance, Terminuses_Kharkiv destination, uint32_t vehicle_num, std::initializer_list<Deviations_Kharkiv> deviations = {});
void get_line_info();

const uint16_t _16A = (uint16_t)Lines_Kharkiv::_16A;

Line transit1 = transmitter_make_line(16,   Terminuses_Kharkiv::Saltivska,             Terminuses_Kharkiv::Saltivska,       4553);
Line transit2 = transmitter_make_line(_16A, Terminuses_Kharkiv::Saltivska,             Terminuses_Kharkiv::Saltivska,       704);
Line transit3 = transmitter_make_line(23,   Terminuses_Kharkiv::Saltivska,             Terminuses_Kharkiv::Pivdenno_Shidna, 8023);
Line transit4 = transmitter_make_line(27,   Terminuses_Kharkiv::Saltivska,             Terminuses_Kharkiv::Novozhanove,     575);
Line transit5 = transmitter_make_line(8,    Terminuses_Kharkiv::Saltivske_tram_depot,  Terminuses_Kharkiv::mikrorayon_602,  3016);
Line transit6 = transmitter_make_line(26,   Terminuses_Kharkiv::Saltivske_tram_depot,  Terminuses_Kharkiv::Lisopark,        707, {Deviations_Kharkiv::Horkyi_park});

Line route_code = transit1; // test route code to transmit

boolean allowed_to_response = 0;

void setup()
{
  pinMode(rx, INPUT_PULLUP);
  pinMode(tx, OUTPUT);
  Serial.begin(9600);
  // start communication with the HC-05 using 9600
  BTserial.begin(38400);
  Serial.println("BTserial started at 57600");
  lcd.begin(20, 2);
  
  /*
  if (fLines != NULL && fTerminuses != NULL && fDeviations != NULL) {
    char *num_lines, *s_line, *s_term, *s_dev;
    uint16_t i = 0;
    fgets(num_lines, 5, fLines);

    while (fgets(s_line, 10, fLines)) {
      Lines_[atoi(num_lines)+1+i] = Line(s_line);
      ++i;
    }
    i = 0;
    while (fgets(s_term, 100, fTerminuses)) {
      Terminuses_[i] = Terminus(s_term);
      ++i;
    }
    i = 0;
    while (fgets(s_term, 100, fDeviations)) {
      Deviations_[i] = Deviation(s_dev);
      ++i;
    }
    fclose(fLines);
    fclose(fTerminuses);
    fclose(fDeviations);
  }
  else Serial.println("Unable to open files");  */
}

void get_line_info() {
  lcd.print("Enter line number");
  lcd.setCursor(0, 1);
}

Line transmitter_make_line(const uint16_t num,
                           const Terminuses_Kharkiv provenance,
                           const Terminuses_Kharkiv destination,
                           const uint32_t vehicle_num,
                           std::initializer_list<Deviations_Kharkiv> deviations = {}) {
  // only involved transit signal controllers will handle deviations transmitted
  // after certain sequence of defined values, others will ignore it
  // Vehicle number won't be analized during line-to-give-priority definition
  Line line = "{{" + String(num, DEC) + 
              "<<" + String((uint16_t)provenance, HEX) + 
              ">>" + String((uint16_t)destination, HEX) + 
              "^^";
  const Deviations_Kharkiv *p_deviations = deviations.begin();
  while (p_deviations != deviations.end()) {
    line += String((uint8_t)*p_deviations, HEX) + '-';
    ++p_deviations;
  }
  line = line + 
         "##" + String(vehicle_num, HEX) + '}';

  /* while ((line.length()*sizeof(char)) % 16 != 0) { // block size must be 128 bit i.e. 16 bytes
    line += char(rand() % 255);
  }*/
  return line;  
}

void loop()
{ 
  // get_line_info();
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

    Serial.print(millis() / 1000);
    Serial.print(" s -> ");
    Serial.print(" request is: ");
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

      // while ((response.length()*sizeof(char)) % 16 != 0) {
      //   response += char(rand() % 255);
      // }
    }
  }

  if (allowed_to_response) {
    BTserial.print(response);
    BTserial.flush();
    Serial.print(millis() / 1000);
    Serial.print(" s -> ");
    Serial.print(" response is: ");
    Serial.println(response);

  } else {
    BTserial.print(route_code);
    BTserial.flush();
    Serial.print(millis() / 1000);
    Serial.print(" s -> ");
    Serial.print(" current line is: ");
    Serial.println(route_code);
  }
  delay(700);
}

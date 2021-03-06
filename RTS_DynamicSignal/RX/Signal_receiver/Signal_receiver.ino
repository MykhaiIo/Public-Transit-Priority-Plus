/* Signal_receiver.ino file created by Mykhailo Lytvynenko
 *
 * Created:   Wed Mar 10 2020
 * Processor: Arduino Uno
 *
 * Active START button level is LOW
 *
 * Modes of operation that enables specified directions are following:
 * individual:
 *          Phase 1 - { > }
 *          Phase 2 - { ^ }
 *          Phase 3 - { < }
 *
 * left_forward_and_right:
 *          Phase 1 - { <, ^ }
 *          Phase 2 - { > }
 *
 * forward_right_and_left:
 *          Phase 1 - { ^, > }
 *          Phase 2 - { < }
 *
 * left_right_and_forward:
 *          Phase 1 - { <, > }
 *          Phase 2 - { ^ }
 *
 * universal:
 *          Phase - { <, ^, > }
 * This mode is completely timed and does not react to transit except phase
 * extension condition checking.
 *
 * Extension of phase allows for transit to pass intrsection just after previous
 * transit if needed direction is subset of current direction(s) e.g. { <, ^}
 * can be extended to { < } as well as to { ^ }  but not to { > }. Similary
 * { > } can be extended to { > } only.
 *
 *
 * States S2 and S3 are inside so-called hyperstate and are used in all modes
 * except "universal". The purpose is to indicate that transit signal controller
 * is going to enable necessary phase for first waiting transit in the queue.
 *
 * Transit departure, after enabling necessary phase and related timeout, is checked
 * using check_old_line_departure() function before transition to the next state analysis
 */

#include "Signal_receiver.h"

#define rx 15
#define tx 14

#define START_BTN 26

// SoftwareSerial BTserial(rx,tx);

// Timers to leave state after specified time
unsigned long state_timer = 0;
unsigned long hyperstate_timer = 0;

unsigned long finish_at = 0;

// Sets of used lines (from enum Lines) for the specific intersection
// Lines are splitted by directions where transit need to go
const std::vector<Line> left_lines{};

const uint16_t _16A = (uint16_t)Lines_Kharkiv::_16A;

const std::vector<Line> forward_lines{
    receiver_make_line(23, Terminuses_Kharkiv::Saltivska, Terminuses_Kharkiv::Pivdenno_Shidna),
    receiver_make_line(26, Terminuses_Kharkiv::Saltivske_tram_depot, Terminuses_Kharkiv::Pivdenno_Shidna),
    receiver_make_line(8, Terminuses_Kharkiv::Saltivske_tram_depot, Terminuses_Kharkiv::mikrorayon_602),
};

const std::vector<Line> right_lines{
    receiver_make_line(16, Terminuses_Kharkiv::Saltivska, Terminuses_Kharkiv::Saltivska),
    receiver_make_line(_16A, Terminuses_Kharkiv::Saltivska, Terminuses_Kharkiv::Saltivska),
    receiver_make_line(26, Terminuses_Kharkiv::Saltivske_tram_depot, Terminuses_Kharkiv::Lisopark),
    receiver_make_line(27, Terminuses_Kharkiv::Saltivska, Terminuses_Kharkiv::Novozhanove),
};

// Queue to store waiting transit
ArduinoQueue<Line> q_lines(3);

typedef struct {
  uint8_t mode : 3;

  boolean phase_extension_left : 1;    // 1 enables left extension (XL on state diagram)

  boolean phase_extension_forward : 1; // 1 enables forward extension (XF on state diagram)

  boolean phase_extension_right : 1;   // 1 enables right extended (XR on state diagram)

  boolean deviations_handled : 1; // 0 makes system ignore deviations in route's code
} input_config;

boolean Start = 0; // configuration part

boolean allowed_to_form_line = 0;
boolean allowed_to_handle_response = 0;

input_config system_config = {
    universal, 0, 0, 0, 0 // defines operation mode of transit signal controller
    };

State state = S0_IDLE;

/* detected and old lines template: {{NNN<<PP>>DD^^DV- ... DV-##VEHNUM}, where
*  NNN — line number (2 bytes, DEC)
*  PP — provenance (2 bytes, HEX)
*  DD — destination (2 bytes, HEX)
*  DV — deviation (1 byte, HEX, optional)
*  VEHNUM — vehicle number (4 byte, HEX)
*  These data is received directly from vehicle transmitter. */
Line old_transit_line = ""; // previous received line from BTserial (with vehicle num)
Line detected_transit_line = ""; // new received line from BTserial (with vehicle num)

/* current line and line_to_enqueue template: NNN<<PP>>DD^^DV- ... DV-#\n
*  Vehicle number is ommited as it is used only during adding to the queue analysis and 
*  cannot be known during system configuration by lines sets initialization
*  These data is formed after detecting line, splitting its data 
*  and reassembling it in line_to_enqueue variable. */
Line current_transit_line = ""; // first transit line from the queue (without vehicle num)

void setup() {
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(MIDDLE_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  pinMode(BOTTOM_LED_PIN, OUTPUT);
  pinMode(AUX_LED_PIN, OUTPUT);

  pinMode(START_BTN, INPUT_PULLUP);

  pinMode(rx, INPUT_PULLUP);
  pinMode(tx, OUTPUT);
  Serial.begin(9600);
  Serial.println("Arduino with HC-06 is ready");
 
  // start communication with the HC-06 using 9600
  Serial3.begin(38400);  
  Serial.println("BTserial started at 38400");

  state_timer = millis(); // initial state_timer initialization
}

Line receiver_make_line(const uint16_t num,
                        const Terminuses_Kharkiv provenance,
                        const Terminuses_Kharkiv destination,
                        std::initializer_list<Deviations_Kharkiv> deviations = {}) {
  // only involved signals will handle deviations transmitted
  // after certain sequence of defined values, others will ignore it
  Line line = String(num, DEC) + 
              "<<" + String((uint16_t)provenance, HEX) + 
              ">>" + String((uint16_t)destination, HEX) + 
              "^^";
  const Deviations_Kharkiv *p_deviations = deviations.begin();
  while (p_deviations != deviations.end())
  {
    line += String((uint8_t)*p_deviations, HEX) + '-';
    ++p_deviations;
  }
  return line + "#\n";
}

void read_transit_line() {
  if (!q_lines.isEmpty()) {
    current_transit_line = q_lines.front(); // get first wating transit in queue
  } else {
    current_transit_line = "";
  }  
}

void handle_detected_line() {
  static String line_data = "";
  while (Serial3.available() > 0) {
    char buff = Serial3.read();
    if (buff == '{') { // start of route code frame
      allowed_to_form_line = 1;
    }

    if (allowed_to_form_line) {
      line_data.concat(buff);
      if (buff == '}') { // end of route code frame
        allowed_to_form_line = 0;
        detected_transit_line = line_data;
        line_data = "";
      }
    }
    
    Line line_to_enqueue = "";

    String number, provenance, destination, deviations;
    
    // proceed to new line analysis if it is different vehicle detected
    if (detected_transit_line != old_transit_line) {
      // check validity of detected line data by verifying presence of any number between "{{" and "<<"
      if ((
            detected_transit_line.indexOf("<<") - detected_transit_line.indexOf("{{") + 2
          ) > 2) {
        number = detected_transit_line.substring(
                                      detected_transit_line.indexOf("{{") + 2, 
                                      detected_transit_line.indexOf("<<")
                                                );
        provenance = detected_transit_line.substring(
                                          detected_transit_line.indexOf("<<") + 2, 
                                          detected_transit_line.indexOf(">>")
                                                    );
        destination = detected_transit_line.substring(
                                          detected_transit_line.indexOf(">>") + 2, 
                                          detected_transit_line.indexOf("^^")
                                                     );
        deviations = detected_transit_line.substring(
                                          detected_transit_line.indexOf("^^") + 2, 
                                          detected_transit_line.indexOf('#')
                                                    );
        line_to_enqueue = number + "<<" + 
                          provenance + ">>" + 
                          destination + "^^";
        if (system_config.deviations_handled) {
          line_to_enqueue.concat(deviations + "#\n");
        } else {
          line_to_enqueue.concat("#\n");
        }
        // prevent irrelevant transit lines adding to queue
        if (is_present_in_set(line_to_enqueue, left_lines) |
            is_present_in_set(line_to_enqueue, forward_lines) |
            is_present_in_set(line_to_enqueue, right_lines)) {
          q_lines.enqueue(line_to_enqueue);
          Serial.print(millis() / 1000);
          Serial.print(" s -> ");
          Serial.print(line_to_enqueue);
          Serial.println(" PUSHED");
          Serial.print("Lines in queue: ");
          Serial.println(q_lines.item_count());
          Serial.print("Current state is: S");
          Serial.print(state);
          Serial.println(" ");
          old_transit_line = detected_transit_line;
        }
      }
    }
  }
}

boolean is_present_in_set(Line line,
                            const std::vector<Line> &lines_set) {
  for (uint8_t line_no = 0; line_no < lines_set.size(); ++line_no) {
    // OR operation of line and each set element comparison
    if (line == lines_set[line_no]) {
      return 1;
    }
  }
  return 0;
}

boolean is_current_line_present() {
  String request = "++" + detected_transit_line + '?';
  Serial.print("Request: ");
  Serial3.print(request);
  Serial.println(request);
  Serial3.flush();


  delay(200);

  // Response template: --{{NNN<<PP>>DD^^DV- ... DV-##VEHNUM}!;
  String response = "";
  static String response_data = "";

  while (Serial3.available() > 0) {
    char buff = Serial3.read();
    if(buff == '-') { // begin to handle response if start char matches
      allowed_to_handle_response = 1;
    }
    if(allowed_to_handle_response) {
      response_data = Serial3.readStringUntil(';'); // to trigger timeout procedure
      response = response_data;
      response_data = "";
      allowed_to_handle_response = 0;
    }
  }

    Serial.print("Response: ");
    Serial.println(response);

  if (
        (response.substring(
                response.indexOf('-') + 1, 
                response.indexOf('!')
                           ) == // if response's line matches to currently interacting
        detected_transit_line.substring(
                            detected_transit_line.indexOf("{{"), 
                            detected_transit_line.indexOf('}') + 1
                                       )
        ) && // and this line is only one to be currently detected
          q_lines.item_count() < 2
      ) {
         Serial.println("Vehicle is still on intersection");
        return 1;
  } else { // if response not detected (departure) or next line detected (previous is disconnected == departure)
        Serial.println("Vehicle has cleared intersection");
        return 0;
  }
  
}

void check_old_line_departure() {
  Serial.print(millis() / 1000);
  Serial.print(" s -> ");
  if(!is_current_line_present()) {
    Serial.print(current_transit_line);
    Serial.println(" SERVED");

    if (!q_lines.isEmpty()) {
      q_lines.dequeue();
      Serial.print("Waiting is ");
      Serial.println(q_lines.front());
    }
  }
  Serial.print("Lines in queue: ");
  Serial.println(q_lines.item_count());
  Serial.print("Current state is: S");
  Serial.print(state);
  Serial.println(" ");
  read_transit_line();
}

void timed_automaton_run(const Line line) {
  switch (state) {
  case S0_IDLE:
    // after entering in idle state all stored lines leave the system
    while (!q_lines.isEmpty()) {
      q_lines.dequeue();
    }

    if (!Start) {
      state = S0_IDLE;
    } else if (system_config.mode == universal) {
      state = S14_INHIBIT_ALL_MODE;
      Serial.println("Current mode is { <, ^, > }");

      system_config.phase_extension_left
          ? Serial.println("Left phase extension is on")
          : Serial.println("Left phase extension is off");
      system_config.phase_extension_forward
          ? Serial.println("Forward phase extension is on")
          : Serial.println("Forward phase extension is off");
      system_config.phase_extension_right
          ? Serial.println("Right phase extension is on")
          : Serial.println("Right phase extension is off");
      system_config.deviations_handled
          ? Serial.println("Deviations are handled")
          : Serial.println("Deviations are not handled");

      Serial.println(" ");
    } else {
      state = S1_WAIT;
      switch (system_config.mode) {
      case individual:
        Serial.println("Current mode is { < } { ^ } { > }");
        break;
      case left_forward_and_right:
        Serial.println("Current mode is { <, ^ } { > }");
        break;
      case forward_right_and_left:
        Serial.println("Current mode is { ^, > } { < }");
        break;
      case left_right_and_forward:
        Serial.println("Current mode is { <, > } { ^ }");
        break;
      case universal:                                  //
        Serial.println("Current mode is { <, ^, > }"); // nonsense, just to cover enum
        break;
      }

      system_config.phase_extension_left
          ? Serial.println("Left phase extension is on")
          : Serial.println("Left phase extension is off");
      system_config.phase_extension_forward
          ? Serial.println("Forward phase extension is on")
          : Serial.println("Forward phase extension is off");
      system_config.phase_extension_right
          ? Serial.println("Right phase extension is on")
          : Serial.println("Right phase extension is off");
      system_config.deviations_handled
          ? Serial.println("Deviations are handled")
          : Serial.println("Deviations are not handled");

      Serial.println(" ");
    }
    break;

  case S1_WAIT:
    if (!Start) {
      state = S0_IDLE;
    } else if (is_present_in_set(line, left_lines) |
               is_present_in_set(line, forward_lines) |
               is_present_in_set(line, right_lines)) {
      state = S2_BLINKING_ON;
      finish_at =
          millis() + blinking_duration; // define moment of stopping blinking
      hyperstate_timer = millis();
    } else {
      state = S1_WAIT;
    }
    state_timer = millis(); // re-initialize state_timer for next state
    break;

  case S2_BLINKING_ON:
    if (millis() < finish_at) {
      unsigned long currentMillis = millis();
      if (currentMillis - hyperstate_timer >= t_blink_half_period) {
        hyperstate_timer = currentMillis;
        state = S3_BLINKING_OFF;
      }
    }

    // cheking for ellapsed time after entering state (non-blocking delay)
    if (millis() - state_timer >= blinking_duration) {
      do_at_leaving_hyperstate(line);
    }
    break;

  case S3_BLINKING_OFF:
    if (millis() < finish_at) {
      unsigned long currentMillis = millis();
      if (currentMillis - hyperstate_timer >= t_blink_half_period) {
        hyperstate_timer = currentMillis;
        state = S2_BLINKING_ON;
      }
    }

    if (millis() - state_timer >= blinking_duration) {
      do_at_leaving_hyperstate(line);
    }
    break;

  case S4_LEFT:
    if (millis() - state_timer >= t_turn_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, left_lines) & 
          system_config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S5_EXTENDED_LEFT:
    if (millis() - state_timer >= t_extended_turn_phase) {
      check_old_line_departure();
      if (system_config.mode == universal) {
        state = S14_INHIBIT_ALL_MODE;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S6_LEFT_FORWARD:
    if (millis() - state_timer >= t_turn_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, left_lines) & 
          system_config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
      } else if (is_present_in_set(line, forward_lines) &
                 system_config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S7_FORWARD:
    if (millis() - state_timer >= t_forward_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, forward_lines) &
          system_config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S8_EXTENDED_FORWARD:
    if (millis() - state_timer >= t_extended_forward_phase) {
      check_old_line_departure();
      if (system_config.mode == universal) {
        state = S14_INHIBIT_ALL_MODE;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S9_FORWARD_RIGHT:
    if (millis() - state_timer >= t_turn_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, forward_lines) &
          system_config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
      } else if (is_present_in_set(line, right_lines) &
                 system_config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S10_RIGHT:
    if (millis() - state_timer >= t_turn_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, right_lines) &
          system_config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S11_EXTENDED_RIGHT:
    if (millis() - state_timer >= t_extended_turn_phase) {
      check_old_line_departure();
      if (system_config.mode == universal) {
        state = S14_INHIBIT_ALL_MODE;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S12_LEFT_RIGHT:
    if (millis() - state_timer >= t_turn_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, left_lines) & 
          system_config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
      } else if (is_present_in_set(line, right_lines) &
                 system_config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S13_INHIBIT:
    if (millis() - state_timer >= t_inhibit) {
      state = S1_WAIT;
      state_timer = millis();
    }
    break;

  case S14_INHIBIT_ALL_MODE:
    if (millis() - state_timer >= t_inhibit_all_mode) {
      if (Start == 0) {
        state = S0_IDLE;
      } else {
        state = S15_LEFT_FORWARD_RIGHT;
      }
      state_timer = millis();
    }
    break;

  case S15_LEFT_FORWARD_RIGHT:
    if (millis() - state_timer >= t_all_directions_phase) {
      check_old_line_departure();
      if (is_present_in_set(line, left_lines) & 
          system_config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
      } else if (is_present_in_set(line, forward_lines) &
                 system_config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
      } else if (is_present_in_set(line, right_lines) &
                 system_config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
      } else {
        state = S14_INHIBIT_ALL_MODE;
      }
      state_timer = millis();
    }
    break;

  default:
    state = S0_IDLE;
  }
}

void do_at_leaving_hyperstate(const Line line) {
  if (is_present_in_set(line, left_lines) &
      ((system_config.mode == forward_right_and_left) | 
       (system_config.mode == individual))) {
    state = S4_LEFT;
  } else if ((is_present_in_set(line, left_lines) |
              is_present_in_set(line, forward_lines)) &
             (system_config.mode == left_forward_and_right)) {
    state = S6_LEFT_FORWARD;
  } else if (is_present_in_set(line, forward_lines) &
             ((system_config.mode == left_right_and_forward) ||
              (system_config.mode == individual))) {
    state = S7_FORWARD;
  } else if ((is_present_in_set(line, forward_lines) |
              is_present_in_set(line, right_lines)) &
             (system_config.mode == forward_right_and_left)) {
    state = S9_FORWARD_RIGHT;
  } else if (is_present_in_set(line, right_lines) &
             ((system_config.mode == left_forward_and_right) |
              (system_config.mode == individual))) {
    state = S10_RIGHT;
  } else if ((is_present_in_set(line, left_lines) |
              is_present_in_set(line, right_lines)) &
             (system_config.mode == left_right_and_forward)) {
    state = S12_LEFT_RIGHT;
  } else {
    state = S1_WAIT;
  }
  state_timer = millis();
}

void loop() {
  if (digitalRead(START_BTN) == LOW) {
    delay(10);
    if (digitalRead(START_BTN) == LOW) {
      Start = 1;
    }
  } else {
    Start = 0;
  }

  delay(10);

  handle_detected_line();

  delay(10);

  read_transit_line();

  delay(10);

//  Serial.print("Current state is: S");
//  Serial.println(state);
 
  delay(10);

  timed_automaton_run(current_transit_line);

  delay(10);

  digitalWrite(LEFT_LED_PIN,
               (state == S1_WAIT || state == S2_BLINKING_ON ||
                state == S3_BLINKING_OFF || state == S4_LEFT ||
                state == S5_EXTENDED_LEFT || state == S6_LEFT_FORWARD ||
                state == S12_LEFT_RIGHT || state == S13_INHIBIT ||
                state == S14_INHIBIT_ALL_MODE ||
                state == S15_LEFT_FORWARD_RIGHT)
                   ? HIGH
                   : LOW);

  digitalWrite(MIDDLE_LED_PIN,
               (state == S1_WAIT || state == S2_BLINKING_ON ||
                state == S3_BLINKING_OFF || state == S6_LEFT_FORWARD ||
                state == S7_FORWARD || state == S8_EXTENDED_FORWARD ||
                state == S9_FORWARD_RIGHT || state == S13_INHIBIT ||
                state == S14_INHIBIT_ALL_MODE ||
                state == S15_LEFT_FORWARD_RIGHT)
                   ? HIGH
                   : LOW);

  digitalWrite(RIGHT_LED_PIN,
               (state == S1_WAIT || state == S2_BLINKING_ON ||
                state == S3_BLINKING_OFF || state == S9_FORWARD_RIGHT ||
                state == S10_RIGHT || state == S11_EXTENDED_RIGHT ||
                state == S12_LEFT_RIGHT || state == S13_INHIBIT ||
                state == S14_INHIBIT_ALL_MODE ||
                state == S15_LEFT_FORWARD_RIGHT)
                   ? HIGH
                   : LOW);

  digitalWrite(BOTTOM_LED_PIN,
               (state == S2_BLINKING_ON || state == S4_LEFT ||
                state == S5_EXTENDED_LEFT || state == S6_LEFT_FORWARD ||
                state == S7_FORWARD || state == S8_EXTENDED_FORWARD ||
                state == S9_FORWARD_RIGHT || state == S10_RIGHT ||
                state == S11_EXTENDED_RIGHT || state == S12_LEFT_RIGHT ||
                state == S15_LEFT_FORWARD_RIGHT)
                   ? HIGH
                   : LOW);

  digitalWrite(AUX_LED_PIN, state == S0_IDLE ? HIGH : LOW);

  delay(10);
}
/* RTS_DynamicTransitSignal.ino file created by Mykhailo Lytvynenko
 *
 * Created:   Wed Nov 13 2019
 * Processor: Arduino Uno
 * Compiler:  Arduino AVR (Proteus)
 *
 * Active buttons level is LOW
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
 * Transit is supposed to leave intersecrion after enabling necessary phase
 */

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <Queue.h>
#include <stdarg.h>
#include <map>
#include <vector>


#define START_BTN 12
#define LINE_16_BTN 11
#define LINE_15_BTN 10
#define LINE_26_BTN 9

const uint8_t LEFT_LED_PIN = 6;
const uint8_t MIDDLE_LED_PIN = 5;
const uint8_t RIGHT_LED_PIN = 4;
const uint8_t BOTTOM_LED_PIN = 7;
const uint8_t AUX_LED_PIN = 8;

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

// Timers to leave state after specified time
unsigned long state_timer = 0;
unsigned long hyperstate_timer = 0;

unsigned long finish_at = 0;

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

enum Line {
  L1 = 1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12, L13, L14, L15, L16, 
  L17, L18, L19, L20, L21, L22, L23, L24, L25, L26, L27, L28, L29, L30, L31,
  L32, L33, L34, L35, L36, L37, L38, L39, L40, L41, L42, L43, L44, L45, L46,
  L47, L48, L49, L50, L51, L52, L53, L54, L55, L56, L57, L58, L59, L60, L61,
  L62, L63, L64, L65, L66, L67, L68, L69, L70, L71, L72, L73, L74, L75, L76,
  L77, L78, L79, L80, L81, L82, L83, L84, L85, L86, L87, L88, L89, L90, L91,
  L92, L93, L94, L95, L96, L97, L98, L99, L100, L101, L102, L103, L104, L105
};

// Sets of used lines (from enum Line) for the specific intersection
// Lines are splitted by directions where transit need to go
const std::vector<uint16_t> left_lines{L16};
const std::vector<uint16_t> forward_lines{L15};
const std::vector<uint16_t> right_lines{L26};

// Queue to store all waiting transit
DataQueue<uint16_t> q_lines(10);

typedef struct {
  uint8_t mode : 3;
  boolean phase_extension_left : 1;    // 1 enables left extension (XL on state
                                       // diagram)
  boolean phase_extension_forward : 1; // 1 enables forward extension (XF on
                                       // state diagram)
  boolean phase_extension_right : 1;   // 1 enables right extended (XR on state
                                       // diagram)
} input_config;

boolean Start = 0;

input_config transit_signal_mode = {
    left_right_and_forward, 1, 0,
    1}; // defines operation mode of transit signal controller

void timed_automaton_run(input_config mode, uint16_t line);
void do_at_leaving_hyperstate(input_config config, uint16_t line);
boolean check_line_to_serve(uint16_t line, std::vector<uint16_t> &lines_set);
void serve_transit_line(uint16_t line);
void read_transit_line();
void add_line_to_queue(uint8_t line_btn_pin, uint16_t line);
void print_transit_signal_mode();

State state = S0_IDLE;
uint16_t current_transit_line = 0; // first transit line from the queue
uint8_t line_btn_pin_to_analize = 0;

void setup() {
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(MIDDLE_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  pinMode(BOTTOM_LED_PIN, OUTPUT);
  pinMode(AUX_LED_PIN, OUTPUT);

  pinMode(LINE_15_BTN, INPUT_PULLUP);
  pinMode(LINE_16_BTN, INPUT_PULLUP);
  pinMode(LINE_26_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);

  state_timer = millis(); // initial state_timer initialization
  Serial.begin(2400);
}

void read_transit_line() {
  if (!q_lines.isEmpty()) {
    current_transit_line = q_lines.front(); // get first wating transit in queue
  } else {
    current_transit_line = 0;
  }  
}

void add_line_to_queue(uint8_t line_btn_pin, uint16_t line_no) {
  static std::map<uint8_t, uint32_t> cnt = {
      {LINE_15_BTN, 0}, {LINE_16_BTN, 0}, {LINE_16_BTN, 0}};
  if (digitalRead(line_btn_pin) == LOW) {
    delay(10);
    if (digitalRead(line_btn_pin) == LOW && cnt[line_btn_pin] < 1) {
      q_lines.enqueue(line_no);
      Serial.print(millis() / 1000);
      Serial.print(" s -> ");
      Serial.print("L");
      Serial.print(line_no);
      Serial.println(" PUSHED");
      Serial.print("Lines in queue: ");
      Serial.println(q_lines.item_count());
      Serial.print("Current state is: S");
      Serial.println(state);
      Serial.println(" ");
      cnt[line_btn_pin]++;
    }
  } else if (digitalRead(line_btn_pin) == HIGH) {
    cnt[line_btn_pin] = 0;
  }
}

boolean check_line_to_serve(uint16_t line,
                            const std::vector<uint16_t> &lines_set) {
  for (size_t line_no = 0; line_no < lines_set.size(); ++line_no) {
    if (line == lines_set[line_no]) {
      return 1;
    }
  }
  return 0;
}

void serve_transit_line(const uint16_t line) {
  if (!q_lines.isEmpty()) {
    if (line == q_lines.front()) {
      current_transit_line = q_lines.dequeue();
      Serial.print(millis() / 1000);
      Serial.print(" s -> ");
      Serial.print("L");
      Serial.print(line);
      Serial.println(" SERVED");
      Serial.print("Lines in queue: ");
      Serial.println(q_lines.item_count());
      if (!q_lines.isEmpty()) {
        Serial.print("Waiting is L");
        Serial.println(q_lines.front());
      }
      Serial.print("Current state is: S");
      Serial.println(state);
      Serial.println(" ");
    }
  }
}

void timed_automaton_run(const input_config config,
                                    const uint16_t line) {
  switch (state) {
  case S0_IDLE:
    // after entering in idle state universal stored lines leave computer system
    while (!q_lines.isEmpty()) {
      q_lines.dequeue();
    }

    if (!Start) {
      state = S0_IDLE;
    } else if (config.mode == universal) {
      state = S14_INHIBIT_ALL_MODE;
      Serial.println("Current mode is { <, ^, > }");

      transit_signal_mode.phase_extension_left
          ? Serial.println("Left phase extension is on")
          : Serial.println("Left phase extension is off");
      transit_signal_mode.phase_extension_forward
          ? Serial.println("Forward phase extension is on")
          : Serial.println("Forward phase extension is off");
      transit_signal_mode.phase_extension_right
          ? Serial.println("Right phase extension is on")
          : Serial.println("Right phase extension is off");

      Serial.println(" ");
    } else {
      state = S1_WAIT;
      switch (transit_signal_mode.mode) {
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
      case universal:
        Serial.println("Current mode is { <, ^, > }");
        break;
      }

      transit_signal_mode.phase_extension_left
          ? Serial.println("Left phase extension is on")
          : Serial.println("Left phase extension is off");
      transit_signal_mode.phase_extension_forward
          ? Serial.println("Forward phase extension is on")
          : Serial.println("Forward phase extension is off");
      transit_signal_mode.phase_extension_right
          ? Serial.println("Right phase extension is on")
          : Serial.println("Right phase extension is off");

      Serial.println(" ");
    }
    break;

  case S1_WAIT:
    if (!Start) {
      state = S0_IDLE;
    } else if (check_line_to_serve(line, left_lines) |
               check_line_to_serve(line, forward_lines) |
               check_line_to_serve(line, right_lines)) {
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
      do_at_leaving_hyperstate(config, line);
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
      do_at_leaving_hyperstate(config, line);
    }
    break;

  case S4_LEFT:
    if (millis() - state_timer >= t_turn_phase) {
      if (check_line_to_serve(line, left_lines) & config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
        serve_transit_line(line);
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S5_EXTENDED_LEFT:
    if (millis() - state_timer >= t_extended_turn_phase) {
      if (config.mode == universal) {
        state = S14_INHIBIT_ALL_MODE;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S6_LEFT_FORWARD:
    if (millis() - state_timer >= t_turn_phase) {
      if (check_line_to_serve(line, left_lines) & config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
        serve_transit_line(line);
      } else if (check_line_to_serve(line, forward_lines) &
                 config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
        serve_transit_line(line);
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S7_FORWARD:
    if (millis() - state_timer >= t_forward_phase) {
      if (check_line_to_serve(line, forward_lines) &
          config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
        serve_transit_line(line);
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S8_EXTENDED_FORWARD:
    if (millis() - state_timer >= t_extended_forward_phase) {
      if (config.mode == universal) {
        state = S14_INHIBIT_ALL_MODE;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S9_FORWARD_RIGHT:
    if (millis() - state_timer >= t_turn_phase) {
      if (check_line_to_serve(line, forward_lines) &
          config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
        serve_transit_line(line);
      } else if (check_line_to_serve(line, right_lines) &
                 config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
        serve_transit_line(line);
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S10_RIGHT:
    if (millis() - state_timer >= t_turn_phase) {
      if (check_line_to_serve(line, right_lines) &
          config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
        serve_transit_line(line);
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S11_EXTENDED_RIGHT:
    if (millis() - state_timer >= t_extended_turn_phase) {
      if (config.mode == universal) {
        state = S14_INHIBIT_ALL_MODE;
      } else {
        state = S13_INHIBIT;
      }
      state_timer = millis();
    }
    break;

  case S12_LEFT_RIGHT:
    if (millis() - state_timer >= t_turn_phase) {
      if (check_line_to_serve(line, left_lines) & config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
        serve_transit_line(line);
      } else if (check_line_to_serve(line, right_lines) &
                 config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
        serve_transit_line(line);
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
        serve_transit_line(line);
      }
      state_timer = millis();
    }
    break;

  case S15_LEFT_FORWARD_RIGHT:
    if (millis() - state_timer >= t_all_directions_phase) {
      if (check_line_to_serve(line, left_lines) & config.phase_extension_left) {
        state = S5_EXTENDED_LEFT;
        serve_transit_line(line);
      } else if (check_line_to_serve(line, forward_lines) &
                 config.phase_extension_forward) {
        state = S8_EXTENDED_FORWARD;
        serve_transit_line(line);
      } else if (check_line_to_serve(line, right_lines) &
                 config.phase_extension_right) {
        state = S11_EXTENDED_RIGHT;
        serve_transit_line(line);
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

void do_at_leaving_hyperstate(const input_config config, const uint16_t line) {
  if (check_line_to_serve(line, left_lines) &
      ((config.mode == forward_right_and_left) | (config.mode == individual))) {
    state = S4_LEFT;
    serve_transit_line(line);
  } else if ((check_line_to_serve(line, left_lines) |
              check_line_to_serve(line, forward_lines)) &
             (config.mode == left_forward_and_right)) {
    state = S6_LEFT_FORWARD;
    serve_transit_line(line);
  } else if (check_line_to_serve(line, forward_lines) &
             ((config.mode == left_right_and_forward) ||
              (config.mode == individual))) {
    state = S7_FORWARD;
    serve_transit_line(line);
  } else if ((check_line_to_serve(line, forward_lines) |
              check_line_to_serve(line, right_lines)) &
             (config.mode == forward_right_and_left)) {
    state = S9_FORWARD_RIGHT;
    serve_transit_line(line);
  } else if (check_line_to_serve(line, right_lines) &
             ((config.mode == left_forward_and_right) |
              (config.mode == individual))) {
    state = S10_RIGHT;
    serve_transit_line(line);
  } else if ((check_line_to_serve(line, left_lines) |
              check_line_to_serve(line, right_lines)) &
             (config.mode == left_right_and_forward)) {
    state = S12_LEFT_RIGHT;
    serve_transit_line(line);
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

  read_transit_line();

  delay(10);

  add_line_to_queue(LINE_15_BTN, L15);
  add_line_to_queue(LINE_16_BTN, L16);
  add_line_to_queue(LINE_26_BTN, L26);
 
  delay(10);

  timed_automaton_run(transit_signal_mode, current_transit_line);

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
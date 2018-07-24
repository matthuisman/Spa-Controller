#include <Arduino.h>

// D0	IO, Pull-down                   GPIO16   (MUST BE OUTPUT)
// D1	IO, SCL                         GPIO5
// D2	IO, SDA                         GPIO4
// D3	IO, 10k Pull-up                 GPIO0    (MUST BE OUTPUT)
// D4	IO, 10k Pull-up, BUILTIN_LED    GPIO2    (MUST BE OUTPUT)
// D5	IO, SCK                         GPIO14
// D6	IO, MISO                        GPIO12
// D7	IO, MOSI                        GPIO13
// D8	IO, 10k Pull-down, SS           GPIO15   (MUST BE OUTPUT)

#define HI_LIMIT_MAX  450   // (RANGE 200 => 900) (higher = hotter)

//INPUT PINS
#define HI_LIMIT       A0  // GRAY
#define LIGHT_REQUEST  D1  // ORANGE?
#define JETS_REQUEST   D2  // BROWN?
#define HEAT_REQUEST   D5  // GREEN
#define FLOW_OK        D6  // PURPLE

//OUTPUT PINS
#define LIGHT_GO       D3  // WHITE
#define JETS_GO        D0  // GREEN
#define HEAT_GO        D7  // YELLOW
#define RECIRC_GO      D8  // BLUE
#define STATUS_LED     D4  // BLUE/BLACK


#define FAST_FLASH  250
#define SLOW_FLASH  1000

#define STATUS_NORMAL       1
#define STATUS_NO_FLOW      2
#define STATUS_LOCKED_OUT   3

#define STATUS_LED_ON  1
#define STATUS_LED_OFF 0

#define ON  LOW
#define OFF HIGH

#define LIGHTS_DEBOUNCE 50 //ms
#define JETS_DEBOUNCE   50 //ms
#define FLOW_DEBOUNCE   500 //ms
#define HEAT_DEBOUNCE   500 //ms

#define ANALOG_OPEN_CIRCUIT  900 // Any analog value below this means open-circuit

void setup() {
    Serial.begin(9600);
    Serial.println();
    Serial.println();
    
    // INPUTS
    pinMode(HI_LIMIT,      INPUT_PULLUP);
    pinMode(LIGHT_REQUEST, INPUT_PULLUP);
    pinMode(JETS_REQUEST,  INPUT_PULLUP);
    pinMode(HEAT_REQUEST,  INPUT_PULLUP);
    pinMode(FLOW_OK,       INPUT_PULLUP);

    // OUTPUTS
    digitalWrite(LIGHT_GO,   OFF);
    digitalWrite(JETS_GO,    OFF);
    digitalWrite(HEAT_GO,    OFF);
    digitalWrite(RECIRC_GO,  ON);
    
    pinMode(LIGHT_GO,   OUTPUT);
    pinMode(JETS_GO,    OUTPUT);
    pinMode(HEAT_GO,    OUTPUT);
    pinMode(RECIRC_GO,  OUTPUT);
    pinMode(STATUS_LED, OUTPUT);

    Serial.println("SETUP COMPLETE!");
}

unsigned long last_flash = 0;
void do_status(int new_state) {
    int current_state = digitalRead(STATUS_LED);

    if (new_state == STATUS_NORMAL) {
        digitalWrite(STATUS_LED, STATUS_LED_ON);
    }
    else if(new_state == STATUS_NO_FLOW) {
        if ((millis() - last_flash) > FAST_FLASH) {
            digitalWrite(STATUS_LED, !current_state);
            last_flash = millis();
        }
    }
    else if(new_state == STATUS_LOCKED_OUT) {
        if ((millis() - last_flash) > SLOW_FLASH) {
            digitalWrite(STATUS_LED, !current_state);
            last_flash = millis();
        }
    }
    else {
        digitalWrite(STATUS_LED, STATUS_LED_OFF);
    }
}

int lights_state = OFF;
int lights_request = OFF;
int last_lights_request = OFF;
unsigned long lights_requst_debounce = 0;

void read_light_request() {
    int reading = digitalRead(LIGHT_REQUEST);

    if (reading != last_lights_request) {
        lights_requst_debounce = millis();
    }

    if ((millis() - lights_requst_debounce) > LIGHTS_DEBOUNCE) {
        if (reading != lights_request) {
            lights_request = reading;

            if (lights_request == ON) {
                lights_state = !lights_state;
            }
        }
    }

    last_lights_request = reading;
}

int jets_state   = OFF;
int jets_request = OFF;
int last_jets_request = OFF;
unsigned long jets_requst_debounce = 0;

void read_jets_request() {
    int reading = digitalRead(JETS_REQUEST);

    if (reading != last_jets_request) {
        jets_requst_debounce = millis();
    }

    if ((millis() - jets_requst_debounce) > JETS_DEBOUNCE) {
        if (reading != jets_request) {
            jets_request = reading;

            if (jets_request == ON) {
                jets_state = !jets_state;
            }
        }
    }

    last_jets_request = reading;
}

int flow_state   = OFF;
int last_flow_state = OFF;
unsigned long flow_state_debounce = 0;

void read_flow_state() {
    int reading = digitalRead(FLOW_OK);

    if (reading != last_flow_state) {
        flow_state_debounce = millis();
    }

    if ((millis() - flow_state_debounce) > FLOW_DEBOUNCE) {
        flow_state = reading;
    }

    last_flow_state = reading;
}

int heat_state   = OFF;
int last_heat_state = OFF;
unsigned long heat_state_debounce = 0;

void read_heat_request() {
    int reading = digitalRead(HEAT_REQUEST);

    if (reading != last_heat_state) {
        heat_state_debounce = millis();
    }

    if ((millis() - heat_state_debounce) > HEAT_DEBOUNCE) {
        heat_state = reading;
    }

    last_heat_state = reading;
}

int read_temperature() {
    int temperature = analogRead(HI_LIMIT);
    temperature = (ANALOG_OPEN_CIRCUIT - temperature);
    return temperature;
}

int locked_out = false;
void loop() {
    if (locked_out) {
        digitalWrite(LIGHT_GO, OFF);
        digitalWrite(JETS_GO,  OFF);
        digitalWrite(HEAT_GO,  OFF);
        do_status(STATUS_LOCKED_OUT);
        delay(10);
        return;
    }

    int status_state = STATUS_NORMAL;

    int temperature = read_temperature();
    if (temperature < 0 || temperature > HI_LIMIT_MAX) {
        locked_out = true;
        return;
    }

    read_flow_state();

    if (flow_state == OFF) {
        status_state  = STATUS_NO_FLOW;
        jets_state    = OFF;
        heat_state    = OFF;
    }
    else {
        read_heat_request();
        read_jets_request();
    }

    read_light_request();

    digitalWrite(LIGHT_GO, lights_state);
    digitalWrite(JETS_GO,  jets_state);
    digitalWrite(HEAT_GO,  heat_state);

    do_status(status_state);
}
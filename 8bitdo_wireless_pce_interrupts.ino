#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#undef EXTRADEBUG

// SEL pins is used as interrupt, uno uses 2,3 as interrupt pins
// 0,1 are reserved
// Setting these to non-PWM pins (2, 4, 7) makes digitalRead() faster?
#define PIN_SEL 2
#define PIN_CLR 4

// D0 D1 D2 D3 lines should be on pins A0 A1 A2 A3, hardcoded

volatile uint8_t button_arrow;
volatile uint8_t button_low;

const uint8_t n = B0001;
const uint8_t e = B0010;
const uint8_t s = B0100;
const uint8_t w = B1000;
const uint8_t arrow_map[16] = {0, n, n|e, e, s|e, s, s|w, w, n|w, 0, 0, 0, 0, 0, 0, 0};

class JoystickReportParser : public HIDReportParser {
  public:
    void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
      const uint8_t btn = buf[0];
      const uint8_t startsel = buf[1];
      const uint8_t dpad = buf[2];

      button_low =  ~((startsel << 2) |     // run & sel in right order
                    (btn & B10) |           // ii, already shifted to right place
                    ((btn & B100) >> 2));   // i
    
      // 0xF nul should roll over to 0x0 as a uint8
      button_arrow = ~(arrow_map[(dpad + 1) & 0xf]);

#ifdef EXTRADEBUG
      Serial.print(PIND, BIN);
      Serial.print(" ");
      Serial.print(button_low, BIN);
      Serial.print(" ");
      Serial.print(button_arrow, BIN);
      Serial.print("\n");
#endif
    }
};

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickReportParser Joy;

// http://www.archaicpixels.com/Controllers
// https://github.com/asterick/TurboSharp/blob/master/Text/pcetech.txt
// The 2-button controller has a four-way directional pad and four buttons:
// Select, Run, II and I. A multiplexer is used to determine which values
// (directions or buttons) are returned when D3-D0 are read. The SEL line of
// the I/O port selects directions when high, and buttons when low. The state
// of D3-D0 are inverted, so '0' means a switch is closed and '1' means a
// switch is open.
//
//        SEL = 0                SEL = 1 // seems reversed to me
// D3 :   Run                    Left
// D2 :   Select                 Right // incorrect, this is DOWN
// D1 :   Button II              Down // incorrect, this is RIGHT
// D0 :   Button I               Up
//
// Games use a small delay after changing the SEL line, before the new data is
// read (a common sequence is PHA PLA NOP NOP). This ensures the multiplexer
// has had enough time to change it's state and return the right data.
//
// When the CLR line is low, the joypad can be read normally. When CLR is
// high, input from the joypad is disabled and D3-D0 always return '0'.


void isr_sel() {
  PORTC = digitalRead(PIN_SEL) ? button_low : button_arrow;
}


void setup() {
  //Configure PCE pins
  pinMode(A0, OUTPUT); //D0 PORTC0
  pinMode(A1, OUTPUT); //D1 PORTC1
  pinMode(A2, OUTPUT); //D2 PORTC2
  pinMode(A3, OUTPUT); //D3 PORTC3

  // keep PIND clear of noise?
  for (int i = 2; i < 8; i++) {
    pinMode(i, INPUT);
  }
  
  pinMode(PIN_CLR, INPUT); //CLR  PIND2
  pinMode(PIN_SEL, INPUT); //SEL  PIND3
  attachInterrupt(digitalPinToInterrupt(PIN_SEL), &isr_sel, CHANGE);

  Serial.begin(115200);
  Serial.println("Start");

  if (Usb.Init() == -1) {
    Serial.println("ERROR: OSC did not start.");
  }
  delay(200);
  if (!Hid.SetReportParser(0, &Joy)) {
    Serial.println("ERROR: Failed to set HID parser");
  }
  Serial.println("Setup complete.");
}

unsigned long lastms = 0;
unsigned long ticks = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastms > 15) {
    Usb.Task();
    lastms = now;
  }
  
#ifdef EXTRADEBUG  
  if (++ticks % 1000 == 0) { 
    Serial.print("SEL LOW ");
    Serial.print(debugSelLowCount, DEC);
    Serial.print("\tHIGH ");
    Serial.print(debugSelHighCount, DEC);
    Serial.print("\tCLR ");
    Serial.print(debugClrCount, DEC);
    Serial.println();
    debugSelLowCount = debugSelHighCount = debugClrCount = 0;
  }  
#endif
}

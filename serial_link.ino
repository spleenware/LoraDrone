

// ------------------------------------ private data -----------------------------------

static char sbuf[128];
static int slen;

enum SerialStateDef { SER_IDLE = 0, SER_STARTED = 1 };
static SerialStateDef state = SER_IDLE;

// ------------------------------------- private code -----------------------------------

static void processSerial() {
    // now using radio 'command' packets
}

// ------------------------------------- public code -----------------------------------

void serial_loop() 
{
  if (Serial.available()) {
    char c = Serial.read();
    switch (state) {
      case SER_IDLE:
        if (c == '$') {
          state = SER_STARTED;
          slen = 0;
        }
        break;

      case SER_STARTED:
        if (c == '\n') {
          sbuf[slen] = 0;
          processSerial();
          state = SER_IDLE;
        } else if (slen < 127) {
          sbuf[slen++] = c;
        }
        break;
    }
  }
}

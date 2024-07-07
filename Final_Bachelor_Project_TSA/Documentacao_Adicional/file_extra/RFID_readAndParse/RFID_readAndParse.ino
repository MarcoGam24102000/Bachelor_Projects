void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


// Try to handle / parse an RFID tag.
bool
RFIDTagHandled ()
{
  byte value = 0;       // temporary data received from RFID reader.
  byte code[6];         // code + checksum data of RFID tag received.
  byte checksum = 0;    // checksum data of RFID tag received.
  byte bytesRead = 0;   // number of received data from RFID reader.
  byte tempByte = 0;    // temporary value used for checksum calculation.

  bool handled = false; // flag indicating if an RFID tag was handled.

  const byte RFID_SIZE = 10; // the code size (digits) of the RFID tag.
  const byte CSUM_SIZE = 2;  // the csum size (digits) of the RFID tag.

  // if there are any data coming from the RFID reader.
  if (RFID.available () > 0) {
    // disable all interrupts.
    detachInterruptsISRs ();

    // check for the STX header (0x02 ASCII value).
    if (0x02 == (value = RFID.read ())) {
      // read the RFID digits & the checksum digits.
      while (bytesRead < (RFID_SIZE + CSUM_SIZE)) {
        // if there are any data coming from the RFID reader.
        if (RFID.available () > 0) {
          // get a byte from the RFID reader.
          value = RFID.read ();

          // check for ETX | STX | CR | LF.
          if ((0x0D == value) ||
              (0x0A == value) ||
              (0x03 == value) ||
              (0x02 == value)) {
            // stop reading - there is an error.
            break;
          }

          // store the RFID code digits to an external array.
          if (bytesRead < RFID_SIZE)
            RFIDCode[bytesRead] = value;

          // convert hex tag ID.
          if ((value >= '0') && (value <= '9'))
            value = value - '0';
          else if ((value >= 'A') && (value <= 'F'))
            value = 10 + value - 'A';

          // every two hex-digits, add byte to code.
          if (bytesRead & 1 == 1) {
            // make some space for this hex-digit by shifting
            // the previous hex-digit with 4 bits to the left.
            code[bytesRead >> 1] = (value | (tempByte << 4));

            if (bytesRead >> 1 != 5)
              // if this is checksum byte, calculate the checksum (XOR).
              checksum ^= code[bytesRead >> 1];
          }
          else
            tempByte = value;

          // ready to read next digit.
          bytesRead++;
        }
      }

      // handle the RFID digits & the checksum digits.
      if (bytesRead == (RFID_SIZE + CSUM_SIZE)) {
        // check if the RFID code is correct.
        if (code[5] == checksum)
          // set that the tag was handled.
          handled = true;
      }
    }

    // enable all external interrupts.
    attachInterruptsISRs ();
  }

  return handled;
}

//
// Arduino magnet levitation sketch
//
// Norbert Pozar 2009
//
// License: do whatever you want with the code.
//
/*
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// clear bit and set bit
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


// intRes = intIn1 * intIn2 >> 16 + rounding
// rounding adds 1 to the result if the MSB of the byte 1 is set
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 32bit result
// 21 cycles
#define MultiU16X16toH16Round(intRes, intIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %A2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %B2, %A1 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsl r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"a" (intIn1), \
"a" (intIn2) \
: \
"r26" , "r27" \
) 

void analogSetup(unsigned char prescaler, unsigned char reference, unsigned char adjust) {
  //value	prescaler	adc clock	max freq
  //1	2	8000000	615385.  Hz
  //2	4	4000000	307692.  Hz
  //3	8	2000000	153846.  Hz
  //4	16	1000000	76923.1  Hz
  //5	32	500000	38461.5  Hz
  //6	64	250000	19230.8  Hz
  //7	128	125000	9615.38  Hz

  // reference
  // 0 	AREF
  // 1	Vcc
  // 2	reserved
  // 3	1.1V

  // adjust
  // 0	right adjust
  // 1	left adjust

    ADMUX = (ADMUX & 0x1f) | ((reference << 6) | adjust << 5);

  ADCSRA = (ADCSRA & 0xf8) | prescaler;
}

// start conversion
void analogStart(unsigned char pin) {
  ADMUX = (ADMUX & 0xf0) | pin;

  sbi(ADCSRA, ADSC);
}

// waits until another conversion is finished by waiting for
// the interrupt flag to be set
// reads the 10bit result and resets the interrupt flag
unsigned int analogNext() {
  while (!(ADCSRA & _BV(ADIF))) ;

  // reset the flag
  sbi(ADCSRA, ADIF);

  //	unsigned char low = ADCL;
  //	unsigned char high = ADCH;
  //return	(high << 8) + low;

  // this is here just for fun
  // it annoys me that AVR GCC wastes cycles on combining ADCL and ADCH
  unsigned int res;

  asm volatile ( 
  "lds %A0, 0x0078 \n\t" // ADCL
  "lds %B0, 0x0079 \n\t" // ADCH
: 
  "=&r" (res) 
    ) ;

  return res;	
}

inline void digitalWriteD(unsigned char bit, unsigned char state) {
  if (state) {
    sbi(PORTD, bit);
  } 
  else {
    cbi(PORTD, bit);
  }
}

// disable timer 0 overflow interrupt
void disableMillis() {
  cbi(TIMSK0, TOIE0);
}

#define SAMPLES 4096
unsigned int baseline = 0;
unsigned int coilPower = 0;

void setup() 
{

  // open serial connection, 1,000,000 baud
  beginSerial(1000000L);

  // setup 19230.8Hz sampling rate
  // setup Vcc as ref, right aligned output
  analogSetup(6, 1, 0);

  // free running mode
  ADCSRB = ADCSRB & 0xf8;

  // setup pin 3 as output
  pinMode(3,OUTPUT);

  // enable autotrigger
  sbi(ADCSRA, ADATE);

  // start conversion from pin 0
  analogStart(0);

  // calibration
  unsigned long gather;

  // turn the coil off for baseline
  digitalWriteD(3, 0);
  // wait for some time
  delay(200);
  // gather info
  gather = 0;
  for (int i = 0; i < SAMPLES; i ++) {
    // wait for the next value and read it
    gather += analogNext();
  }
  baseline = gather / SAMPLES;

  // turn the coil on for coilPower
  digitalWriteD(3, 1);
  // wait for some time
  delay(200);
  // gather info
  gather = 0;
  for (int i = 0; i < SAMPLES; i ++) {
    // wait for the next value and read it
    gather += analogNext();
  }
  coilPower = gather / SAMPLES - baseline + 1;
  coilPower *= 2;  // this is to compensate for the filter resolution 0 ... 32767

  // turn the coil off
  digitalWriteD(3, 0);

  // disable Timer0 for function millis() to 
  // avoid interference
  disableMillis();


}

// action parameters
// 0 -- 3 coil simulation
// 4 -- 7 power generation
// 8 -- 14 experimentatal
// 15 -- square wave generator
#define NPARAMS 16
#define HEADERSIZE (NPARAMS * 2)
unsigned int ap[NPARAMS];

int counter = 0;

void loop()
{

  unsigned char signal = 0;
  unsigned char action = 0;
  unsigned int filter = 0;
  unsigned int oldfilter = 0;

  while (true) {
    // read commands
    if (!action) {
      if (serialAvailable() >= HEADERSIZE) {
        char* a = (char*) ap;
        for (char i = 0; i < HEADERSIZE; i ++) {
          char c = serialRead();
          *a = c;
          a++;
        }
        action = true;

        filter = 0;
        oldfilter = 0;
        signal = 0;

        for (char i = 0; i < HEADERSIZE; i ++) {
          serialWrite(i);
        }
      }

    } 
    else {
      if (serialAvailable() > 0) {
        unsigned char c = serialRead();
        switch (c) {
        case 0:
          action = false;
          digitalWriteD(3,0);
          break;
        case 1: 
          ap[4] ++;
          break;
        case  2: 
          ap[4] --;
          break;
        case 3: 
          ap[5] ++;
          break;
        case 4: 
          ap[5] --;
          break;
        }
      }
    }

    if (action) {
      // wait for the next value and read it
      unsigned int field = analogNext();
      // write previously computed signal
      digitalWriteD(3, signal);

      // do the computations

      unsigned int result;
      // mag = field - coilPower * oldFilter >> 16 - baseline;
      MultiU16X16toH16Round(result, coilPower, oldfilter);
      int mag = field - result - baseline;

      // update filter
      oldfilter = filter;
      if (signal == 0) {
        //filter -= ((unsigned long) ap[2] * filter) >> 16;
        // dropping to diode voltage ap[3]
        MultiU16X16toH16Round(result, ap[2], filter + ap[3]);
        if (result > filter) {
          filter = 0;
        } 
        else {
          filter -= result;
        }
      } 
      else {
        //filter += ((unsigned long) ap[0] *(32768 - filter)) >> 16;
        unsigned int temp = 32768 - filter;
        MultiU16X16toH16Round(result, ap[0], temp);
        filter += result;
      }
      // estimate required power 0 -- 255
      // linear decay of intensity

      int power = 255 + (ap[4] - (mag)) * ap[5];

      if (power < 0) power = 0;
      if (power > 255) power = 255;
      // compare with the actual coil field
      if (power > (filter >> 7)) {
        // need to energize
        signal = 1;
      } 
      else {
        // need to deenergize
        signal = 0;
      }

      // emergency shutdown
      // magnet is too far, turn off the coil
      if (mag < 8) signal = 0;

      // test
      if (ap[15] !=0) {
        counter += ap[15];
        if (counter > 0) {
          signal = 0;
        } 
        else {
          signal = 1;
        }
      }

      // send data
      serialWrite(field);
      serialWrite(((field >> 8) << 1) | signal);   
      serialWrite(mag);
    }
  }
}

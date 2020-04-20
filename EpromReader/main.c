/*
 * EpromReader.c
 */
#define F_CPU 8000000UL
#define BAUD 9600
#define TX_PIN PB4 // Pin 3
#define TX_PIN_SET DDB4
#define OUT_DATA PB0 // Pin 5
#define OUT_DATA_SET DDB0
#define OUT_LATCH PB1      // Pin 6
#define OUT_LATCH_SET DDB1 // Pin 6
#define OUT_CLOCK PB2      // Pin 7
#define OUT_CLOCK_SET DDB2 // Pin 7
#define MANUAL_TICK PB3    // Pin 2
#define ADDRESS_BASE 0x8000
#define EPROM_SIZE 0x1FFF
#define DIGITAL_DELAY 20
#define ADDRESS_BIT_COUNT 8

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
//#include <util/setbaud.h>

// basically kill me
volatile uint16_t tx_shift_reg = 0;
volatile uint16_t addr_tmp = 0;

// void uart_init(void)
void setPin(uint8_t pin, uint8_t state);
void uartTx(char character);
void uartTxString(char *string);
void blit(int8_t address, int8_t chipSelect);

ISR(TIM0_COMPA_vect) {
  // setPin(TX_PIN,1);
  // setPin(TX_PIN,0);
  // output LSB of the TX shift register at the TX pin
  if (tx_shift_reg & 0x01) {
    PORTB |= (1 << TX_PIN);
  } else {
    PORTB &= ~(1 << TX_PIN);
  }
  // shift the TX shift register one bit to the right
  tx_shift_reg = (tx_shift_reg >> 1);
  // if the stop bit has been sent, the shift register will be 0
  // and the transmission is completed, so we can stop & reset timer0
  if (!tx_shift_reg) {
    TCCR0B = 0;
    TCNT0 = 0;
  }
}

ISR(PCINT0_vect) {
  if (PINB & (1 << MANUAL_TICK)) {
    addr_tmp += 1;
    blit(addr_tmp, 0b1111);
  }
}

int main(void) {
  // CLKPR = 0b10000000;
  CLKPR = 0x80; // set system clock to 8mhz with no prescale
  CLKPR = 0x00; // these 2 CLKPR instructions have to be run together in order

  DDRB |= (1 << TX_PIN_SET | 1 << OUT_DATA_SET | 1 << OUT_LATCH_SET |
           1 << OUT_CLOCK_SET); // mark output pins as such

  // CLKPCE = 1; // look this up
  // CLKPS0 = 0;
  // CLKPS1 = 0;
  // CLKPS2 = 0;
  // CLKPS3 = 0;

  // no prescaling cs02: 0, cs01: 0, cs00: 0
  // CTC mode 2, wgm02: 0, wgm01: 1, wgm00: 0
  TCCR0A = (1 << WGM01);
  TCCR0B = 0b00000000;

  // 833 = 8,000,000 / baudrate. 8mill = clock speed?
  //	OCR0A = 833 >> 8;  // first 3 binary digs
  // OCR0B = 833 & 0xFF; // last byte.
  OCR0A = 103; // adjust somewhere around 103

  TIMSK |= (1 << OCIE0A);

  GIMSK |= (1 << PCIE);   // pin change interrupt enable
  PCMSK |= (1 << PCINT3); // pin change interrupt for PB3

  sei();

  blit(0, 0b1111); // reset the shift registers
  uartTxString("initialized...");

  /* Replace with your application code */
  while (1) {

    // Cart slot has 15 address pins and 4 chip select pins. so going to need 3
    // shift registers (8 outputs a piece) for 19 outputs. 8 dataports so 1 piso
    // should be enough for reading. though I haven't looked into them yet.
    // uartTxString("hello");
    //_delay_ms(100);
  }
}

uint8_t piso() {
  /*
  PL (Parallel load) has to be tied high. pulse it low to latch the inputs.
  */
  return 0;
}

void iterateCart() {
  /*
  The four chip select pins on the cartridge correspond to the four ROM chips
  that may be present in the cartridge. The first chip select pin (Chip Select
  0x8000) should be toggled (high to low) when reading any addresses between
  0x8000 and 0x9FFF. Likewise, Chip Select 0xA000 should be toggled for
  addresses between 0xA000 and 0xBFFF, Chip Select 0xC000 for 0xC000 through
  0xDFFF, and Chip Select 0xE000 for 0xE000 through 0xFFFF.
  */

  /*	tiny pins
          1 reset (for now)
          2 pb input (for now)
          3
          4 ground
          5 sipo data
          6 sipo latch
          7 spio clock
  */

  /*
   design shift register output
    0 ADDRESS_01
    1 ADDRESS_02
    2 ADDRESS_03
    3 ADDRESS_04
    4 ADDRESS_05
    5 ADDRESS_06
    6 ADDRESS_07
    7 ADDRESS_08

    8 ADDRESS_09
    9 ADDRESS_10
   10 ADDRESS_11
   11 ADDRESS_12
   12 ADDRESS_13
   13 ADDRESS_14
   14 ADDRESS_15
   15 CHIP_SELECT_1 // 0x8000 - 0x9FFF

   16 CHIP_SELECT_2 // 0xA000 - 0xBFFF
   17 CHIP_SELECT_3 // 0xC000 - 0xDFFF
   18 CHIP_SELECT_4 // 0xE000 - 0xFFFF
   19 LATCH_PISO
   20
   21
   22
   23
  */
}

void shiftOut(int8_t bit) {
  setPin(OUT_DATA, 1 & bit);

  setPin(OUT_CLOCK, 1);
  _delay_ms(DIGITAL_DELAY);
  setPin(OUT_CLOCK, 0);
  _delay_ms(DIGITAL_DELAY);
}

void blit(int16_t address, int8_t chipSelect) {
  // shift all the bits
  int16_t local_address = address;
  for (int8_t i = 0; i < ADDRESS_BIT_COUNT; i++) {
    shiftOut(local_address & 0b1);
    local_address = local_address >> 1;
  }
  // pulse latch
  setPin(OUT_LATCH, 1);
  _delay_ms(DIGITAL_DELAY);
  setPin(OUT_LATCH, 0);
  _delay_ms(DIGITAL_DELAY);
}

void uartTx(char character) {
  uint16_t local_tx_shift_reg = tx_shift_reg;
  // if sending the previous character is not yet finished, return
  // transmission is finished when tx_shift_reg == 0
  if (local_tx_shift_reg) {
    return;
  }
  // fill the TX shift register witch the character to be sent and the start &
  // stop bits (start bit (1<<0) is already 0)
  local_tx_shift_reg = (character << 1) | (1 << 9); // stop bit (1<<9)
  tx_shift_reg = local_tx_shift_reg;
  // start timer0 with a prescaler of 8
  TCCR0B = (1 << CS01);
}

void uartTxString(char *string) {
  while (*string) {
    uartTx(*string++);
    // wait until transmission is finished
    while (tx_shift_reg)
      ;
  }
}

void setPin(uint8_t pin, uint8_t state) {
  if (state) {
    PORTB |= (1 << pin);
  } else {
    PORTB &= ~(1 << pin);
  }
}

/*
void uart_init(void) {
//UBRR0H = UBRRH_VALUE;
//UBRR0L = UBRRL_VALUE;
//
//#if USE_2X
//UCSR0A |= _BV(U2X0);
//#else
//UCSR0A &= ~(_BV(U2X0));
//#endif
}
*/

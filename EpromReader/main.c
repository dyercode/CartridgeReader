/*
 * EpromReader.c
 */

//#define FOSC 1843200 // Clock Speed, but why this speed?
#define F_CPU 8000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1
#define OUT_DATA PB0 // Pin 23
#define OUT_DATA_SET DDB0
#define OUT_LATCH PB1      // Pin 24
#define OUT_LATCH_SET DDB1 // Pin 24
#define OUT_CLOCK PB2      // Pin 25
#define OUT_CLOCK_SET DDB2 // Pin 25
#define IN_DATA PB3        // Pin 17
#define IN_DATA_SET DDB3   // Pin 17
#define IN_LATCH PB4       // Pin 18
#define IN_LATCH_SET DDB4  // Pin 18
#define IN_CLOCK PB5       // Pin 19
#define IN_CLOCK_SET DDB5  // Pin 19
//#define MANUAL_TICK PC3               // Pin 26
//#define MANUAL_TICK_INTERRUPT PCINT11 // Pin 26
#define EPROM_SIZE 0x1FFF
#define DIGITAL_DELAY_US 10
#define ADDRESS_BIT_COUNT 15 // for testing since I'm short a shift register.
#define EOF -1

#include <avr/interrupt.h>
#include <avr/io.h>

void set_pin(uint8_t pin, uint8_t state);
void uart_tx(char character);
void uart_tx_string(char *string);
void set_address(uint16_t address, uint8_t chip_select);
void iterate_cart();
void characterize(uint8_t d);
void pulse(uint8_t pin, uint8_t high);
void pulse_high(uint8_t pin);
void pulse_low(uint8_t pin);
const unsigned char hexies[] = "0123456789ABCDEF";

/*
ISR(PCINT1_vect) {
  if (PINC & (1 << MANUAL_TICK)) {
  }
}
*/

void usart_init(unsigned int ubrr) {
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  /* Enable receiver and transmitter */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  /* Set frame format: 8data, 1stop bit */
  UCSR0C = (0 << USBS0) | (3 << UCSZ00);
}

int main(void) {
  // CLKPR = 0b10000000;
  CLKPR = 0x80; // set system clock to 8mhz with no prescale
  CLKPR = 0x00; // these 2 CLKPR instructions have to be run together in order

  usart_init(MYUBRR);

  DDRB |= (1 << OUT_DATA_SET | 1 << OUT_LATCH_SET | 1 << OUT_CLOCK_SET |
           1 << IN_LATCH_SET | 1 << IN_CLOCK_SET); // mark output pins as such

  // CLKPCE = 1; // look this up
  // CLKPS0 = 0;
  // CLKPS1 = 0;
  // CLKPS2 = 0;
  // CLKPS3 = 0;

  // no prescaling cs02: 0, cs01: 0, cs00: 0
  // CTC mode 2, wgm02: 0, wgm01: 1, wgm00: 0
  // TCCR0A = (1 << WGM01);
  // TCCR0B = 0b00000000;

  // PCICR |= (1 << PCIE1);   // pin change interrupt enable
  // PCMSK1 |= (1 << PCINT3); // pin change interrupt for PB3

  sei();

  set_pin(IN_LATCH, 1);
  set_address(0, 0b1111); // reset the shift registers
  uart_tx_string("initialized...\r\n");

  iterate_cart(); // TODO - trigger by a serial read so it can started by pc and
                  // automated instead of just happeneing when it gets power.
  while (1) {

    // Cart slot has 15 address pins and 4 chip select pins. so going to need 3
    // shift registers (8 outputs a piece) for 19 outputs. 8 dataports so 1 piso
    // should be enough for reading. though I haven't looked into them yet.
    // uart_tx_string("hello");

    // iterate_cart(); this needs to be triggered by something so it only
    // happens once. or just drop it out of the while loop.
  }
}

uint8_t read_eprom_byte() {
  /*
  PL (Parallel load) has to be tied high. pulse it low to latch the inputs.
  */
  pulse_low(IN_LATCH);

  uint8_t fish = 0;
  for (uint8_t i = 0; i < 8; i++) {
    pulse_high(IN_CLOCK);
    fish |= (PINB & (1 << IN_DATA)) << i;
  }
  return fish;
}

uint8_t chip_bin(uint8_t n) { return 0b1111 & ~(1 << (4 - n)); }

void iterate_cart() {
  /*
  The four chip select pins on the cartridge correspond to the four ROM chips
  that may be present in the cartridge. The first chip select pin (Chip Select
  0x8000) should be toggled (high to low) when reading any addresses between
  0x8000 and 0x9FFF. Likewise, Chip Select 0xA000 should be toggled for
  addresses between 0xA000 and 0xBFFF, Chip Select 0xC000 for 0xC000 through
  0xDFFF, and Chip Select 0xE000 for 0xE000 through 0xFFFF.
  */

  const uint16_t range = 0x1fff;
  const uint16_t base = 0x8000;

  for (uint8_t cs = 1; cs < 5; cs++) {
    uint8_t chip_select = chip_bin(cs);
    uint16_t offset = 0x2000 * (cs - 1);
    uint16_t top = base + offset + range;
    for (uint32_t addr = base + offset; addr <= top; addr++) {
      set_address(addr, chip_select);

      // latch_piso();
      uint8_t byte = read_eprom_byte();
      characterize(byte);
    }
  }
  uart_tx(EOF); // not sure if will is work are

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
  */
}

void shift_out(uint8_t bit) {
  set_pin(OUT_DATA, 1 & bit);

  pulse_high(OUT_CLOCK);
}

void shift_many(uint16_t data, uint8_t size) {
  int16_t local = data;
  for (int8_t i = 0; i < size; i++) {
    shift_out(local & 0b1);
    local = local >> 1;
  }
}

void characterize(uint8_t d) {
  unsigned char first_digit = ((d >> 4) & 0b1111);
  unsigned char second_digit = (d & 0b1111);
  uart_tx(hexies[first_digit]);
  uart_tx(hexies[second_digit]);
  uart_tx(' ');
}

void set_address(uint16_t address, uint8_t chip_select) {
  // shift in chip_select first
  shift_many(chip_select, 4);

  // shift all the bits
  shift_many(address, ADDRESS_BIT_COUNT);

  pulse_high(OUT_LATCH);
}

void uart_tx(char character) {
  /* Wait for empty transmit buffer */
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  /* Put data into buffer, sends the data */
  UDR0 = character;
}

void uart_tx_string(char *string) {
  while (*string) {
    uart_tx(*string++);
    // wait until transmission is finished
    // while (!(UCSR0A & (1 << UDRE0)))
    //;
  }
}

void set_pin(uint8_t pin, uint8_t state) {
  if (state) {
    PORTB |= (1 << pin);
  } else {
    PORTB &= ~(1 << pin);
  }
}

void pulse_high(uint8_t pin) { pulse(pin, 1); }

void pulse_low(uint8_t pin) {
  set_pin(pin, 0);
  set_pin(pin, 1);
}

void pulse(uint8_t pin, uint8_t high) {
  set_pin(pin, high & 0b1);
  set_pin(pin, (~high & 0b1) >> 1);
}

/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

#define RX1_RING_BUFFER (RX1_BUFFER_SIZE+1)
#define TX1_RING_BUFFER (TX1_BUFFER_SIZE+1)

uint8_t serial1_rx_buffer[RX1_RING_BUFFER];
uint8_t serial1_rx_buffer_head = 0;
volatile uint8_t serial1_rx_buffer_tail = 0;

uint8_t serial1_tx_buffer[TX1_RING_BUFFER];
uint8_t serial1_tx_buffer_head = 0;
volatile uint8_t serial1_tx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial1_get_rx_buffer_available()
{
  uint8_t rtail = serial1_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial1_rx_buffer_head >= rtail) { return(RX1_BUFFER_SIZE - (serial1_rx_buffer_head-rtail)); }
  return((rtail-serial1_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial1_get_rx_buffer_count()
{
  uint8_t rtail = serial1_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial1_rx_buffer_head >= rtail) { return(serial1_rx_buffer_head-rtail); }
  return (RX1_BUFFER_SIZE - (rtail-serial1_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial1_get_tx_buffer_count()
{
  uint8_t ttail = serial1_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial1_tx_buffer_head >= ttail) { return(serial1_tx_buffer_head-ttail); }
  return (TX1_RING_BUFFER - (ttail-serial1_tx_buffer_head));
}


void serial1_init()
{
  // Set baud rate
  #if BAUD_RATE1 < 57600
    uint16_t UBRR1_value = ((F_CPU / (8L * BAUD_RATE1)) - 1)/2 ;
    UCSR1A &= ~(1 << U2X1); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR1_value = ((F_CPU / (4L * BAUD_RATE1)) - 1)/2;
    UCSR1A |= (1 << U2X1);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR1H = UBRR1_value >> 8;
  UBRR1L = UBRR1_value;

  // enable rx, tx, and interrupt on complete reception of a byte
  UCSR1B |= (1<<RXEN1 | 1<<TXEN1 | 1<<RXCIE1);

  // defaults to 8-bit, no parity, 1 stop bit
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial1_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial1_tx_buffer_head + 1;
  if (next_head == TX1_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial1_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial1_tx_buffer[serial1_tx_buffer_head] = data;
  serial1_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR1B |=  (1 << UDRIE1);
}


// Data Register Empty Interrupt handler
ISR(SERIAL1_UDRE)
{
  uint8_t tail = serial1_tx_buffer_tail; // Temporary serial1_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  UDR1 = serial1_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX1_RING_BUFFER) { tail = 0; }

  serial1_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial1_tx_buffer_head) { UCSR1B &= ~(1 << UDRIE1); }
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial1_read()
{
  uint8_t tail = serial1_rx_buffer_tail; // Temporary serial1_rx_buffer_tail (to optimize for volatile)
  if (serial1_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial1_rx_buffer[tail];

    tail++;
    if (tail == RX1_RING_BUFFER) { tail = 0; }
    serial1_rx_buffer_tail = tail;

    return data;
  }
}


ISR(SERIAL1_RX)
{
  uint8_t data = UDR1;
  uint8_t next_head;

  next_head = serial1_rx_buffer_head + 1;
  if (next_head == RX1_RING_BUFFER) { next_head = 0; }

  // Write data to buffer unless it is full.
  if (next_head != serial1_rx_buffer_tail) {
	  serial1_rx_buffer[serial1_rx_buffer_head] = data;
	  serial1_rx_buffer_head = next_head;
  }

}


void serial1_reset_read_buffer()
{
  serial1_rx_buffer_tail = serial1_rx_buffer_head;
}

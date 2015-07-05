// EC3K decoding using a JeeLink(v3)
//
// $Id: ec3kSerial.ino, v 1.02 2015/07/05 10:00:00 Sevenwatt.com $
// Adapted to latest Arduinbo IDE. Mostly PROGMEM is declared as const
// Copyright 2015 SevenWatt.com <info@sevenwatt.com>
// under the MIT license http://opensource.org/licenses/mit-license.php
//
// original sketch maintained at http://www.fhemwiki.de/wiki/JeeLink
//
// $Id: ec3kSerial,v 1.01 2014/12/30 24:00:00 Fuchks $
// -Whrs calculation corrected, Decoding status updated (derived from RF12test15 http://forum.jeelabs.net/comment/6761.html#comment-6761)
//
// #Id: ec3kSerial,v 1.00 2013/06/30 09:00:00 ohweh $
//
// based on the excellent work of Thomas Schulz and other,
//      see http://forum.jeelabs.net/node/341.html
//
// derived from
//  #Id: RF12test.pde,v 1.48 2012/01/09 22:10:49 tsch Stab $
//  Copyright 2011 Thomas Schulz <t.schulz@xs4.de>
//  under the MIT license http://opensource.org/licenses/mit-license.php
//
// derived from:
//  #Id: RF12demo.pde 7754 2011-08-22 11:38:59Z jcw $
//  2009-05-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php

#define PROGNAME  "ec3kSerial"
#define PROGVERS  "1"

#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdarg.h>

//--- ohweh mod start ---------------------------------------------------------------
#define AUTOSTART       1       // enable to autostart decoding for housemon
#define NODEID         22       // node id to be used in "decoded" packet
#define R2_FREQ    0xA679       // center frequency used to receive ec3k data
//--- ohweh mod end -----------------------------------------------------------------

// #define DATAFLASH    1       // check for presence of DataFlash memory on JeeLink
#define FLASH_MBIT  16       // JeeLink v3 dataflash 16 Mbit, other sizes: 4/8 Mbit
// RF12test does not yet support 4/8 Mbit dataflash

#define LED_PIN   4 // activity LED on DIO1, comment out to disable
// #define VERBOSE         1       // enable for extended output

// #define EXECTIMES  1 // report execution times of code
// #define USE_TESTFIFO 1 // test RFM12B fifo levels and access methods
#define USE_XMODEM  1 // dataflash upload with XMODEM protocol
#define USE_XMODEMCRC 1 // XMODEM protocol with CRC option

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// utilities

#define alenof(array) (sizeof(array)/sizeof(array[0]))

static long   combaud; // console baudrate, see setup()

struct datadtsyn {
  unsigned long msec;  // millis() of last time sync
  unsigned long utcsec;  // UTC seconds and ..
  unsigned int  utcmsec; // ... milliseconds where millis() == 0
};
static struct datadtsyn dtsync;  // data for timesync with UTC from host

static const char _spcqm[3] PROGMEM = " ?";
#define     _qm   (_spcqm+1)
#define     _nulstr (_spcqm+2)

static void activityLed (byte on) {
#ifdef LED_PIN
  digitalWrite(LED_PIN, !on);
#endif
}

static int freeRam(void) {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

static void mprintf_P( PGM_P format, ... );
static void mprintnl( void );
static void hexdump( uint8_t * data, size_t dlen );

#if EXECTIMES
static unsigned long  _exectbeg;

static void exectimebeg( void )
{
  _exectbeg = micros();
}
static void exectimeprt( PGM_P text )
{
  mprintf_P(PSTR(" %S %Lus\n"), text, micros() - _exectbeg);
}
#else
#define exectimebeg()   /* no exec times */
#define exectimeprt(text) /* no exec times */
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RFM12B support for JeeLink ATmega328p
// (derived from jeelabs RF12.zip 2011-08-22)

// ATmega328, etc.
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2       // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8
#define SPI_SS      10      // do not change, must point to h/w SPI pin
#define SPI_MOSI    11
#define SPI_MISO    12
#define SPI_SCK     13

// RFM12B commands
#define RF_STATUS_READ    0x0000
#define RF_RX_FIFO_READ   0xB000
#define RF_SLEEP_MODE   0x8205  // 2 power mgmt: !er !ebb !et !es !ex  EB !ew DC
#define RF_IDLE_MODE    0x820D  // 2 power mgmt: !er !ebb !et !es  EX  EB !ew DC
#define RF_RECEIVER_ON    0x82DD  // 2 power mgmt:  ER  EBB !et  ES  EX  EB !ew DC
#define RF_SW_RESET   0xFE00  // see RFM12B.pdf p.35 "SW Reset Command"

// RFM12B status bits
#define RFM12_STATUS_RGIT   0x8000
#define RFM12_STATUS_FFIT   0x8000
#define RFM12_STATUS_POR  0x4000
#define RFM12_STATUS_RGUR   0x2000
#define RFM12_STATUS_FFOV   0x2000
#define RFM12_STATUS_WKUP   0x1000
#define RFM12_STATUS_EXT  0x0800
#define RFM12_STATUS_LBD  0x0400
#define RFM12_STATUS_FFEM   0x0200
#define RFM12_STATUS_ATS  0x0100
#define RFM12_STATUS_RSSI   0x0100
#define RFM12_STATUS_DQD  0x0080
#define RFM12_STATUS_CRL  0x0040
#define RFM12_STATUS_ATGL   0x0020

#define RF_MAX  128 // size of receive buffer

/* RFM12 control registers
*/
struct rfm12reg {
  uint16_t  cmd;  // control command
  uint16_t  mask; // fixed bits in control command
  uint16_t  dflt; // default value used here
};

static const struct rfm12reg rfm12dfl[16] PROGMEM = {
  { 0x8000,  // 0x8008   1 configuration setting      80xx
    0xFF00,   0x80E7
  },  // EL (ena TX), EF (ena RX FIFO), 868MHz, 12.0pF
  { 0x8200,  // 0x8208   2 power management       82xx
    0xFF00,     0x820D
  },  // idle: !er !ebb !et !es  ex  eb !ew dc
  { 0xA000,  // 0xA680   3 frequency setting        Axxx
    // 0xF000,     0xA686 },  // 868.350 MHz
    0xF000,     R2_FREQ
  },     // ohweh: changed from A686 to A679
  { 0xC600,  // 0xC623   4 data rate          C6xx
    0xFF00,     0xC610
  },  // approx 20.3 kbps, i.e. 10000/29/(1+n) kbps
  { 0x9000,  // 0x9080   5 receiver control       9xxx
    0xF800,     0x94A2
  },  // VDI, FAST, 134kHz, lna 0dBm, rssi -91dBm
  { 0xC228,  // 0xC22C   6 data filter        C2xx
    0xFF28,     0xC2AC
  },  // AL, ml, DIG, DQD4
  { 0xCA00,  // 0xCA80   7 FIFO and reset mode      CAxx
    0xFF00,     0xCA8F
  },  // FIFO8, 1-SYNC, AL, FF, DR
  { 0xCE00,  // 0xCED4   8 synchron pattern       CExx
    0xFF00,     0xCEAA
  },  // SYNC=AA
  { 0xB000,  // 0x0000   9 receiver fifo read command     B000
    0x0000,     0x0000
  },  // NO SETUP: readonly
  { 0xC400,  // 0xC4F7  10 AFC (AutoFrequencyControl)     C4xx
    // 0xFF00,     0xC4A7 },  // keep with VDI, +7..-8, !st, FI, OE, EN
    0xFF00,     0xC4D4
  },     // ohweh: changed from C4A7 to C4D4
  { 0x9800,  // 0x9800  11 transmitter configuration      98xx
    0xFE00,     0x9820
  },  // !mp, 45kHz, 0dB power (max)
  { 0xCC12,  // 0xCC77  12 PLL setting        CCxx
    0xFF92,     0xCC77
  },  // OB1, OB0, LPX, !ddy, DDIT, BW0
  { 0xB800,  // 0xB8AA  13 transmitter register write command   B800
    0x0000,     0x0000
  },  // NO SETUP: only for transmit
  { 0xE000,  // 0xE196  14 wake-up timer (0xFE00 = software reset)  Exxx
    0xE000,     0xE000
  },  // wake-up timer disabled
  { 0xC800,  // 0xC80E  15 low duty cycle       C8xx
    0xFF00,     0xC800
  },  // low duty cycle disabled
  { 0xC000,  // 0xC000  16 low battery detect and uC clock divide C0xx
    0xFF10,     0xC049
  },  // 1.66MHz, 3.15V
};

static uint16_t rfm12regs[16];    // RFM12 control register cache
static volatile uint16_t rfm12istat;  // RFM12 status read in interrupt
static volatile uint8_t  rxbuf[RF_MAX]; // receive buffer
static volatile uint8_t  rxstat[RF_MAX]; // receive status buffer
static volatile struct rxinfo {
  unsigned long tusec;    // micros() of first received byte
  uint8_t len;      // number of received bytes in rxbuf
  uint8_t ovr;      // receive overrun counter
  uint16_t  st_rssi;    // RFM12 status read at DRSSI off->on
  uint16_t  st_beg;     // RFM12 status read at receive begin
  uint16_t  st_end;     // RFM12 status read at receive end
  int   dus_rssi;   // delta usec of RSSI rise to tusec
  int   dus_end;    // delta usec of receive end to tusec
} rxinfo;
#define rxfill    rxinfo.len    // number of received bytes in rxbuf
#define rxovr     rxinfo.ovr    // receive overrun counter
static volatile uint16_t rxcrl;   // receive ClockRecoveryLocked counter
static volatile unsigned long rxrssius; // receive begin rssi rise [usec]
#define rxtbegus  rxinfo.tusec    // receive begin timestamp [usec]
#define statrssi  rxinfo.st_rssi  // RFM12 status read at DRSSI off->on
#define statbeg   rxinfo.st_beg   // RFM12 status read at receive begin
static volatile uint16_t statbeg8;  // RFM12 status read at receive 8th byte
static volatile unsigned long rxtendus; // receive end timestamp [usec]
#define statend   rxinfo.st_end   // RFM12 status read at receive end

#define EC3KLEN   41    // length of Energy Count 3000 packet
struct dataec3k {
  struct rxinfo rxinfo;   // info about received block (like 'I')
  byte    crl;    // ClockRecoveryLocked count for pkt[]
  byte    boffs;    // number of bits preceeding EC3k packet
  byte    pkt[EC3KLEN]; // Energy Count 3000 packet data
};


// map RFM12 command to RFM12 register index
static byte rf12regidx( byte cmdhi )
{
  if ( cmdhi == 0x00 )            // 17 status read command
    return alenof(rfm12regs) + 1;  // do not cache RF12 register value
  switch ( cmdhi & 0xF0 ) {
    //   0x80xx                         1 configuration setting
    //   0x82xx                         2 power management
    case 0x80:   return cmdhi & 0x02 ?  2 - 1 : 1 - 1;
    //   0x90xx                         5 receiver control
    //   0x98xx                        11 transmitter configuration
    case 0x90:   return cmdhi & 0x08 ? 11 - 1 : 5 - 1; break;
    case 0xA0:   return 3 - 1;      //  3 frequency setting
    //   0xB000  NOCACHE:readonly       9 receiver fifo read command
    //   0xB8xx  NOCACHE               13 transmit register write command
    case 0xC0:
      switch ( cmdhi ) {
        case 0xC0: return 16 - 1;     // 16 low battery detect and uC clock divide
        case 0xC2: return  6 - 1;     //  6 data filter
        case 0xC4: return 10 - 1;     // 10 AFC (AutoFrequencyControl)
        case 0xC6: return  4 - 1;     //  4 data rate
        case 0xC8: return 15 - 1;     // 15 low duty cycle
        case 0xCA: return  7 - 1;     //  7 FIFO and reset mode
        case 0xCC: return 12 - 1;     // 12 PLL setting
        case 0xCE: return  8 - 1;     //  8 synchron pattern
      }
      break;
    case 0xE0:                      // 14 wake-up timer (0xFE00 = software reset)
      if ( cmdhi < 0xE0 + 29 + 1 ) // wakeup exponent R 0..29
        return 14 - 1;
      break;
  }
  return alenof(rfm12regs) + 1;   // default: do not cache RF12 register value
}


static void spi_initialize( void )
{
  static byte spi_initdone;

  if ( spi_initdone )
    return;
  bitSet(SS_PORT, SS_BIT);
  bitSet(SS_DDR, SS_BIT);
  digitalWrite(SPI_SS, 1);
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_SCK, OUTPUT);
#ifndef SPCR
# error "SPCR not defined, should be for ATmega328p"
#endif
#if F_CPU <= 10000000
# error "F_CPU <= 10MHz, expect 16MHz for ATmega328p"
#endif
  // use clk/8 (2x 1/16th) to avoid exceeding RF12's SPI specs of 2.5 MHz
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
  SPSR |= _BV(SPI2X);
  spi_initdone = 1;
}

static uint8_t rf12_byte( uint8_t out )
{
#ifndef SPDR
# error "SPDR not defined, should be for ATmega328p"
#endif
  SPDR = out;
  // this loop spins 4 usec with a 2 MHz SPI clock
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

static uint16_t rf12_xfer( uint16_t cmd )
{
  bitClear(SS_PORT, SS_BIT);
  uint16_t reply = rf12_byte(cmd >> 8) << 8;
  reply |= rf12_byte(cmd);
  bitSet(SS_PORT, SS_BIT);
  return reply;
}

#if USE_TESTFIFO
static uint32_t rf12_statfifo( void )
{
  uint32_t statfifo = 0;

  bitClear(SS_PORT, SS_BIT);
  statfifo  = rf12_byte(0x00) << 24;
  statfifo |= rf12_byte(0x00) << 16;
  statfifo |= rf12_byte(0x00) << 8;
  statfifo |= rf12_byte(0x00);
  bitSet(SS_PORT, SS_BIT);
  return statfifo;
}
#endif /*USE_TESTFIFO*/

// access to the RFM12B internal registers with interrupts disabled
// cache cmd in rfm12regs[index] mapping index from cmd hibyte
uint16_t rf12_control_ec3k( uint16_t cmd )
{
#ifndef EIMSK
# error "EIMSK not defined, should be for ATmega328p"
#endif
  byte index = rf12regidx(cmd >> 8);
  byte intron = bitRead(EIMSK, INT0);
  bitClear(EIMSK, INT0);
  if ( index < alenof(rfm12regs) )
    rfm12regs[index] = cmd;
  uint16_t r = rf12_xfer(cmd);
  if ( intron )
    bitSet(EIMSK, INT0);
  return r;
}

// RFM12B interrupt service
static void rf12_interrupt( void )
{
  // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
  rfm12istat = rf12_xfer(0x0000);

  if ( rfm12istat & RFM12_STATUS_FFIT ) {
    uint8_t in = rf12_xfer(RF_RX_FIFO_READ);
    if ( rfm12istat & RFM12_STATUS_RSSI ) {
      if ( rxtbegus == 0 && rxfill == 0 ) {
        rxtbegus = micros();
        statbeg = rfm12istat;
        activityLed(1);
      }
      if ( rxfill < alenof(rxbuf) ) {
        if ( rxfill == 7 )
          statbeg8 = rfm12istat;
        rxstat[rxfill] = (uint8_t)(rfm12istat & 0xFF);
        rxbuf[rxfill++] = in;
      } else {
        if ( rxtendus == 0 ) {
          rxtendus = micros();
        }
        ++rxovr;
      }
      if ( rfm12istat & RFM12_STATUS_CRL )
        ++rxcrl;
    } else if ( rxfill ) {
      if ( rxtendus == 0 ) {
        rxtendus = micros();
        statend = rfm12istat;
      }
      activityLed(0);
      // restart FIFO fill after DRSSI went off
      rf12_xfer(rfm12regs[7 - 1] & ~0x0002); // FIFO fill disable
      // FIFO fill enable shall not be done here (might cause RFM12 hang),
      // but after rxbuf[] processed in e.g. drecvintr():
      //??? rf12_xfer(rfm12regs[7-1] |  0x0002);  // FIFO fill enable
    }
  }
}

static void rf12_recvStart( void )
{
  byte intron = bitRead(EIMSK, INT0);
  bitClear(EIMSK, INT0);
  rxcrl = rxovr = rxfill = 0;     // reset for next receive block
  rxrssius = rxtbegus = rxtendus = 0;
  rf12_xfer(RF_RECEIVER_ON);    // (re)start receiver
  rf12_xfer(rfm12regs[7 - 1] & ~0x0002); // stop FIFO fill and ...
  delayMicroseconds(100);     //??? delay really needed ?
  rf12_xfer(rfm12regs[7 - 1] |  0x0002); // ... (re)start FIFO fill
  if ( intron )
    bitSet(EIMSK, INT0);
}

static void rf12_recvStop( void )
{
  rf12_control_ec3k(RF_IDLE_MODE);
}

static void rf12_regsinit( void )
{
  byte  i;

  for ( i = 0; i < alenof(rfm12regs); ++i )
    rfm12regs[i] = pgm_read_word(&rfm12dfl[i].dflt);
}

void rf12_initialize( byte irq_on )
{
  byte  i;
  uint16_t  nval;

  bitClear(EIMSK, INT0);  // disable interrupt from RFM12

  spi_initialize();

  pinMode(RFM_IRQ, INPUT);
  digitalWrite(RFM_IRQ, 1); // pull-up

  rf12_xfer(0x0000); // intitial SPI transfer added to avoid power-up problem

  rf12_xfer(0x8205); // DC (disable clk pin), enable lbd, disable TX,RX,xosc


  // wait until RFM12B is out of power-up reset, this takes several *seconds*
  rf12_xfer(0xB800); // txreg_write in case we're still in OOK mode
  while (digitalRead(RFM_IRQ) == 0) // wait for RFM12B nIRQ off
    rf12_xfer(0x0000);

  if ( rfm12regs[0] == 0 ) {
    // initially setup RFM12B register caches with defaults
    rf12_regsinit();
  }
  for ( i = 0; i < alenof(rfm12regs); ++i ) {
    nval  = rfm12regs[i] & ~pgm_read_word(&rfm12dfl[i].mask);
    nval |= pgm_read_word(&rfm12dfl[i].cmd);
    rfm12regs[i] = nval;
    switch ( i ) {
      case  2-1:  // skip power management, will enable finally
      case  9-1:  // skip receiver fifo read command
      case 13-1:  // skip transmitter write command
        continue;
    }
    rf12_xfer(nval);
  }
  rf12_xfer(rfm12regs[2 - 1] = RF_IDLE_MODE);

  if ( irq_on != 0) {
    attachInterrupt(0, rf12_interrupt, LOW);
    bitSet(EIMSK, INT0);  // enable interrupt from RFM12
  } else
    detachInterrupt(0);
}


#if DATAFLASH
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DataFlash code

#define DF_ENABLE_PIN   8           // PB0
#define DF_ENABLE_PBbit 0

#if FLASH_MBIT == 4
// settings for 0.5 Mbyte flash in JLv2
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0700      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x0800      // 2048 pages, i.e. 0.5 Mbyte
#define DF_DEVICE_ID    0x1F44      // see AT25DF041A datasheet
#define DF_DEVICE_NAME  "AT25DF041A"
#define DF_SECT_ERASE   0x20        // erase one block of flash memory
#endif

#if FLASH_MBIT == 8
// settings for 1 Mbyte flash in JLv2
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x1000      // 4096 pages, i.e. 1 Mbyte
#define DF_DEVICE_ID    0x1F45      // see AT26DF081A datasheet
#define DF_DEVICE_NAME  "AT26DF081A"
#define DF_SECT_ERASE   0x20        // erase one block of flash memory
#endif

#if FLASH_MBIT == 16
// settings for 2 Mbyte flash in JLv3
#define DF_BLOCK_SIZE   256         // number of pages erased at same time
#define DF_LOG_BEGIN    0x0200      // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x1F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x2000      // 8192 pages, i.e. 2 Mbyte
#define DF_DEVICE_ID    0x2020      // see M25P16 datasheet
#define DF_DEVICE_NAME  "M25P16"
// M25P16 commands:
#define DF_READ_ID        0x9F  // o: --        i: mm tt cc
#define DF_READ_IDUID     0x9E  // o: --        i: mm tt cc ll uu... (16)
#define DF_READ_ELSIG     0xAB  // o: xx xx xx  i: es
#define DF_READ_STATUS    0x05  // o: --        i: st
#define DF_WRITE_STATUS   0x01  // o: st        i: --
#define DF_WRITE_ENABLE   0x06  // o: --        i: --
#define DF_WRITE_DISABLE  0x04  // o: --        i: --
#define DF_READ_DATA      0x03  // o: aa aa aa  i: dd...
#define DF_PAGE_PROGRAM   0x02  // o: aa aa aa dd... (1..256)
#define DF_SECT_ERASE     0xD8  // o: aa aa aa  i: --
#define DF_BULK_ERASE     0xC7
// M25P16 status bits:
#define DF_STATUS_WIP 0x01  // write in progress
#define DF_STATUS_WEL 0x02  // write enable latch
#define DF_STATUS_BP  0x1C  // block protect bits
#define DF_STATUS_SRWD  0x80  // status register write protect
#endif

// structure of each page in the log buffer, size must be exactly 256 bytes
typedef struct {
  byte data[248];
  word seqnum;
  long timestamp;
  word crc;
} FlashPage;

// types of records in data area of each FlashPage
#define DFR_RESET 'R' // JeeLink reset with reason
#define DFR_DTSYN 'T' // timesync with host: millis() UTC sec.ms base
#define DFR_RINFO 'I' // info data for received block
#define DFR_RDATA 'D' // data of received block
#define DFR_RSTAT 'S' // RFM12B status LSByte per received byte
#define DFR_REC3K 'E' // valid Energy Count 3000 packet

// structures of data according to record type
struct datareset {
  unsigned long tusec;  // micros() when setup() called
  byte    mcusr;  // MCU status register
};

// structure of consecutive records in the data area of each FlashPage
// records are TLV-encoded: Type,Length,Value
typedef struct flashrecord {
  byte type;      // record type determines fields in data[]
  byte length;    // length of data[]
  union {     // union of all record types:
    struct datareset R; //  JeeLink reset with reason as MCUSR contents
    struct datadtsyn T; //  timesync with host: millis() UTC sec.ms base
    struct dataec3k  E; //  valid Energy Count 3000 packet with rxinfo
    struct rxinfo    I; //  info data for received block, D,S recs follow
    byte             D[]; //  data of received block, after I record
    byte             S[]; //  RFM12B status LSByte per received byte
    byte             data[];  //  length databytes, struct according to type
  }  u;
} FlashRecord;

static FlashPage dfBuf;     // for data not yet written to flash
static word dfLastPage;     // page number last written
static byte dfFill;         // next byte available in buffer to store entries


static byte df_present( void )
{
  return dfLastPage != 0;
}

static void df_enable( void )
{
  // digitalWrite(DF_ENABLE_PIN, 0);
  bitClear(PORTB, DF_ENABLE_PBbit);
}

static void df_disable( void )
{
  // digitalWrite(DF_ENABLE_PIN, 1);
  bitSet(PORTB, DF_ENABLE_PBbit);
}

static byte df_xfer( byte cmd )
{
  SPDR = cmd;
  while (!bitRead(SPSR, SPIF))
    ;
  return SPDR;
}

byte df_status( void )
{
  byte  status;

  cli();    // use SPI exclusively, i.e. block RFM12 interrupts
  df_enable();
  df_xfer(DF_READ_STATUS);
  status = df_xfer(0);
  df_disable();
  sei();    // release SPI, RFM12 interrupt may use SPI
}

// wait for no more "work in progress" (WIP off) or no dataflash
void df_waitWIPoff( void )
{
  byte  status;

  do {
    status = df_status();
    // don't wait for ready bit if there is clearly no dataflash connected
    if ( status == 0xFF )
      break;
  } while ( status & DF_STATUS_WIP );
}

void df_command( byte cmd )
{
  byte  status;

  df_waitWIPoff();

  cli();
  df_enable();
  df_xfer(cmd);
}

static void df_deselect( void )
{
  df_disable();
  sei();
}

static void df_writeCmd( byte cmd )
{
  df_command(DF_WRITE_ENABLE);
  df_deselect();
  df_command(cmd);
}

void df_read( word block, word off, void* buf, word len )
{
  df_command(DF_READ_DATA); // Read Array (Low Frequency SPIclk <= 20MHz)
  df_xfer(block >> 8);  // sector address
  df_xfer(block);   // page address in sector
  df_xfer(off);   // offset in page
  for (word i = 0; i < len; ++i)
    ((byte*) buf)[(byte) i] = df_xfer(0);
  df_deselect();
}

void df_write( word block, const void* buf )
{
  df_writeCmd(DF_PAGE_PROGRAM);
  df_xfer(block >> 8);  // sector address
  df_xfer(block);   // page address in sector
  df_xfer(0);     // begin of page
  for (word i = 0; i < 256; ++i)
    df_xfer(((const byte*) buf)[(byte) i]);
  df_deselect();
}

// wait for current command to complete
void df_flush( void )
{
  df_waitWIPoff();
}

static void df_wipe( void )
{
  mprintf_P(PSTR("DF W\n"));
  df_writeCmd(DF_BULK_ERASE); // Chip Erase all sectors
  df_deselect();
  df_flush();     // wait for completion (M25P16 10 .. 20 sec)
}

static void df_erase( word block )
{
  mprintf_P(PSTR("DF E %W\n"), block);
  df_writeCmd(DF_SECT_ERASE);
  df_xfer(block >> 8);  // sector address
  df_xfer(block);   // any page in sector
  df_xfer(0);
  df_deselect();
  df_flush();     // wait for completion (M25P16 0.8 .. 3.0 sec)
}       // measured 0.618 (0.593 .. 0.659) sec

static word df_wrap( word page )
{
  return page < DF_LOG_LIMIT ? page : DF_LOG_BEGIN;
}

static void df_saveBuf( void )
{
  if (dfFill == 0)
    return;

  exectimebeg();
  dfLastPage = df_wrap(dfLastPage + 1);
  if (dfLastPage == DF_LOG_BEGIN)
    ++dfBuf.seqnum; // bump to next seqnum when wrapping


  // set remainder of buffer data to 0xFF and calculate crc over entire buffer
  dfBuf.crc = ~0;
  for (byte i = 0; i < sizeof dfBuf - 2; ++i) {
    if (dfFill <= i && i < sizeof dfBuf.data)
      dfBuf.data[i] = 0xFF;
    dfBuf.crc = _crc16_update(dfBuf.crc, dfBuf.data[i]);
  }

  df_write(dfLastPage, &dfBuf);
  dfFill = 0;

  // wait for write to finish before reporting page, seqnum, and time stamp
  // takes 1.5 (35%) or 2.5 (65%) msec
  df_flush();
  exectimeprt(PSTR("df_saveBuf"));
  mprintf_P(PSTR("DF S %W %W %L\n"), dfLastPage, dfBuf.seqnum, dfBuf.timestamp);

  // erase next block if we just saved data into a fresh block
  // at this point in time dfBuf is empty, so a lengthy erase cycle is ok
  if (dfLastPage % DF_BLOCK_SIZE == 0) {
    exectimebeg();
    df_erase(df_wrap(dfLastPage + DF_BLOCK_SIZE));
    exectimeprt(PSTR("df_erase"));
  }
}

static void df_appendrec( const byte rectype, const void* buf, byte len )
{
  byte maxlen = sizeof(dfBuf.data) - offsetof(FlashRecord, u.data) - dfFill;
  if ( len > maxlen )
    len = maxlen; // truncate len prevents write beyond end of dfBuf
  dfBuf.data[dfFill++] = rectype;
  dfBuf.data[dfFill++] = len;
  memcpy(dfBuf.data + dfFill, buf, len);
  dfFill += len;
}
static void df_append( const byte rectype, const void* buf, byte len )
{
  //FIXME the current logic can't append incoming packets during a save!

  if ( !df_present() )
    return;

  if ( dfFill + offsetof(FlashRecord, u.data) + len > sizeof(dfBuf.data) )
    df_saveBuf(); // record does not fit in page: flush dfBuf to dataflash

  // fill in page time stamp when appending to a fresh page
  if ( dfFill == 0 ) {
    unsigned long now = millis();
    if ( now < dfBuf.timestamp )  // 49-day overflow
      ++dfBuf.seqnum;   //  note by increase sequence number
    dfBuf.timestamp = now;
    if ( df_wrap(dfLastPage + 1) % DF_BLOCK_SIZE == 0 && dtsync.msec ) {
      // insert a timesync record at begin of sector
      df_appendrec(DFR_DTSYN, &dtsync, sizeof(dtsync));
    }
  }

  // append new record to flash buffer
  df_appendrec(rectype, buf, len);
}

// go through entire log buffer to figure out which page was last saved
static void scanForLastSave( void )
{
  dfBuf.seqnum = 0;
  dfLastPage = DF_LOG_LIMIT - 1;
  // look for last page before an empty page
  for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
    word currseq = 0xFFFF;
    df_read(page, sizeof dfBuf.data, &currseq, sizeof currseq);
    if (currseq != 0xFFFF) {
      dfLastPage = page;
      dfBuf.seqnum = currseq + 1;
    } else if (dfLastPage == page - 1)
      break; // careful with empty-filled-empty case, i.e. after wrap
  }
}

static void df_initialize( void )
{
  word  info;
  byte  cap, status;

  spi_initialize();
  df_disable();
  pinMode(DF_ENABLE_PIN, OUTPUT);
  df_command(DF_READ_ID); // Read Manufacturer and Device ID
  info =  df_xfer(0) << 8;  // manufacturer
  info |= df_xfer(0);   // memory type
  cap  =  df_xfer(0);   // capacity
  df_deselect();
  status = df_status();
  mprintf_P(PSTR(" dataflash %B %B %B"), info >> 8, info & 0xFF, cap);

  if (info == DF_DEVICE_ID) {
    status = df_status();
    mprintf_P(PSTR(" " DF_DEVICE_NAME " status %B"), status);
    if ( status & (DF_STATUS_SRWD | DF_STATUS_BP) ) {
      df_writeCmd(DF_WRITE_STATUS); // Write Status Register ...
      df_xfer(0x00);      // ... Global Unprotect
      df_deselect();
      status = df_status();
      mprintf_P(PSTR(" unprotect %B"), status);
    }


    scanForLastSave();
    mprintf_P(PSTR("\n DF I %W %W"), dfLastPage, dfBuf.seqnum);

    // df_wipe();
    //??? df_saveBuf(); //XXX
  } else {
    mprintf_P(PSTR(" NOT SUPPORTED"));
  }
  mprintnl();
}

#else /*not DATAFLASH*/

#define df_present()      (false)
#define df_initialize()
#define df_append(rectype, buf,len)

#endif


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// console communication

#ifndef ABORT
#define ABORT -2
#endif
#ifndef ERR
#define ERR -1
#endif
#ifndef OK
#define OK  0
#endif
#define bABORT  (byte)(ABORT)
#define bERR  (byte)(ERR)
#define bOK (byte)(OK)

#define LINELEN 31
static char linebuf[LINELEN + 1];

#define HIBYTE(word)  ((word) >> 8)
#define LOBYTE(word)  ((word) & 0xFF)

static int
comgetc( void )
{
  if ( Serial.available() )
    return Serial.read();
  return ERR;
}
static void
computc( byte chr )
{
  Serial.write(chr);
}

static char
to_upper( char chr )    // convert character 'a'..'z' to uppercase
{
  if ( chr >= 'a' && 'z' >= chr )
    return chr - 'a' + 'A';
  return chr;
}
static char
to_asc( byte nibble )   // convert hex-nibble to ASCII-hex
{
  if ( (byte)(nibble & 0x0F) >= 0x09 + 1 )
    nibble += 'A' - 1 - '9';
  return ( (char)(nibble + '0') );
}
static byte
to_hex( byte chr )    // convert char '0'..'9','A'..'F' to hex nibble
{
  if ( chr < '9' + 1 )  chr -= '0';
  else chr = to_upper(chr) - 'A' + 0x0A;
  if ( chr & 0xF0 ) return bERR;
  return chr;
}

static void
putchr( char chr )    // send character, expand newline
{
  if ( chr == '\n' )
    computc( '\r' );
  computc( chr );
}
static void
puthbyte( byte hbyte )    // send byte in ASCII-hex
{
  computc(to_asc(hbyte >> 4));    /* send MSDigit   */
  computc(to_asc((byte)(hbyte & 0x0F)));  /* send LSDigit   */
}
static void
_putdlongwf( unsigned long num, byte width, char fill )
{
  char  buf[3 * sizeof(long)];
  byte  i = 0;

  if ( num == 0 ) {
    buf[i++] = 0;
  }
  while ( num > 0 ) {
    buf[i++] = num % 10;
    num /= 10;
  }
  for ( ; width > i && fill; width-- )
    computc(fill);
  for ( ; i > 0; i-- )
    computc('0' + buf[i - 1]);
}
static void
putdlong( unsigned long num ) // send dword in ASCII decimal
{
  _putdlongwf(num, 0, '\0');
}
static void
putdword( unsigned int num )  // send word in ASCII decimal
{
  putdlong((unsigned long)num);
}

/* printf for monitor console
* the format string contains normal text to print and control-codes
* for output-conversion of the following data-arguments.
* control-codes after escaping '%' are:
*   W 2byte hex-Word, 4 digits with leading zero
*   B hex-Byte, 2 digits with leading zero
*   D 2byte Decimal unsigned integer
*   L 4byte Long decimal unsigned integer
*   C Character, convert non-printable to '.'
*   S String in PROGMEM, 0-terminated
*   s String in RAM, 0-terminated
*   % print a '%'
* newline \n in format and string argument is printed as carriage-
* return,line-feed \r\n.
*/
#define RAMem (false)
#define PGMem (true)
#define GETCHR(isPGMem, ptr)  ({  \
    unsigned char __c;          \
    __c = (isPGMem) ? pgm_read_byte(ptr) : *ptr;  \
    ptr++;            \
    __c;            \
  })

static void
vmprintf( boolean fmem, const char *fptr, va_list argp )
{
  char  fchr;
  boolean smem = RAMem;
  char *  str;
  uint16_t  varg = 0;

  /* process print-format ...     */
  while ( (fchr = GETCHR(fmem, fptr)) != '\0' ) {
    if ( fchr == '%' ) {                /* format introducer        */
      fchr = GETCHR(fmem, fptr);
      switch ( fchr ) {   /* pre-process format selector  */
        case '%':       /* format "%%" print '%'*/
          break;
        case 'L':       /* 4byte dec-dword  */
          putdlong(va_arg(argp, unsigned long));
          continue;
        default:
          varg = va_arg(argp, uint16_t);  /* get next argument  */
          break;
      }
      switch ( fchr ) {   /* process format selector  */
        case 'D':       /* 2byte dec-word */
          putdword(varg);
          continue;
        case 'W':       /* 2byte hex-word       */
          puthbyte( HIBYTE(varg) );
        /* fall through for LO-byte */
        case 'B':       /* hex-byte             */
          puthbyte( LOBYTE(varg) );
          continue;
        case 'C':       /* byte as character    */
          computc( LOBYTE(varg) < ' ' || '~' < LOBYTE(varg) ?
                   '.' : LOBYTE(varg) );
          continue;
        case 'S':       /* string in PROGMEM  */
          smem = PGMem;
        /* fall through for print string */
        case 's':       /* string in RAM  */
          str = (char*)varg;
          while ( (fchr = GETCHR(smem, str)) != 0 )
            putchr(fchr);
          smem = RAMem;
          continue;
        default:        /* bad format selector  */
          putchr('?');
          putchr('%');
          putchr(fchr);
          putchr('?');
          continue;
        case '%':       /* format "%%" print '%'*/
          break;
      }
    }
    putchr(fchr);       /* plain char in format */
  }
}

static void
mprintf( char *format, ... )    /* printf with format in RAM */
{
  va_list argp;

  va_start(argp, format);
  vmprintf(RAMem, format, argp);
  va_end(argp);
}

static void
mprintf_P( PGM_P format, ... )    /* printf with format in PROGMEM */
{
  va_list argp;

  va_start(argp, format);
  vmprintf(PGMem, format, argp);
  va_end(argp);
}

static void
mprintnl( void )
{
  putchr('\n');
}

/* receive (command-)line from monitor console
* characters are handled special for line-editing:
*   BS  delete character: echo BS ' ' BS, decrement ptr
*   DEL   - dito -
*   CR  accept input line: append end-of-string and return
*   LF    - dito -
*   ESC abort input: clear line buffer and return
*   ctl-C   - dito -
* receive-timeout is ignored within command-line
* line-buffer must have space for trailing zero
*/
static void
getline( char *line, byte len )
{
  char *  lptr;
  int   chr;

  lptr = line;      /* init ptr into command-line */
  for ( ; ; ) {
    if ( (chr = comgetc()) == ERR ) /* receive character    */
      continue;         /* ignore timeout   */
    switch ( (byte)chr ) {    /* process character    */
      default:          /* normal character   */
        computc(*lptr = chr);   /* store char & echo it */
        if ( ++lptr < line + len - 1 )  /* command-line full ?  */
          continue;     /*  no, ->next char */
        /* fall thru */     /*  ya, ->echo new line */
      case '\r':                          /* carriage-return          */
      case '\n':                          /* line-feed                */
        putchr('\n');       /* echo new line  */
        break;        /* exit     */
      case '\b':                          /* back-space               */
      case '\377':                        /* DEL-character            */
        if ( line < lptr ) {    /* delete last character*/
          mprintf_P(PSTR("\b \b"));
          --lptr;
        }
        continue;
      case '\033':                        /* ESC-character            */
      case '\003':                        /* ctrl-C                   */
        mprintf_P(PSTR("<ABORT>\n")); /* input aborted  */
        lptr = line;      /* clear command-line   */
        break;
    }
    break;
  }
  *lptr = '\0';                       /* append end-of-string         */
}

/* skip leading blanks from string
*   returns:
* (char*) ptr after leading blanks
* NULL  empty string or only blanks
*/
static char *
skip_spc( char * str )
{
  if ( str != NULL ) {
    while ( *str == ' ' ) ++str;
    if ( *str != '\0' )
      return str;
  }
  return NULL;
}

/* compare text-strings ignoring upper/lower case
* compares 'text' and 'string' up to end of text or blank in text
* and end of string. if either is reached, they are considered
* equal and a ptr into 'string' with the matching text skipped is
* returned. otherwise return NULL for mismatch.
* called from cmdexec() to compare command line 'string' with
* command-name 'text' from cmdtab[].
*/
static char *
text_cmp( PGM_P text, char * string )
{
  while ( to_upper(pgm_read_byte(text++))
          == to_upper(*string++) )     /* walk thru while matching */
    if (  pgm_read_byte(text) == '\0'       /* text ends at NUL or  */
          || (pgm_read_byte(text) == ' '        /*  blank & string ends */
              && *string == '\0') )
      return string;      /*   string after match */
  return NULL;
}

/* evaluate argument from command-string
* skip characters until ASCII-hex character found
* then convert hex-characters to 2byte-word until
* non-hex character and return ptr to the latter
* if no ASCII-hex character in string, return NULL
*   returns:
* (char*) ptr into string after scanned & evaluated argument
* NULL  invalid argument, no ASCII-hex characters in string
*/
static char *
evalword( char * str, uint16_t * pword )
{
  char *  pstr = skip_spc(str);
  uint16_t  word = 0x0000;
  byte  ndig = 0;

  while ( pstr != NULL      /* non-empty string ... */
          && to_hex(*pstr) != bERR ) {   /* and hex-digit found  */
    word <<= 4; word += to_hex(*pstr++);  /*  add in hex-digit  */
    ++ndig;         /*  and count digits  */
  }
  if ( ndig != 0        /* found hex-value and  */
       && ndig < sizeof(word) * 2 + 1 ) { /*  not too many digits */
    *pword = word;        /*  store scanned word  */
    return pstr;        /* ptr after hex-digits */
  }
  return NULL;        /* no hex-digit in str  */
}           /*  or too long hex-str */

static char *
evalbyte( char * str, byte * pbyte )
{
  uint16_t  word;
  char *  pstr;

  if ( (pstr = evalword(str, &word)) != NULL
       && HIBYTE(word) == 0 ) {
    *pbyte = (byte)word;
    return pstr;
  }
  return NULL;
}

static char *
evaltext( char * str, char ** ptext )
{
  char *  ptxt = skip_spc(str);
  char *  pstr = ptxt;

  while ( pstr != NULL      /* non-empty string ... */
          && *pstr && *pstr != ' ' ) {   /* and non-blank found  */
    ++pstr;         /*  continue search */
  }
  if ( pstr ) {       /* found text   */
    if ( *pstr )        /* not end of cmdline */
      *pstr++ = '\0';       /*  terminate string  */
    *ptext = ptxt;        /*  store ptr to text */
    return pstr;        /* ptr after trail-'\0' */
  }
  return NULL;        /* no hex-digit in str  */
}           /*  or too long hex-str */

static char *
evalnone( void )
{
  return NULL;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// communication protocols: serial stream, xmodem

struct prot {
  byte  (*init)(void);    // start file transfer
  byte  (*putchr)(byte);  // send character (buffered)
  byte  (*exit)(void);    // terminate file transfer
};

static const char _aborted[] PROGMEM = "aborted";
static const char _failed[]  PROGMEM = "failed";
static const char _done[]    PROGMEM = "done";
static const char *results[] = {
  _aborted,
  _failed,
  _done,
  _qm
};

static void
reportrc( PGM_P text, byte rc )
{
  delay(1000);
  if ( rc != bOK )
    mprintnl();
  rc += 2;
  if ( rc >= alenof(results))
    rc = alenof(results) - 1;
  mprintf_P(text);
  mprintf_P((PGM_P)pgm_read_word(&results[rc]));
  mprintnl();
}


// standard serial stream protocol
//  returnOK() implements init,exit doing nothing
//  sendchr()  aborts if any user input available
//
static byte returnOK( void )    // serial stream init,exit do nothing
{
  return bOK;
}

static byte ssputchr( byte chr )  // serial stream send character
{
  if ( Serial.available() )
    return bABORT;
  computc(chr);
  return bOK;
}

#define ssinit  returnOK
#define ssexit  returnOK

const struct prot ssprot PROGMEM = {  // serial stream protocol functions
  ssinit, ssputchr, ssexit
};


#if USE_XMODEM
// XMODEM protocol
//  see https://secure.wikimedia.org/wikipedia/en/wiki/XMODEM
//  and http://www.techfest.com/hardware/modem/xymodem.htm
//

#define XMTIMEOUT 3000  // 3 sec, xmodem standard is 10 sec
#define XMRETRYMAX  10
#define SOH   '\x01'
#define EOT   '\x04'
#define ACK   '\x06'
#define NAK   '\x15'
#define CAN   '\x18'
#define XMPAD   '\x1A'
#define XMWANTCRC 'C'

static byte txbuff[128];
static byte txfill;
static byte xmbnum;
#if USE_XMODEMCRC
static boolean xmwantcrc;
#endif
static byte xmlastrchr;

static int  xmgetc( void )    // xmodem get character with timout
{
  word  tstart = (word)millis();
  int   chr;

  while ( (word)millis() - tstart < XMTIMEOUT ) {
    chr = Serial.read();
    if ( chr >= 0 ) {
      if ( (byte)chr == CAN && xmlastrchr == CAN )
        return ABORT;
      xmlastrchr = chr;
#if USE_XMODEMCRC
      if ( (byte)chr == XMWANTCRC && xmbnum <= 1 ) {
        xmwantcrc = true;
        chr = NAK;
      }
#endif
      return chr;
    }
  }
  return ERR;
}

static byte xminit( void )    // xmodem init: start transfer
{
  byte  retry;

  txfill = xmbnum = xmlastrchr = 0;
#if USE_XMODEMCRC
  xmwantcrc = false;
#endif
  mprintf_P(PSTR("start XMODEM receiver!\n"));
  // wait for NAK / CAN CAN / 'C' from receiver
  for ( retry = 0; retry < XMRETRYMAX; ++retry ) {
    switch ( xmgetc() ) {
      case NAK:
        return bOK;
      case ABORT:
        return bABORT;    // abort by CAN CAN
    }
  }
  return bERR;      // retries exceeded
}

static byte xmputchr( byte chr )  // xmodem buffered send character
{
#if USE_XMODEMCRC
  uint16_t  crc;
#endif
  byte  retry, checksum, i;

  txbuff[txfill++] = chr;
  if ( txfill < sizeof(txbuff) )
    return bOK;
  ++xmbnum;
  xmlastrchr = 0;
  for ( retry = 0; retry < XMRETRYMAX; ++retry ) {
    // send xmodem packet
    computc(SOH);
    computc(xmbnum);
    computc(255 - xmbnum);
    checksum = 0;
#if USE_XMODEMCRC
    crc = 0;
#endif
    for ( i = 0; i < sizeof(txbuff); ++i ) {
      computc(txbuff[i]);
      checksum += txbuff[i];
#if USE_XMODEMCRC
      crc = _crc_xmodem_update(crc, txbuff[i]);
#endif
    }
#if USE_XMODEMCRC
    if ( xmwantcrc ) {
      computc(crc >> 8);
      computc(crc & 0xFF);
    } else
#endif
      computc(checksum);
    // wait for xmodem response on sent packet
    switch ( xmgetc() ) {
      case ACK:
        txfill = 0;     // start fill next packet
        return bOK;     // OK, packet acknowledged
      case ABORT:
        return bABORT;    // abort by CAN CAN
      case NAK:
      case ERR:
        break;
    }
  }
  return bERR;      // ACK retries exceeded
}

static byte xmexit( void )  // xmodem exit: terminate transfer
{
  byte  rc, retry;

  // fill sendbuffer with PAD characters and send last packet
  while ( txfill ) {
    rc = xmputchr(XMPAD);   // will set txfill = 0 after sent and ACKed
    if ( rc != bOK )
      return rc;
  }
  for ( retry = 0; retry < XMRETRYMAX; ++retry ) {
    computc(EOT);
    if ( xmgetc() == ACK )
      return bOK;
  }
  return bERR;      // retries exceeded
}

const struct prot xmprot PROGMEM = {  // xmodem protocol functions
  xminit, xmputchr, xmexit
};
#endif /*USE_XMODEM*/


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RFM12B register commands

static byte regcmd2idx( byte regn ) // map RFM12 command hibyte to regindex
{
  byte  regidx;
  PGM_P ignbad = PSTR("ignored command");

  if ( regn >= 0x80 && (regidx = rf12regidx(regn)) < alenof(rfm12regs) )
    regn = regidx;
  switch ( regn ) {
    default:
      if ( regn < alenof(rfm12regs) )
        break;
      ignbad = PSTR("bad");
    // fallthru: invalid register
    case  9-1:    // ignore receiver fifo read command
    case 0xB0:
    case 13-1:    // ignore transmitter write command
    case 0xB8:
      mprintf_P(PSTR("%S regnum: %B\n"), ignbad, regn);
      regn = alenof(rfm12regs) + 1; // tell caller invalid regn
  }
  return regn;
}

static void dmpreg( byte regn )
{
  if ( (regn = regcmd2idx(regn)) >= alenof(rfm12regs) )
    return;   // bad regn, reported in regcmd2idx()
  mprintf_P(PSTR("  R%C %W\n"), to_asc(regn), rfm12regs[regn]);
}

static void dmpregs( void )
{
  byte  regn;

  for ( regn = 0; regn < alenof(rfm12regs); ++regn ) {
    switch ( regn ) {
      case  9-1:  // skip receiver fifo read command
      case 13-1:  // skip transmitter write command
        continue;
    }
    dmpreg(regn);
  }
}

static void modreg( byte regn, word rval )
{
  if ( (regn = regcmd2idx(regn)) >= alenof(rfm12regs) )
    return;   // bad regn, reported in regcmd2idx()
  rval &= ~pgm_read_word(&rfm12dfl[regn].mask);
  rval |=  pgm_read_word(&rfm12dfl[regn].cmd);
  rfm12regs[regn] = rval;
  dmpreg(regn);
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// frequency scanning

static void setFrequency( word f )
{
  if ( f < 96 )   f =   96;
  if ( f > 3903 ) f = 3903;
  rf12_control_ec3k(0xA000 | f);
}

static void setRSSI( byte level )
{
  if (level > 5)
    level = 5;
  rf12_control_ec3k(0x9400 | (rfm12regs[5 - 1] & 0x03F8) | level);
  delayMicroseconds(1000);
}

static void setRSSIbwLNA( byte level, byte bw, byte LNA )
{
  if (level > 5) level = 5; // -73dB .. -103dB 0
  if (bw == 0) bw = 1;  // 400kHz
  if (bw >  6) bw = 6;  //  67kHz
  if (LNA > 3) LNA = 3; // -20dB .. 0dB 0
  rf12_control_ec3k(0x9400 | ((word)bw << 5) | (LNA << 3) | level);
  delayMicroseconds(1000);
}

static byte getRSSI( byte do_DRSSI )
{
  byte  rssi;
  uint16_t  regval;

  if ( do_DRSSI ) {
    /*
    * fast return RSSI threshold+1 if DRSSI status bit on, else 0
    */
    if ( ((rf12_control_ec3k(0x0000) >> 8) & (RFM12_STATUS_RSSI >> 8)) )
      return (rfm12regs[5 - 1] & 0x07) + 1;
    return 0;
  }
  /*
  * determine RSSI value by set rising RSSI threshold until DRSSI status
  * bit is found off, much slower because delay 1000 usec is needed after
  * after each set threshold according to experiments
  * no delay caused RSSI read too low for 2 next frequencies
  * delay < 1000 usec caused RSSI read too low for next frequency
  */
  regval = 0x9400 | (rfm12regs[5 - 1] & 0x03F8);
  for ( rssi = 0; rssi < 6; ++rssi ) {
    rf12_control_ec3k(regval | rssi);
    delayMicroseconds(1000);
    if ( ((rf12_control_ec3k(0x0000) >> 8) & (RFM12_STATUS_RSSI >> 8)) == 0 )
      break;
  }
  return rssi;
}

// scan all frequencies in current band (433/868/915MHz)
// RFM12B.pdf / Si4421.pdf:
// - 868MHz band:
//  freq = 10 * 2 * (43 + F/4000) MHz
//  freq_kHz = 860000 + F * 5 kHz
//  F = (freq_kHz - 860000) / 5
//   F = 1600 .. 1720 = 0x640 .. 0x6B8 for 868.000 .. 868.600 MHz
// - RSSI threshold respects also LNA gain
// - DRSSI response time is 500 usec (p.10)
// http://www.controller-designs.de "rfm12_scanner":
// - delay 70..2550 usec between frequency changes
// - receiver bandwidths can be scanned simultaneously
// hints/improvements:
// + do_DRSSI true:  faster scan, but reports only RSSI above threshold
//   do_DRSSI false: report RSSI values, but scans frequencies slower
// + disable AFC (Automatic Frequency Control) for exact frequency
// + report RSSI value incl. LNA gain setting, i.e. 0..5 up to 3..8
// + report only scans with any RSSI >= current threshold
// - set bandwith according to frequency steps
//
static void scanfrequband( byte do_DRSSI, word flo, word fhi, word fstep, word fdly )
{
  uint16_t R2old = rfm12regs[3 - 1];
  uint16_t R4old = rfm12regs[5 - 1];
  uint16_t R9old = rfm12regs[10 - 1];
  uint16_t bwfstep = rfm12regs[1 - 1] & 0x30;
  byte LNAgain = (rfm12regs[5 - 1] & 0x18) >> 3;
  byte RSSIthr =  rfm12regs[5 - 1] & 0x07;
  byte bw;

  if ( flo < 96 || fhi > 3903 || flo > fhi ) {
    mprintf_P(PSTR("invalid flo/fhi\n"));
    return;
  }
  if ( !fdly )
    fdly = 250;   // default delay 250us after change frequency
#if 0
  // set bandwidth according to frequency steps:
  switch ( bwfstep ) {
    case 0x10: bwfstep = fstep *  5 / 2; break; // 433MHz: 2.5kHz steps
    case 0x20: bwfstep = fstep * 10 / 2; break; // 868MHz: 5.0kHz steps
    case 0x30: bwfstep = fstep * 15 / 2; break; // 915MHz: 7.5kHz steps
    default:
      mprintf_P(PSTR("invalid freq.band %B in "), bwfstep);
      dmpregs(1 - 1);
      return;
  }
  /*   bwfstep >  340*/ bw = 0x01 << 5;
  if ( bwfstep <= 340 ) bw = 0x02 << 5;
  if ( bwfstep <= 270 ) bw = 0x03 << 5;
  if ( bwfstep <= 200 ) bw = 0x04 << 5;
  if ( bwfstep <= 134 ) bw = 0x05 << 5;
  if ( bwfstep <=  65 ) bw = 0x06 << 5;
  rfm12regs[5 - 1] = (rfm12regs[5 - 1] & 0xFF1F) | bw;
#endif
  rfm12regs[10 - 1] = 0xC400; // disable AFC for exact frequency scan
  rf12_initialize(0);   // setup RFM12B, no interrupts
  rf12_recvStart();
  dmpreg(1 - 1);      // 80xx  1 configuration (Fband)
  mprintf_P(PSTR("R2 %W -> "), R2old); dmpreg(3 - 1); // Axxx  3 frequency setting
  mprintf_P(PSTR("R4 %W -> "), R4old); dmpreg(5 - 1); // 9xxx  5 receiver control
  mprintf_P(PSTR("R9 %W -> "), R9old); dmpreg(10 - 1); // C4xx 10 AFC
  mprintf_P(PSTR("change freq delay %Dus\n"), fdly);
  mprintf_P(PSTR("scanfrequband start t=%L\n"), millis());
  while ( ! Serial.available() ) {
    word f;
    byte rssi, i;
    byte maxrssi = 0;
    unsigned long tbeg = millis();
    activityLed(0);
    for ( f = flo, i = 0; f <= fhi && i < RF_MAX - 1; f += fstep, ++i ) {
      setFrequency(f);
      delayMicroseconds(fdly);
      rssi = getRSSI(do_DRSSI);
      if ( rssi > maxrssi ) {
        maxrssi = rssi;
        activityLed(maxrssi > RSSIthr);
      }
      rxstat[i] = '0' + rssi + LNAgain; // absolute RSSI, RFM12.pdf p.19
    }
    // report only scans where current RSSI threshold met:
    if ( maxrssi > RSSIthr ) {
      rxstat[i] = '\0';
      mprintf_P(PSTR("%L %s %L\n"), tbeg, rxstat, millis());
    }
  }
  rf12_recvStop();
  rf12_control_ec3k(R2old); // restore Axxx  3 frequency setting
  rf12_control_ec3k(R4old); // restore 9xxx  5 receiver control
  rf12_control_ec3k(R9old); // restore C4xx 10 AFC
  activityLed(0);
  mprintf_P(PSTR("scanfrequband exit t=%L\n"), millis());
}
static void scanfrequval( word flo, word fhi, word fstep, word fdly )
{
  scanfrequband(false, flo, fhi, fstep, fdly);
}
static void scanfrequthr( word flo, word fhi, word fstep, word fdly )
{
  scanfrequband(true,  flo, fhi, fstep, fdly);
}

// scan RSSI on/off pulses
//
static uint8_t  fill;
static uint16_t pulses[256];
static uint16_t lastus;
static unsigned long pbegms, pendms;
static uint16_t pbegstat, pendstat;

static void scanpulses( word freq, byte bw, byte LNA, byte drssi )
{
  byte  rssi, orssi;
  uint16_t  statreg, width, usec, scans, pkts;

  rf12_initialize(0);   // setup RFM12B, no interrupts
  setFrequency(freq);
  setRSSIbwLNA(drssi, bw, LNA);
  dmpreg(3 - 1);  // Axxx  3 frequency setting reg
  dmpreg(5 - 1);  // 9xxx  5 receiver control reg
  dmpreg(10 - 1); // C4xx 10 AFC
  rf12_recvStart();
  mprintf_P(PSTR("scanpulses start t=%L\n"), millis());

  pkts = 0;
  orssi = 0;
  fill = 0;
  scans = 0;
  for (;;) {
    rssi = (((statreg = rf12_control_ec3k(0x0000)) >> 8) & 1);
    usec = micros();
    width = usec - lastus;
    ++scans;
    if (rssi == orssi) {
      if (!rssi && fill && width > 60000) {
        activityLed(0);
        pendms = millis();
        pendstat = statreg;
        // pulse n=nn b=msec stat e=msec stat scans=nn 1wid 0wid ...
        mprintf_P(PSTR("pulse n=%D b=%L %W e=%L %W scans=%D"),
                  (int)fill, pbegms, pbegstat, pendms, pendstat, scans);
        for ( byte i = 0; i < fill; ++i )
          mprintf_P(PSTR(" %D"), pulses[i]);
        mprintnl();
        fill = 0;
        scans = 0;
        ++pkts;
      }
      if (width > 60000 && Serial.available()) {
        rf12_recvStop();
        mprintf_P(PSTR("scanpulses exit t=%L np=%D\n"), millis(), pkts);
        break;
      }
      continue;
    }
    orssi = rssi;
    lastus = usec;
    if (fill == 0 && rssi) {
      pbegms = millis();
      pbegstat = statreg;
      scans = 0;
      activityLed(1);
      continue;
    }
    pulses[fill++] = width;
  }
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Energy Count 3000 utilities

static uint16_t
mem2word( uint8_t * data )  // convert 2 bytes to word
{
  return data[0] << 8 | data[1];
}

static uint32_t
mem2long( uint8_t * data )  // convert 4 bytes to long
{
  return (uint32_t)mem2word(data + 0) << 16 | mem2word(data + 2);
}

// left shift block of bytes
//
static void
lshift( unsigned char * buff, uint8_t blen, uint8_t shift )
{
  uint8_t offs, bits, slen, i;
  uint16_t  wbuf;

  if ( shift == 0 )
    return;
  offs = shift / 8;
  bits = 8 - shift % 8;
  slen = blen - offs - 1;
  wbuf = buff[offs];
  for ( i = 0; i < slen; ++i ) {
    wbuf = wbuf << 8 | buff[i + offs + 1];
    buff[i] = wbuf >> bits;
  }
  buff[slen] = wbuf << (uint8_t)(8 - bits);
}

// delete 0-bits inserted after 5 consecutive 1-bits and reverse bit-sequence
//  EC3k packets are transmitted with HDLC bit-stuffing and LSBit first
//  decode EC3k packets needs these 0-bits deleted and bits reversed to
//  MSBit first
//
static void
del0bitins_revbits( unsigned char * buff, uint8_t blen )
{
  uint8_t sval, dval, bit;
  byte  si, sbi, di, dbi, n1bits;

  di = dval = dbi = n1bits = 0;
  for ( si = 0; si < blen; ++si ) {
    sval = buff[si];      // get source byte
    for ( sbi = 0; sbi < 8; ++sbi ) {
      bit = sval & 0x80;      // get source-bit
      sval <<= 1;       // process source MSBit to LSBit
      if ( n1bits >= 5 && bit == 0 ) {  // 5 1-bits and 1 0-bit
        n1bits = 0;     //  reset counter
        continue;     //  and skip 0-bit
      }
      if ( bit )  n1bits++;   // count consecutive 1-bits
      else        n1bits = 0;   // 0-bit: reset counter
      dval = dval >> 1 | bit;   // add source-bit to destination
      ++dbi;        //  reversing destin bit-sequence
      if ( dbi == 8 ) {     // destination byte complete
        buff[di++] = dval;    //  store it
        dval = dbi = 0;     //  reset for next destin byte
      }
    }
  }
  if ( dbi )
    buff[di] = dval >> (byte)(8 - dbi);
}

// count 1-bits in 32bit value
//  see http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetTable
//
static const byte tab1bits[256] PROGMEM = { // lookup-table for 1-bits in byte
#   define B2(n)  n,     n+1,     n+1,     n+2
#   define B4(n)  B2(n), B2(n+1), B2(n+1), B2(n+2)
#   define B6(n)  B4(n), B4(n+1), B4(n+1), B4(n+2)
  B6(0), B6(1),   B6(1),   B6(2)
};
static byte
count1bits( uint32_t value )
{
  byte  n1bits = 0;
  byte  ibyte;
  uint8_t vbyte;

  for ( ibyte = 0; ibyte < sizeof(value); ++ibyte ) {
    vbyte = value & 0xFF;
    if ( vbyte )
      n1bits += pgm_read_byte(&tab1bits[vbyte]);
    value >>= 8;
  }
  return n1bits;
}

// compare blocks reporting number of bit mismatches
//  rblock in RAM, pattern in PROGMEM
//
static uint8_t
bitcmp( uint8_t *rblock, uint8_t PROGMEM *pattern, uint8_t plen )
{
  uint8_t bxor, ebits = 0;

  while ( plen-- ) {
    bxor = (uint8_t)(*rblock++ ^ pgm_read_byte(pattern++));
    ebits += pgm_read_byte(&tab1bits[bxor]);
  }
  return ebits;
}

// generic multiplicative (self-synchronizing) de-scrambler
//  descramble Energy Count 3000 packets needs only the polynomial
//  x^18+x^17+x^13+x^12+x+1 (0x31801) plus bit-inversion, equivalant
//  to Non-Return-to-Zero-Space (NRZS) and polynomial x^17+x^12+1 and
//  to Non-Return-to-Zero-Inverted (NRZI) and polynomial x^17+x^12+1
//  plus bit-inversion
//  scrambler polynomial hexadecimal representation with exponents
//  as 1-relative bit numbers and an implicit + 1 term
//  e.g. 0x8810 for CRC-16-CCITT polynomial x^16 + x^12 + x^5 + 1
//         x^16       + x^12                + x^5             + 1
//  0x8810  1  0  0  0   1  0  0  0   0  0  0  1   0  0  0  0
//         16 15 14 13  12 11 10  9   8  7  6  5   4  3  2  1
//  see http://www.hackipedia.org/Checksums/CRC/html/Cyclic_redundancy_check.htm
//      "Specification of CRC" and "Representations...(reverse of reciprocal)"
//  and http://www.ece.cmu.edu/%7Ekoopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
//      "hexadecimal representation" on page 2 in section "2.Background"
//  descramblers for Energy Count 3000 found with Berlekamp-Massey algorithm:
//    2111 1111 111             polynomial exponents 20..1
//          0987 6543 2109 8765 4321
//  0x10800    1 0000 1000 0000 0000  x^17+x^12+1
//  0x31801   11 0001 1000 0000 0001  x^18+x^17+x^13+x^12+x+1
//  0x52802  101 0010 1000 0000 0010  x^19+x^17+x^14+x^12+x^2+1
//  0xF7807 1111 0111 1000 0000 0111  20,19,18,17,15,14,13,12,3,2,1
//  ITU-T V.52 PN9 PseudoRandomBitSequence, see RFM22 and trc103 manuals:
//    0x110              1 0001 0000  x^9+x^5+1
//  IEEE 802.11b scrambler, see "4.4 Scrambler" page 16 (PDF page 17)
//  in http://epubl.ltu.se/1402-1617/2001/066/LTU-EX-01066-SE.pdf
//     0x48                 100 1000  x^7+x^4+1
//  descramb() takes 3328 .. 3524 usec per 70 byte block
//
#define SCRAMPOLY  0x31801  // polynomial x^18+x^17+x^13+x^12+x+1

static void
descramb( uint8_t* buff, size_t len )
{
  byte  ibit, obit;
  byte  bit;
  uint8_t inpbyte, outbyte;
  uint32_t  scramshift;

  exectimebeg();
  scramshift = 0xFFFFFFFF;
  while ( len-- ) {
    inpbyte = *buff;
    for ( bit = 0; bit < 8; ++bit ) {
      // RFM12B receives MSBit first into bytes:
      ibit = (inpbyte & 0x80) >> 7;
      obit = ibit ^ (count1bits(scramshift & SCRAMPOLY) & 0x01);
      scramshift = scramshift << 1 | ibit;
      inpbyte <<= 1;
      outbyte = outbyte << 1 | obit;
    }
    *buff++ = outbyte ^ 0xFF;
  }
  exectimeprt(PSTR("descramb"));
}

// get start of Energy Count 3000 packet
//  packets start with common sequence before changing ID field:
//    [00 C6 3F] 33 DF 97 25 8A F9 59 DA 02 7E 15 ED 67 13 F1 85 D3 AC D2
//  leading 00 C6 3F may be missing
//  inspect packet at offsets 13..16 for match of the common sequence's
//  trailing bytes (13 F1 85 D3[AC]) left-shifted 0..7 bits
//  tolerate match with up to 3 bit erros
//  takes 24..228 usec
//  returns:
//  bitoffset of ID field, block needs left-shift bitoffset % 8 bits
//  EC3KNOMATCH if no match found
//
#define EC3KNOMATCH 0xFF
#define EC3KMAXEBITS   3  /* max. bit errors in match */
#define EC3KBEGOFFS 13  /* offset of ec3kbeg in rblock  */
#define EC3KMAXOFFS  3  /* how far to search in rblock  */
#define EC3KBLEN   4  /* length of EC3K begin pattern */
#define BLS(b1,b2,b3,b4,b5,s)         \
  ((b1 << s) | (b2 >> (8-s))),    \
  ((b2 << s) | (b3 >> (8-s))),    \
  ((b3 << s) | (b4 >> (8-s))),  \
  ((b4 << s) | (b5 >> (8-s)))

static const uint8_t  ec3kbeg[] PROGMEM = {
  0x13, 0xF1, 0x85, 0xD3,
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 7),
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 6),
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 5),
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 4),
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 3),
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 2),
  BLS(0x13, 0xF1, 0x85, 0xD3, 0xAC, 1),
};

static byte
ec3kmatch( uint8_t *rblock, size_t rblen )
{
  uint8_t   offs, bit, bitoffs, ebits, mebits;

  exectimebeg();
  /*
  * require packet at least 21 bytes long prevents check beyond received data
  * check shorter (18..20 bytes) packets would be possible, but is not useful,
  * because they anyway are too short for a valid EC3k packet
  */
  if ( rblen < EC3KBEGOFFS + EC3KMAXOFFS + EC3KBLEN + 1 )
    return EC3KNOMATCH;
  mebits  = EC3KMAXEBITS + 1;
  bitoffs = EC3KNOMATCH;
  /*
  * compare received block with ec3kbeg shifted 0,7..1 bits at some offsets
  * both for-loops stop if match without bit errors (mebits == 0) found
  */
  for ( bit = 0; bit < 8 && mebits; ++bit ) {
    for ( offs = EC3KBEGOFFS; offs <= EC3KBEGOFFS + EC3KMAXOFFS && mebits; ++offs ) {
      ebits = bitcmp(rblock + offs, (uint8_t *) ec3kbeg + EC3KBLEN * bit, EC3KBLEN);
      if ( ebits < mebits ) { // less required or previous bit erorrs
        mebits = ebits;   //  use bitoffs with least bit errors
        /*
        * for bit == 0 add 8 bits to offset, else no addition,
        * because the shifted patterns (bit 7..1) include an
        * additional byte of the common sequence
        */
        bitoffs = (offs + EC3KBLEN + 1) * 8 + bit + (bit ? 0 : 8);
      }
    }
  }
  exectimeprt(PSTR("ec3kmatch"));
  return bitoffs;
}

// convert descrambled Energy Count 3000 packets to interpretable format
//  packet 'rblock' is expected descrambled with common sequence at 'offs'
//  delete 0-bits inserted after 5 consecutive 1-bits starting at ID byte
//  and reverse bits per byte, i.e. move bit-7,6..0 to bit-0,1..7
//  left-shift packet by 4 bits and delete common sequence before ID byte
//  takes 684..728 usec
//
static uint16_t ec3kcrc;

static byte
ec3krevshift( byte idoffs, uint8_t *rblock, size_t rblen )
{
  size_t  i, ec3klen;
  uint16_t  crc;

  exectimebeg();
  ec3klen = rblen - idoffs;
  del0bitins_revbits(rblock + idoffs, ec3klen);
  crc = 0xFFFF;
  if ( ec3klen >= EC3KLEN )
    for ( i = 0; i < EC3KLEN; ++i )
      crc = _crc_ccitt_update(crc, rblock[idoffs + i]);
  ec3kcrc = crc;
  lshift(rblock, rblen, 4 + idoffs * 8);
  exectimeprt(PSTR("ec3krevshift"));
  return ec3klen;
}

// count bytes received with CRL on within EC3k data
//
static byte
ec3kcntCRL( byte idoffs, byte len )
{
  byte  i;
  byte  endoffs = idoffs + (len < EC3KLEN ? len : EC3KLEN);
  byte  CRL = 0;

  for ( i = idoffs; i < endoffs; ++i )
    if ( rxstat[i] & RFM12_STATUS_CRL )
      ++CRL;
  return CRL;
}

#if DATAFLASH
// check if EC3k packet changed versus last saved
//  used for longer dataflash log by not storing (almost) same EC3k packets
//  EC3k packets are regarded same if only the total time LSWord differs,
//  occurs only with Watt == 0 i.e. device at EC3k switched off
//  regard packets different around wrap of the total time LSWord and
//  every 10 minutes
//
struct ec3klast {
  word  id; // EC3k ID
  word  page; // dataflash page where last packet from EC3k ID saved
  byte  offs; // offset of 'E' record in page
};
static struct ec3klast ec3klast[6]; // last EC3k packets per ID for compare

static boolean
ec3kchange( uint8_t *ec3kpkt )
{
  word    page = 0;
  byte    offs, i;
  uint16_t    id, ntsec, otsec;
  struct {
    byte      type;
    byte      length;
    struct dataec3k   E;
  }     Erec;

  ntsec = mem2word(ec3kpkt + 2);
  if ( ntsec < 0x0010 || 0xFFF0 < ntsec )
    return true;    // changed: always save around wrap total time

  // lookup last page,offs for EC3k ID
  id = mem2word(ec3kpkt + 0);
  for ( i = 0; i < alenof(ec3klast); ++i )
    if ( ec3klast[i].id == id )
      break;
  if ( i >= alenof(ec3klast) )
    return true;    // changed: id not in ec3klast

  // get last EC3k data with this ID for compare
  page = ec3klast[i].page;
  offs = ec3klast[i].offs;
  if ( page == df_wrap(dfLastPage + 1) )  // previous EC3k still in dfBuf
    memcpy(&Erec, &dfBuf.data[offs], sizeof(Erec));
  else          // saved, read from dataflash
    df_read(page, offs, &Erec, sizeof(Erec));
  otsec = mem2word(&Erec.E.pkt[2]);
  if ( ntsec - otsec >= 10 * 60 )
    return true;    // changed: save every 10 minutes
  // compare this and last EC3k data between time total and checksum exclusive
  if ( memcmp(&Erec.E.pkt[4], ec3kpkt + 4, 38 - 4) != 0 )
    return true;    // changed: EC3k data different
  return false;   // same EC3k data
}

static void
ec3kchsave( uint8_t *ec3kpkt, word page, byte offs )
{
  byte    i, ifound, ifree;
  uint16_t    id;

  ifound = ifree = 0;
  id = mem2word(ec3kpkt + 0);
  for ( i = 0; i < alenof(ec3klast); ++i ) {
    if ( ec3klast[i].id == id )
      ifound = i + 1;
    if ( ec3klast[i].id == 0 && !ifree )
      ifree = i + 1;
  }
  if ( !ifound ) {    // ID not (yet) in ec3klast table
    if ( !ifree )
      return;   //  cannot save: no free ec3klast entry
    ifound = ifree;   //  save in first free entry (if any)
  }
  i = ifound - 1;
  ec3klast[i].id   = id;
  ec3klast[i].page = page;
  ec3klast[i].offs = offs;
}
#else  /*not DATAFLASH*/

#define ec3kchange(ec3kpkt)   (true)
#define ec3kchsave(ec3kpkt,page,offs)

#endif /*DATAFLASH*/

// append received EC3k packet to dataflash log
//
static void
df_append_ec3k( byte bitoffs, byte crl )
{
  struct dataec3k dataec3k;

  if ( !df_present() )
    return;
  memcpy(&dataec3k.rxinfo, (void*)&rxinfo, sizeof(dataec3k.rxinfo));
  dataec3k.boffs = bitoffs; // number of bits preceeding EC3k packet
  dataec3k.crl   = crl; // ClockRecoveryLocked count for pkt[]
  memcpy(dataec3k.pkt, (void*)rxbuf, sizeof(dataec3k.pkt));
  df_append(DFR_REC3K, (void*)&dataec3k, sizeof(dataec3k));
  // note dataflash page,offs of last EC3k packet per ID for eck3change()
  ec3kchsave((uint8_t*)rxbuf, df_wrap(dfLastPage + 1), dfFill - sizeof(dataec3k) - 2);
}

// report Energy Count 3000 packet data
//      EC3k packet offsets (offs9) below start with "9" before ID
//                             1     1   1   1                   2   3   3   3   3 3 3   4 4
//-2-1 0 1 2   4   6   8       2     5   7   9                   9   1   3   5   7 8 9   1 2
// IbBfSID--TsecZeroOsecZero---Wslo---WattWmax?unknown-----------?ThiZero-WshiOhiRcOZCsumEfIb....
// 557E96EA19B1300009AAF0000000EB3FC88032E068F84946848B4853D884DFB14100000000314105802FB87E5555555
//  0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 3 3 3 4 4 4 4 4
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4
//     ^^------v  EC3k packet offsets (offs) above start with ID
//     |       v
//     `offs9  offs len
//   -2      -   1   Ib   idle byte 0x55 preceding HDLC frame
//   -1      -   1   Bf   HDLC begin flag 0x7E
//    0      -   0.4 S    start mark, always 9
//    0.4    0   2   ID   Energy Count 3000 transmitter ID
//    2.4    2   2   Tsec time total in seconds lower 16 bits
//    4.4    4   2   Zero 00-bytes
//    6.4    6   2   Osec time ON in seconds lower 16 bits
//    8.4    8   3.4 Zero 00-bytes
//   12     11.4 3.4 Wslo energy in watt-seconds lower 28 bits
//   15.4   15   2   Watt current watt*10
//   17.4   17   2   Wmax maximum watt*10
//   19.4   19  10   ?unknown? (seems to be used by EC3k internal calculations)
//   29.4   29   1.4 Thi  time total in seconds upper 12 bits
//   31     30.4 2.4 Zero 00-bytes
//   33.4   33   2   Wshi energy in watt-seconds upper 16 (or 12?) bits
//   35.4   35   1.4 Ohi  time ON in seconds upper 12 bits
//   36     36.4 1   Rc   reset counter: number of EC3k transmitter resets
//   37     37.4 0.4 O    device on/off flag: 0xX8 = ON (watt!=0), 0xX0 = off (watt==0)
//   38.4   38   0.4 Z    0-nibble
//   39     38.4 2   Csum checksum CRC-CCITT starting with start mark 9
//   41     40.4 1   Ef   HDLC end flag 0x7E
//   42     41.4 1+  Ib   idle bytes 0x55 following HDLC frame
//  trailing ".4" means half byte (4 bits)
//
static byte
ec3kdecode( uint8_t *ec3kpkt, byte crl )
{
  uint16_t  id, lswsec, mswsec, losec, Watt, Wmax;
  uint16_t  Wshi, Wsmid, Wslo;
  uint8_t   nres, onoff;
  uint32_t  tsec, osec, Whrs;
  byte    nocrl;

  id   = mem2word(ec3kpkt + 0);
  lswsec = mem2word(ec3kpkt + 2);
  mswsec = mem2word(ec3kpkt + 29) >> 4;
  tsec = (uint32_t)mswsec << 16 | lswsec;
  lswsec = mem2word(ec3kpkt + 6);
  mswsec = mem2word(ec3kpkt + 35) >> 4;
  osec = (uint32_t)mswsec << 16 | lswsec;
  Watt = mem2word(ec3kpkt + 15);
  Wmax = mem2word(ec3kpkt + 17);
  Wslo  = mem2word(ec3kpkt + 13);
  Wsmid = mem2word(ec3kpkt + 11);
  Wshi  = mem2word(ec3kpkt + 33);
  Wsmid = (Wsmid & 0x0FFF) | (Wshi << 12);
  Wshi >>= 4;
  nres = (uint8_t)(ec3kpkt[36] << 4) | (uint8_t)(ec3kpkt[37] >> 4);
  onoff = ec3kpkt[37] & 0x08;
  //generates much more code for 64bit (+4,2k), could be avoided by an own implementation of division without 64bit
  //Ws are 40bits, 3600 = 2^4 * 225 -> shift Ws 4bits right = 36Bits left to divide by 225
  //Option: use only 36Bits, means 32bits left after shifting which gets max 19088 kWh - could be enough ;-), but we have no problem with flash-size at the moment)
  Whrs   = ((uint64_t)Wshi << 32 | (uint32_t)Wsmid << 16 | (uint32_t)Wslo) / 3600; //with only 32bits: max 1193kWh

  //  report EC3k packet contents
#if VERBOSE
  mprintf_P(PSTR("EC3K %L ID=%W %L sT %L sON"), rxtbegus, id, tsec, osec);
#endif

  //old - gets wrong Ws because of rubbish if-clause and wrong calculation of Ws
  // #if 1    // avoid big code with 64bit arithmetic for Ws
  // if ( ec3kpkt[10] == 0 ) {
  // if ( ec3kpkt[11] == 0 && ec3kpkt[12] == 0 ) {
  // #if VERBOSE
  // mprintf_P(PSTR(" %D Ws"), mem2word(ec3kpkt+13));
  // #endif
  // } else {
  // Whrs = mem2long(ec3kpkt+11) / 3600;
  // #if VERBOSE
  // mprintf_P(PSTR(" %L Wh"), Whrs);
  // #endif
  // }
  // } else {
  // #if VERBOSE
  // mprintf_P(PSTR(" %B%B%B%B%B Ws"),
  // ec3kpkt+10, ec3kpkt+11, ec3kpkt+12, ec3kpkt+13, ec3kpkt+14);
  // #endif
  // }
  // #else    // 64bit arithmetic for Ws makes code 3.2kB bigger
  // uint32_t Wh;
  // uint64_t Ws;
  // Ws = mem2long(ec3kpkt+10) << 8 | ec3kpkt[14];
  // Wh = Ws / 3600;
  // Whrs = Wh;
  // #if VERBOSE
  // mprintf_P(PSTR(" %L"), Wh);
  // #endif
  // #endif
#if VERBOSE
  mprintf_P(PSTR(" %D.%D W %D.%D Wmax %D resets"),
            Watt / 10, Watt % 10, Wmax / 10, Wmax % 10, nres);
  if ( !ec3kchange(ec3kpkt) )
    mprintf_P(PSTR(" same"));
#endif
  nocrl = EC3KLEN - crl;  // checks above assert ec3klen >= EC3KLEN
#if VERBOSE
  if ( nocrl )  // EC3k bytes received with ClockRecoveryLock bit off
    mprintf_P(PSTR(" (%D noCRL)"), nocrl);
  mprintnl();
#endif

  // ---------------------------------------------------------------------------------------
  // - crc calc algorithm is unknown, therefore rely on ClockRecoveryLock for each bit
  // - the higher the nocrl counter, more likely it is that an incorrect packet was received
  // - therefore a filter could be used (like: if(nocrl <= 2)...)
  //
  // output format:
  //   01 - static jeenode id 22 (defined in nodemap.local)
  //   02 - ec3k sender id
  //   03 - seconds total
  //   04 - seconds on
  //   05 - watt hours
  //   06 - actual consumption (shifted by 10)
  //   07 - max. consumption (shifted by 10)
  //   08 - number of resets
  //   09 - bits without ClockRecoveryLock on
  // ---------------------------------------------------------------------------------------

  if (nocrl >= 0) {
    byte payl [20];
    payl[0]  = id >> 8;       // ec3k sender id
    payl[1]  = id;
    payl[2]  = tsec >> 24;    // seconds total
    payl[3]  = tsec >> 16;
    payl[4]  = tsec >> 8;
    payl[5]  = tsec;
    payl[6]  = osec >> 24;    // seconds on
    payl[7]  = osec >> 16;
    payl[8]  = osec >> 8;
    payl[9]  = osec;
    payl[10] = Whrs >> 24;    // watt hours
    payl[11] = Whrs >> 16;
    payl[12] = Whrs >> 8;
    payl[13] = Whrs;
    payl[14] = Watt >> 8;     // actual watt consumption
    payl[15] = Watt;
    payl[16] = Wmax >> 8;     // watt max
    payl[17] = Wmax;
    payl[18] = nres;          // resets
    payl[19] = nocrl;         // bits without ClockRecoverLock on

    mprintf_P(PSTR("OK %D"), NODEID);
    for (int ix = 0; ix < sizeof payl; ix++)
      mprintf_P(PSTR(" %D"), payl[ix]);
    mprintnl();

#if VERBOSE
    mprintf_P(PSTR("$ 22 %W %L %L %L %D %D %D %D"), id, tsec, osec, Whrs, Watt, Wmax, nres, nocrl);
    mprintnl();
#endif
  }

  return bOK;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// data receive

// hexadecimal dump data bytes
//
static void hexdumpspc( uint8_t * data, size_t dlen, uint8_t ispc )
{
  uint8_t spcnt;

  for ( spcnt = 0; dlen != 0; --dlen, ++spcnt ) {
    if ( ispc && spcnt == ispc ) {
      spcnt = 0;
      putchr(' ');
    }
    mprintf_P(PSTR(" %B"), *data++);
  }
  mprintnl();
}
static void hexdump( uint8_t * data, size_t dlen )
{
  hexdumpspc(data, dlen, 0);
}

// append received block to dataflash log
//
static void
df_append_recv( uint8_t *rxdata )
{
  df_append(DFR_RINFO, (void*)&rxinfo, sizeof(rxinfo));
  df_append(DFR_RDATA, rxdata,  rxfill);
  df_append(DFR_RSTAT, (void*)rxstat, rxfill);
}

// process and report received data block
//
static void reportrxinfo( byte bitoffs )
{
  char  tdsign = '+';
  int   tdrssi;

  if ( (tdrssi = -rxinfo.dus_rssi) < 0 ) {
    tdsign = '-';
    tdrssi = -tdrssi;
  }

#if VERBOSE
  mprintf_P(PSTR("recv len=%D ovr=%D r=%L %W b=%L %C%D %W %W e=%L +%D %W stat=%W crl=%D"),
            rxfill, rxovr,
            rxrssius, statrssi,
            rxtbegus, tdsign, tdrssi, statbeg, statbeg8,
            rxtendus, rxinfo.dus_end, statend, rfm12istat, rxcrl);
  if ( bitoffs != EC3KNOMATCH )
    mprintf_P(PSTR(" boffs %D"), bitoffs);
  mprintnl();
#endif
}

static void processrecv( boolean do_EC3K, uint8_t descram )
{
  unsigned long trxrssius;
  byte  bitoffs = EC3KNOMATCH;
  byte  ec3klen, ec3kCRL;
  uint8_t rxorig[rxfill];

  // setup rxinfo delta times, used by df_append_recv() and df_append_ec3k():
  rxinfo.dus_rssi = (trxrssius = rxrssius) ? trxrssius - rxtbegus : 0;
  rxinfo.dus_end  = rxtendus - rxtbegus;

  // save original receive data, because EC3k decode modifies rxbuf contents,
  // but on decode failure the original data shall be written to dataflash
  memcpy(rxorig, (void*)rxbuf, rxfill);

  if ( do_EC3K
       && (bitoffs = ec3kmatch((uint8_t*)rxbuf, rxfill)) != EC3KNOMATCH ) {
    // left shift received block by bitoffset % 8 of EC3k match
    // left-shift less than 8 bits (not whole bitoffs) keeps leading
    // bytes needed for descramble and rxbuf[] match rxstat[] indexes
    // takes 6 or 128..258 usec for no shift or shift 1..7 bits
    exectimebeg();
    lshift((uint8_t*)rxbuf, rxfill, bitoffs % 8);
    exectimeprt(PSTR("lshift"));
  }
  if ( descram != 0x0C ) {
    if ( descram != 0x0D )
      df_append_recv(rxorig); // append received block to dataflash log
    reportrxinfo(bitoffs);
    mprintf_P(PSTR("data")); hexdump((uint8_t*)rxbuf, rxfill);
  }
  if ( descram && bitoffs != EC3KNOMATCH ) {
    descramb((uint8_t*)rxbuf, rxfill);
    if ( descram != 0x0C ) {
      mprintf_P(PSTR("dscr")); hexdump((uint8_t*)rxbuf, rxfill);
    }
    if ( descram == 0x0C || descram == 0x0D ) {
      ec3klen = ec3krevshift(bitoffs / 8, (uint8_t*)rxbuf, rxfill);
      ec3kCRL = ec3kcntCRL(bitoffs / 8, ec3klen);
      if ( descram != 0x0C ) {
        mprintf_P(PSTR("ec3k")); hexdump((uint8_t*)rxbuf, ec3klen);
      }
      if ( ec3klen >= EC3KLEN
           && ec3kcrc == 0xF0B8  /* from ec3krevshift() */
           && ec3kdecode((uint8_t*)rxbuf, ec3kCRL) == OK ) {
        if ( ec3kchange((uint8_t*)rxbuf) )
          df_append_ec3k(bitoffs, ec3kCRL);
      } else if ( descram == 0x0C ) {
        // EC3k decode failed: log receive data and report rxinfo
        df_append_recv(rxorig);
        reportrxinfo(bitoffs);
      }
    }
  }
  if ( descram != 0x0C ) {
    mprintf_P(PSTR(  "stat")); hexdump((uint8_t*)rxstat, rxfill);
  }
}

// restart RFM12B and report state
//  called from drecvintr() and drecvpoll() if RFM12B apparently
//  hanging or stuck, i.e. no receive data for several seconds
//
static void
rfm12_restart( byte irq_on )
{
  byte    eint0 = bitRead(EIMSK, INT0);
  uint16_t    stat  = rf12_control_ec3k(0x0000);

  mprintf_P(PSTR("RFM12 hang: rfm12istat=%W stat=%W rxfill=%B rxovr=%W EI0=%B regs:\n"),
            rfm12istat, stat, rxfill, rxovr, eint0);
  dmpregs();
  mprintf_P(PSTR("RFM12 restart "));
  rf12_recvStop();
  activityLed(0);
  exectimebeg();
  if ( (rfm12regs[7 - 1] & 0x01) == 0 ) {
    // sw reset command needs sensitive reset mode enabled (dr=0)
    // see RFM12B.pdf p.35 "SW Reset Command" and p.20 "7. FIFO"
    mprintf_P(PSTR("swreset "));
    rf12_control_ec3k(RF_SW_RESET);
  }
  rf12_initialize(irq_on);  // setup RFM12B, interrupts enabled/disabled
  exectimeprt(PSTR(" took"));
  rf12_recvStart();
}


// receive data with interrupts
//
#define TMORESTART  15500 // restart RFM12B after 15.5 sec no receive
#define RECVRAW   false // receive and dump raw data
#define RECVEC3K  true  // receive and match Energy Count 3000 data

static void drecvintr( boolean do_EC3K, uint8_t descram )
{
  size_t    pkts;
  unsigned long tlastrecv;

  switch ( descram ) {
    case 0:   // do not descramble data
    case 0x03:    // 0x31801 descrambler 1,12,13,17,18 + bit-inversion
    case 0x0C:    // descramble and decode EC3k packets, silent
    case 0x0D:    // descramble and decode EC3k packets, verbose
      break;
    default:
      mprintf_P(PSTR("bad descrambler: %B\n"), descram);
      return;
  }
  rf12_initialize(1);   // setup RFM12B, interrupts enabled
  dmpreg(3 - 1);  // 3 frequency
  dmpreg(4 - 1);  // 4 data rate
  dmpreg(5 - 1);  // 5 receiver control reg
  dmpreg(6 - 1);  // 6 data filter
  dmpreg(7 - 1);  // 7 fifo and reset mode
  dmpreg(8 - 1);  // 8 sync pattern
  dmpreg(10 - 1); // 10 AFC
  rf12_recvStart();

  mprintf_P(PSTR("\n  drecvintr start t=%L\n\n"), millis());

  pkts = 0;
  tlastrecv = 0;
  while ( ! Serial.available() ) {
    if ( rxfill == 0 && rxrssius == 0 ) {
      statrssi = rf12_control_ec3k(0x0000);
      if ( statrssi & RFM12_STATUS_RSSI )
        rxrssius = micros();
    }
    if ( rxfill && !(rfm12istat & RFM12_STATUS_RSSI) ) {
      rf12_recvStop();
      ++pkts;
      processrecv(do_EC3K, descram);  // process+report received block
      rf12_recvStart();     // reset vars, restart FIFO fill
      tlastrecv = millis();
      continue;
    }
    if ( tlastrecv && millis() - tlastrecv > TMORESTART ) {
      rfm12_restart(1);   // restart RFM12B with interrupts enabled
      tlastrecv = millis();
    }
  }
  rf12_recvStop();
  activityLed(0);
  mprintf_P(PSTR("drecvintr exit t=%L np=%D\n"), millis(), pkts);
}

static void
datarecvintr( void )
{
  drecvintr( RECVRAW, 0 );
}

static void
ec3krecvintr( uint8_t descram )
{
  drecvintr( RECVEC3K, descram );
}

static void
ec3krecvdec( void )
{
  drecvintr( RECVEC3K, 0x0C );
}


// receive data with polling
//
static void drecvpoll( boolean do_EC3K, uint8_t descram )
{
  size_t    pkts;
  unsigned long tlastrecv;
  unsigned long rssirise;
  byte    receive;

  switch ( descram ) {
    case 0:   // do not descramble data
    case 0x03:    // 0x31801 descrambler 1,12,13,17,18 + bit-inversion
    case 0x0C:    // descramble and decode EC3k packets, silent
    case 0x0D:    // descramble and decode EC3k packets, verbose
      break;
    default:
      mprintf_P(PSTR("bad descrambler: %B\n"), descram);
      return;
  }
  rf12_initialize(0);   // setup RFM12B, interrupts disabled
  dmpreg(3 - 1);  // 3 frequency
  dmpreg(4 - 1);  // 4 data rate
  dmpreg(5 - 1);  // 5 receiver control reg
  dmpreg(6 - 1);  // 6 data filter
  dmpreg(7 - 1);  // 7 fifo and reset mode
  dmpreg(8 - 1);  // 8 sync pattern
  dmpreg(10 - 1); // 10 AFC
  rf12_recvStart();
  mprintf_P(PSTR("drecvpoll start t=%L\n"), millis());

  pkts = 0;
  tlastrecv = 0;
  receive = 0;
  while ( ! Serial.available() ) {
    rfm12istat = rf12_xfer(0x0000);
    rxrssius = micros();
    if ( rfm12istat & RFM12_STATUS_RSSI ) {
      if ( !receive ) {
        statrssi = rfm12istat;
        rssirise = rxrssius;
        receive = 1;
        activityLed(1);
        continue;
      }
      if ( rfm12istat & RFM12_STATUS_FFIT ) {
        uint8_t rbyte = rf12_xfer(RF_RX_FIFO_READ);
        if ( rxfill == 0 ) {
          statbeg  = rfm12istat;
          rxtbegus = rxrssius;
        } else if ( rxfill == 7 )
          statbeg8 = rfm12istat;
        if ( rxfill < alenof(rxbuf) ) {
          rxbuf[rxfill]  = rbyte;
          rxstat[rxfill] = (uint8_t)rfm12istat & 0xFF;
          ++rxfill;
        } else
          ++rxovr;
        if ( rfm12istat & RFM12_STATUS_CRL )
          ++rxcrl;
        continue;
      }
    } else {
      if ( receive ) {
        activityLed(0);
        statend  = rfm12istat;
        rxtendus = rxrssius;
        rxrssius = rssirise;
        ++pkts;       // count and
        processrecv(do_EC3K, descram);  //  process+report received block
        rf12_recvStart();   // reset vars, restart FIFO fill
        tlastrecv = millis();
        continue;
      }
    }
    if ( tlastrecv && millis() - tlastrecv > TMORESTART ) {
      rfm12_restart(0);   // restart RFM12B with interrupts disabled
      tlastrecv = millis();
    }
  }
  rf12_recvStop();
  activityLed(0);
  mprintf_P(PSTR("drecvpoll exit t=%L np=%D\n"), millis(), pkts);
}

static void
datarecvpoll( void )
{
  drecvpoll( RECVRAW, 0 );
}

static void
ec3krecvpoll( uint8_t descram )
{
  drecvpoll( RECVEC3K, descram );
}


#if USE_TESTFIFO
// test RFM12 fifo
//    receive with polling every single bit to check if LSB or MSB is 1st in FIFO
//    20.284kbps yields 49.3usec/bit
//    rf12_control_ec3k() SPI transfer 2 bytes needs ca. 13/10 usec with/without caching
//    invent rf12xfer3() and rf12xfer4() for read status and 8,16 FIFO bits
//
static void testfifo( uint16_t fifolvl, uint16_t a_rfcmdB0 )
{
  uint16_t    R6old = rfm12regs[7 - 1];
  byte    rfcmdB0 = (a_rfcmdB0 != 0);
  byte    receive, i;
  unsigned long statfifo;

  rfm12regs[7 - 1]  = 0xCA00 | (rfm12regs[7 - 1] & 0x000F);
  rfm12regs[7 - 1] |= (fifolvl & 0x0F) << 4;
  rf12_initialize(0);   // setup RFM12B, no interrupts
  dmpreg(3 - 1);  // Axxx  3 frequency
  dmpreg(4 - 1);  // C6xx  4 data rate
  dmpreg(5 - 1);  // 9xxx  5 receiver control reg
  dmpreg(6 - 1);  // C2xx  6 data filter
  mprintf_P(PSTR("R6 %W -> "), R6old);
  dmpreg(7 - 1);  // CAxx  7 FIFO and reset mode
  dmpreg(8 - 1);  // CExx  8 sync pattern
  dmpreg(10 - 1); // C4xx 10 AFC
  rf12_recvStart();
  mprintf_P(PSTR("testfifo start t=%L\n"), millis());
  receive = 0;
  while ( ! Serial.available() ) {
    rfm12istat = rf12_xfer(0x0000);
    rxrssius = micros();
    if ( rfm12istat & RFM12_STATUS_RSSI ) {
      if ( !receive ) {
        statbeg  = rfm12istat;
        rxtbegus = rxrssius;
        receive = 1;
        activityLed(1);
        continue;
      }
      if ( rfm12istat & RFM12_STATUS_FFIT
           && rxfill < alenof(rxbuf)
           && rxfill < alenof(pulses) ) {
        if ( rfcmdB0 ) {
          rxbuf[rxfill + 0] = rf12_xfer(RF_RX_FIFO_READ);
        } else {
          statfifo = rf12_statfifo();
          rxbuf[rxfill + 0]  = (statfifo >> 8) & 0xFF;
          rxbuf[rxfill + 1]  =  statfifo & 0xFF;
        }
        rxstat[rxfill + 0] = (rfm12istat >> 8) & 0xFF;
        rxstat[rxfill + 1] =  rfm12istat & 0xFF;
        pulses[rxfill + 0] = (rxrssius >> 16) & 0xFFFF;
        pulses[rxfill + 1] =  rxrssius & 0xFFFF;
        rxfill += 2;
        continue;
      }
    } else {
      if ( receive ) {
        activityLed(0);
        statend  = rfm12istat;
        rxtendus = rxrssius;
        // report
        mprintf_P(PSTR("recvfifo len=%D b=%L %W e=%L %W\nusec"),
                  rxfill, rxtbegus, statbeg, rxtendus, statend);
        for ( i = 0; i < rxfill; i += 2 ) {
          mprintf_P(PSTR(" %W"),
                    pulses[i + 1] - (i ? pulses[i - 1] : (rxtbegus & 0xFFFF)));
        }
        mprintf_P(PSTR("\nstat"));
        for ( i = 0; i < rxfill; i += 2 )
          mprintf_P(PSTR(" %W"), rxstat[i + 0] << 8 | rxstat[i + 1]);
        mprintf_P(PSTR("\nfifo"));
        for ( i = 0; i < rxfill; i += 2 )
          if ( rfcmdB0 )
            mprintf_P(PSTR("   %B"), rxbuf[i + 0]);
          else
            mprintf_P(PSTR(" %W"), rxbuf[i + 0] << 8 | rxbuf[i + 1]);
        mprintnl();
        // reset record buffers, waiting for receive
        rxfill = 0;
        receive = 0;
        // restart FIFO fill after DRSSI went off
        rf12_xfer(rfm12regs[7 - 1] & ~0x0002); // FIFO fill disable
        rf12_xfer(rfm12regs[7 - 1] |  0x0002); // FIFO fill enable
      }
    }
  }
  rf12_recvStop();
  rf12_control_ec3k(R6old); // restore CAxx  7 FIFO and reset mode
  activityLed(0);
  mprintf_P(PSTR("testfifo exit t=%L\n"), millis());
}
#endif /*USE_TESTFIFO*/


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// miscellaneous commands

// date+time synchronization with host
//  DT utcsecs.msec
//    calculate millis() offset to Unix UTC seconds
//    report previous offset and new offset
//  DT yyyy-mm-dd hh:mm:ss.uuu
//  DT
//    report current offset, ignore ISO date+time arg
//  under Linux/Unix with GNU 'date' send UTC seconds to USB device
//  where JeeLink is connected:
//    stty -F /dev/ttyUSB0 57600 -hupcl
//    date '+DT %s.%3N' > /dev/ttyUSB0
//
#define MINUTCSEC 946684800 // 2000-01-01 00:00:00 UTC

#define tsyncmsec dtsync.msec // millis() of last time sync
#define tsyncutcsec dtsync.utcsec // UTC time ...
#define tsyncutcmsec  dtsync.utcmsec  // ... where millis() == 0

static void
reportsync( PGM_P pfx, struct datadtsyn *pdtsync )
{
  mprintf_P(PSTR("%Ssync "), pfx);
  _putdlongwf(pdtsync->msec / 1000, 10, ' ');
  putchr('.');
  _putdlongwf(pdtsync->msec % 1000, 3, '0');
  mprintf_P(PSTR(" local "));
  _putdlongwf(pdtsync->utcsec, 10, ' ');
  putchr('.');
  _putdlongwf((unsigned long)pdtsync->utcmsec, 3, '0');
  mprintf_P(PSTR(" UTC base\n"));
}
static void
dtime( char * arg1, char * arg2 )
{
  unsigned long nmsec = millis();
  unsigned long nutcsec;
  int     nutcmsec = 0;
  char    *pdot;
  byte    utcmslen, i;
  boolean   newsync;

  pdot = strchr(arg1, '.');
  nutcsec  = atol(arg1);
  if ( pdot != NULL ) {
    utcmslen = strlen(pdot + 1);
    if ( utcmslen > 3 )
      *(pdot + 4) = '\0'; // adjust .123456 .12345 .1234 to 123 msec
    nutcmsec = atoi(pdot + 1);
    for ( i = utcmslen; i < 3; ++i )
      nutcmsec *= 10; // adjust . .1 .12 to 0 100 120 msec
  }
  newsync = (nutcsec >= MINUTCSEC);
  reportsync(newsync ? PSTR("old ") : _nulstr, &dtsync);
  if ( newsync ) {
    nutcsec  -= nmsec / 1000;
    nutcmsec -= nmsec % 1000;
    if ( nutcmsec < 0 ) {
      nutcmsec += 1000;
      nutcsec  -= 1;
    }
    tsyncmsec    = nmsec;
    tsyncutcsec  = nutcsec;
    tsyncutcmsec = nutcmsec;
    reportsync(PSTR("new "), &dtsync);
    df_append(DFR_DTSYN, &dtsync, sizeof(dtsync));
  }
}

// set console baudrate
//  baudrates below worked with Linux minicom version 2.4
//  baudrate 460k8 did not work due 21% difference
//  baudrate 2000k would work, but no higher throughput (30kB/s) than 1000k
//
static void
comsetbaud( byte baud )
{
  long  baudrate;

  switch ( baud ) {
    case 0x05:  baudrate =   57600; break;
    case 0x01:  baudrate =  115200; break;
    case 0x02:  baudrate =  230400; break;
    case 0x03:  baudrate =  500000; break;
    case 0x04:  baudrate = 1000000; break;
    case 0x00:  baudrate = combaud; break;  // report current
    default:
      mprintf_P(PSTR("bad baud: %B\n"), baud);
      return;
  }
  if ( baud ) {
    combaud = baudrate;
    Serial.begin(baudrate);
  }
#if VERBOSE
  mprintf_P(PSTR("%L baud\n"), baudrate);
#endif
}





#if DATAFLASH
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// dataflash commands

// check dataflash not found and report
//
static boolean
df_notfound( void )
{
  if ( !df_present() ) {
    mprintf_P(PSTR("DataFlash not present\n"));
    return true;
  }
  return false;
}

// dataflash hex dump
//
static void
df_hexdump( word page, word offs, unsigned long len )
{
  size_t  dlen;
  uint8_t dbuf[32];

  if ( df_notfound() )
    return;
  while ( len && ! Serial.available() ) {
    if ( page >= DF_MEM_TOTAL )
      break;
    dlen = len;
    if ( dlen > sizeof(dbuf) )
      dlen = sizeof(dbuf);
    df_read(page, offs, dbuf, dlen);
    mprintf_P(PSTR("%W%B:"), page, offs);
    hexdumpspc(dbuf, dlen, 8);
    offs += dlen;
    if ( offs & 0xFF00 ) {
      offs &= 0xFF;
      ++page;
    }
    len  -= dlen;
  }
}
static void
df_hexdumpol( word page, byte offs, word len )
{
  df_hexdump(page, (word)offs, (unsigned long)len);
}
static void
df_hexdumpbe( word begpage, word endpage )
{
  df_hexdump(begpage, 0, (unsigned long)((endpage - begpage) * 256));
}

// dataflash scan sectors for filled pages
//  report dataflash address and seqnum,timestamp for
//  - first filled page per sector
//  - last filled before empty page
//  - pages around changed seqnum
//
static void
df_scanpages( void )
{
  word  page, prevpage;
  struct {
    word seqnum;
    long timestamp;
    word crc;
  }   curr, prev;
  struct {
    byte              type;   // 'T'
    byte              length; // 10
    struct datadtsyn  T;    // timesync with host: millis() UTC sec.ms base
  }   Trec;

  if ( df_notfound() )
    return;
  exectimebeg();
  prev.seqnum = 0xFFFF;
  for ( page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page ) {
    if ( Serial.available() ) // user abort
      break;
    // read marker from page in flash
    df_read(page, sizeof(dfBuf.data), &curr, sizeof curr);
    // report 1st filled page per sector, filled after empty page
    // and pages around changed sequence number
    if ( curr.seqnum != prev.seqnum && prev.seqnum != 0xFFFF )
      mprintf_P(PSTR("%W00: %D %L\n"),
                page - 1, prev.seqnum, prev.timestamp);
    if ( curr.seqnum != 0xFFFF
         && (curr.seqnum != prev.seqnum || page % DF_BLOCK_SIZE == 0) ) {
      mprintf_P(PSTR("%W00: %D %L"),
                page, curr.seqnum, curr.timestamp);
      df_read(page, 0, &Trec, sizeof(Trec));
      if ( Trec.type == DFR_DTSYN && Trec.length == sizeof(Trec.T) )
        reportsync(PSTR(" "), &Trec.T);
      else
        mprintnl();
    }
    prev = curr;
  }
  exectimeprt(PSTR("df_scanpages"));
}

// dataflash scan filled pages for TLV records
//
static void
df_scanprecs( word begpage, word endpage )
{
#define badmark   (_spcqm)
#define goodmark  (_nulstr)
  word  page, offs, crc;
  struct {
    word seqnum;
    long timestamp;
    word crc;
  }   curr;
  struct {
    struct {
      byte type;
      byte length;
    }    h;
    union {
      unsigned long     Rusec;  // receive time
      struct {
        struct rxinfo R;
        byte          crl;
        byte          boffs;
        byte          id[2];    // Energy Count 3000 ID at packet begin
      }                 E;
      struct datadtsyn  T;      // timesync millis() with host UTC
    }    u;
  }   rec;
  byte  i;
  uint8_t dbuf[sizeof(dfBuf) / 4];

  if ( df_notfound() )
    return;
  exectimebeg();
  for ( page = begpage; page < endpage; ++page ) {
    if ( Serial.available() ) // user abort
      break;
    if ( page >= DF_MEM_TOTAL ) // DataFlash size exceeded
      break;
    // read marker from page in flash
    df_read(page, sizeof(dfBuf.data), &curr, sizeof curr);
    if ( curr.seqnum == 0xFFFF )
      continue;     // skip empty pages
    // test checksum of page
    crc = ~0;
    for ( offs = 0; offs < sizeof(dfBuf); offs += sizeof(dbuf) ) {
      df_read(page, offs, dbuf, sizeof(dbuf));
      for ( i = 0; i < sizeof(dbuf); ++i )
        crc = _crc16_update(crc, dbuf[i]);
    }
    mprintf_P(PSTR("%W00: %D %L%S\n"),
              page, curr.seqnum, curr.timestamp,
              crc != 0 ? badmark : goodmark);
    offs = 0;
    do {
      df_read(page, offs, &rec, sizeof(rec));
      if ( rec.h.type == 0xFF && rec.h.length == 0xFF )
        break;
      mprintf_P(PSTR("%W%B:  "), page, offs);
      mprintf_P(rec.h.type < ' ' || '~' < rec.h.type
                ? PSTR("%B %D") : PSTR("%C %D"),
                rec.h.type, rec.h.length);
      offs = offs + sizeof(rec.h) + rec.h.length;
      if ( offs <= sizeof(dfBuf.data) ) {
        switch ( rec.h.type ) {
          case DFR_REC3K:
          case DFR_RINFO:
            mprintf_P(PSTR(" %L"), rec.u.Rusec);
            if ( rec.h.type == DFR_REC3K )
              mprintf_P(PSTR(" %B%B"), rec.u.E.id[0], rec.u.E.id[1]);
            break;
          case DFR_DTSYN:
            reportsync(PSTR(" "), &rec.u.T);
            continue;   // reportsync() prints newline
        }
      } else {
        mprintf_P(badmark); // bad record length
      }
      mprintnl();
    } while ( offs < sizeof(dfBuf.data) );
  }
  exectimeprt(PSTR("df_scanprecs"));
}

// dataflash intel-hex upload
//  see https://secure.wikimedia.org/wikipedia/en/wiki/Intel_HEX
//  intel-hex record types:
//    ll aaaa 00 dd dd .. cc  data, adress aaaa LSWord of 32bit address
//    00 0000 01 FF           end of file
//    02 0000 04 aa aa cc     extended linear address, aaaa 32bit addr MSW
//  ll length of data after record type up checksum exclusive
//  cc checksum is 0x100 - sum of bytes, i.e. all record bytes sum to 0x00
//  bytes are encoded as 2 characters 0..9,A..F
//  records start with ':' and end with CR LF ('\x0D '\x0A')
//  read whole 2 MB dataflash takes 15.003 msec = ca. 136.5 kB/sec
//
#define MAXIHEXLEN  32
struct ihex {       // intel-hex record
  uint8_t len;      //  length of data
  uint16_t  addr;     //  address for data, big-endian!
  uint8_t type;     //  record type
  uint8_t data[MAXIHEXLEN]; //  data bytes
  uint8_t csum;     //  checksum
};

static byte
upldhbyte( byte (*pputchr)(byte), uint8_t hbyte )
{
  byte  rc;

  rc = (*pputchr)(to_asc(hbyte >> 4));
  if ( rc != bOK )
    return rc;
  rc = (*pputchr)(to_asc((byte)(hbyte & 0x0F)));
  return rc;
}
static byte
upldihex( byte (*pputchr)(byte), struct ihex * pihex )
{
  uint16_t  addr = pihex->addr;
  uint8_t *dptr = &pihex->len;
  uint8_t chksum;
  byte  len, rc;

  // make address big-endian:
  pihex->addr = ((addr & 0x00FF) << 8) | ((addr & 0xFF00) >> 8);
  rc = (*pputchr)(':');   // send start mark
  chksum = 0x00;
  for ( len = 0; rc == bOK && (uint8_t)(4 + pihex->len) != len; ++len ) {
    rc = upldhbyte(pputchr, *dptr); // send header and data
    chksum += *dptr++;    // update checksum
  }
  chksum = (uint8_t)(0x100 - chksum);
  if ( rc == bOK )
    rc = upldhbyte(pputchr, chksum); // send checksum
  if ( rc == bOK )
    rc = (*pputchr)('\r');    // send trailing CR
  if ( rc == bOK )
    rc = (*pputchr)('\n');    //  and LF
  // back to original address to avoid confusing caller:
  pihex->addr = addr;
  return rc;
}
static void
df_upldihex( const struct prot PROGMEM *pprot, word begpage, word endpage )
{
  struct ihex ihex;
  byte  (*pputchr)(byte);
  word  page, offs;
  byte  rc, i;

  if ( df_notfound() )
    return;
  exectimebeg();
  rc = (*(byte (*)(void))pgm_read_word(&pprot->init))();
  pputchr = (byte (*)(byte))pgm_read_word(&pprot->putchr);
  for ( page = begpage; rc == bOK && page < endpage; ++page ) {
    if ( page >= DF_MEM_TOTAL ) // DataFlash size exceeded
      break;
    // extended linear address record at begin and 64kB boundary
    if ( page == begpage || (page & 0x00FF) == 0 ) {
      // exectimeprt(PSTR("read64kB"));
      ihex.len  = 2;
      ihex.addr = 0x0000;
      ihex.type = 0x04;
      ihex.data[0] = 0x00;
      ihex.data[1] = (page & 0xFF00) >> 8;
      rc = upldihex(pputchr, &ihex);
      // exectimebeg();
    }
    for ( offs = 0x00; rc == bOK && offs < 0x100; offs += MAXIHEXLEN ) {
      df_read(page, offs, ihex.data, MAXIHEXLEN);
      for ( i = 0; i < MAXIHEXLEN; ++i )
        if ( ihex.data[i] != 0xFF )
          break;
      if ( i >= MAXIHEXLEN )
        continue;     // do not send all-FF data
      ihex.len  = MAXIHEXLEN;
      ihex.addr = ((page & 0x00FF) << 8) | offs;
      ihex.type = 0x00;
      rc = upldihex(pputchr, &ihex);
    }
  }
  if ( rc == bOK ) {
    // end-of-file record
    ihex.len  = 0;
    ihex.addr = 0x0000;
    ihex.type = 0x01;
    upldihex(pputchr, &ihex);
    rc = (*(byte (*)(void))pgm_read_word(&pprot->exit))();
  }
  exectimeprt(PSTR("df_upldihex"));
  reportrc(PSTR("upload "), rc);
}

static void
df_upldssbe( word begpage, word endpage )
{
  df_upldihex(&ssprot, begpage, endpage);
}
static void
df_upldsslog( void )
{
  df_upldssbe(DF_LOG_BEGIN, DF_LOG_LIMIT);
}

#if USE_XMODEM
static void
df_upldxmbe( word begpage, word endpage )
{
  df_upldihex(&xmprot, begpage, endpage);
}
static void
df_upldxmlog( void )
{
  df_upldxmbe(DF_LOG_BEGIN, DF_LOG_LIMIT);
}
#endif /*USE_XMODEM*/

#endif /*DATAFLASH*/


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// command line interface

const char helpText[] PROGMEM =
  "commands are short words, have hexadecimal arguments and end with CR" "\n"
  " L n        turn activity LED at PB1 on (1) or off (0)" "\n"
  " R [reg]    show all or specified RFM12B register" "\n"
  " R reg val  modify RFM12B register, reg = 0..F or 80,82..CE,E0" "\n"
  " FV flo fhi fstep [us]  frequency scan in current band, RSSI values" "\n"
  " FT flo fhi fstep [us]  frequency scan in current band, RSSI threshold" "\n"
  "    flo,fhi 60..F3F  lo,hi frequency * 2.5/5.0/7.5kHz" "\n"
  "    fstep    0..FF   fstep * 2.5/5.0/7.5kHz in 433/868/915MHz band" "\n"
  "    us       0..FFFF delay after change frequency, default 250 usec" "\n"
  " P freq bw lna rssi     scan radio pulses" "\n"
  "   freq 60..F3F 686=868.350MHz" "\n"
  "   bw    1..6    400..67kHz" "\n"
  "   lna   0..3      0..-20dBm" "\n"
  "   rssi  0..5   -103..-72dBm" "\n"
  " DI         data receive with interrupts" "\n"
  " DP         data receive with polling" "\n"
  " E [dscr]   Energy Count 3000 receive, descramble with dscr = 3,D" "\n"
  " EC         Energy Count 3000 receive and decode" "\n"
  " DT [secs[.ms]]         synchronize millis() time with UTC secs.ms" "\n"
  " SB baud                set baudrate 5:57k6 1:115k2 2:230k4 3:500k 4:1000k" "\n"
#if USE_TESTFIFO
  " T flvl 0|1 test receive fifo with 0:read status+fifo, 1:readfifo command" "\n"
  "   flvl  0..F    FIFO fill level" "\n"
#endif
  ;
const char helpTextDF[] PROGMEM =
#if DATAFLASH
  " DFM page offs len      dataflash memory hexdump" "\n"
  "     begp endp           with end page address" "\n"
  " DFU[X] [begp endp]     upload dataflash as intel-hex [with Xmodem]" "\n"
  " DFS [begp endp]        scan dataflash filled pages [and records]" "\n"
#else
  ""
#endif
  ;
const char helpTail[] PROGMEM =
  " ?          help" "\n"
  "any console input stops running command" "\n"
  ;

static void showProg( void )
{
  unsigned long usec = micros();
  unsigned long msec = millis();
  mprintf_P(PSTR("\n[" PROGNAME "." PROGVERS "]\n\n"
                 "  freeram %D\n"
                 "  msectime %L usectime %L\n"), freeRam(), msec, usec);
}

static void showHelp( void )
{
  mprintf_P(helpText);
  if ( df_present() )
    mprintf_P(helpTextDF);
  mprintf_P(helpTail);
}

static void help( void )
{
  showProg();
  showHelp();
}

/* command argument types */
#define Ano 0
#define ABYTE 1
#define AWORD 2
#define ATEXT 3
#define args(n,a1,a2,a3,a4) (uint8_t)(n), (uint8_t)( a1 | a2<<2 | a3<<4 | a4<<6 )
#define argtype(atyps,n)  (uint8_t)( (atyps >> 2*n) & 0x03 )

#define EVAFPTR   char * (*)( char*, uint16_t* )
#define CMDFPTR   char (*)( uint16_t, uint16_t, uint16_t, uint16_t )

struct cmdrec {       /* command record   */
  PGM_P     name;     /*  name of command   */
  uint8_t   narg;     /*  required number of arguments*/
  uint8_t   atyp;     /*  argument types    */
  char    (*exec)( uint16_t, uint16_t, uint16_t, uint16_t ); /*cmd.function*/
};

/* argument evaluation table */
char * (* const   evatab[])(char*, uint16_t*) PROGMEM = {
  (EVAFPTR)evalnone,  /* Ano:    no argument  */
  (EVAFPTR)evalbyte,  /* ABYTE: byte argument */
  (EVAFPTR)evalword,  /* AWORD: word argument */
  (EVAFPTR)evaltext,  /* ATEXT: string        */
};

/* command table */
static const char _DT[] PROGMEM = "DT";
static const char _L[]  PROGMEM = "L";
static const char _R[]  PROGMEM = "R";
static const char _FV[] PROGMEM = "FV";
static const char _FT[] PROGMEM = "FT";
static const char _P[]  PROGMEM = "P";
static const char _DI[] PROGMEM = "DI";
static const char _DP[] PROGMEM = "DP";
static const char _ED[] PROGMEM = "ED";
static const char _E[]  PROGMEM = "E";
static const char _SB[] PROGMEM = "SB";
static const char _H[]  PROGMEM = "H";
#if USE_TESTFIFO
static const char _T[]  PROGMEM = "T";
#endif
#if DATAFLASH
static const char _DFM[]  PROGMEM = "DFM";
static const char _DFUX[] PROGMEM = "DFUX";
static const char _DFU[]  PROGMEM = "DFU";
static const  char _DFS[]  PROGMEM = "DFS";
#endif
const struct cmdrec cmdtab[] PROGMEM = {
  { _DT,  args(2, ATEXT, ATEXT, Ano,  Ano ), (CMDFPTR)dtime        },
  { _DT,  args(1, ATEXT, Ano,  Ano,  Ano ), (CMDFPTR)dtime        },
  { _DT,  args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)dtime        },
  { _L,   args(1, ABYTE, Ano,  Ano,  Ano ), (CMDFPTR)activityLed  },
  { _R,   args(2, ABYTE, AWORD, Ano,  Ano ), (CMDFPTR)modreg       },
  { _R,   args(1, ABYTE, Ano,  Ano,  Ano ), (CMDFPTR)dmpreg       },
  { _R,   args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)dmpregs      },
  { _FV,  args(4, AWORD, AWORD, ABYTE, AWORD), (CMDFPTR)scanfrequval },
  { _FV,  args(3, AWORD, AWORD, ABYTE, Ano ), (CMDFPTR)scanfrequval },
  { _FT,  args(4, AWORD, AWORD, ABYTE, AWORD), (CMDFPTR)scanfrequthr },
  { _FT,  args(3, AWORD, AWORD, ABYTE, Ano ), (CMDFPTR)scanfrequthr },
  { _P,   args(4, AWORD, ABYTE, ABYTE, ABYTE), (CMDFPTR)scanpulses   },
  { _DI,  args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)datarecvintr },
  { _DP,  args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)datarecvpoll },
  { _ED,  args(1,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)ec3krecvdec  },
  { _E,   args(1, ABYTE, Ano,  Ano,  Ano ), (CMDFPTR)ec3krecvintr },
  { _E,   args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)ec3krecvintr },
#if USE_TESTFIFO
  { _T,   args(2, ABYTE, ABYTE, Ano,  Ano ), (CMDFPTR)testfifo     },
#endif
#if DATAFLASH
  { _DFM, args(3, AWORD, ABYTE, AWORD, Ano ), (CMDFPTR)df_hexdumpol },
  { _DFM, args(2, AWORD, AWORD, Ano,  Ano ), (CMDFPTR)df_hexdumpbe },
#if USE_XMODEM
  { _DFUX, args(2, AWORD, AWORD, Ano,  Ano ), (CMDFPTR)df_upldxmbe  },
  { _DFUX, args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)df_upldxmlog },
#endif
  { _DFU, args(2, AWORD, AWORD, Ano,  Ano ), (CMDFPTR)df_upldssbe  },
  { _DFU, args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)df_upldsslog },
  { _DFS, args(2, AWORD, AWORD, Ano,  Ano ), (CMDFPTR)df_scanprecs },
  { _DFS, args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)df_scanpages },
#endif
  { _SB,  args(1, ABYTE, Ano,  Ano,  Ano ), (CMDFPTR)comsetbaud   },
  { _SB,  args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)comsetbaud   },
  { _H,   args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)help         },
  { _qm,  args(0,  Ano,  Ano,  Ano,  Ano ), (CMDFPTR)help         },
  {  NULL }
};

/* parse & execute command-line
*  ----------------------------
* matches command at begin of command-line against command-name in
* records from command-table. command-names starting with the same
* character and commands with variable number of arguments will be
* detected with multiple records in table. first matching is done
* with max.name-length and/or max.number of arguments.
* arguments in command-line are parsed with their required types
* from the command-record.
* if command-name and -arguments the execution-routine is called
* with the max.number of arguments, it will ignore unneeded args.
*/
static void
cmdexec( char * cmdline )
{
  const struct cmdrec *cmdrec;
  char          *cmdptr;
  char *       (*evalfunc)(char*, uint16_t*);
  char         (*cmdfunc)(uint16_t, uint16_t, uint16_t, uint16_t);
  uint8_t       argc;
  uint16_t      argv[4];
  PGM_P         whatbad = PSTR("command");

  /* search command name in table matching required arguments */
  for ( cmdrec = cmdtab; pgm_read_word(&cmdrec->name) != NULL; ++cmdrec ) {
    if ( (cmdptr = skip_spc(cmdline)) == NULL )
      return;     /* got no command, don't parse */
    /* compare with cmd-name of this cmdtab entry, match skips command  */
    if ( (cmdptr
          = text_cmp((PGM_P)pgm_read_word(&cmdrec->name), cmdptr)) == NULL )
      continue;     /* cmd-name not matched, try next   */
    whatbad = PSTR("arg(s)");
    /* evaluate required command arguments */
    for ( argc = 0; argc < pgm_read_byte(&cmdrec->narg); ++argc ) {
      argv[argc] = 0;
      evalfunc      /* get+call evaluation for arg type */
        = (EVAFPTR)pgm_read_word(&evatab[ argtype(pgm_read_byte(
                                            &cmdrec->atyp), argc) ]);
      if ( (cmdptr = (*evalfunc)(cmdptr, &argv[argc])) == NULL )
        break;      /*  bad argument, stop evaluation   */
    }       /*  argument ok, evaluate next      */
    if ( cmdptr == NULL   /* found bad argument             */
         || skip_spc(cmdptr) != NULL ) /* or trailing input ?              */
      continue;     /*  yes, try next cmdtab entry      */
    /* get+call command function with evaluated args */
    cmdfunc = (CMDFPTR)pgm_read_word(&cmdrec->exec);
    (*cmdfunc)(argv[0], argv[1], argv[2], argv[3]);
    return;
  }
  /* didn't find cmd-name + required args */
  mprintf_P(PSTR("bad %S: '%s'\n"), whatbad, cmdline);
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// M A I N - program
//
void setup() {

  // cannot report reset reason, because optiboot clears MCUSR

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  activityLed(0);
#endif


  comsetbaud(5);  // 57600 baud

  showProg();

  rf12_regsinit();
  df_initialize();

#if VERBOSE
  showHelp();
  mprintf_P(PSTR(" msectime %L usectime %L\n"), millis(), micros());
#endif

#if AUTOSTART
  ec3krecvdec();
#endif
}

void loop() {
  mprintf_P(PSTR(">"));
  getline(linebuf, sizeof(linebuf));
  cmdexec(linebuf);
}



////////////////////////////////////////////////////////////////////////////////
//
//  FX201P HD36106 RAM Replacement
//
////////////////////////////////////////////////////////////////////////////////
//
// Emulates two HD36106 RAM chips, specificslly for the FX201P calculator
//
// Both RAM chips emulated
// RAM can be saved and loaded to and from RP2040 flash
// 
//
////////////////////////////////////////////////////////////////////////////////

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/structs/bus_ctrl.h"

#define DEBUG_STOP while(1) {}

// Some logic to analyse:
#include "hardware/structs/pwm.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#include "fx201p.h"

// Single shot trae, don't repeatedly capture data
#define SINGLE_SHOT     0

// whether to display bus transations over USB, it can mess the CLI up
#define NO_TRACE_USB    1

// On falling edge of CE, are both cycle counters reset?
#define RESET_BOTH_CYCLES	  1

////////////////////////////////////////////////////////////////////////////////
//
// Command codes
//

#define CMD_CLR      0x94          // Zero all
#define CMD_CLR_M    0x95          // Zero memories
#define CMD_CLR_P    0x96          // Zero program

#define CMD_LOAD     0x11          // Loads program and memory areas
#define CMD_LOAD_M   0x12          // Loads memory areas
#define CMD_LOAD_P   0x13          // Loads program areas

#define CMD_SAVE     0x15          // Saves program and memory areas
#define CMD_SAVE_M   0x16          // Saves memory areas
#define CMD_SAVE_P   0x17          // Saves program areas

#define CMD_CATALOG  0x47          // Shows empty slots

#define STATUS_CHAR_GOOD 0x0B
#define STATUS_CHAR_BAD  0x0D

#define GET_SPLIT_BYTE(NNN) (((*(command_ptr+(NNN+1))) & 0x0F) << 4) + (((*(command_ptr+NNN)) & 0xF0)>>4);

////////////////////////////////////////////////////////////////////////////////

#define TEXT_PARAMETER_LEN 40

const uint CAPTURE_PIN_BASE = 0;
const uint CAPTURE_PIN_COUNT = 32;
const uint CAPTURE_N_SAMPLES = 3000;

#define CLOCK_DIV  30.0f

const int P_A0     = 0;
const int P_DOUT   = 10;    // DOUT sense, for tracing

const int P_DOUTDRV = 12;   // DOUT that we drive
const int P_DOUTOE = 11;    // DOUT output enable

const int P_DIN    = 4;
const int P_RW     = 5;
const int P_CE     = 6;

#if PROTO
const int P_CLKA   = 9;
const int P_CLKB   = 8;
#else
const int P_CLKA   = 8;
const int P_CLKB   = 9;
#endif

const int P_CE2    = 14;
const int P_STAT1  = 16;

// Which clock we sample on
const int P_SAMPLE_CLK = P_CLKB;
  
#define READ_GPIO(NNN)          (((gpio_states = sio_hw->gpio_in) & (1 << NNN)) >> NNN)
#define CHECK_GPIO_STATE(NNN)   (((gpio_states                  ) & (1 << NNN)) >> NNN)
#define GET_GPIO_STATE(NNN)     (((gpio_states                  ) & (1 << NNN)) >> NNN)

#define ALL_HIGH 0xFFFFFFFFFFFFFFFF
#define GPIO_VALUE(XXX,NNN)     (((XXX) & (1 << NNN)) >> NNN)

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Serial loop command structure

typedef void (*FPTR)(void);

typedef struct
{
  char key;
  char *desc;
  FPTR fn;
} SERIAL_COMMAND;

int keypress = 0;
int parameter = 0;
int auto_increment_parameter = 0;
int auto_increment_address   = 0;

// Used when building data in a buffer
char hex_buffer[2048];
char key_buffer[2048];
char keystroke_buffer[20];

#define PRINT_LF     0
#define PRINT_NO_LF  1

char text_parameter[TEXT_PARAMETER_LEN+1] = "";

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//

#define NUM_BUS_T  1500

int queue_overflow = 0;

volatile int data_in = 0;
volatile int data_out = 0;

volatile int       bus_addr[NUM_BUS_T];
volatile int       bus_clk_count[NUM_BUS_T];

int last_ce        = 0;
int last_ce2       = 0;
int last_clk       = 0;
uint64_t ce_cycle  = 0;
uint64_t ce2_cycle = 0;

int ce          = 0;
int ce2         = 0;
int trans_i     = 0;
int dout        = 0;
int din         = 0;
int rw          = 0;
int address     = 0;
int clk         = 0;
uint64_t doutlo = 0;
uint64_t douthi = 0;

// Tracing of reads and writes is done in this buffer
volatile int stopped = 0;

#define TRACE_BIT  7
#define NUM_TRACE_ENTRIES  2000
volatile uint8_t exec_trace_address[NUM_TRACE_ENTRIES];
volatile uint8_t exec_trace_data[NUM_TRACE_ENTRIES];
volatile uint8_t exec_trace_flags[NUM_TRACE_ENTRIES];
volatile int     exec_trace_i = 0;

#define EXEC_TRACE_START_128    1
#define EXEC_TRACE_FLAG_R       1
#define EXEC_TRACE_FLAG_W       2

#define NUM_DATA_BITS 256
int bit_din[NUM_DATA_BITS];
int bit_dout[NUM_DATA_BITS];
int bit_rw[NUM_DATA_BITS];

int clk_count   = 0;

volatile int loop_count  = 0;

////////////////////////////////////////////////////////////////////////////////
//
// RAM emulation data
//
// Two HD36106s are emulated, one stores the program, the other stores memories.
//


#define DO_MP_MEMORIES    1
#define DO_MP_PROGRAM     2
#define DO_MP_BOTH        3

// Data is stored for all emulated RAMs as bytes
uint8_t ram_data[RAM_SIZE];

// Where are commands looked for?
#define ADDRESS_OF_MEM(NNN) (&(ram_data[8*NNN]))
#define MEM_OF_ADDRESS(AAA) ((AAA/8)-1)

#define COMMAND_RAM_ADDR ADDRESS_OF_MEM(8)

// We have a pointer to the commands so it can be changed, if required
uint8_t *command_ptr = COMMAND_RAM_ADDR;

////////////////////////////////////////////////////////////////////////////////
//
// Flash data
//
// We have several flash program slots.
// Slots are in the top 1Mbyte of flash
//
// The flash has to be erased in 4096 byte aligned blocks
// Writes can be 256 byte aligned but due to erase being 4096 we use that slot size.

// Program and dat aram is stored at the same time, so it is a snapshot of the entire
// RAM. RAM plkus program is 256 bytes.

#define FLASH_PROGRAM_DATA_SIZE         256
#define FLASH_PROGRAM_SLOT_SIZE         4096
#define FLASH_PROGRAM_SLOT_AREA_SIZE    (1000*1024)
#define FLASH_PROGRAM_NUM_SLOTS         (FLASH_PROGRAM_SLOT_AREA_SIZE / FLASH_PROGRAM_SLOT_SIZE)

#define FLASH_SLOT_OFFSET (1024*1024)
uint8_t *flash_slot_contents   = (uint8_t *) (XIP_BASE + FLASH_SLOT_OFFSET);

// general buffer for flash read and write
uint8_t slot_buffer[FLASH_PROGRAM_DATA_SIZE];

////////////////////////////////////////////////////////////////////////////////
//
// Slot layout:
//
// RAM_SIZE bytes      :  Program
// TEXT_PARAMETER_LEN  :  Slot label
//
////////////////////////////////////////////////////////////////////////////////
//
// Prototypes
//
////////////////////////////////////////////////////////////////////////////////

void print_keystroke_to_buffer(int byte, int no_lf);
void serial_help(void);
double mem_to_dbl(uint8_t *m);
char *mem_to_str(uint8_t *m);
void dbl_to_mem(double value, uint8_t *m);

////////////////////////////////////////////////////////////////////////////////

void set_gpio_input(int gpio_pin)
{
  gpio_init(gpio_pin);
  gpio_set_dir(gpio_pin, GPIO_IN);
  gpio_set_pulls(gpio_pin, 0, 0);
}

void set_gpio_output(int gpio_pin)
{
  gpio_init(gpio_pin);
  gpio_set_dir(gpio_pin, GPIO_OUT);
  gpio_put(gpio_pin, 0);
}

////////////////////////////////////////////////////////////////////////////////

void display_keystrokes_at(uint8_t *d)
{

  char *dest = key_buffer;
  *dest = '\0';
  
  for(int i=0; i<128; i++)
    {
      print_keystroke_to_buffer(*(d+128), PRINT_LF);
      d++;
      strcat(dest, keystroke_buffer);
    }

  printf("\nKeystrokes:\n%s\n", dest);
}

void display_keystrokes(void)
{
  display_keystrokes_at(&(ram_data[0]));
}

void display_ram_at(uint8_t *dat)
{
  printf("\n");
  
  for(int z = 0; z<RAM_SIZE; z++)
    {
      int byte = 0;
      
      if( (z % 8) == 0)
	{
	  if( (z != 0) && (z <= 128))
	    {
	      switch(z/8)
		{
		case 11:
		case 12:
		  printf("    Jump Indices");
		  break;
		  
		default:
		  if( ((*(dat-1)) & 0xF0) == 0xF0,1 )
		    {
		      printf("    M%02d %s %lg", MEM_OF_ADDRESS(z), mem_to_str(dat-8), mem_to_dbl(dat-8));

		      if( MEM_OF_ADDRESS(z)==4 )
			{
			  dbl_to_mem(mem_to_dbl(dat-8), dat-16);
			}
		    }
		  break;
		}
	    }
	  printf("\n%03X: ", z);
	}
      
      printf("%02X ", *(dat++));
    }
  
  printf("\n");

}

void display_ram(void)
{
  display_ram_at(&(ram_data[0]));

  // Display keystrokes after RAM hex data
  display_keystrokes();

}

////////////////////////////////////////////////////////////////////////////////
//
// Traces the GPIOs at every clock edge. Also stores data written in emulation RAM
// for both HD devices.
//
////////////////////////////////////////////////////////////////////////////////

int last_sample_clk = 0;
int sample_clk = 0;

void core1_main(void)
{
  int gpio_states = 0;
  clk_count = 0;
  
  irq_set_mask_enabled(0xffffffff, false);
    
  gpio_states = sio_hw->gpio_in;

  last_ce = GET_GPIO_STATE(P_CE);
  last_ce2 = GET_GPIO_STATE(P_CE2);
  
  // We run round this look only when the sample clock edge is seen
  while(1)
    {
      loop_count++;

      // All samples of signals have to be done on an edge of a clock
      // Wait here until that edge appears
      last_sample_clk = READ_GPIO(P_SAMPLE_CLK);

      while(1)
	{
	  sample_clk = READ_GPIO(P_SAMPLE_CLK);

	  if( (last_sample_clk == 1) && (sample_clk == 0) )
	    {
	      break;
	    }
	  
	  last_sample_clk = sample_clk;
	}

      //----------------------------------------
      // Clock edge, process signals
      //----------------------------------------
      
      ce  = GET_GPIO_STATE(P_CE);
      ce2 = GET_GPIO_STATE(P_CE2);
      rw  = GET_GPIO_STATE(P_RW);
      
      // Check for CE falling edges
      if( (last_ce == 1) && (ce == 0 ))
	{
	  // reset cycle counter
	  ce_cycle = 0;
#if RESET_BOTH_CYCLES	  
	  ce2_cycle = 0;
#endif
	}

      if( (last_ce2 == 1) && (ce2 == 0 ))
	{
	  // reset cycle counter
	  ce2_cycle = 0;
#if RESET_BOTH_CYCLES	  
	  ce_cycle = 0;
#endif
	}

      // Check for CE rising edges
      if( (last_ce == 0) && (ce == 1 ))
	{
	  // Disable DOUT as output
	  gpio_put(P_DOUTOE, 1);
	}

      if( (last_ce2 == 0) && (ce2 == 1 ))
	{
	  // Disable DOUT as output
	  gpio_put(P_DOUTOE, 1);
	}
      
      // We have gpio_states from the clock edge

      address = gpio_states;

      int addr = address & 0xF;
      
      // If ce is low and this is a write then write data bit to emulation RAM
      if( (ce==0) && (rw==0) )
	{
	  int bitno  = (ce_cycle & 0x7);
	  int byteno = (ce_cycle & 0x38) >> 3;
	  int din = GET_GPIO_STATE(P_DIN);

	  // Write bit
	  ram_data[(addr << 3)+byteno] &= ((1   << bitno) ^ 0xFF);
	  ram_data[(addr << 3)+byteno] |=  (din << bitno);

	  // trace only lsb bit of every byte
	  if( !stopped && (bitno == TRACE_BIT))
	    {
	      exec_trace_address[exec_trace_i] = (addr << 3)+byteno;
	      exec_trace_data[exec_trace_i] = ram_data[(addr << 3)+byteno];
	      exec_trace_flags[exec_trace_i] = EXEC_TRACE_FLAG_W;
	      exec_trace_i++;
	      if( exec_trace_i >= NUM_TRACE_ENTRIES )
		{
		  stopped = 1;
		}
	    }
	}

      if( (ce2==0) && (rw==0) )
	{
	  int bitno  = (ce2_cycle & 0x7);
	  int byteno = (ce2_cycle & 0x38) >> 3;
	  int din = GET_GPIO_STATE(P_DIN);
	  
	  // Write bit
	  ram_data[128+(addr << 3)+byteno] &= ((1   << bitno) ^ 0xFF);
	  ram_data[128+(addr << 3)+byteno] |= (din << bitno);
	  // trace only lsb bit of every byte
	  if( !stopped && (bitno == TRACE_BIT))
	    {
	      exec_trace_address[exec_trace_i] = 128+(addr << 3)+byteno;
	      exec_trace_data[exec_trace_i] = ram_data[128+(addr << 3)+byteno];
	      exec_trace_flags[exec_trace_i] = EXEC_TRACE_FLAG_W;
	      exec_trace_i++;
	      if( exec_trace_i >= NUM_TRACE_ENTRIES )
		{
		  stopped = 1;
		}
	    }
	}

      // If CE is low and RW is high then this is a read.
      // We drive DOUT from now until the time that CE goes high
      if( (ce==0) && (rw==1) )
	{
	  int bitno  = (ce_cycle & 0x7);
	  int byteno = (ce_cycle & 0x38) >> 3;
	  int doutval = (ram_data[(addr << 3)+byteno] & (1 << bitno)) >> bitno;
	  
	  // Drive DOUT signal and drive the data level
	  gpio_put(P_DOUTOE, 0);
	  gpio_put(P_DOUTDRV, doutval);
	}

      if( (ce2==0) && (rw==1) )
	{
	  int bitno  = (ce2_cycle & 0x7);
	  int byteno = (ce2_cycle & 0x38) >> 3;
	  int doutval = (ram_data[128+(addr << 3)+byteno] & (1 << bitno)) >> bitno;
	  
	  // Drive DOUT signal and drive the data level
	  gpio_put(P_DOUTOE, 0);
	  gpio_put(P_DOUTDRV, doutval);

	  // trace only lsb bit of every byte
	  if( !stopped && (bitno == TRACE_BIT))
	    {
	      exec_trace_address[exec_trace_i] = 128+(addr << 3)+byteno;
	      exec_trace_data[exec_trace_i] = ram_data[128+(addr << 3)+byteno];
	      exec_trace_flags[exec_trace_i] = EXEC_TRACE_FLAG_R;
	      exec_trace_i++;
	      if( exec_trace_i >= NUM_TRACE_ENTRIES )
		{
		  stopped = 1;
		}
	    }
	}
	    
      // Capture GPIO states
      // Only capture writes
      if( (address & (1 <<P_CE2)) == 0 )
	{
	  bus_addr[data_in]      = address;
	  bus_clk_count[data_in] = clk_count;
	  
	  int last_data_in = data_in;
	  
	  data_in = (data_in+1) % NUM_BUS_T;
	  
	  if( data_out == data_in )
	    {
#if SINGLE_SHOT
	      // We have stopped tracing, sit in loop
	      printf("\nCore 1 has stopped.\n");
	      
	      stopped = 1;
	      while(stopped)
		{
		}
	      // Reset
	      data_in = 0;
	      data_out = 0;
	      
#else
	      // No space
	      queue_overflow = 1;
	      data_in = last_data_in ;
#endif
	    }
	  else
	    {
	    }
	}

      clk_count++;
      last_ce  = ce;
      last_ce2 = ce2;
      ce_cycle++;
      ce2_cycle++;
    }
  
  
}

////////////////////////////////////////////////////////////////////////////////
//
// Display low level signal trace
//
////////////////////////////////////////////////////////////////////////////////

// This is a simple dump of the GPIOs, decode here

void process_bus(void)
{
  //Process data coming in from the bus via core 1
  while( data_in != data_out )
    {
      int gpios = bus_addr[data_out];
      
      printf("\n%04d: CC:%08X AD:%01X DI:%01X DO: %01X CE: %01X CE2:%01X  RW:%01X",
	     data_out,
	     bus_clk_count[data_out],
	     gpios & 0xF,
	     GPIO_VALUE(gpios, P_DIN),
	     GPIO_VALUE(gpios, P_DOUT),
	     GPIO_VALUE(gpios, P_CE),
	     GPIO_VALUE(gpios, P_CE2),
	     GPIO_VALUE(gpios, P_RW)
	     );
      
      data_out = (data_out + 1) % NUM_BUS_T;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Flash write/erase
//
////////////////////////////////////////////////////////////////////////////////

// Erase a slot
void erase_slot(int n)
{
  flash_range_erase(FLASH_SLOT_OFFSET+n*FLASH_PROGRAM_SLOT_SIZE, FLASH_PROGRAM_SLOT_SIZE);
}

// Checksum a slot
int checksum_slot(int slot_num)
{
  int csum = 0;

  for(int i=0; i<FLASH_PROGRAM_DATA_SIZE; i++)
    {
      csum += *(flash_slot_contents+slot_num*FLASH_PROGRAM_SLOT_SIZE+i);
    }

  return(csum);
}

void do_auto_increment_parameter(void)
{
  if( auto_increment_parameter )
    {
      parameter++;
    }
}

void do_auto_increment_address(void)
{
  if( auto_increment_address )
    {
      address++;
    }
}

////////////////////////////////////////////////////////////////////////////////

void print_keystroke_to_buffer(int byte, int no_lf)
{
  char * dest = keystroke_buffer;
  *dest = '\0';
  
  switch(byte)
    {
    case 0x00:
      break;
      
    case 0x80:
    default:
      sprintf(dest, "?(%02x)", byte);
      break;
      
    case 0xC4:
      if( no_lf )
	{
	  sprintf(dest, "  ST# ");
	}
      else
	{
	  sprintf(dest, "\n  ST# ");
	}
      break;
      
    case 0xC1:
      sprintf(dest, "MAC ");
      break;
      
    case 0xC2:
      sprintf(dest, "GOTO ");
      break;
      
    case 0xC3:
      sprintf(dest, "SUB# ");
      break;
      
    case 0xB1:
      sprintf(dest, "SQRT ");
      break;
    case 0xB2:
      sprintf(dest, "log ");
      break;
    case 0xB3:
      sprintf(dest, "ln ");
      break;
    case 0xB4:
      sprintf(dest, "e^x ");
      break;
    case 0xB5:
      sprintf(dest, "x^y ");
      break;
    case 0xB6:
      sprintf(dest, "10^x ");
      break;
    case 0xA2:
      sprintf(dest, "K ");
      break;
    case 0xA1:
      sprintf(dest, "+/- ");
      break;
    case 0xA3:
      sprintf(dest, "IF	 ");
      break;
    case 0xA4:
      sprintf(dest, "arc ");
      break;
    case 0xA5:
      sprintf(dest, "sin ");
      break;
    case 0xA6:
      sprintf(dest, "cos ");
      break;
    case 0xA7:
      sprintf(dest, "tan ");
      break;
    case 0xC6:
      sprintf(dest, "MJ ");
      break;
    case 0xC5:
      sprintf(dest, " : ");
      break;
    case 0xD1:
      sprintf(dest, "IM ");
      break;
    case 0xD0:
      sprintf(dest, "= ");
      break;
    case 0xD2:
      sprintf(dest, "ENT ");
      break;
    case 0xD3:
      sprintf(dest, "ANS ");
      break;
    case 0xD4:
      sprintf(dest, "M+ ");
      break;
    case 0xD6:
      sprintf(dest, "* ");
      break;
    case 0xD5:
      sprintf(dest, "/ ");
      break;
    case 0xD7:
      sprintf(dest, "+ ");
      break;
    case 0xD8:
      sprintf(dest, "- ");
      break;
    case 0xDD:
      sprintf(dest, "EXP ");
      break;
    case 0xD9:
      sprintf(dest, ". ");
      break;
    case 0xF0:
      sprintf(dest, "0 ");
      break;
    case 0xF1:
      sprintf(dest, "1");
      break;
    case 0xF2:
      sprintf(dest, "2");
      break;
    case 0xF3:
      sprintf(dest, "3");
      break;
    case 0xF4:
      sprintf(dest, "4");
      break;
    case 0xF5:
      sprintf(dest, "5");
      break;
    case 0xF6:
      sprintf(dest, "6");
      break;
    case 0xF7:
      sprintf(dest, "7");
      break;
    case 0xF8:
      sprintf(dest, "8");
      break;
    case 0xF9:
      sprintf(dest, "9");
      break;

    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Convert memory bytes to a double value
//
////////////////////////////////////////////////////////////////////////////////

double bcd_to_dec(uint8_t *b)
{
  double result = ((*b) & 0xF);

  result += (((*b) & 0xF0) >> 4) * 10.0;
  return(result);
}

int bcd_to_int(uint8_t *b)
{
  int result = ((*b) & 0xF);
  result += (((*b) & 0xF0) >> 4) * 10.0;
  return(result);
}

void dbl_to_bcd(double v, uint8_t *m)
{
  int byte;
  double intpart;
  double fracpart = modf(v/10.0, &intpart);

  byte = intpart * 16.0;
  byte += (fracpart * 10.0);

  *m = byte;
}

//------------------------------------------------------------------------------


void dbl_to_mem(double value, uint8_t *m)
{
  int sign = 0;
  int exp_sign = 0;

  // Build up sign byte
  if( value < 0.0 )
    {
      sign = 4;
    }
  else
    {
      sign = 0;
    }

  printf("\nSign:%d", sign);
  
  value = fabs(value);
  
  double exponent = floor(log10(value));

  printf("\nExponent:%g", exponent);

  // Normalise  
  value /= pow(10, exponent);
  
  if( exponent < 0.0 )
    {
      exponent = 100+exponent;
    }
  else
    {
      sign |= 0x1;
    }
  
  printf("\nSign byte:%02X", sign);
  
  // get digits
  double dpair;

  dpair = floor(value);
  dbl_to_bcd(dpair, m+5);
  value = value - floor(value);
  value *=100.0;

  printf("\ndpair: %g Value:%g", dpair, value);
  
  dpair = floor(value);
  dbl_to_bcd(dpair, m+4);
  value = value - floor(value);
  value *=100.0;

  printf("\ndpair: %g Value:%g", dpair, value);
    
  dpair = floor(value);
  dbl_to_bcd(dpair, m+3);
  value = value - floor(value);
  value *=100.0;

  printf("\ndpair: %g Value:%g", dpair, value);
  dpair = floor(value);
  dbl_to_bcd(dpair, m+2);
  value = value - floor(value);
  value *=100.0;

  dpair = floor(value);
  dbl_to_bcd(dpair, m+1);
  value = value - floor(value);
  value *=100.0;

  dpair = floor(value);
  dbl_to_bcd(dpair, m+0);
  value = value - floor(value);
  value *=100.0;

  // Write exponent and signs
  dbl_to_bcd(exponent, m+6);

  int signs = 0xF0;

  *(m+7) = sign; 
  
}

////////////////////////////////////////////////////////////////////////////////
//
// Converts a memory to a string representation
// Fixed format with exponent
//
////////////////////////////////////////////////////////////////////////////////

#define MEM_STR_LEN  40

char memstr[MEM_STR_LEN];

char *mem_to_str(uint8_t *m)
{
  char man_sgn_str[2] = " ";
  char exp_sgn_str[2] = " ";
  
  int exp = bcd_to_int((m+6));
  
  int exp_sign =  (*(m+7)) & 1;
  int man_sign = ((*(m+7)) & 4) >> 2;

  exp_sgn_str[0] = (exp_sign==0)? '-':' ';
  if( exp_sign == 0 )
    {
      exp = 100-exp;
    }
  
  man_sgn_str[0] = (man_sign==0)? ' ':'-';
  
  int r1 = bcd_to_int( (m+5));
  int r2 = bcd_to_int( (m+4));
  int r3 = bcd_to_int( (m+3));
  int r4 = bcd_to_int( (m+2));
  int r5 = bcd_to_int( (m+1));
  int r6 = bcd_to_int( (m+0));

  sprintf(memstr, "%s%02d.%02d%02d%02d%02d%02d %sE%02d",
	  man_sgn_str,
	  r1,
	  r2,
	  r3,
	  r4,
	  r5,
	  r6,
	  exp_sgn_str,
	  exp
	  );
  return(memstr);
}


double mem_to_dbl(uint8_t *m)
{
  double result = 0.0;
  double exp = bcd_to_dec((m+6));
  int exp_sign =  (*(m+7)) & 1;
  int man_sign = ((*(m+7)) & 4) >> 2;
  double exp_sgn = (exp_sign==0)? -1.0 : 1.0;
  double man_sgn = (man_sign==0)?  1.0 :-1.0;
  
  result  = bcd_to_dec( (m+5)) * 1.0;
  result += bcd_to_dec( (m+4)) * 0.01;
  result += bcd_to_dec( (m+3)) * 0.0001;
  result += bcd_to_dec( (m+2)) * 0.000001;
  result += bcd_to_dec( (m+1)) * 0.00000001;
  result += bcd_to_dec( (m+0)) * 0.0000000001;

  result *= man_sgn;
  if( exp_sign == 0 )
    {
      result *= pow(10, (100.0-exp)*-1);
    }
  else
    {
      result *= pow(10, exp);
    }
  
  return(result);
}

void print_keystroke(int byte, int no_lf)
{
  print_keystroke_to_buffer(byte, no_lf);
  printf("%s", keystroke_buffer);
}

void cli_boot_mass(void)
{
  reset_usb_boot(0,0);
}

// Another digit pressed, update the parameter variable
void cli_digit(void)
{
  parameter *= 10;
  parameter += keypress-'0';
}

void cli_zero_parameter(void)
{
  parameter = 0;
}

void cli_set_address(void)
{
  address = parameter;
}

void cli_information(void)
{
  printf("\n");

  //  printf("\nNumber of reads  :%d", num_rd);
  //printf("\nNumber of writes :%d", num_wr);
  printf("\nNumber of loops: %d", loop_count);
  printf("\n");
}

void cli_start_tracing(void)
{
  stopped = 0;
  exec_trace_i = 0;

  for(int i=0; i<NUM_TRACE_ENTRIES; i++)
    {
      exec_trace_address[i] = 0;
      exec_trace_data[i]    = 0;
      exec_trace_flags[i]   = 0;
    }
}

#define NUM_SLICES  5
#define TRACE_SLICE_SIZE (NUM_TRACE_ENTRIES/NUM_SLICES)
void cli_display_trace(void)
{
  printf("\n");
  
  for(int i=0; i<TRACE_SLICE_SIZE; i++)
    {
      for(int j = 0; j<NUM_SLICES; j++)
	{
	  char flags = '.';
	  switch(exec_trace_flags[i+j*TRACE_SLICE_SIZE] )
	    {
	    case EXEC_TRACE_FLAG_R:
	      flags = 'R';
	      break;
	      
	    case EXEC_TRACE_FLAG_W:
	      flags = 'W';
	      break;
	    }
	  
	  printf("  %05d: %c %02X %02X  %c",
		 i+j*TRACE_SLICE_SIZE,
		 (i+j*TRACE_SLICE_SIZE)==exec_trace_i?'*':' ',
		 exec_trace_address[i+j*TRACE_SLICE_SIZE],
		 exec_trace_data[i+j*TRACE_SLICE_SIZE],
		 flags);
	}
      printf("\n");
    }

  printf("\n");
}

void cli_write_byte(void)
{
  ram_data[address] = parameter;

  do_auto_increment_address();
  do_auto_increment_parameter();
  
  printf("\nWritten %02X to %03X\n", parameter, address);
}

void cli_erase_program_slot(void)
{
  printf("\nErasing program slot %d...", parameter);
  erase_slot(parameter);
  printf("\ndone.\n");

  do_auto_increment_parameter();
}

void cli_display_program_slot(void)
{
  
  printf("\nSlot %d\n", parameter);

  // First dump in hex
  display_ram_at(flash_slot_contents+parameter*FLASH_PROGRAM_SLOT_SIZE);

  printf("\n\n");

  // Then decode keystrokes
  display_keystrokes_at(flash_slot_contents+parameter*FLASH_PROGRAM_SLOT_SIZE);
  printf("\n");
}

void save_ram(int slotnum, int what_mp)
{
  // Save slot data in a buffer so we can write just memories or program and keep what was in the
  // other half
  memcpy(slot_buffer, flash_slot_contents+slotnum*FLASH_PROGRAM_SLOT_SIZE, RAM_SIZE);
  
  // Erase slot
  erase_slot(slotnum);
  
  // Write to slot buffer

  if ( what_mp & DO_MP_MEMORIES )
    {
      printf("\nWriting memory data to slot %d", slotnum);
      memcpy(&(slot_buffer[RAM_OFF_M]), &(ram_data[RAM_OFF_M]), RAM_SIZE_M);
    }

  if ( what_mp & DO_MP_PROGRAM )
    {
      printf("\nWriting program data to slot %d", slotnum);
      memcpy(&(slot_buffer[RAM_OFF_P]), &(ram_data[RAM_OFF_P]), RAM_SIZE_P);
    }

  // Write the buffer back
  flash_range_program(FLASH_SLOT_OFFSET + (FLASH_PROGRAM_SLOT_SIZE * slotnum), (uint8_t *) &(slot_buffer[0]), RAM_SIZE);

  printf("\nData written\n");
  
}

int choose_what_mp(void)
{
  int what = 0;
  int key = 0;
  
  printf("\nMemories or Program (M/P)?");

  if( ((key = getchar_timeout_us(10000000)) != PICO_ERROR_TIMEOUT))
    {
      switch(key)
	{
	case 'm':
	case 'M':
	  what = DO_MP_MEMORIES;
	  break;

	case 'p':
	case 'P':
	  what = DO_MP_PROGRAM;
	  break;
	  
	default:
	  break;
	}
      
    }

return(what);
}


void cli_save_ram_mp(void)
{
  int do_what_mp = choose_what_mp();
  
  save_ram(parameter, do_what_mp);
}

void cli_save_ram(void)
{
  save_ram(parameter, DO_MP_BOTH);
}

void load_ram(int slotnum, int do_which_mp)
{
  if( do_which_mp & DO_MP_MEMORIES )
    {
      printf("\nLoading memories from flash slot %03d", slotnum);
      memcpy(ram_data+RAM_OFF_M, flash_slot_contents+slotnum*FLASH_PROGRAM_SLOT_SIZE+RAM_OFF_M, RAM_SIZE_M);
    }

  if( do_which_mp & DO_MP_PROGRAM )
    {
      printf("\nLoading program from flash slot %03d", slotnum);
      memcpy(ram_data+RAM_OFF_P, flash_slot_contents+slotnum*FLASH_PROGRAM_SLOT_SIZE+RAM_OFF_P, RAM_SIZE_P);
    }
  
  printf("\n");
}

void cli_load_ram_mp(void)
{
  int do_what_mp = choose_what_mp();
  
  load_ram(parameter, do_what_mp);
}

void cli_load_ram(void)
{
  load_ram(parameter, DO_MP_BOTH);
}

void copy_slot_to_buffer(int slot, uint8_t *buffer)
{
  memcpy(buffer, flash_slot_contents+slot*FLASH_PROGRAM_SLOT_SIZE, FLASH_PROGRAM_DATA_SIZE);
}

////////////////////////////////////////////////////////////////////////////////
//
// Write the text parameter as a slot label to the slot
// with the number of the parameter
// We have to copy the slot program, erase the slot then write the program
// back and then write the label.
//
////////////////////////////////////////////////////////////////////////////////

uint8_t label_buffer[FLASH_PROGRAM_SLOT_SIZE];

void cli_label_slot(void)
{
  copy_slot_to_buffer(parameter, &(label_buffer[0]));
  erase_slot(parameter);

  // Add label data to buffer before we write it back
  for(int i=0; i<TEXT_PARAMETER_LEN;i++)
    {
      label_buffer[FLASH_PROGRAM_DATA_SIZE+i] = text_parameter[i];
    }

  // Write the buffer back
  flash_range_program(FLASH_SLOT_OFFSET + (FLASH_PROGRAM_SLOT_SIZE * parameter), (uint8_t *) &(label_buffer[0]), FLASH_PROGRAM_SLOT_SIZE);
  
}

////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

void cli_slot_program_list(void)
{
  for(int i=0; i<FLASH_PROGRAM_NUM_SLOTS; i++)
    {
      printf("\n%03d:", i);
      
      if( *((flash_slot_contents+i*FLASH_PROGRAM_SLOT_SIZE)) == 0xFF )
	{
	  printf(" Blank");
	  continue;
	}

      for(int j=0; j<TEXT_PARAMETER_LEN;j++)
	{
	  uint8_t c=*((flash_slot_contents+i*FLASH_PROGRAM_SLOT_SIZE)+FLASH_PROGRAM_DATA_SIZE+j);
	  if( isprint(c) )
	    {
	      printf("%c", c);
	    }
	}

      // Now display the keystrokes
      display_keystrokes_at(flash_slot_contents+i*FLASH_PROGRAM_SLOT_SIZE);
    
      printf("\n");
    }

  printf("\n");
}

void cli_toggle_auto_increment_parameter(void)
{
  auto_increment_parameter = !auto_increment_parameter;
}

void cli_toggle_auto_increment_address(void)
{
  auto_increment_address = !auto_increment_address;
}

// Enter a text string that can be used by other commands

void cli_text_param(void)
{
  int key;
  int done = 0;
  int i = 0;

  printf("\nType string and end with RETURN...\n");
  
  while( !done )
    {
      if( ((key = getchar_timeout_us(1000)) != PICO_ERROR_TIMEOUT))
	{
	  printf("%c", key);
	  
	  if( key == 13 )
	    {
	      done = 1;
	      continue;;
	    }

	  text_parameter[i++] = key;
	  
	  if( i >= (TEXT_PARAMETER_LEN-1) )
	    {
	      i--;
	    }
	}
    }
  
  text_parameter[i] = '\0';
}

void cli_slot_label_list(void)
{
  for(int i=0; i<FLASH_PROGRAM_NUM_SLOTS; i++)
    {
      if( *((flash_slot_contents+i*FLASH_PROGRAM_SLOT_SIZE)+FLASH_PROGRAM_DATA_SIZE+0) == 0xFF)
	{
	  // Probably erased slot, don't display
	  continue;
	}
      
      printf("\n%03d: ", i);
      for(int j=0; j<TEXT_PARAMETER_LEN;j++)
	{
	  uint8_t c=*((flash_slot_contents+i*FLASH_PROGRAM_SLOT_SIZE)+FLASH_PROGRAM_DATA_SIZE+j);
	  if( isprint(c) )
	    {
	      printf("%c", c);
	    }
	}
    }
  printf("\n");
}

void clear_ram_mp(int what)
{
  if( what & DO_MP_MEMORIES )
    {
      printf("\nClearing memories");
      memset(&(ram_data[RAM_OFF_M]), 0x00, RAM_SIZE_M);
    }

  if( what & DO_MP_PROGRAM )
    {
      printf("\nClearing program");
      memset(&(ram_data[RAM_OFF_P]), 0x00, RAM_SIZE_P);
    }
  
  printf("\n");
}

void cli_clear_ram_mp(void)
{
  int what = choose_what_mp();

  clear_ram_mp(what);
}

void cli_clear_ram(void)
{
  clear_ram_mp(DO_MP_BOTH);
}

void cli_clear_memories(void)
{
  clear_ram_mp(DO_MP_MEMORIES);
}

void cli_clear_program(void)
{
  clear_ram_mp(DO_MP_PROGRAM);
}


////////////////////////////////////////////////////////////////////////////////

SERIAL_COMMAND serial_cmds[] =
  {
   {
    'h',
    "Serial command help",
    serial_help,
   },
   {
    'l',
    "Load emulation RAM from flash slot",
    cli_load_ram,
   },
   {
    'L',
    "Load memory or program from flash slot",
    cli_load_ram_mp,
   },
   {
    's',
    "Save emulation RAM to flash slot",
    cli_save_ram,
   },
   {
    'S',
    "Save memory or program to flash slot",
    cli_save_ram_mp,
   },
   {
    '?',
    "Serial command help",
    serial_help,
   },
   {
    'I',
    "Information",
    cli_information,
   },
   {
    'd',
    "Display RAM",
    display_ram,
   },
   {
    '+',
    "Start tracing",
    cli_start_tracing,
   },
   {
    '=',
    "Display trace",
    cli_display_trace,
   },
   {
    'z',
    "Zero parameter",
    cli_zero_parameter,
   },
   {
    'E',
    "Erase program slot",
    cli_erase_program_slot,
   },
   {
    'D',
    "Display program slot",
    cli_display_program_slot,
   },
   {
    'a',
    "Display All Programs",
    cli_slot_program_list,
   },
   {
    'A',
    "Set Address",
    cli_set_address,
   },
   {
    'W',
    "Write Byte",
    cli_write_byte,
   },
   {
    '_',
    "Clear All RAM",
    cli_clear_ram,
   },
   {
    '-',
    "Clear Memory or Program RAM",
    cli_clear_ram_mp,
   },
   {
    '0',
    "*Digit",
    cli_digit,
   },
   {
    '1',
    "*Digit",
    cli_digit,
   },
   {
    '2',
    "*Digit",
    cli_digit,
   },
   {
    '3',
    "*Digit",
    cli_digit,
   },
   {
    '4',
    "*Digit",
    cli_digit,
   },
   {
    '5',
    "*Digit",
    cli_digit,
   },
   {
    '6',
    "*Digit",
    cli_digit,
   },
   {
    '7',
    "*Digit",
    cli_digit,
   },
   {
    '8',
    "*Digit",
    cli_digit,
   },
   {
    '9',
    "*Digit",
    cli_digit,
   },
   {
    'B',
    "Label Slot",
    cli_label_slot,
   },
   {
    'b',
    "List Slot Labels",
    cli_slot_label_list,
   },
   {
    't',
    "Enter Text Parameter",
    cli_text_param,
   },
   {
    '.',
    "Toggle auto increment of parameter",
    cli_toggle_auto_increment_parameter,
   },
   {
    ',',
    "Toggle auto increment of address",
    cli_toggle_auto_increment_address,
   },
   {
    '!',
    "Boot to mass storage",
    cli_boot_mass,
   },
  };



void serial_help(void)
{
  printf("\n");
  
  for(int i=0; i<sizeof(serial_cmds)/sizeof(SERIAL_COMMAND);i++)
    {
      if( *(serial_cmds[i].desc) != '*' )
	{
	  printf("\n%c:   %s", serial_cmds[i].key, serial_cmds[i].desc);
	}
    }
  printf("\n0-9: Enter parameter digit");
}


void prompt(void)
{
  printf("\n\n(Text Parameter:'%s'", text_parameter);
  printf("\n(Parameter:%d (%04X) %c, Address:%d (%04X) %c) >",
	 parameter, parameter, auto_increment_parameter?'A':' ',
	 address,   address,   auto_increment_address?  'A':' ');
}


////////////////////////////////////////////////////////////////////////////////
//
// Serial CLI Handling
//
////////////////////////////////////////////////////////////////////////////////

int pcount = 0;
int periodic_read = 0;

void serial_loop()
{
  int  key;
  
  if( ((key = getchar_timeout_us(1000)) != PICO_ERROR_TIMEOUT))
    {
      for(int i=0; i<sizeof(serial_cmds)/sizeof(SERIAL_COMMAND);i++)
	{
	  if( serial_cmds[i].key == key )
	    {

	      keypress = key;
	      (*serial_cmds[i].fn)();
	      prompt();
	      break;
	    }
	}
    }
  else
    {
      // I have found that I need to send something if the serial USB times out
      // otherwise I get lockups on the serial communications.
      // So, if we get a timeout we send a spoace and backspace it. And
      // flush the stdio, but that didn't fix the problem but seems like a good idea.
      stdio_flush();
      printf(" \b");
    }
}

////////////////////////////////////////////////////////////////////////////////

// Process commands
//
// Commands are put into M<command>
//
// Form:
// ccsnnn E 88
//
// (This is normalised to c.csnnn E 93)
//
// cc  : Command number
// s   : Status
// nnn : Numeric parameter
//

void set_status(int n)
{
  int stat_byte = *(command_ptr+4);

  stat_byte &= 0xF0;
  stat_byte |= (n & 0xF);
  
  *(command_ptr+4) = stat_byte;
}

#define CATALOG_WIDTH  30

int slot_blank_ma(int num)
{
  if( *((flash_slot_contents+num*FLASH_PROGRAM_SLOT_SIZE)) == 0xFF )
    {
      // Blank
      return(1);
    }

  // Not blank
  return(0);
}

void cmd_catalog(int num)
{
  int bitmask = 0;
  int bitnum = 2;

  uint8_t *cmd_ptr2 = command_ptr+8;
  
  for(int j=num; j<num+CATALOG_WIDTH; j++)
    {
      int i = ((CATALOG_WIDTH-1) - (j-num))+3;
      //i = j + 3 - num;
      
      bitmask = (1 << bitnum);
      
      *(cmd_ptr2+i/6) &= ~bitmask;

      printf("\n%02X %02X %d %d", bitnum, i/6, num, slot_blank_ma(j));
      
      if( slot_blank_ma(j) )
	{
	}
      else
	{
	  *(cmd_ptr2+i/6) |= bitmask;
	}

      bitnum--;

      switch(bitnum)
	{
	case 3:
	  *(cmd_ptr2+i/6) &= ~(1 << 3);
	  bitnum--;
	  break;

	case -1:
	  
	  *(cmd_ptr2+i/6) &= ~(1 << 7);
	  bitnum = 6;
	  break;
	}
    }

  *(cmd_ptr2+6) = 0x09;
  *(cmd_ptr2+7) = 0xF1;
}

void status_good(void)
{
  set_status(STATUS_CHAR_GOOD);
}

void status_bad(void)
{
    set_status(STATUS_CHAR_GOOD);
}

void process_commands(void)
{

  // Do we have a valid command?
  if( *(command_ptr+6) == 0x93 && *(command_ptr+7) == 0xF1 &&
      
    // Status is zero if we haven't executed the command
      ((*(command_ptr+4) & 0xF)==0)
      )
    {
      
      // This could be a command
      int cmd_code = GET_SPLIT_BYTE(4);
      int num      = bcd_to_dec(command_ptr+3) * 10;
      num += bcd_to_dec(command_ptr+2)/10;

      printf("\nCmd code:%02X", cmd_code);
      printf(" Num = %04X %d",num, num);

      switch( cmd_code )
	{
	case CMD_LOAD:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS) )
	    {
	      status_good();
	      load_ram(num, DO_MP_BOTH);
	    }
	  else
	    {
	      status_bad();
	    }
	  break;

	case CMD_LOAD_M:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS) )
	    {
	      status_good();
	      load_ram(num, DO_MP_MEMORIES);
	    }
	  else
	    {
	      status_bad();
	    }
	  break;
	  
	case CMD_LOAD_P:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS) )
	    {
	      status_good();
	      load_ram(num, DO_MP_PROGRAM);
	    }
	  else
	    {
	      status_bad();
	    }
	  break;

	case CMD_SAVE:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS) )
	    {
	      status_good();
	      save_ram(num, DO_MP_BOTH);
	    }
	  else
	    {
	      status_bad();
	    }
	  break;

	case CMD_SAVE_M:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS) )
	    {
	      status_good();
	      save_ram(num, DO_MP_MEMORIES);
	    }
	  else
	    {
	      status_bad();
	    }
	  break;
	  
	case CMD_SAVE_P:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS) )
	    {
	      save_ram(num, DO_MP_PROGRAM);
	      status_good();
	    }
	  else
	    {
	      status_bad();
	    }
	  break;

	case CMD_CLR:
	  status_good();
	  clear_ram_mp(DO_MP_BOTH);
	  break;

	case CMD_CLR_M:
	  status_good();
	  clear_ram_mp(DO_MP_MEMORIES);
	  break;

	case CMD_CLR_P:
	  status_good();
	  clear_ram_mp(DO_MP_PROGRAM);
	  break;

	case CMD_CATALOG:
	  if( (num >=0) && (num<FLASH_PROGRAM_NUM_SLOTS-CATALOG_WIDTH) )
	    {
	    cmd_catalog(num);
	    status_good();
	    }
	  else
	    {
	      status_bad();
	    }
	  break;
	  
	default:
	  break;
	}
    }
}


////////////////////////////////////////////////////////////////////////////////


int main()
{
  int gpio_states;
  
  
  ////////////////////////////////////////////////////////////////////////////////
  //
  // Overclock as needed
  //
  ////////////////////////////////////////////////////////////////////////////////
  
  //#define OVERCLOCK 135000
  //#define OVERCLOCK 200000
#define OVERCLOCK 270000
  //#define OVERCLOCK 360000
  
#if OVERCLOCK > 270000
  /* Above this speed needs increased voltage */
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  sleep_ms(1000);
#endif
  
  /* Overclock */
  set_sys_clock_khz( OVERCLOCK, 1 );

  stdio_init_all();
  
  // All GPIOs inputs
  
  for(int i=0;i<23; i++)
    {
      set_gpio_input(i);
    }

  // DOUT is done with two outputs
  set_gpio_output(P_DOUTDRV);
  set_gpio_output(P_DOUTOE);

  // Don't drive DOUT yet
  gpio_put(P_DOUTOE, 1);
  
  // Debug status blipper
  set_gpio_output(P_STAT1);
  
  set_gpio_input(26);
  set_gpio_input(27);
  set_gpio_input(28);

  // Core 1 captures data and sends transactions to core 0 which
  // sends them over USB
  multicore_launch_core1(core1_main);

  sleep_ms(2000);

  printf("\nHD36106 Replacement\n");

  // Load slot 0 at startup
  load_ram(0, DO_MP_BOTH);
  
  // Sit in loop sending transatins to USB
  while(1)
    {
      serial_loop();

      process_commands();
      
#if NO_TRACE_USB
#else
      process_bus();
#endif

#if EMULATE_FX201P
      process_fx201p_execution();
#endif
    }
}

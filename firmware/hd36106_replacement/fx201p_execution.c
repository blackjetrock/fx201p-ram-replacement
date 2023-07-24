////////////////////////////////////////////////////////////////////////////////
//
//
//
////////////////////////////////////////////////////////////////////////////////

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fx201p.h"
#include "fx201p_execution.h"

#define NUM_ST 10

////////////////////////////////////////////////////////////////////////////////

double mem_to_dbl(uint8_t *m);
void dbl_to_mem(double value, uint8_t *m);

////////////////////////////////////////////////////////////////////////////////

int tok_loc[NUM_ST];
double xreg = 0;
int dest_reg = 0;
int tok;
int xref = 0;
int pending_op = 0;
int constant_entry = 0;
double constant;

void execution_start(void)
{
  exec_pc = 0x80;
  executing = 1;
  
}

void process_fx201p_execution(void)
{

  if( !executing )
    {
      return;
    }

  // Process one more keystroke
  tok = ram_data[exec_pc];
  double v = 0;
  
  switch(tok)
    {
    case TOK_ST:
      exec_pc++;
      
      // Store location of instruction after label
      tok_loc[ram_data[exec_pc]] = exec_pc+1;
      exec_pc++;
      break;

    case TOK_GOTO:
      exec_pc++;
      exec_pc = tok_loc[ram_data[exec_pc]];
	    
      break;

    case TOK_K:
      constant_entry = 1;
      constant = 0;
      break;

    case TOK_tan:
      // Increment memory 1
      v = mem_to_dbl(ADDRESS_OF_MEM(1));
      v = v + 1;
      dbl_to_mem(v, ADDRESS_OF_MEM(1));

      exec_pc++;
      break;
      
    case TOK_COLON:
      if( pending_op )
	{
	  xreg = xreg + constant;
	}
      
      exec_pc++;
      break;

    case TOK_0:
    case TOK_1:
    case TOK_2:
    case TOK_3:
    case TOK_4:
    case TOK_5:
    case TOK_6:
    case TOK_7:
    case TOK_8:
    case TOK_9:
      if( constant_entry )
	{
	  constant *=10;
	  constant += tok -TOK_0;
	}
      else
	{
	  // Load X register with contents of memory n
	  xreg = mem_to_dbl(ADDRESS_OF_MEM(tok-TOK_0));
	  
	  xref = tok-TOK_0;
	}
      exec_pc++;
      break;

    case TOK_EQ:
      dest_reg = xref;
      exec_pc++;
      break;

    case TOK_PL:
      pending_op = tok;
      exec_pc++;
      break;
    }
#if 0
  printf("\nPC:%03X", exec_pc);
  printf("\nxref=%d  xreg=%g", xref, xreg);
  printf("\n");
#endif
}

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
#include "icd/Icd.h"

#define NUM_ST       10
#define DEBUG_EXEC    0

////////////////////////////////////////////////////////////////////////////////

double mem_to_dbl(uint8_t *m);
void dbl_to_mem(double value, uint8_t *m);
void print_keystroke(int byte, int no_lf);

////////////////////////////////////////////////////////////////////////////////

int executing = 0;
int exec_pc = 0x80;

int tok_loc[NUM_ST];
double xreg = 0;

#define NO_DEST -1
int dest_reg = NO_DEST;
int tok;

#define NO_XREF  -1
int xref = NO_XREF;

#define NO_PENDING_OP 0
int pending_op = NO_PENDING_OP;

int constant_entry = 0;
double constant;

// Some statements an't be exeuted, we skip them.
int run_to_colon = 0;

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

  if( run_to_colon )
    {
      if( tok != TOK_COLON )
	{
	  exec_pc++;
	}
    }
  
  switch(tok)
    {
    case TOK_NONE:
      executing = 0;
      break;
      
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

    case TOK_ANS:
      exec_pc++;
      run_to_colon = 1;
      break;
      
    case TOK_K:
      constant_entry = 1;
      constant = 0.0;
      exec_pc++;
      break;

    case TOK_TAN:
      // Increment memory 1
      v = mem_to_dbl(ADDRESS_OF_MEM(1));
      v = v + 1;
      dbl_to_mem(v, ADDRESS_OF_MEM(1));

      exec_pc++;
      break;
      
    case TOK_COLON:
      if( pending_op != NO_PENDING_OP )
	{
	  double argval;
	  
	  if( constant_entry )
	    {
	      argval = constant;
	    }
	  else
	    {
	      argval = mem_to_dbl(ADDRESS_OF_MEM(xref));
	    }
	  
	  switch(pending_op)
	    {
	    case TOK_PL:
	      xreg += argval;
	      break;
	    }
	}
      else
	{
	  if( constant_entry )
	    {
	      xreg = constant;
	    }
	}
      
      // Assign if there was one
      if( dest_reg != NO_DEST )
	{
#if DEBUG_EXEC
	  printf("\nAssigning %g to M%d", xreg, dest_reg);
#endif
	  dbl_to_mem(xreg, ADDRESS_OF_MEM(dest_reg));
	}
      
      // Clear up for next statement
      constant_entry = 0;
      pending_op = NO_PENDING_OP;
      xreg = 0.0;
      xref = NO_XREF;
      dest_reg = NO_DEST;      
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
	  if( pending_op != NO_PENDING_OP )
	    {
	      switch(pending_op)
		{
		case TOK_PL:
		  xreg += mem_to_dbl(ADDRESS_OF_MEM(tok-TOK_0));
		  break;
		}

	      pending_op = NO_PENDING_OP;
	      xref = NO_XREF;
	    }
	  else
	    {
	      // Load X register with contents of memory n
	      xreg = mem_to_dbl(ADDRESS_OF_MEM(tok-TOK_0));
	      xref = tok-TOK_0;
	    }
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

#if DEBUG_EXEC
  printf("\nPC:%03X", exec_pc);
  printf("\nToken:");
  print_keystroke(tok, 1);
  printf("\nxref=%d  xreg=%g", xref, xreg);
  printf("\nPending op:%d", pending_op);
  printf("\nConstant entry:%d  Constant:%g", constant_entry, constant);
  printf("\nDest Reg:%d", dest_reg);
  printf("\n");
#endif
}

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

int tok_loc[NUM_ST];
int xreg = 0;
int dest_reg = 0;

void process_fx201p_execution(void)
{

  if( !executing )
    {
      return;
    }

  // Process one more keystroke
  switch(ram_data[exec_pc])
    {
    case TOK_ST:
      exec_pc++;
      
      // Store location of instruction after label
      tok_loc[ram_data[exec_pc]] = exec_pc+1;
      exec_pc++;
      break;

    case TOK_COLON:
      // Ignore for now
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
      // Store X register
      xreg = ram_data[exec_pc] - TOK_0;
      break;

    case TOK_EQ:
      dest_reg = xreg;
      break;

    case TOK_PL:
      
      break;
    }
}

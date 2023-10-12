/*
 * FX201P simulator
 *
 */

#include <math.h>
#include <stdlib.h>

#include <stdio.h>
#include <string.h>
#include <climits>
#include <string>

#include "icd/Icd.h"
#include "fx201p.h"
#include "fx201p_execution.h"

#include <ncurses.h>
#include <termio.h>
#include <unistd.h>

char display[20];
uint8_t ram_data[];

#if 0
int executing = 0;
int exec_pc = 0x80;
#endif

void init_curses(void)
{
  //initscr();  
  //timeout(1);

  initscr();

  cbreak();
  noecho();
  nodelay(stdscr, TRUE);

    //scrollok(stdscr, TRUE);
}

void end_curses(void)
{
  endwin();
}

void echoOff(void)
{
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);

    term.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

void echoOn(void)
{
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);

    term.c_lflag |= ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

////////////////////////////////////////////////////////////////////////////////

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

  //  printf("\nSign:%d", sign);
  
  value = fabs(value);
  
  double exponent = floor(log10(value));

  //printf("\nExponent:%g", exponent);

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
  
  // get digits
  double dpair;

  dpair = floor(value);
  dbl_to_bcd(dpair, m+5);
  value = value - floor(value);
  value *=100.0;

  dpair = floor(value);
  dbl_to_bcd(dpair, m+4);
  value = value - floor(value);
  value *=100.0;

  dpair = floor(value);
  dbl_to_bcd(dpair, m+3);
  value = value - floor(value);
  value *=100.0;

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

void set_display(icd n)
{
  strcpy(display, n.AsString().c_str());
}

// Process a keystroke in comp mode

void mode_comp(CALCULATOR_STATE *state, int token)
{
  int val;

  if( state->clear_on_next_key )
    {
      state->y = state->x;
      state->x = 0.0;
      
      state->clear_on_next_key = 0;
      
    }
  
  switch(token)
    {
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
      state->x *= 10.0;
      val = token - TOK_0;
      state->x += (long int)val;
      break;

    case TOK_PL:
      state->clear_on_next_key = 1;
      state->pending_operator = token;
      break;

    case TOK_MN:
      state->clear_on_next_key = 1;
      state->pending_operator = token;
      break;

    case TOK_TIMES:
      state->clear_on_next_key = 1;
      state->pending_operator = token;
      break;

    case TOK_DIV:
      state->clear_on_next_key = 1;
      state->pending_operator = token;
      break;

    case TOK_EQ:
      if( state->pending_operator != TOK_NONE )
	{
	  switch(state->pending_operator)
	    {
	    case TOK_TIMES:
	      state->x = state->y * state->x;
	      state->y = 0.0;
	      state->pending_operator = TOK_NONE;
	      break;
	      
	    case TOK_DIV:
	      state->x = state->y / state->x;
	      state->y = 0.0;
	      state->pending_operator = TOK_NONE;
	      break;
	      
	    case TOK_PL:
	      state->x = state->y + state->x;
	      state->y = 0.0;
	      state->pending_operator = TOK_NONE;
	      break;
	      
	    case TOK_MN:
	      state->x = state->y - state->x;
	      state->y = 0.0;
	      state->pending_operator = TOK_NONE;
	      break;
	    }
	}
      
      break;

    case TOK_AC:
      state->x = 0.0;
      state->y = 0.0;
      state->pending_operator = TOK_NONE;
      state->clear_on_next_key = 0;
      
      break;

    case TOK_C:
      state->y = 0.0;
      break;

    case TOK_SIN:
      state->x = sin(state->x);
      break;

    case TOK_COS:
      state->x = cos(state->x);
      break;

    case TOK_TAN:
      state->x = tan(state->x);
      break;

    case TOK_PI:
      state->x = 3.14159265358979323;
      break;
    }
}

void mode_run(CALCULATOR_STATE *s, int token)
{

}

void mode_write(CALCULATOR_STATE *s, int token)
{

}

void mode_clear(CALCULATOR_STATE *s, int token)
{

}

void display_status(CALCULATOR_STATE *s)
{
  char xs[1000], ys[1000];
  char ts[1000];
  
  strcpy(xs, s->x.AsString().c_str());
  strcpy(ys, s->y.AsString().c_str());

  sprintf(ts, "X:%s Y:%s", xs, ys);
  mvaddstr(20, 12, xs);
}

KEY_TABLE fx201p_key_table[] =
  {

   {"MAC"  , 0xC1},
   {"1/x"  , 0xC2},
   {"GOTO" , 0xC2},
   {"SUB#" , 0xC3},
   {"ST#"  , 0xC4},
   {":"    , 0xC5},
   {"MJ"   , 0xC6},
   {"SQRT" , 0xB1},
   {"log"  , 0xB2},
   {"ln"   , 0xB3},
   {"e^x"  , 0xB4},
   {"x^y"  , 0xB5},
   {"10^x" , 0xB6},
   {"+-"   , 0xA1},
   {"K"    , 0xA2},
   {"IF"   , 0xA3},
   {"arc"  , 0xA4},
   {"sin"  , 0xA5},
   {"cos"  , 0xA6},
   {"tan"  , 0xA7},
   {"EQ"   , 0xD0},
   {"="    , 0xD0},
   {"IM"   , 0xD1},
   {"ENT"  , 0xD2},
   {"ANS"  , 0xD3},
   {"M+"   , 0xD4},
   {"*"    , 0xD5},
   {"/"    , 0xD6},
   {"+"    , 0xD7},
   {"-"    , 0xD8},
   {"."    , 0xD9},
   {"EXP"  , 0xDD},
   {"0"    , 0xF0},
   {"1"    , 0xF1},
   {"2"    , 0xF2},
   {"3"    , 0xF3},
   {"4"    , 0xF4},
   {"5"    , 0xF5},
   {"6"    , 0xF6},
   {"7"    , 0xF7},
   {"8"    , 0xF8},
   {"9"    , 0xF9},
   {"AC"   , 0x100},
   {"C"    , 0x101},
   {"NONE" , 0x00},
  };

#define NUM_FX201P_KEY_TABLE (sizeof(fx201p_key_table)/sizeof(KEY_TABLE))

int find_keystroke(char *k)
{
  for(int i=0; i<NUM_FX201P_KEY_TABLE; i++)
    {
      if( strncmp(k, fx201p_key_table[i].key_str, strlen(fx201p_key_table[i].key_str))==0)
	{
	  //Found key code
	  return(fx201p_key_table[i].keycode);
	}
    }
  return(-1);
}


char keystr[10];
int keystr_i = 0;

int poll_keyboard(void)
{
  int c = wgetch(stdscr);
  int keystroke = -1;
  
  if( c != ERR )
    {
      char hexc[10];
      sprintf(hexc, "%02x", c);
      mvaddch(5,5, c);
      mvaddstr(5, 10, hexc);

      // Add to string and see if we have a keystroke
      keystr[keystr_i++] = toupper(c);

      if( (keystroke = find_keystroke(keystr)) != -1 )
	{
	  // Found one, reset string
	  keystr_i = 0;
	  
	  sprintf(hexc, "%02x", keystroke);
	  mvaddch(6,5, keystroke);
	  mvaddstr(7, 10, hexc);
	  mvaddstr(8, 10, keystr);
	}
      
      return(keystroke);
    }
  
  return(-1);
}

CALCULATOR_STATE state;

int main(void)
{
  icd x,y;
  int key;
  init_curses();
  
  x = icd(0L);
  y = icd(10L);
  x = x / y;
  
  printf("\n%s", x.AsString().c_str());
  
  set_display(x);
  
  printf("\n%s", display);
  
  printf("\n");

  state.x = icd(0L);
  state.y = icd(0L);
  state.state = CALC_STATE_COMP;
  state.clear_on_next_key = 0;
  state.pending_operator - TOK_NONE;
  
  while(1)
    {
#if 0
      state.x += 1.0/7;
      state.y++;
#endif
      
      key = poll_keyboard();

      display_status(&state);

      if( key != -1 )
	{
	  switch( state.state )
	    {
	    case CALC_STATE_COMP:
	      mode_comp(&state, key);
	      break;
	      
	    case CALC_STATE_RUN:
	      mode_run(&state, key);
	      break;
	      
	    case CALC_STATE_WRITE:
	      mode_write(&state, key);
	      break;
	    }
	}
    }

  end_curses();
}


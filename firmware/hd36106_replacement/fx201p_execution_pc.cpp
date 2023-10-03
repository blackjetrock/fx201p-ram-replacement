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

char display[20];
uint8_t ram_data[];

int executing = 0;
int exec_pc = 0x80;

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

int main(void)
{
  icd x,y;

  x = icd(1234L);
  y = icd(10L);
  x = x / y;
  
  printf("\n%s", x.AsString().c_str());
  
  set_display(x);
  
  printf("\n%s", display);
  
  printf("\n");
  
  while(1)
    {
      
    }
}


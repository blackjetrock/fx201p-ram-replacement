

// Instruction tokens

#define TOK_MAC	  0xC1
#define TOK_1x	  0xC2
#define TOK_SUB	  0xC3
#define TOK_ST	  0xC4
#define TOK_COLON 0xC5
#define TOK_MJ	  0xC6

#define TOK_SQRT  0xB1
#define TOK_log	  0xB2
#define TOK_ln	  0xB3
#define TOK_ex	  0xB4
#define TOK_xy	  0xB5
#define TOK_10x   0xB6

#define TOK_PLMN 0xA1
#define TOK_K	 0xA2
#define TOK_IF	 0xA3
#define TOK_arc	 0xA4
#define TOK_sin	 0xA5
#define TOK_cos	 0xA6
#define TOK_tan	 0xA7

#define TOK_EQ	  0xD0
#define TOK_IM	  0xD1
#define TOK_ENT	  0xD2
#define TOK_ANS	  0xD3
#define TOK_MPL	  0xD4
#define TOK_TIMES 0xD5
#define TOK_DIV	  0xD6
#define TOK_PL	  0xD7
#define TOK_MN	  0xD8
#define TOK_DOT	  0xD9
#define TOK_EXP	  0xDD

#define TOK_0	 0xF0
#define TOK_1	 0xF1
#define TOK_2	 0xF2
#define TOK_3	 0xF3
#define TOK_4	 0xF4
#define TOK_5	 0xF5
#define TOK_6	 0xF6
#define TOK_7	 0xF7
#define TOK_8	 0xF8
#define TOK_9	 0xF9


// We need access to emulation ram
uint8_t ram_data[RAM_SIZE];

void process_fx201p_execution(void);

int executing = 0;
int exec_pc = 0x80;


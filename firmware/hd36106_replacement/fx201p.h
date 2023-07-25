////////////////////////////////////////////////////////////////////////////////
//
// FX201 Specific information
//
////////////////////////////////////////////////////////////////////////////////

#define RAM_SIZE    256
#define RAM_SIZE_M  128
#define RAM_SIZE_P  128



#define RAM_OFF_M   0
#define RAM_OFF_P   128


#define ADDRESS_OF_MEM(NNN) (&(ram_data[8*(NNN)]))
#define MEM_OF_ADDRESS(AAA) (((AAA)/8)-1)
void process_fx201p_execution(void);
void execution_start(void);


#include "iodefine_RL78.h"
#include "YRPBRL78G13.h"

void dummy(void) {
}

int main(void)
{
  unsigned long uLcounter = 0;
  LED01_PIN = 0; // Make Pin as O/P
  LED01 = 0; // Turn LED ON  
  while(1)
  {
     LED01 = ~LED01; // toggle LED
     for(uLcounter=0;uLcounter<80000;uLcounter++);
	 {
	    asm("nop"); // do nothing
	 }
     
  }  
  return 0;
}
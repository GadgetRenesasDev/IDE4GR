

typedef struct
{
  unsigned char B0:1;
  unsigned char B1:1;
  unsigned char B2:1;
  unsigned char B3:1;
  unsigned char B4:1;
  unsigned char B5:1;
  unsigned char B6:1;
  unsigned char B7:1;
} __BITS8;

struct st_crc0ctl { 
        union {
                unsigned char BYTE;
                __BITS8 BIT;
        } CRC0CTL;
};

struct st_iawctl {
        union {
                unsigned char BYTE;
                __BITS8 BIT;
        } IAWCTL;
};

struct st_port7 {
	union {
		unsigned char BYTE;
		__BITS8 BIT;
	} PM7;
};

struct st_portu7 {
	union {
		unsigned char BYTE;
		__BITS8 BIT;
	} PU7;
};

struct st_portp7 { 
        union {
                unsigned char BYTE;
                __BITS8 BIT;
        } P7;
};

//#define PORT_P7 (*(volatile unsigned char*)0xFFF07)
//#define PORT_P7_DIR (*(volatile unsigned char*)0xFFF27)

#define PORT7 (*(volatile struct st_port7 *)0xFFF27)
#define PORTU7 (*(volatile struct st_portu7 *)0xF0037)
#define PORTP7 (*(volatile struct st_portp7 *) 0xFFF07)
#define REGCRC0CTL (*(volatile struct st_crc0ctl *) 0xF02F0)
#define REGIAWCTL (*(volatile struct st_iawctl *) 0xF0078)
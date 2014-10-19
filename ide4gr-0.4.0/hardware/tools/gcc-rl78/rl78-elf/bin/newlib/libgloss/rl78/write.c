/* These definitions are for the RL78/G13, which the simulator
   simulates.  */

typedef unsigned char UQI __attribute__((mode(QI)));
typedef unsigned int UHI __attribute__((mode(HI)));

#define s8(x)	(*(volatile UQI *)(x))
#define s16(x)	(*(volatile UHI *)(x))

#define P(x)	s8(0xfff00+(x))
#define PM(x)	s8(0xfff20+(x))

#define PER0	s8(0xf00f0)

#define SSR00	s16(0xf0100)
#define SMR00	s16(0xf0110)
#define SCR00	s16(0xf0118)
#define SS0	s16(0xf0122)
#define SPS0	s16(0xf0126)
#define SO0	s16(0xf0128)
#define SOE0	s16(0xf012a)
#define SOL0	s16(0xf0134)

#define SDR00	s16(0xfff10)

static int initted = 0;

static void
init_uart0 ()
{
  initted = 1;

  PER0 = 0xff;
  SPS0 = 0x0011; /* 16 MHz */
  SMR00 = 0x0022; /* uart mode */
  SCR00 = 0x8097; /* 8-N-1 */
  SDR00 = 0x8a00; /* baud in MSB - 115200 */
  SOL0 = 0x0000; /* not inverted */
  SO0 = 0x000f; /* initial value */
  SOE0 = 0x0001; /* enable TxD0 */
  P(1)  |= 0b00000100;
  PM(1) &= 0b11111011;
  SS0 = 0x0001;
}

static void
tputc (char c)
{
  /* Wait for transmit buffer to be empty.  */
  while (SSR00 & 0x0020)
    asm("");
  /* Transmit that byte.  */
  SDR00 = c;
}

int
_write(int fd, char *ptr, int len)
{
  int rv = len;

  if (!initted)
    init_uart0 ();

  while (len != 0)
    {
      if (*ptr == '\n')
	tputc ('\r');
      tputc (*ptr);
      ptr ++;
      len --;
    }
  return rv;
}

char * write (int) __attribute__((weak, alias ("_write")));

/*
 * MasterMind implementation: template; see comments below on which parts need to be completed

 * Compile: 
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:     
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY   200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			     0
#define	HIGH			 1

/*
 * #define NAN1 8
#define NAN2 9
 * */


// =======================================================
// Wiring (see inlined initialisation routine)

//pin of lcd
#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22


/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char* color_names[] = { "red", "green", "blue" };

static int* theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;




/* --------------------------------------------------------------------------- */






// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

static int lcdControl ;

//
/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04







// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

static unsigned int gpiobase ;
static uint32_t *gpio ;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes



int failure (int fatal, const char *message, ...);                        //finish

void waitForEnter (void);                                                 //finish
void waitForButton (uint32_t *gpio, int button);                          //finish

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ********************************************************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */

/* 在引脚号 @pin@ 上发送一个 @value@ (LOW 或 HIGH)；@gpio@ 是映射的 GPIO 基地址 */


void digitalWrite (uint32_t *gpio, int pin, int value);                    //finish



/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode);                           //finish

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value);                          //finish

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button);                                   //finish

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq() {
  /* ***  COMPLETE the code here  ***  */
    // allocate memory for theSeq variable dynamically
    theSeq = (int*)malloc(seqlen * sizeof(int));
    //new random sequence is generated everytime
    srand(time(NULL));
    // generate numbers
    int i=0;
    while(i<seqlen){
        //generate number between 1 and seqlen
        int random = rand()%seqlen + 1;
        //create the sequence
        theSeq[i] = random;
        i++;
    }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq) {
  /* ***  COMPLETE the code here  ***  */
    fprintf(stdout,"The value of sequence is : ");
    //prinit the value
    int i=0;
    while(i<seqlen){
        fprintf(stdout,"%d",seq[i]);
        i++;
    }
    fprintf(stdout,".\n");
}

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */

//?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????




extern int* countMatches(int *seq1, int *seq2);






//?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int* code,int *seq1, int *seq2,int lcd_format) {
  /* ***  COMPLETE the code here  ***  */
    printf("Exact Matches are      : %d\n",code[0]);
    printf("Approx Matches are    : %d\n",code[1]);
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */

/* 将整数值解析为数字列表，并将它们放入 @seq@ 中 */
/* 用于处理带有选项 -s 或 -u 的命令行 */
void readSeq(int *seq, int val) {
  /* ***  COMPLETE the code here  ***  */

    for(int i=0;i<seqlen;i++)
    {
        //Stor the value of modulus in a variable (Using Reverse Number Logic)
        int value = val%10;
        //Stor the value in sequence variable
        seq[i]=value;
        //divide it by 10
        val = val/10;
    }
    //change the position of seq[0] and seq[2]
    int connect=seq[2];
    seq[2]=seq[0];
    seq[0]=connect;
}






/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */



/* 从标准输入读取猜测序列，并将值存储在 arr 中 */
/* 仅用于测试游戏逻辑，没有按钮输入时需要 */
//???????????????????????????????????????????????????????????????
int readNum(int max) {                                                    //waiting
  /* ***  COMPLETE the code here  ***  */

}



/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */


//关于时间间隔的几个代码

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
/* 在下面的 timer_handler() 函数中可能需要使用此函数 */
/* 使用 libc 的 gettimeofday() 函数来实现它 */
uint64_t timeInMicroseconds(){
  /* ***  COMPLETE the code here  ***  */
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
/* 这应该是一个回调函数，通过定时器触发 */
/* 通过在主函数中调用 sigaction() 来设置定时器 */
void timer_handler (int signum) {                                              //waiting
  /* ***  COMPLETE the code here  ***  */
}


/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
/* 初始化时间戳，设置一个间隔定时器，并安装 timer_handler 回调函数 */
void initITimer(uint64_t timeout){                                              //waiting
  /* ***  COMPLETE the code here  ***  */
}



/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal) //  && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
  vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter (void)
{
  printf ("Press ENTER to continue: ") ;
  (void)fgetc (stdin) ;
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0){
           return ;
        }

    #if 0
        else if (howLong  < 100)
             delayMicrosecondsHard (howLong) ;
    #endif
        else
        {
            sleeper.tv_sec  = wSecs ;
            sleeper.tv_nsec = (long)(uSecs * 1000L) ;
            nanosleep (&sleeper, NULL) ;
        }
}





//???????????????????????????????????????????????????????????????????????????
void pinMode(uint32_t *gpio, int pin, int mode)
{
    int result;
    int functionSelect = pin / 10;               // Calculate the value for the function select register
    int shiftAmount = (pin % 10) * 3;            // Calculate the shift amount

    asm volatile (
            "ldr r1, %[gpio]\n"                      // Load the gpio value into R1
            "add r0, r1, %[functionSelect]\n"         // Add the function select value to R1 and store the result in R0
            "ldr r1, [r0, #0]\n"                      // Load the value from R0 into R1
            "mov r2, #0b111\n"                        // Put the binary value of 7 into R2
            "lsl r2, %[shiftAmount]\n"                // Shift 111 left by the shift amount
            "bic r1, r1, r2\n"                        // Bitwise clear those bits (not 111)
            "mov r2, %[mode]\n"                       // Put the mode into R2
            "lsl r2, %[shiftAmount]\n"                // Shift the mode left by the shift amount
            "orr r1, r1, r2\n"                        // OR the two values together
            "str r1, [r0, #0]\n"                      // Store the value from R1 back into R0
            "mov %[result], r1\n"                     // Store R1 into the output variable result
            : [result] "=r" (result)                  // output operands
    : [gpio] "m" (gpio),                              // Input operands
    [functionSelect] "r" (functionSelect * 4),
    [shiftAmount] "r" (shiftAmount),
    [mode] "r" (mode)
    : "r0", "r1", "r2", "cc"                           //clobbers - the registers being used
    );
}

void writeLED(uint32_t *gpio, int led, int value)
{
    int state = 0;  // Initialize the LED state to 0
    int result;

    // If the value is HIGH, set the state to 7 to turn on the LED
    if (value == HIGH) {
        state = 7;
    }
        // Else, set the state to 10 to turn off the LED
    else {
        state = 10;
    }
    asm volatile (
            "ldr r1, %[gpio]\n"                 // Load the gpio value into R1
            "add r0, r1, %[state]\n"            // Add the state value to R1 and store the result in R0
            "mov r2, #1\n"                      // Store 1 into R2
            "mov r1, %[led]\n"                  // Move the led number into R1
            "and r1, #31\n"                     // Perform bitwise AND with 31 to ensure the LED number is within the valid range
            "lsl r2, r1\n"                      // Shift R2 left by the LED number of times
            "str r2, [r0, #0]\n"                // Store the value in R2 into the memory location pointed to by R0
            "mov %[result], r2\n"               // Store the value of R2 into the output variable result
            : [result] "=r" (result)            // Output operand: result
    : [led] "r" (led)                   // Input operand: led
            , [gpio] "m" (gpio)                 // Input operand: gpio
            , [state] "r" (state * 4)           // Input operand: state
    : "r0", "r1", "r2", "cc"            // Clobbered registers: r0, r1, r2, cc
    );
}


int readButton(uint32_t *gpio, int button)
{
    int result = 0;                     // Initialize the result variable to 0
    int offset = 13 * 4;                // Calculate the offset for GPIO level register
    int buttonValue = LOW;                 // Initialize the button value to LOW

    asm volatile (
            "ldr r0, [%[gpio], %[offset]]\n"     // Load the value at gpio + offset into R0
            "mov r1, %[button]\n"                 // Move the button value into R1
            "and r1, #31\n"                       // Perform bitwise AND with 31 to ensure the button number is within the valid range
            "mov r2, #1\n"                        // Store 1 into R2
            "lsl r2, r1\n"                        // Left shift the value in R2 by the button number of times
            "and r0, r2\n"                        // Perform bitwise AND between the value in R0 and the value in R2
            "mov %[result], r0\n"                 // Move the value in R0 to the result variable
            : [result] "=r" (result)              // Output operand: result
    : [button] "r" (button)                // Input operand: button
            , [gpio] "r" (gpio)                    // Input operand: gpio
            , [offset] "r" (offset)                // Input operand: offset
    : "r0", "r1", "r2", "cc"               // Clobbered registers: r0, r1, r2, cc
    );

    if (result == 0) {
        buttonValue = LOW;            // Set the button value to LOW if the result is zero
    } else {

        buttonValue = HIGH;           // Set the button value to HIGH if the result is non-zero
    }

    return buttonValue;                        // Return the button value
}


void digitalWrite(uint32_t *gpio, int pin, int value) {


    // select set or clr registers
    int reg = value ? 7: 10;

    // pins 0-31 in register X0, 32-63 in register X1.
    if (pin > 31) {
        pin =pin- 32;
        reg++;
    }

    gpio[reg] = 1 << pin;
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */
//lcd 代码lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
void strobe (const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite (gpio, lcd->strbPin, 1) ; delayMicroseconds (50) ;
  digitalWrite (gpio, lcd->strbPin, 0) ; delayMicroseconds (50) ;
}

/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
    strobe (lcd) ;

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (myData & 1)) ;
      myData >>= 1 ;
    }
  }
  strobe (lcd) ;
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin,   0, lcd, command);
#endif
  digitalWrite (gpio, lcd->rsPin,   0) ;
  sendDataCmd  (lcd, command) ;
  delay (2) ;
}

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command ;
  register unsigned char i ;

  digitalWrite (gpio, lcd->rsPin,   0) ;

  for (i = 0 ; i < 4 ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], (myCommand & 1)) ;
    myCommand >>= 1 ;
  }
  strobe (lcd) ;
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

void lcdClear (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_CLEAR) ;
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */

void lcdPosition (struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return ;
  if ((y > lcd->rows) || (y < 0))
    return ;

  lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)  /* rowOff [y] */  )) ;

  lcd->cx = x ;
  lcd->cy = y ;
}



/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */

void lcdDisplay (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL ;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursor (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_CURSOR_CTRL ;
  else
    lcdControl &= ~LCD_CURSOR_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursorBlink (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_BLINK_CTRL ;
  else
    lcdControl &= ~LCD_BLINK_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite (gpio, lcd->rsPin, 1) ;
  sendDataCmd  (lcd, data) ;

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0 ;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;
    
    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00)   /* rowOff [lcd->cy] */  )) ;
  }
}


/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

void lcdPuts (struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar (lcd, *string++) ;
}



/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c) { 
  /* ***  COMPLETE the code here  ***  */
    for(int i=0;i<c;i++)
    {
        if ((led & 0xFFFFFFC0) == 0)
        {
            writeLED(gpio,led,HIGH);
            delay(510);
            writeLED(gpio,led,LOW);
            delay(210);
        }
    }
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main (int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use
  struct lcdDataStruct *lcd ;

  int bits, rows, cols ;
  unsigned char func ;
  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;
  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin,  clrOff, setOff, off, res;
  int fd ;
  int  exact, contained;
  char str1[32];
  char str2[32];
  struct timeval t1, t2 ;
  int t ;
  char buf [32] ;

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0;
  int *res_matches;
  
  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  // see the CW spec for the intended meaning of these options
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1) {
      switch (opt) {
          case 'v':
             verbose = 1;
             break;
          case 'h':
             help = 1;
             break;
          case 'd':
             debug = 1;
             break;
          case 'u':
             unit_test = 1;
             break;
          case 's':
             opt_s = atoi(optarg);
             break;
          default:
              fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
              exit(EXIT_FAILURE);
      }
    }


  if (help) {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n"); 
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n"); 
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n"); 
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }
  
  if (unit_test && optind >= argc-1) {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test) {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind+1]);
  }

  if (verbose) {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s){
        fprintf(stdout, "Secret sequence set to %d\n", opt_s);
    }

  }

  seq1 = (int*)malloc(seqlen*sizeof(int));
  seq2 = (int*)malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind+1) { // more arguments to process; only needed with -u 
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind+1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  } else {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s) { // if -s option is given, use the sequence as secret sequence
    if (theSeq==NULL)
      theSeq = (int*)malloc(seqlen*sizeof(int));
    readSeq(theSeq, opt_s);
    if (verbose) {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSeq);
    }
  }
  
  // -------------------------------------------------------
  // LCD constants, hard-coded: 16x2 display, using a 4-bit connection
  bits = 4; 
  cols = 16; 
  rows = 2; 
  // -------------------------------------------------------

  printf ("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring) \n", cols, rows, bits) ;

  if (geteuid () != 0)
    fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int*) malloc(seqlen*sizeof(int));

  //.?????????????????????????????????????????????????????????????????????????
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // -----------------------------------------------------------------------------
  // GPIO base address for Raspberry Pi 2/3
  //gpiobase = 0x3F200000;
  // GPIO base address for Raspberry Pi 4
  gpiobase = 0xFE200000;

  // -----------------------------------------------------------------------------
  // memory mapping 
  // Open the master /dev/memory device




  //lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
  if ((int32_t)gpio == -1)
    return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

  // -------------------------------------------------------
  // Configuration of LED and BUTTON

  /* ***  COMPLETE the code here  ***  */
  
  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *)malloc (sizeof (struct lcdDataStruct)) ;
  if (lcd == NULL)
    return -1 ;

  // hard-wired GPIO pins
  lcd->rsPin   = RS_PIN ;
  lcd->strbPin = STRB_PIN ;
  lcd->bits    = 4 ;
  lcd->rows    = rows ;  // # of rows on the display
  lcd->cols    = cols ;  // # of cols on the display
  lcd->cx      = 0 ;     // x-pos of cursor
  lcd->cy      = 0 ;     // y-pos of curosr

  lcd->dataPins [0] = DATA0_PIN ;
  lcd->dataPins [1] = DATA1_PIN ;
  lcd->dataPins [2] = DATA2_PIN ;
  lcd->dataPins [3] = DATA3_PIN ;


  digitalWrite (gpio, lcd->rsPin,   0) ; pinMode (gpio, lcd->rsPin,   OUTPUT) ;
  digitalWrite (gpio, lcd->strbPin, 0) ; pinMode (gpio, lcd->strbPin, OUTPUT) ;

  for (i = 0 ; i < bits ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], 0) ;
    pinMode      (gpio, lcd->dataPins [i], OUTPUT) ;
  }
  delay (35) ; // mS



  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    func = LCD_FUNC ;					// 4th set: 4-bit mode
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcd->bits = 4 ;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N ;
    lcdPutCommand (lcd, func) ; delay (35) ;
  }

  // Rest of the initialisation sequence
  lcdDisplay     (lcd, TRUE) ;
  lcdCursor      (lcd, FALSE) ;
  lcdCursorBlink (lcd, FALSE) ;
  lcdClear       (lcd) ;

  lcdPutCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;    // set entry mode to increment address counter after write
  lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;  // set display shift to right-to-left

  // END lcdInit ------
  // -----------------------------------------------------------------------------
  // Start of game
  char word[50];
  fprintf(stderr,"Printing welcome message on the LCD display ...\n");
  /* ***  COMPLETE the code here  ***  */

//===============================================================================================
  scanf("%s", word);
  lcdPutCommand(lcd,LCD_DGRAM);
  lcdPuts(lcd,word);

  // Configuration of LED and BUTTON
  pinMode(gpio,pinLED,OUTPUT);
  pinMode(gpio,pin2LED2,OUTPUT);
  pinMode(gpio,pinButton,INPUT);


  /* initialise the secret sequence */


  if (!opt_s){
      initSeq();
  }

  if (opt_s){
      showSeq(theSeq);
      printf("The length of the secret code is : %d.\n",seqlen);
  }





  if (debug) {
      initSeq();
      showSeq(theSeq);
      printf("The length of the secret code is : %d.\n", seqlen);
      while (attempts != 4) {
          attempts++;
          fprintf(stdout, "=======Start round %d========\n", attempts);
          fprintf(stderr, "Input your guess: ");
          int value = 0;

          scanf("%d", &value);

          readSeq(seq1, value); // turn the integer number into a sequence of numbers

          res_matches = countMatches(theSeq, seq1);
          printf("Exact Matches are      : %d\n", res_matches[0]);
          printf("Approx Matches are    : %d\n", res_matches[1]);

          if (res_matches[0] == seqlen) {
              //Then breaking the loop
              found = 1;
              fprintf(stdout, "Success!\n");
              fprintf(stdout, "Used %d attempts\n", attempts);
              exit(EXIT_SUCCESS);
          }
      }
      fprintf(stdout, "Sequence not found\n");
      exit(EXIT_SUCCESS);

  }

  showSeq(theSeq);
  printf("The length of the secret code is : %d.\n",seqlen);

  // optionally one of these 2 calls:
  // waitForEnter () ;
  // waitForButton (gpio, pinButton) ;

  // -----------------------------------------------------------------------------
  // +++++ main loop
  
  while (attempts!=4) {
      attempts++;

      fprintf(stdout,"=======Start round %d========\n",attempts);

//Declare a variable to store weather button is clicked
      int res = 0;            //res

      int current=HIGH;               //current
    /* ******************************************************* */
    /* ***  COMPLETE the code here  ***                        */
    /* this needs to implement the main loop of the game:      */
    /* check for button presses and count them                 */
    /* store the input numbers in the sequence @attSeq@        */
    /* compute the match with the secret sequence, and         */
    /* show the result                                         */
    /* see CW spec for details                                 */
    /* ******************************************************* */
      for(int i=0;i<seqlen;i++)
      {
          //Declaring the variable to count how many times button is pressed
          int count=0;
          //Printing the message
          fprintf(stderr,"----Starting guess %d----\n",(i+1));
          int timer=0;
          while(count<seqlen)
          {
              //Storing whether the button is pressed or not
              res = readButton(gpio,pinButton);
              //Checking if the button is pressed or not
              if((pinButton & 0xFFFFFFC0) == 0)
              {
                  //If the button is pressed and the value is 1 means button is pressed again
                  if(res!=0 && current==HIGH)
                  {
                      current=LOW;
                      //Incrementing the count
                      count++;
                      //Printing that the button is pressed
                      fprintf(stderr,"==== Button Pressed ====\n");
                  }
                      //If the button is released then changing the value of current
                  else if(res==0 && current==LOW )
                  {
                      current = HIGH;
                  }
              }
              else
              {
                  fprintf(stderr, "only supporting on-board pins\n");
              }
              //Delayig the time
              delay(65);
              //Incrementing the time
              timer++;
              //If the timer is equal to 65
              if(timer==65)
              {
                  //Breaking the loop
                  break;
              }
          }
          //If the person does'nt press the button then taking the count as 1
          if(count==0)
          {
              count=1;
          }
          //Printing the message
          fprintf(stderr,"----End of guess %d----\n\n",(i+1));
          fprintf(stderr,"You have pressed button %d times.\n",count);
          //Blinking the red light once as input is accepted
          blinkN(gpio,pin2LED2,1);
          //Blinking the green light count times
          blinkN(gpio,pinLED,count);
          //Assinging the code to the variable
          attSeq[i] = count;
      }


        //blink red LED twice after finishing input
        blinkN(gpio,pin2LED2,2);
        showSeq(attSeq);
        int * res_match = countMatches(theSeq,attSeq);
        showMatches(res_match, theSeq, attSeq,1);

        //blink red LED exact times
        blinkN(gpio,pinLED,res_match[0]);

        int number = res_match[0];

        sprintf(word, "Exact: %d", number );
//==================================================================================
        //show exact times on lcd
        lcdClear(lcd);

        lcdPutCommand(lcd,LCD_DGRAM);
        lcdPuts(lcd,word);
        //blink red LED once as a separator
        blinkN(gpio,pin2LED2,1);
        //Blinking red LED approx times

        blinkN(gpio,pinLED,res_match[1]);
        number = res_match[1];

        sprintf(word, "Approx: %d", number );
        lcdPosition(lcd,1,1);
//=======================================================================================

        lcdPuts(lcd,word);

        if(res_match[0]==seqlen)
        {
            //Then breaking the loop
            found=1;
            break;
        }

        fprintf(stdout,"end of the round %d\n",attempts);
        //Blink Red 3 times as new round starts
        blinkN(gpio,pin2LED2,3);


  }



  if (found) {
      /* ***  COMPLETE the code here  ***  */
      fprintf(stderr,"Congratulations!!! You won the game\n");


      lcdClear(lcd);
      lcdPutCommand(lcd,LCD_DGRAM);
      lcdPuts(lcd,"Success!");
      lcdPosition(lcd,1,1);
      sprintf(word, "Used %d attempts", attempts );
      lcdPuts(lcd,word);

  } else {
      fprintf(stdout, "Sequence not found\n");
      lcdClear(lcd);
      lcdPutCommand(lcd,LCD_DGRAM);
      lcdPuts(lcd,"Sequence not found");
  }
  return 0;
}

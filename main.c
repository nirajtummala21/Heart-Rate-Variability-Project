
// Method → 'User defined'
// Arithmetic = 'Floating Point (Single Precision)';
// Architecture = 'FIR';
// Structure = 'Direct Form';
// Response = 'User defined';
// Fs = 500.0000Hz;


#include "arm_math.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// Sampling frequency (200 Hz)
#define SAMPLE_RATE_HZ 200
#define SYSTICK_RELOAD_VAL (16000000 / SAMPLE_RATE_HZ) - 1  // System clock = 16 MHz

//LEDS
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) //PF2
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define RED_LED_MASK 0x02
#define BLUE_LED_MASK 0x04
#define GREEN_LED_MASK 0x08


// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

void SysTick_Handler(void);
void ADC0_Init(void);
uint16_t Read_ADC(void);

volatile float32_t EcgBuffer1[128];
volatile float32_t EcgBuffer2[128];

volatile float32_t *BackBuffer;
volatile float32_t *ActiveBuffer;

bool BufferPtr = false;

volatile uint8_t CaptureActive = 0;
volatile uint8_t CurrentSample = 0;


// ** FIR Direct Form Implementation **
// y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + ... + bM * x[n-M]

#define NUMTAPS 25  // Filter order = 24

// FIR coefficients
float32_t firCoeffsf32[25] =
          {
            -0.0004194, -0.0011427, -0.0004945,  0.0035871,     // b[24:21]
             0.0088504,  0.0062500, -0.0120312, -0.0366607,     // b[20:17]
            -0.0358595,  0.0231104,  0.1375095,  0.2548769,     // b[16:13]
             0.3048780,  0.2548769,  0.1375095,  0.0231104,     // b[12:9]
            -0.0358595, -0.0366607, -0.0120312,  0.0062500,     // b[8:5]
             0.0088504,  0.0035871, -0.0004945, -0.0011427,     // b[4:1]
            -0.0004194  // b[0]
          };

#define NUMTAPSh 65  // Filter order = 64

// FIR coefficients
float32_t firCoeffsh32[65] =
          {
           -0.0003835, -0.0003952, -0.0004281, -0.0004821,     // b[64:61]
           -0.0005569, -0.0006519, -0.0007662, -0.0008989,     // b[60:57]
           -0.0010488, -0.0012145, -0.0013946, -0.0015873,     // b[56:53]
           -0.0017907, -0.0020031, -0.0022223, -0.0024462,     // b[52:49]
           -0.0026727, -0.0028996, -0.0031246, -0.0033456,     // b[48:45]
           -0.0035604, -0.0037669, -0.0039630, -0.0041469,     // b[44:41]
           -0.0043166, -0.0044706, -0.0046073, -0.0047254,     // b[40:37]
           -0.0048236, -0.0049010, -0.0049569, -0.0049906,     // b[36:33]
            0.9953811, -0.0049906, -0.0049569, -0.0049010,     // b[32:29]
           -0.0048236, -0.0047254, -0.0046073, -0.0044706,     // b[28:25]
           -0.0043166, -0.0041469, -0.0039630, -0.0037669,     // b[24:21]
           -0.0035604, -0.0033456, -0.0031246, -0.0028996,     // b[20:17]
           -0.0026727, -0.0024462, -0.0022223, -0.0020031,     // b[16:13]
           -0.0017907, -0.0015873, -0.0013946, -0.0012145,     // b[12:9]
           -0.0010488, -0.0008989, -0.0007662, -0.0006519,     // b[8:5]
           -0.0005569, -0.0004821, -0.0004281, -0.0003952,     // b[4:1]
           -0.0003835  // b[0]
         };

#define NUMTAPSd 7  // Filter order = 6

// FIR coefficients
float32_t firCoeffsd32[7] =
         {
           -0.1003937,  0.3057444,  0.2646743,  0.0000000,     // b[6:3]
           -0.2646743, -0.3057444,  0.1003937  // b[2:0]
         };

#define NUMTAPSm 25  // Filter order = 24

// FIR coefficients
float32_t firCoeffsm32[25] =
          {
             0.0400000,  0.0400000,  0.0400000,  0.0400000,     // b[24:21]
             0.0400000,  0.0400000,  0.0400000,  0.0400000,     // b[20:17]
             0.0400000,  0.0400000,  0.0400000,  0.0400000,     // b[16:13]
             0.0400000,  0.0400000,  0.0400000,  0.0400000,     // b[12:9]
             0.0400000,  0.0400000,  0.0400000,  0.0400000,     // b[8:5]
             0.0400000,  0.0400000,  0.0400000,  0.0400000,     // b[4:1]
             0.0400000  // b[0]
          };


#define TEST_LENGTH_SAMPLES  128
#define BLOCKSIZE 32
#define NUMBLOCKS  (TEST_LENGTH_SAMPLES/BLOCKSIZE)

float32_t OutputValues[TEST_LENGTH_SAMPLES],OutputValuesH[TEST_LENGTH_SAMPLES],OutputValuesD[TEST_LENGTH_SAMPLES],OutputValuesM[TEST_LENGTH_SAMPLES];
float32_t InputValues[TEST_LENGTH_SAMPLES],InputValuesH[TEST_LENGTH_SAMPLES],InputValuesD[TEST_LENGTH_SAMPLES],InputValuesM[TEST_LENGTH_SAMPLES];

float32_t Differentiated_signal[TEST_LENGTH_SAMPLES];

float32_t *InputValuesf32_ptr = &InputValues[0];   // Low pass filter input pointer
float32_t *OutputValuesf32_ptr = &OutputValues[0]; // Low pass filter Output Pointer

float32_t *InputValuesh32_ptr = &InputValuesH[0];  // High Pass filter Input pointer
float32_t *OutputValuesh32_ptr = &OutputValuesH[0];// High Pass Filter Output Pointer

float32_t *InputValuesd32_ptr = &InputValuesD[0];  // Differentiator Input pointer
float32_t *OutputValuesd32_ptr = &OutputValuesD[0];// Differentiator Output Pointer

float32_t *InputValuesm32_ptr = &InputValuesM[0];  // Moving Average Filter Input pointer
float32_t *OutputValuesm32_ptr = &OutputValuesM[0];// Moving Average Filter Output Pointer


float32_t firStatesf32[BLOCKSIZE+NUMTAPS-1];
float32_t firStatesh32[BLOCKSIZE+NUMTAPSh-1];
float32_t firStatesd32[BLOCKSIZE+NUMTAPSd-1];
float32_t firStatesm32[BLOCKSIZE+NUMTAPSm-1];

float32_t rr_intervals[TEST_LENGTH_SAMPLES];
int num_intervals;

float32_t max_val = 0;
float32_t last_value = 0;
float32_t threshold;
int r_peak_indices[TEST_LENGTH_SAMPLES] = {0};
int r_peak_count = 0;

arm_fir_instance_f32 L,H,D,M;

void EnableFPU(void) {
    NVIC_CPAC_R |= (0xF << 20);  // Enable full access to CP10 and CP11 (FPU)
    __asm(" DSB");           // Data Synchronization Barrier
    __asm(" ISB");           // Instruction Synchronization Barrier
}

// Shell Functions

void reverse(char *str, int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

char* itoa(int num, char *str, int base) {
    int i = 0;
    int isNegative = 0;

    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    if (num < 0 && base == 10) {
        isNegative = 1;
        num = -num;
    }

    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    if (isNegative) {
        str[i++] = '-';
    }

    str[i] = '\0';
    reverse(str, i);

    return str;
}

// Uart Code

void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}


// ADC code

void ADC0_Init(void) {


    SYSCTL_RCGCGPIO_R |=0x10;                //Activate clock for PortE

    while ((SYSCTL_PRGPIO_R & 0x10) == 0);   // Wait for Port E to be ready

    GPIO_PORTE_DIR_R &= ~0x10;              //make PE4 input
    GPIO_PORTE_AFSEL_R|= 0x10;               //Enable alternate Function on PE4
    GPIO_PORTE_DEN_R &= ~0x10;              //disable digital I/O on PE4
    GPIO_PORTE_AMSEL_R|= 0x10;               //enable analog Function on PE4

    SYSCTL_RCGCADC_R |= 0x01;                //Activate ADC0


    // Allow time for the clocks to stabilize
    volatile int delay = SYSCTL_RCGCADC_R;   // Dummy read to delay
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;


     // Configure ADC0 Sequencer 3
    ADC0_PC_R     =0x01;                     //configure for 125K
    ADC0_SSPRI_R  =0x0123;                   //Seq3 is highest priority
    ADC0_ACTSS_R &= ~0x0008;                 // Disable sample sequencer 3
    ADC0_EMUX_R &= ~0xF000;                  // Software trigger conversion
    ADC0_SSMUX3_R = (ADC0_SSMUX3_R& 0xFFFFFFF0)+9;   // Set channel (AIN9 - PE4)
    ADC0_SSCTL3_R = 0x0006;                    // Single-ended, single sample, end-of-sequence
    ADC0_IM_R    &= ~0x0008;                 // disable SS3 interrupts
    ADC0_ACTSS_R |= 0x0008;                    // Enable sample sequencer 3

    // Check if ADC0 peripheral is ready
    while ((SYSCTL_PRADC_R & 0x01) == 0);    // Wait until ADC0 is ready


}


void GPIO_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x08;           // Enable clock for Port D
    while ((SYSCTL_PRGPIO_R & 0x08) == 0); // Wait for Port D to be ready

    GPIO_PORTD_DIR_R &= ~0x0C;           // Set PD2 and PD3 as input
    GPIO_PORTD_DEN_R |= 0x0C;            // Enable digital function on PD2 and PD3

    SYSCTL_RCGC2_R=SYSCTL_RCGC2_GPIOF;
    GPIO_PORTF_DIR_R |= RED_LED_MASK|BLUE_LED_MASK|GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= RED_LED_MASK|BLUE_LED_MASK|GREEN_LED_MASK;
    GPIO_PORTF_DR2R_R|= RED_LED_MASK|BLUE_LED_MASK|GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)

}

void main(void)
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    EnableFPU();
    initUart0();

    ADC0_Init();
 //   GPIO_Init();

    putsUart0("\n\r HRV Project Debugging \n\r");
    // Set up SysTick timer for 200 Hz
    NVIC_ST_CTRL_R = 0;                         // Disable SysTick during setup
    NVIC_ST_RELOAD_R = SYSTICK_RELOAD_VAL;      // Set reload value for 200 Hz
    NVIC_ST_CURRENT_R = 0;                      // Any write to CURRENT clears it
    NVIC_ST_CTRL_R = 0x07;                      // Enable SysTick with core clock and interrupt


    // initialize  foreground ActiveBuffer , ISR BackBuffer

    BufferPtr = false;

    ActiveBuffer = EcgBuffer1;
    BackBuffer   = EcgBuffer2;


  int n,k;


  // setup test sinusoid input
  for (n=0; n<TEST_LENGTH_SAMPLES; n++)
    InputValues[n] = ActiveBuffer[n];    // test sinusoid input

  // Initialise Low pass FIR (Direct Form implementation)
   arm_fir_init_f32 (&L, NUMTAPS, (float32_t *)&(firCoeffsf32[0]), &(firStatesf32[0]), BLOCKSIZE);

  // Initialise High Pass FIR (Direct Form implementation)
   arm_fir_init_f32 (&H, NUMTAPSh, (float32_t *)&(firCoeffsh32[0]), &(firStatesh32[0]), BLOCKSIZE);

   // Initialise Differentiator filter FIR (Direct Form implementation)
    arm_fir_init_f32 (&D, NUMTAPSd, (float32_t *)&(firCoeffsd32[0]), &(firStatesd32[0]), BLOCKSIZE);

   // Initialise Moving Average Filter FIR (Direct Form implementation)
    arm_fir_init_f32 (&M, NUMTAPSm, (float32_t *)&(firCoeffsm32[0]), &(firStatesm32[0]), BLOCKSIZE);

  while (1)

  {

    if(OutputValuesM[0]!=0)
    {
       r_peak_count=0;
       // Step 1: Find the maximum value
       int x = 0;
       for (; x <TEST_LENGTH_SAMPLES; x++)
       {
          if (OutputValuesM[x] > max_val)
          {
              max_val = OutputValuesM[x];
          }
      }

      // Step 2: Set threshold as 60% of max value
      threshold = 0.6 * max_val;

      // Step 3: Detect R-peaks and store their indices
      int z = 1;
      for (; z < TEST_LENGTH_SAMPLES - 1; z++)
       {
          if (OutputValuesM[z] > threshold && OutputValuesM[z] > last_value && OutputValuesM[z] > OutputValuesM[z + 1])
          {
              r_peak_indices[r_peak_count++] = z;
          }
          last_value = OutputValuesM[z];
        }

//         // Step 4: Calculate RR intervals in milliseconds

             if (r_peak_count < 2)
             {
                 int p= (r_peak_count * 60 *SAMPLE_RATE_HZ)/TEST_LENGTH_SAMPLES;
                 char Buffer[10];

                 itoa(p,Buffer,10);
                 putsUart0("\n\r hear rate is:");
                 putsUart0(Buffer);
                 putsUart0("\n\r");

             }
     if(r_peak_count >1)
      {
             num_intervals = 0;
             int j = 1;
             for (; j < r_peak_count; j++)
                {
                  rr_intervals[num_intervals] = (float32_t)(r_peak_indices[j] - r_peak_indices[j - 1]) * 1000.0 / SAMPLE_RATE_HZ;
                  (num_intervals)++;
                }


          float32_t mean_rr = 0;
          float32_t sum_rmssd = 0;
          float32_t sdnn = 0, rmssd = 0;

          // Step 1: Use CMSIS-DSP to calculate the mean
          arm_mean_f32(rr_intervals, num_intervals, &mean_rr);

          // Step 2: Use CMSIS-DSP to calculate the standard deviation (SDNN)
          arm_std_f32(rr_intervals, num_intervals, &sdnn);

          // Step 3: Calculate RMSSD
          int i = 1;
          for (; i < num_intervals; i++)
          {
              sum_rmssd += powf(rr_intervals[i] - rr_intervals[i - 1], 2);
          }

          rmssd = sqrtf(sum_rmssd / (num_intervals - 1));

          const float SDNN_THRESHOLD = 50.0;  // High variability in RR intervals
          const float RMSSD_THRESHOLD = 40.0; // Lower successive differences

          // AFib is likely if SDNN > threshold and RMSSD < threshold
          if (sdnn > SDNN_THRESHOLD && rmssd < RMSSD_THRESHOLD)
          {
            putsUart0("\n\rAFib Detected\n\r");                 // AFib detected
          }

          else
           putsUart0("\n\rNO AFib Condition\n\r");
       }
     }

      // setup
        for (n=0; n<TEST_LENGTH_SAMPLES; n++)
          InputValues[n] = ActiveBuffer[n];    // test raw ecg input

              // Perform Low Pass Filter FIR filtering
        for (k=0; k < NUMBLOCKS; k++)
          arm_fir_f32 (&L, InputValuesf32_ptr + (k*BLOCKSIZE), OutputValuesf32_ptr + (k*BLOCKSIZE), BLOCKSIZE);    // perform FIR filtering


        for (n=0; n<TEST_LENGTH_SAMPLES; n++)
            InputValuesH[n] = OutputValues[n];    // test low pass filter input - Passing Low pass Output signal to High pass filter


           // Perform High Pass Filter FIR filtering
        for (k=0; k < NUMBLOCKS; k++)
          arm_fir_f32 (&H, InputValuesh32_ptr + (k*BLOCKSIZE), OutputValuesh32_ptr + (k*BLOCKSIZE), BLOCKSIZE);    // perform FIR filtering


        for (n=0; n<TEST_LENGTH_SAMPLES; n++)
             InputValuesD[n] = OutputValuesH[n];   // test High pass filter input - Passing High pass Output signal to Differentiator filter


           // Perform Differentiator FIR filtering
        for (k=0; k < NUMBLOCKS; k++)
         arm_fir_f32 (&D, InputValuesd32_ptr + (k*BLOCKSIZE), OutputValuesd32_ptr + (k*BLOCKSIZE), BLOCKSIZE);    // perform FIR filtering


        // Square the signal using arm_mult_f32
            arm_mult_f32(OutputValuesD,OutputValuesD,Differentiated_signal,TEST_LENGTH_SAMPLES);


        for (n=0; n<TEST_LENGTH_SAMPLES; n++)
             InputValuesM[n] = Differentiated_signal[n];   // test High pass filter input - Passing High pass Output signal to Differentiator filter

        // Perform Moving Average Filter FIR filtering
        for (k=0; k < NUMBLOCKS; k++)
        arm_fir_f32 (&M, InputValuesm32_ptr + (k*BLOCKSIZE), OutputValuesm32_ptr + (k*BLOCKSIZE), BLOCKSIZE);    // perform FIR filtering
//



         while(CaptureActive == 1)
         {

         }

         if(BufferPtr == false)
         {
             BufferPtr = true;
             ActiveBuffer = EcgBuffer2;
             BackBuffer   = EcgBuffer1;
         }

         else
         {
           BufferPtr = false;
           ActiveBuffer = EcgBuffer1;
           BackBuffer   = EcgBuffer2;

         }

         CaptureActive = 1;



  }


}

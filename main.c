//*****************************************************************************
//
// Application Name     - Heart Beat Rate Detection
// Application Overview - This application use data from SPI bus to calculate
//                        the main frequency. The main purpose is to detect
//                        the heart beat rate
//
//*****************************************************************************

//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <assert.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"
#include "systick_if.h"
#include "timer_if.h"
#include "pin_mux_config.h"



#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  12000 //12000 bits per second
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

#define NUMSAMPLE       4096

#define SAMPLES_PER_SECOND  SPI_IF_BIT_RATE/16

#define PROCESS_SIZE    4096

#define SYSCLK          80000000

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned short g_ucTxBuff[TR_BUFF_SIZE];
static unsigned short g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;
unsigned long TimerInts;

static unsigned long g_ulA2IntCount;

#ifndef PI
# define PI 3.14159265358979323846264338327950288
#endif

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

typedef float real;
typedef struct{real Re; real Im;} complex;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI module as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{
    unsigned long ulUserData;
    unsigned long ulDummy;

    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_16));

    //
    // Enable SPI for communication
    //
    SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Master Mode\n\r");


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Initialize variable
    //
    ulUserData = 0;


}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}



/***************************************************************************************
*    Title: Filter Code Definitions
*    Author: Shawn
*    Date: 10/29/2019
*    Code version: 1.0
*    Availability: https://sestevenson.wordpress.com/implementation-of-fir-filtering-in-c-part-1/
*
***************************************************************************************/

/*
// FIR init
void firFloatInit( void )
{
    memset( insamp, 0, sizeof( insamp ) );
}

// the FIR filter function
void firFloat( double *coeffs, double *input, double *output,
       int length, int filterLength )
{
    double acc;     // accumulator for MACs
    double *coeffp; // pointer to coefficients
    double *inputp; // pointer to input samples
    int n;
    int k;

    // put the new samples at the high end of the buffer
    memcpy( &insamp[filterLength - 1], input,
            length * sizeof(double) );

    // apply the filter to each input sample
    for ( n = 0; n < length; n++ ) {
        // calculate output n
        coeffp = coeffs;
        inputp = &insamp[filterLength - 1 + n];
        acc = 0;
        for ( k = 0; k < filterLength; k++ ) {
            acc += (*coeffp++) * (*inputp--);
        }
        output[n] = acc;
    }
    // shift input samples back in time for next time
    memmove( &insamp[0], &insamp[length],
            (filterLength - 1) * sizeof(double) );

}
*/

//*****************************************************************************
//! Export to csv file
//!
//! This function export the array to a csv file
//!
//! \param filename the filename of output file
//!        a[] a double array will be output
//!        n size of a[]
//!
//! \return None.
//
//*****************************************************************************
/*
void exportToCSV(char const *filename, double a[],int n){

    printf("\n Creating %s.csv file",filename);

    FILE *fp;

    int i,j;

    filename=strcat(filename,".csv");

    fp=fopen(filename,"w+");


    // fopen() return NULL if last operation was unsuccessful
    if(fp == NULL)
    {
        // File not created hence exit
        printf("Unable to create file.\n");
        exit(EXIT_FAILURE);
    }

    for(j=0;j<n;j++)    //n is the row
        fprintf(fp,"%lf\n",a[j]);

    fclose(fp);

    Report(" %s file created\n\r",filename);

    return;
}


/***************************************************************************************
*    Title: FFT
*    Author: Unknown
*    Date: Unknown
*    Code version: 1.0
*    Availability: https://www.math.wustl.edu/~victor/mfmm/fourier/fft.c
*
***************************************************************************************/

/*
   fft(v,N):
   [0] If N==1 then return.
   [1] For k = 0 to N/2-1, let ve[k] = v[2*k]
   [2] Compute fft(ve, N/2);
   [3] For k = 0 to N/2-1, let vo[k] = v[2*k+1]
   [4] Compute fft(vo, N/2);
   [5] For m = 0 to N/2-1, do [6] through [9]
   [6]   Let w.re = cos(2*PI*m/N)
   [7]   Let w.im = -sin(2*PI*m/N)
   [8]   Let v[m] = ve[m] + w*vo[m]
   [9]   Let v[m+N/2] = ve[m] - w*vo[m]
 */
void
fft( complex *v, int n, complex *tmp )
{
  if(n>1) {         /* otherwise, do nothing and return */
    int k,m;    complex z, w, *vo, *ve;
    ve = tmp; vo = tmp+n/2;
    for(k=0; k<n/2; k++) {
      ve[k] = v[2*k];
      vo[k] = v[2*k+1];
    }
    fft( ve, n/2, v );      /* FFT on even-indexed elements of v[] */
    fft( vo, n/2, v );      /* FFT on odd-indexed elements of v[] */
    for(m=0; m<n/2; m++) {
      w.Re = cos(2*PI*m/(double)n);
      w.Im = -sin(2*PI*m/(double)n);
      z.Re = w.Re*vo[m].Re - w.Im*vo[m].Im; /* Re(w*vo[m]) */
      z.Im = w.Re*vo[m].Im + w.Im*vo[m].Re; /* Im(w*vo[m]) */
      v[  m  ].Re = ve[m].Re + z.Re;
      v[  m  ].Im = ve[m].Im + z.Im;
      v[m+n/2].Re = ve[m].Re - z.Re;
      v[m+n/2].Im = ve[m].Im - z.Im;
    }
  }
  return;
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    uint16_t ulRecvData;
    int sampSize = 0;
    int i = 0;
    int j = 0;
    int k = 0;
    double realV = 0;

    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initializing the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        Heart Rate Detection Application  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    //
    // Reset the peripheral
    //
    PRCMPeripheralReset(PRCM_GSPI);

    //Initialize the SPI master mode
    MasterMain();

    //Full dataset
    double floatInputFull[NUMSAMPLE];          //4096
    //Half dataset after down sampling
    double floatInputHalf[NUMSAMPLE/2];          //2048
    //Carrier signal of modulation
    double demodulatedSig[NUMSAMPLE];

    j=0;
    printf("%d\n\r", j);
    Message("Detecting....Don't move.\n\r");
    while(j<NUMSAMPLE)
    {
        //
        // Enable Chip select
        //
        SPICSEnable(GSPI_BASE);

        SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;

        //Get the data from ADC
        SPIDataGet(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        //Shift the data and masking
        ulRecvData = ulRecvData >> 3;
        ulRecvData = (ulRecvData & 0x7FF);
        //Convert the data to volt
        realV = (double)ulRecvData * 3.3 / 1024;
        //Store the voltage value in an array
        floatInputFull[j] = realV;

        ucRxBuffNdx++;
        j++;

       //
       // Disable chip select
       //
       SPICSDisable(GSPI_BASE);
    }

    //Demodulate the raw signal by detecting raw data
    for(j = 0; j < NUMSAMPLE; j++){
        //Multiply by 1 if greater than 1
        if(floatInputFull[j] > 1)
            demodulatedSig[j] = floatInputFull[j];
        //Multiply by 0 if low
        else
            demodulatedSig[j] = 0;
    }

    //Down sampling
    k = 1;
    sampSize=0;
    for(j = 0; j < NUMSAMPLE; j++){
        //Sampling for every 2 samples
        if(k == 2){
            floatInputHalf[sampSize] = demodulatedSig[j];
            sampSize++;
            k = 0;
        }
        k++;
    }

    //Raw data and a scratch array
    complex v[PROCESS_SIZE/2], scratch[PROCESS_SIZE/2];

    //Assign the real part and imaginary part for v[]
    for(k=0; k < PROCESS_SIZE/2; k++) {
        v[k].Re = floatInputHalf[k];
        v[k].Im = 0.0;
    }

    //Calculate the FFT for v[]
    fft( v, PROCESS_SIZE/2, scratch );

    double tempPeak = 0;
    int peakIdx = 0;
    double Fc = 0.0;
    //Find the peak between 1 to 3 Hz
    for(j = 5; j < 20; j++){
        if(sqrt((v[j].Re*v[j].Re)+(v[j].Im*v[j].Im)) > tempPeak){
            peakIdx = j;
            tempPeak = sqrt((v[j].Re*v[j].Re)+(v[j].Im*v[j].Im));
        }
    }

    //Calculate the frequency and heart beat rate
    Fc = (double)peakIdx / (PROCESS_SIZE/2) * SAMPLES_PER_SECOND/2;

    exit(EXIT_SUCCESS);

}

#include "F28x_Project.h"
#include <stdio.h>

// define macros
#define ACQPS_SETTING 9u
#define ADC_WARMUP_TIME 1000
#define DAC_MAX_VALUE 4096u
#define DAC_OFFSET_VALUE 2048u
#define DAC_WARMUP_TIME 10
#define DT 1.0
#define KP 1.0
#define KI 0.0
#define KD 0.0
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define NUM_INPUTS_PER_OUTPUT 2
#define NUM_OUTPUTS 1
#define HOLD_OUTPUT_GPIO_PIN 89
#define LED_GPIO_PIN 90
#define LED_THRESHOLD 205

// declare array holding ADC values and store this in ramgs0 for use in DMA
#pragma DATA_SECTION(analog_ins, "ramgs0");
Uint16 analog_ins[NUM_OUTPUTS][NUM_INPUTS_PER_OUTPUT];

// declare array holding DAC values
Uint16 analog_outs[NUM_OUTPUTS];

// move this code to RAM
#pragma CODE_SECTION(dma_ch2_int_isr, ".TI.ramfunc");
__interrupt void dma_ch2_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK2 = PIEACK_GROUP7;
	EDIS;
}

// move this code to RAM
#pragma CODE_SECTION(dma_ch3_int_isr, ".TI.ramfunc");
__interrupt void dma_ch3_int_isr(void)
{
	// acknowledge that the interrupt was received
	EALLOW;
	PieCtrlRegs.PIEACK.bit.ACK3 = PIEACK_GROUP7;
	EDIS;
}

void configure_adcs(void)
{
	// allow writing to protected registers
	EALLOW;

	// set the ADC clocks to one quarter of the input clock
	AdcaRegs.ADCCTL2.bit.PRESCALE = 0x6;

	// set the ADCs to have twelve bit resolution and put it in single signal mode
	// in single signal mode, the input pin voltage is referenced against VREFLO
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	// generate interrupt pulses at the end of conversions
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	// power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	// give time for the ADCs to power up
	DELAY_US(ADC_WARMUP_TIME);

	// disallow writing to protected registers
	EDIS;
}

void setup_adc_channels(void)
{
	// allow writing to protected registers
	EALLOW;

	// set channels
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //ADCINA2
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //ADCINA3

	// set sample windows
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_SETTING;

	// set what SOCs trigger ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
	AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1;

	// enable the first two ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
	AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;

	// enable continuous mode for the first two ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1;

    // reset the first two ADC interrupt registers
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;

	// disallow writing to protected registers
	EDIS;
}

void configure_dacs(void)
{
	// allow writing to protected registers
	EALLOW;

	// use the same voltage references as the ADC
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    DacbRegs.DACCTL.bit.DACREFSEL = 1;
    DaccRegs.DACCTL.bit.DACREFSEL = 1;

    // enable the DACs
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    DaccRegs.DACOUTEN.bit.DACOUTEN = 1;

    // zero the DACs
    DacaRegs.DACVALS.bit.DACVALS = 0x0;
    DacbRegs.DACVALS.bit.DACVALS = 0x0;
    DaccRegs.DACVALS.bit.DACVALS = 0x0;

    // give time for the DACs to power up
    DELAY_US(DAC_WARMUP_TIME);

	// disallow writing to protected registers
	EDIS;
}

void configure_dma(void)
{
	// disconnect CLA and connect DMA
	EALLOW;
	CpuSysRegs.SECMSEL.bit.PF1SEL = 1;
	EDIS;

	// initialize DMA
	DMAInitialize();

	// setup DMA channel two
	DMACH2AddrConfig((volatile Uint16 *)(&analog_ins[0][0]), (volatile Uint16 *)(&AdcaResultRegs.ADCRESULT0));

	// send one 16 bit word in a burst
	DMACH2BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH2TransferConfig(1, 0, 0);

	// configure DMA channel two
	DMACH2ModeConfig(1, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);

	// setup DMA channel three
	DMACH3AddrConfig((volatile Uint16 *)(&analog_ins[0][1]), (volatile Uint16 *)(&AdcaResultRegs.ADCRESULT1));

	// send one 16 bit word in a burst
	DMACH3BurstConfig(1, 0, 0);

	// send one burst per transfer
	DMACH3TransferConfig(1, 0, 0);

	// configure DMA channel three
	DMACH3ModeConfig(2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
					 SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
					 CHINT_END, CHINT_ENABLE);
}

// move this code to RAM
#pragma CODE_SECTION(pid_loop, ".TI.ramfunc");
void pid_loop(int16 errors[NUM_OUTPUTS],
			  int16 integrals[NUM_OUTPUTS],
			  int16 derivatives[NUM_OUTPUTS],
			  int16 previous_errors[NUM_OUTPUTS])
{
	// start the ADC SOCs
	AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC1 = 1;

	// perform negative PID on all outputs
	size_t i;
	for (i = 0; i < NUM_OUTPUTS; i++) {
		errors[i] = analog_ins[i][1] - analog_ins[i][0];
		integrals[i] += errors[i] * DT;
		derivatives[i] = (errors[i] - previous_errors[i]) / DT;
		analog_outs[i] = MIN(KP * errors[i] + KI * integrals[i] + KD * derivatives[i] + DAC_OFFSET_VALUE, DAC_MAX_VALUE - 1);
		previous_errors[i] = errors[i];
	}
	// printf("%d %d \n", errors[0], integrals[0]);

	// output on the DACs if the hold signal is off
	if (GPIO_ReadPin(HOLD_OUTPUT_GPIO_PIN) == 0) {
		DacaRegs.DACVALS.bit.DACVALS = DAC_OFFSET_VALUE; //Adcsoc0 // adcina2 //
		DacbRegs.DACVALS.bit.DACVALS = analog_outs[0];
		DaccRegs.DACVALS.bit.DACVALS = MIN(errors[0] + DAC_OFFSET_VALUE, DAC_MAX_VALUE - 1);
	}


	// turn on LED if locked
	if (errors[0] < LED_THRESHOLD)
		GPIO_WritePin(LED_GPIO_PIN, 0);
	else
		GPIO_WritePin(LED_GPIO_PIN, 1);
}

int main(void)
{
	// declare PID arrays
	int16 errors[NUM_OUTPUTS];
	int16 integrals[NUM_OUTPUTS];
	int16 derivatives[NUM_OUTPUTS];
	int16 previous_errors[NUM_OUTPUTS];

	// initialize system control
	InitSysCtrl();

	// initialize GPIO
	InitGpio();
	GPIO_SetupPinMux(HOLD_OUTPUT_GPIO_PIN, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(LED_GPIO_PIN, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(HOLD_OUTPUT_GPIO_PIN, GPIO_INPUT, GPIO_PULLUP);
	GPIO_SetupPinOptions(LED_GPIO_PIN, GPIO_OUTPUT, GPIO_PUSHPULL);

	// disallow CPU interrupts
	DINT;

	// initialize PIE interrupts
	InitPieCtrl();

	// disable PIE interrupts
	IER = 0x0;

	// clear all PIE interrupts
	IFR = 0x0;

	// populates the PIE vector table for debug purposes
	InitPieVectTable();

	// enable the PIE block
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

	// enable DMA interrupts
	PieCtrlRegs.PIEIER7.bit.INTx2 = 1;
	PieCtrlRegs.PIEIER7.bit.INTx3 = 1;
	IER = M_INT7;

	// enable global interrupts
	EINT;

	// enable real time debug interrupts
	ERTM;

	// map DMA interrupts to functions
	EALLOW;
	PieVectTable.DMA_CH2_INT= &dma_ch2_int_isr;
	PieVectTable.DMA_CH3_INT= &dma_ch3_int_isr;
	EDIS;

	// copy the necessary code from flash memory to RAM
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)(&RamfuncsLoadSize));

	// setup flash wait states
    InitFlash_Bank0();
    InitFlash_Bank1();

	// configure the ADCs
	configure_adcs();

	// setup acquisition on ADCINs
	setup_adc_channels();

	// configure the DACs
	configure_dacs();

	// configure DMA
	configure_dma();

	// zero all arrays
	size_t i;
	size_t j;
	for (i = 0; i < NUM_OUTPUTS; i++) {
		for (j = 0; j < NUM_INPUTS_PER_OUTPUT; j++)
			analog_ins[i][j] = 0;

		analog_outs[i] = 0;
		errors[i] = 0;
		integrals[i] = 0;
		derivatives[i] = 0;
		previous_errors[i] = 0;
	}

	// start the DMA channels
	StartDMACH2();
	StartDMACH3();

	while (1) {
		// run the PID loop
        pid_loop(errors, integrals, derivatives, previous_errors);

		// delay for a time DT
		DELAY_US(DT);
	}
}

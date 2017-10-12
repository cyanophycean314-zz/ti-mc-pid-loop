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

// declare PID arrays
int16 errors[NUM_OUTPUTS];
int16 integrals[NUM_OUTPUTS];
int16 derivatives[NUM_OUTPUTS];
int16 previous_errors[NUM_OUTPUTS];
int16 analog_ins[NUM_OUTPUTS][NUM_INPUTS_PER_OUTPUT];
int16 analog_outs[NUM_OUTPUTS];

void configure_adcs(void)
{
	// allow writing to protected registers
	EALLOW;

	// set the ADC clocks to one quarter of the input clock
	AdcaRegs.ADCCTL2.bit.PRESCALE = 0x6;
	AdcbRegs.ADCCTL2.bit.PRESCALE = 0x6;

	// set the ADCs to have twelve bit resolution and put it in single signal mode
	// in single signal mode, the input pin voltage is referenced against VREFLO
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	// generate interrupt pulses at the end of conversions
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	// power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

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
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //socket 0 = adcin a2
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //socket 1 = adcin a3
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4; //adcin a4
	AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5; //adcin a5
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2; //adcin b2
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3; //adcin b3
	AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4; //adcin b4
	AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5; //adcin b5

	// set sample windows
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = ACQPS_SETTING;
	AdcaRegs.ADCSOC3CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = ACQPS_SETTING;
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = ACQPS_SETTING;

	// set what SOCs trigger ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
	AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT3SEL = 2;
	AdcaRegs.ADCINTSEL3N4.bit.INT4SEL = 3;
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
	AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT3SEL = 2;
	AdcbRegs.ADCINTSEL3N4.bit.INT4SEL = 3;

	// enable the ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
	AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT3E = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT4E = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT3E = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT4E = 1;

	// enable continuous mode for the ADC interrupt registers
	AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT3CONT = 1;
	AdcaRegs.ADCINTSEL3N4.bit.INT4CONT = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT3CONT = 1;
	AdcbRegs.ADCINTSEL3N4.bit.INT4CONT = 1;

    // reset the ADC interrupt registers
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;

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

void reset_arrays(void)
{
	// zero all arrays
	size_t i, j;
	for (i = 0; i < NUM_OUTPUTS; i++) {
		for (j = 0; j < NUM_INPUTS_PER_OUTPUT; j++) {
			analog_ins[i][j] = 0;
		}
		analog_outs[i] = 0;
		errors[i] = 0;
		integrals[i] = 0;
		derivatives[i] = 0;
		previous_errors[i] = 0;
	}
}

void pid_loop(void)
{
	// start the ADC SOCs
	AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC2 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC3 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC2 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC3 = 1;

	// perform PID on all outputs
	size_t i;
	long pout;
	for (i = 0; i < NUM_OUTPUTS; i++) {
		errors[i] = AdcaResultRegs.ADCRESULT1 - AdcaResultRegs.ADCRESULT0;
		integrals[i] += errors[i] * DT;
		derivatives[i] = (errors[i] - previous_errors[i]) / DT;
		pout = (long) errors[i] * AdcaResultRegs.ADCRESULT2 / 3000;
		// printf("%d\n", pout);
		analog_outs[i] = MIN(pout + KI * integrals[i] + KD * derivatives[i] + DAC_OFFSET_VALUE, DAC_MAX_VALUE - 1);
		previous_errors[i] = errors[i];
	}
	// printf("%d %d %d\n", analog_ins[0][0], kp, AdcaResultRegs.ADCRESULT2);
	// printf("%d %d\n", errors[0], AdcaResultRegs.ADCRESULT2);

	// output on the DACs if the hold signal is off
	if (GPIO_ReadPin(HOLD_OUTPUT_GPIO_PIN) == 0) {
		DacaRegs.DACVALS.bit.DACVALS = DAC_OFFSET_VALUE; //Adcsoc0 // adcina2 //
		DacbRegs.DACVALS.bit.DACVALS = pout;
		DaccRegs.DACVALS.bit.DACVALS = errors[0];
	}

	// turn on LED if locked
	GPIO_WritePin(LED_GPIO_PIN, errors[0] < LED_THRESHOLD);
}

int main(void)
{
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

	// enable global interrupts
	EINT;

	// enable real time debug interrupts
	ERTM;

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

	// zero the arrays
	reset_arrays();

	while (1) {
		// run the PID loop
        pid_loop();

		// delay for a time DT
		DELAY_US(DT);
	}
}

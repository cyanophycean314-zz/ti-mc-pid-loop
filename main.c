#include "F28x_Project.h"
#include <stdio.h>

// define macros
#define ACQPS_SETTING 9u
#define ADC_SCALE_FACTOR 40
#define ADC_WARMUP_TIME 1000
#define ADC_MAX_VALUE 3200
#define DAC_MAX_VALUE 2343 // 4096
#define DAC_OFFSET_VALUE 1171 // 2048
#define DAC_WARMUP_TIME 10
#define DT 1.0
#define KP 1.0
#define KI 0.0
#define KD 0.0
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define SGN(X) ((X > 0) ? 1 : ((X < 0) ? -1 : 0))
#define MAXVAL 32767
#define NUM_INPUTS_PER_OUTPUT 2
#define NUM_OUTPUTS 1
#define PID_OUTPUT_GPIO_PIN 89
#define MANUAL_SET_OUTPUT_GPIO_PIN 90
#define LED_GREEN_GPIO_PIN 65
#define LED_RED_GPIO_PIN 69
#define LED_THRESHOLD 20

// declare PID arrays
int16 errors[NUM_OUTPUTS];
int16 integrals[NUM_OUTPUTS];
int16 derivatives[NUM_OUTPUTS];
int16 previous_errors[NUM_OUTPUTS];
int16 previous_outputs[NUM_OUTPUTS];
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

void setup_gpio(void) {
	// System set up GPIO
	InitGpio();
	// Set up multiplexer options
	GPIO_SetupPinMux(PID_OUTPUT_GPIO_PIN, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(MANUAL_SET_OUTPUT_GPIO_PIN, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(LED_RED_GPIO_PIN, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinMux(LED_GREEN_GPIO_PIN, GPIO_MUX_CPU1, 0);
	// Set up pin options
	GPIO_SetupPinOptions(PID_OUTPUT_GPIO_PIN, GPIO_INPUT, GPIO_PULLUP);
	GPIO_SetupPinOptions(MANUAL_SET_OUTPUT_GPIO_PIN, GPIO_INPUT, GPIO_PULLUP);
	GPIO_SetupPinOptions(LED_RED_GPIO_PIN, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinOptions(LED_GREEN_GPIO_PIN, GPIO_OUTPUT, GPIO_PUSHPULL);
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
		previous_outputs[i] = 0;
	}
}

void pid_loop(void)
{
	// start the ADC SOCs
	AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC2 = 1;
	AdcaRegs.ADCSOCFRC1.bit.SOC3 = 1;
//	AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcbRegs.ADCSOCFRC1.bit.SOC2 = 1;
//	AdcbRegs.ADCSOCFRC1.bit.SOC3 = 1;
	// perform PID on all outputs
	int pout, dout, iout;
	// Calculate error
	errors[0] = AdcaResultRegs.ADCRESULT1 - AdcaResultRegs.ADCRESULT0;

	integrals[0] += errors[0] * DT;
	derivatives[0] = (errors[0] - previous_errors[0]) / DT;
	// printf("%d %d\n", derivatives[0], AdcaResultRegs.ADCRESULT2);

	// Scale outputs with ADC inputed constants
	pout = errors[0] / ADC_SCALE_FACTOR * (AdcaResultRegs.ADCRESULT3 / ADC_SCALE_FACTOR);
	dout = derivatives[0] * (AdcaResultRegs.ADCRESULT2 / ADC_SCALE_FACTOR);

	// to calculate integrals you need special code to ensure that out of bounds are handled correctly.
	int ifac = integrals[0] / ADC_SCALE_FACTOR;
	int ki = AdcbResultRegs.ADCRESULT1 / ADC_SCALE_FACTOR;
	iout = (ki == 0 || SGN(ifac * ki) == SGN(ifac)) ? (ifac * ki) : (MAXVAL * SGN(ifac));
	// printf("%d %d %d %d\n", ifac, ki, ifac * ki, iout);

	previous_errors[0] = errors[0];
	//printf("%d %d %d\n", AdcaResultRegs.ADCRESULT3, AdcbResultRegs.ADCRESULT1, AdcaResultRegs.ADCRESULT2);
	//printf("%d %d %d\n", pout, iout, dout);
	int sumout = pout + iout + dout;

	DacaRegs.DACVALS.bit.DACVALS = DAC_OFFSET_VALUE;
	DaccRegs.DACVALS.bit.DACVALS = MAX(MIN(DAC_OFFSET_VALUE + errors[0], DAC_MAX_VALUE - 1), 0);

	char pid_out = GPIO_ReadPin(PID_OUTPUT_GPIO_PIN);

	if (pid_out) {
		previous_outputs[0] = MAX(MIN(DAC_OFFSET_VALUE + sumout, DAC_MAX_VALUE - 1), 0);
		//printf("Out: %d\n", previous_outputs[0]);
	}

	// output PID signal on the DACs if the PID signal is on
	DacbRegs.DACVALS.bit.DACVALS = (pid_out || !GPIO_ReadPin(MANUAL_SET_OUTPUT_GPIO_PIN)) ? previous_outputs[0] : (AdcbResultRegs.ADCRESULT2 * 3 / 4);

	// turn on LED if locked
	char locked = (errors[0] < 0 ? -errors[0] : errors[0]) < LED_THRESHOLD;
	GPIO_WritePin(LED_GREEN_GPIO_PIN, locked);
	GPIO_WritePin(LED_RED_GPIO_PIN, !locked);

}

int main(void)
{
	// initialize system control
	InitSysCtrl();

	// initialize GPIO
	setup_gpio();

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

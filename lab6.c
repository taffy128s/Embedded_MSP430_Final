#include <msp430.h> 
#include <intrinsics.h>
#define LED1 BIT0
#define LED2 BIT6
#define B1 BIT3
#define UART_TXD 0x02
#define UART_RXD 0x04
#define UART_TBIT_DIV_2	(1000000 / (9600 * 2))
#define UART_TBIT		(1000000 / 9600)
#define SIZE 64

volatile int mem_on, mem_off, mem_id = 0, now_on = 0, sendCount = 1;
volatile int setup = 1, idx = 0, mem_times = 0, my_times = 0, mem_interval = 0;
volatile int mem_1, mem_2, sum, init = 1, send = 0;
volatile int Temp[SIZE];
unsigned int txData;
unsigned char rxBuffer, rxString[4] = {0}, rxIdx = 0;

void flash(char id, int on, int off);
void temp(int interval, int times);
void TimerA_UART_init();
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    ADC10CTL1 = INCH_10 + SHS_1 + CONSEQ_2;
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;

    __enable_interrupt();
	TA1CCR0 = 30;
	TA1CCTL0 |= CCIE;
	TA1CTL = TASSEL_2 + MC_1;
	LPM0;
	TA1CCTL0 &= ~CCIE;
	__disable_interrupt();

    DCOCTL = 0x00;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
    P1OUT = 0x00;
    P1SEL = UART_TXD + UART_RXD;
    P1DIR = (0xFF & ~UART_RXD) | LED1 | LED2;
    __enable_interrupt();

    temp(1000, 1024);
    flash('1', 500, 1500);

    while (1) {
    	if (send) {
    		TimerA_UART_init();
    		TimerA_UART_print("Hot!\r\n");
    		send = 0;
    	}
    	if (rxString[(rxIdx + 3) % 4] == '!' && rxString[(rxIdx + 2) % 4] == 'k' && rxString[(rxIdx + 1) % 4] == 'c' && rxString[rxIdx] == 'A') {
    		__disable_interrupt();
    		rxString[0] = 0;
    		rxString[1] = 0;
    		rxString[2] = 0;
    		rxString[3] = 0;
			TA0CTL = MC_1 | ID_0 | TASSEL_1 | TACLR;
			TA0CCR0 = mem_interval * 12 - 1;
			TA0CCR1 = TA0CCR0 - 1;
			TA0CCTL1 = OUTMOD_3;
			ADC10CTL0 |= ENC;
			flash('1', 500, 1500); // Interrupt enabled.
		}
    }
}

void TimerA_UART_init() {
	TA0CTL = TASSEL_2 + MC_2;
	if (init) TA0CCTL0 = OUT;
	init = 0;
	TA0CCTL1 = SCS + CM1 + CAP + CCIE;
	TA0CCR0 = 0;
	TA0CCR1 = 0;
}

void TimerA_UART_print(char *string) {
	while (*string) TimerA_UART_tx(*string++);
}

void TimerA_UART_tx(unsigned char byte) {
	while (TACCTL0 & CCIE);
	TA0CCR0 = TAR;
	TA0CCR0 += UART_TBIT;
	TA0CCTL0 = OUTMOD0 + CCIE;

	unsigned int myData = byte, myCounter = 0;
	myCounter ^= (myData & 0x01);
	myData >>= 1;
	myCounter ^= (myData & 0x01);
	myData >>= 1;
	myCounter ^= (myData & 0x01);
	myData >>= 1;
	myCounter ^= (myData & 0x01);
	myData >>= 1;
	myCounter ^= (myData & 0x01);
	myData >>= 1;
	myCounter ^= (myData & 0x01);
	myData >>= 1;
	myCounter ^= (myData & 0x01);

	txData = (byte & 0x7F);
	if (myCounter) txData |= 0x80;
	if (byte == '\r') txData = '\r'; // It seems that the terminal doesn't accept \r using 7E1.
	txData |= 0x100;
	txData <<= 1;
}

void flash(char id, int on, int off) {
	mem_on = on;
	mem_off = off;
	mem_id = id - '0';
	TA1CTL = MC_1 | ID_0 | TASSEL_1 | TACLR;
	BCSCTL3 |= LFXT1S_2;
	TA1CCR0 = mem_on * 12 - 1;
	TA1CCTL0 = CCIE;
	if (mem_id == 0) { // red
		P1OUT ^= LED1;
		P1OUT &= ~LED2;
	} else { // green
		P1OUT ^= LED2;
		P1OUT &= ~LED1;
	}
	__bis_SR_register(LPM3_bits + GIE);
}

void temp(int interval, int times) {
	mem_interval = interval;
	mem_times = times;
	my_times = 0;
	TA0CTL = MC_1 | ID_0 | TASSEL_1 | TACLR;
	TA0CCR0 = mem_interval * 12 - 1;
	TA0CCR1 = TA0CCR0 - 1;
	TA0CCTL1 = OUTMOD_3;
	ADC10CTL0 |= ENC;
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
	static unsigned char txBitCnt = 10;
	TA0CCR0 += UART_TBIT;
	if (txBitCnt == 0) {
		TA0CCTL0 &= ~CCIE;
		txBitCnt = 10;
	} else {
		if (txData & 0x01) {
			TA0CCTL0 &= ~OUTMOD2;
		} else {
			TA0CCTL0 |= OUTMOD2;
		}
		txData >>= 1;
		txBitCnt--;
	}
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR() {
	static unsigned char rxBitCnt = 8;
	static unsigned char rxData = 0;
	switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {
		case TA0IV_TACCR1:
			TA0CCR1 += UART_TBIT;
			if (TA0CCTL1 & CAP) {
				TA0CCTL1 &= ~CAP;
				TA0CCR1 += UART_TBIT_DIV_2;
			} else {
				rxData >>= 1;
				if (TA0CCTL1 & SCCI) {
					rxData |= 0x80;
				}
				rxBitCnt--;
				if (rxBitCnt == 0) {
					rxBuffer = rxData & 0x7F;
					rxString[rxIdx++] = rxBuffer;
					rxIdx %= 4;
					rxBitCnt = 8;
					TA0CCTL1 |= CAP;
				}
			}
			break;
	}
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void TA1_ISR() {
	if (setup) {
		setup = 0;
		TA1CTL = 0;
		LPM0_EXIT;
		return;
	}
	if (mem_id == 0) { // red
		P1OUT ^= LED1;
		P1OUT &= ~LED2;
		sendCount--;
		if (sendCount == 0) {
			send = 1;
			sendCount = 4;
		}
	} else { // green
		P1OUT ^= LED2;
		P1OUT &= ~LED1;
	}
	if (now_on) {
		TA1CCR0 = mem_off * 12 - 1;
		now_on = 0;
	} else {
		TA1CCR0 = mem_on * 12 - 1;
		now_on = 1;
	}
}

#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR() {
	mem_1 = idx, mem_2 = (idx - 1 + SIZE) % SIZE;
	int x = ADC10MEM;
	Temp[idx++] = x;
	sum = Temp[mem_1] + Temp[mem_2];
	if (sum > 1474) {
		ADC10CTL0 &= ~ENC;
		mem_on = 200;
		mem_off = 300;
		mem_id = 0;
		_BIC_SR_IRQ(LPM3_bits);
	}
	idx %= SIZE;
	my_times++;
	if (my_times == mem_times) {
		TA0CCTL1 = 0;
		ADC10CTL0 &= ~ENC;
	}
}

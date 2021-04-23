
#include <htc.h>
#include <pic.h>

__CONFIG(FCMEN_OFF & IESO_OFF & CLKOUTEN_OFF & BOREN_OFF & CPD_OFF & CP_OFF & MCLRE_OFF & PWRTE_ON & 
			WDTE_OFF & FOSC_INTOSC);
__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_OFF & BORV_19 & LVP_OFF);

//red (channel A)
#define ENC0CLK PORTAbits.RA4

//white (channel B)
#define ENC0DIR PORTAbits.RA3

#define ENC1CLK PORTCbits.RC3
#define ENC1DIR PORTCbits.RC6
#define ENC2CLK PORTCbits.RC7
#define ENC2DIR PORTBbits.RB7
#define ENC3CLK PORTCbits.RC0
#define ENC3DIR PORTCbits.RC1
#define ENC4CLK PORTCbits.RC2
#define ENC4DIR PORTBbits.RB4
#define ENC5CLK PORTBbits.RB5
#define ENC5DIR PORTBbits.RB6
#define ENC3FWD 'R'
#define ENC3REV 'E'
#define ENC4FWD 'F'
#define ENC4REV 'D'
#define ENC5FWD 'V'
#define ENC5REV 'C'
#define ENC0FWD 'W'
#define ENC0REV 'Q'
#define ENC1FWD 'A'
#define ENC1REV 'S'
#define ENC2FWD 'Z'
#define ENC2REV 'X'
#define SW1REL 'T'
#define SW2REL 'G'
//Bluetooth connection 1:connected 0:none
#define BLUETOOTH_CONNECTED PORTAbits.RA2
#define PUSH_SW2 PORTAbits.RA0
#define PUSH_SW1 PORTAbits.RA1
#define POWER_SW PUSH_SW2
#define MODE_BLUETOOTH 1
#define MODE_RS232C 2

unsigned int iTimer1;
unsigned int iPowerSW_Low;

void interrupt isr(void){
	if(TMR1IF){
		iTimer1++;
		if (POWER_SW) iPowerSW_Low = 0; else iPowerSW_Low++;
		TMR1IF = 0;
	}
	return;
}

void wait(int icycle) {
	for (int i=0; i<icycle; i++) {}
}

#define NDIAL 6
#define NPUSHSW 2
#define NINPUT (NDIAL+NPUSHSW)
#define NTICK 4
#define NTICKMASK (NTICK-1)
unsigned char ucEnc[NINPUT][NTICK];
unsigned char ucEdir[NDIAL][NTICK];
unsigned int uiTick;

void initMCU(void){
	unsigned char uc0, uc1;

	OSCCONbits.SPLLEN = 0;//4xPLL disabled
//	OSCCONbits.SCS = 2;//internal osc
//	OSCCONbits.IRCF = 0xd;//4 MHz
	OSCCONbits.IRCF = 0xe;//8 MHz
	APFCON0bits.RXDTSEL = 1;//RX/DT is on RC5
	APFCON0bits.TXCKSEL = 1;//TX/CK is on RC4

	uiTick = 0;
	for (uc0=0; uc0<NINPUT; uc0++) {
		for (uc1=0; uc1<NTICK; uc1++) {
			ucEnc[uc0][uc1] = 1;
			if (uc0 < NDIAL) ucEdir[uc0][uc1] = 0;
		}
	}

//Timer1: 3.8 Hz interrupt when Fosc = 8 MHz
	iTimer1 = 0;
	iPowerSW_Low = 0;
	T1CONbits.TMR1CS = 0;//Fosc/4 clock
	T1CONbits.T1CKPS0 = 1;//prescale 1/8
	T1CONbits.T1CKPS1 = 1;
	T1CONbits.T1OSCEN = 0;//LP osc off
	T1GCONbits.TMR1GE = 0;//Timer1 gate off
	TMR1H = 0;
	TMR1L = 0;
	T1CONbits.TMR1ON = 1;
	TMR1IE=1;	// timer 1 interrupt enabled
	PEIE=1;		// enable peripheral interrupts
	GIE=1;		// turn on interrupts

//Ports
	ANSELA = 0;//port A to be digital
	ANSELB = 0;//port B to be digital
	ANSELC = 0;//port C to be digital
	INLVLC = 0b11011111;//RC5: TTL level input
	OPTION_REGbits.nWPUEN = 0;//enable indiv pull-ups
	WPUA = 0b00011011;//RA4,3,1,0: pull up; RA2 is hard-wired to pull down
	WPUB = 0b11110000;//RB7-4: pull up
	//210421 WPUC = 0b11011111;//RC7,6,4-0: pull up
	WPUC = 0b11101111;//RC7-5,3-0: pull up, RC4=TX

	TRISA = 0b11011111;//RA5 is output to LTC3531
	TRISB = 0b11111111;//RB7-0:input;
	TRISC = 0b11111111;//RC5/RX for RS232C sense

	PORTAbits.RA5 = 1;//RN42 on

//UART 9600 baud
	SPBRGH = 0;
//9600 baud at 8 MHz
	SPBRG = 12;
	TXSTAbits.BRGH = 0;
	BAUDCONbits.BRG16 = 0;
//invert TX
	BAUDCONbits.SCKP = 1;//210317
//9600 baud at 4 MHz
//	SPBRG = 103;
//	TXSTAbits.BRGH = 1;
//	BAUDCONbits.BRG16 = 1;
	//Asynchronous
	TXSTAbits.SYNC = 0;
	RCSTAbits.SPEN = 1;
	TXSTAbits.TX9 = 0;
	TXSTAbits.TXEN = 1;
	//RC4: UART TX
	//RC5: UART RX
}

void enterSleepMode() {
	OSCCONbits.IRCF = 0x0;//31 kHz
	T1CONbits.TMR1ON = 0;
	TMR1IE=0;	// timer 1 interrupt disabled
	PEIE=0;		// disable peripheral interrupts
	GIE=0;		// turn off all interrupts
	RCSTAbits.SPEN = 0;//turn off UART
	TXSTAbits.TXEN = 0;
//turn off pull ups to reduce sleep current (220 uA ==> 30-50 uA)
	WPUA = 0b00000011;//keep push-SW pull-ups
	WPUB = 0x00;
	WPUC = 0b00100000;//keep unused RC5 pull-up
	TRISA = 0b11111111;
	ANSELA = 0b11111110;//turn off RA4,3,2,1 digital inputs
	ANSELB = 0xff;//port B
	ANSELC = 0b11111111;//port C
	//sleep
	while (POWER_SW == 0) {}//wait for SW released
	IOCAF = 0x00;//clear flags of interrupt on change
	IOCANbits.IOCAN0 = 1;//interrupt on change negative edge
	INTCONbits.IOCIE = 1;//enable interrupt on change
	asm("SLEEP");
	IOCANbits.IOCAN0 = 0;
	INTCONbits.IOCIE = 0;
	IOCAF = 0x00;
}

void initBluetooth() {
	PORTAbits.RA5 = 1;//turn RN42 on
	//SPP mode only
	int i;
	for (int i=0; i<5; i++) {wait(30000);}
	//$$$
	while(!PIR1bits.TXIF);  TXREG = '$';
	while(!PIR1bits.TXIF);  TXREG = '$';
	while(!PIR1bits.TXIF);  TXREG = '$';
	wait(10000);
	//S-,BthDials; device name
	while(!PIR1bits.TXIF);  TXREG = 'S';
	while(!PIR1bits.TXIF);  TXREG = '-';
	while(!PIR1bits.TXIF);  TXREG = ',';
	while(!PIR1bits.TXIF);  TXREG = 'B';
	while(!PIR1bits.TXIF);  TXREG = 'T';
	while(!PIR1bits.TXIF);  TXREG = 'd';
	while(!PIR1bits.TXIF);  TXREG = 'i';
	while(!PIR1bits.TXIF);  TXREG = 'a';
	while(!PIR1bits.TXIF);  TXREG = 'l';
	while(!PIR1bits.TXIF);  TXREG = 's';
	while(!PIR1bits.TXIF);  TXREG = 0x0d;
	wait(10000);
	//SM,0; slave mode
	while(!PIR1bits.TXIF);  TXREG = 'S';
	while(!PIR1bits.TXIF);  TXREG = 'M';
	while(!PIR1bits.TXIF);  TXREG = ',';
	while(!PIR1bits.TXIF);  TXREG = '0';
	while(!PIR1bits.TXIF);  TXREG = 0x0d;
	wait(10000);
	//S~,0; SPP
	while(!PIR1bits.TXIF);  TXREG = 'S';
	while(!PIR1bits.TXIF);  TXREG = '~';
	while(!PIR1bits.TXIF);  TXREG = ',';
	while(!PIR1bits.TXIF);  TXREG = '0';
	while(!PIR1bits.TXIF);  TXREG = 0x0d;
	wait(10000);
	if (!POWER_SW) {//sniffing off
		//SW,0000; no sniff
		while(!PIR1bits.TXIF);  TXREG = 'S';
		while(!PIR1bits.TXIF);  TXREG = 'W';
		while(!PIR1bits.TXIF);  TXREG = ',';
		while(!PIR1bits.TXIF);  TXREG = '0';
		while(!PIR1bits.TXIF);  TXREG = '0';
		while(!PIR1bits.TXIF);  TXREG = '0';
		while(!PIR1bits.TXIF);  TXREG = '0';
		while(!PIR1bits.TXIF);  TXREG = 0x0d;
		wait(10000);
	} else {//sniffing on
		//SW,0320; 500 ms sniff
		while(!PIR1bits.TXIF);  TXREG = 'S';
		while(!PIR1bits.TXIF);  TXREG = 'W';
		while(!PIR1bits.TXIF);  TXREG = ',';
		while(!PIR1bits.TXIF);  TXREG = '0';
		while(!PIR1bits.TXIF);  TXREG = '3';
		while(!PIR1bits.TXIF);  TXREG = '2';
		while(!PIR1bits.TXIF);  TXREG = '0';
		while(!PIR1bits.TXIF);  TXREG = 0x0d;
		wait(10000);
	}
	//R,1
	while(!PIR1bits.TXIF);  TXREG = 'R';
	while(!PIR1bits.TXIF);  TXREG = ',';
	while(!PIR1bits.TXIF);  TXREG = '1';
	while(!PIR1bits.TXIF);  TXREG = 0x0d;
}

void main(void){
	unsigned char uc0, uc1, uc2, uc3;
	unsigned char ucx, ucDirSum;

	initMCU();
	initBluetooth();
	iPowerSW_Low = 0;

	while (1) {
		uiTick++;

		uc0 = (uiTick & NTICKMASK);
		ucEnc[0][uc0] = ENC0CLK;
		ucEnc[1][uc0] = ENC1CLK;
		ucEnc[2][uc0] = ENC2CLK;
		ucEnc[3][uc0] = ENC3CLK;
		ucEnc[4][uc0] = ENC4CLK;
		ucEnc[5][uc0] = ENC5CLK;
		ucEnc[6][uc0] = PUSH_SW1 ? 0 : 1;//detect switch release
		ucEnc[7][uc0] = PUSH_SW2 ? 0 : 1;
		uc1 = (uc0 + NTICK - 1) & NTICKMASK;
		uc2 = (uc0 + NTICK - 2) & NTICKMASK;
		ucEdir[0][uc0] = ENC0DIR;
		ucEdir[1][uc0] = ENC1DIR;
		ucEdir[2][uc0] = ENC2DIR;
		ucEdir[3][uc0] = ENC3DIR;
		ucEdir[4][uc0] = ENC4DIR;
		ucEdir[5][uc0] = ENC5DIR;
		for (uc3=0; uc3<NINPUT; uc3++) {
			ucDirSum = 0;
			for (ucx=2; ucx<NTICK; ucx++) {ucDirSum += ucEnc[uc3][(uc0 + NTICK - ucx) & NTICKMASK];}
			if ((ucDirSum == NTICK-2)&&(ucEnc[uc3][uc1]==0)&&(ucEnc[uc3][uc0]==0)) {
//			if ((ucEnc[uc3][uc2]==1)&&(ucEnc[uc3][uc1]==0)&&(ucEnc[uc3][uc0]==0)) {//detect 'A terminal' = off-on-on
				ucDirSum = 0;
				if (uc3 < NDIAL) {
					for (ucx=0; ucx<NTICK; ucx++) {ucDirSum += ucEdir[uc3][ucx];}
					if (ucDirSum < NTICK/2) ucDirSum = 0;
				}
				switch (uc3) {
					case 0: {
						if (ucDirSum) {while(!PIR1bits.TXIF);  TXREG = ENC0FWD;}
						else {while(!PIR1bits.TXIF);  TXREG = ENC0REV;}
						break;}
					case 1: {
						if (ucDirSum) {while(!PIR1bits.TXIF);  TXREG = ENC1FWD;}
						else {while(!PIR1bits.TXIF);  TXREG = ENC1REV;}
						break;}
					case 2: {
						if (ucDirSum) {while(!PIR1bits.TXIF);  TXREG = ENC2FWD;}
						else {while(!PIR1bits.TXIF);  TXREG = ENC2REV;}
						break;}
					case 3: {
						if (ucDirSum) {while(!PIR1bits.TXIF);  TXREG = ENC3FWD;}
						else {while(!PIR1bits.TXIF);  TXREG = ENC3REV;}
						break;}
					case 4: {
						if (ucDirSum) {while(!PIR1bits.TXIF);  TXREG = ENC4FWD;}
						else {while(!PIR1bits.TXIF);  TXREG = ENC4REV;}
						break;}
					case 5: {
						if (ucDirSum) {while(!PIR1bits.TXIF);  TXREG = ENC5FWD;}
						else {while(!PIR1bits.TXIF);  TXREG = ENC5REV;}
						break;}
					case 6: {
						while(!PIR1bits.TXIF);  TXREG = SW1REL;
						break;}
					case 7: {
						while(!PIR1bits.TXIF);  TXREG = SW2REL;
						break;}
				}
			}
		}
		if (BLUETOOTH_CONNECTED) iTimer1 = 0;
		//210317 if ((iTimer1 > 3000)||(iPowerSW_Low > 6)) {//wait approx 13 min to sleep, or sense continuous SW pressing
		if ((iTimer1 > 300)||(iPowerSW_Low > 6)) {//wait approx 13 min to sleep, or sense continuous SW pressing
			if (BLUETOOTH_CONNECTED) {
				while(!PIR1bits.TXIF);  TXREG = '$';
				while(!PIR1bits.TXIF);  TXREG = '$';
				while(!PIR1bits.TXIF);  TXREG = '$';
				wait(10000);
				while(!PIR1bits.TXIF);  TXREG = 'K';
				while(!PIR1bits.TXIF);  TXREG = ',';
				while(!PIR1bits.TXIF);  TXREG = 0x0d;
				wait(30000);
			}
			enterSleepMode();
			initMCU();
		}
	}//while(1)
}//main()

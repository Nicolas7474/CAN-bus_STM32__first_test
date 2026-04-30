/***************************************************************************************************************
* STM32F469
* USB OTG FS device (CDC) implementation  - Nicolas Prata - 2026
* Lightweight, only two files to integrate to your project: usb_cdc_fs.c and usb_cdc_fs.h
* Usage:
    - USB_CDC_UserRxCallBack_EP1() function triggers when the Rx buffer has been updated with new received data; in argument is the nb of bytes received
    - Use "RxData = read_circBufferRx(yourLen)", it returns RxData.index and RxData.len (declare RxData with 'circBufferAddress' type)
      Make sure the requested length returned is correct (RxData.len == yourLen) or if you read it all Rx buffer has been duly emptied (RxData.len == 0),
      to avoid forgotten wrapped-around data.
      You can store the received bytes to another buffer, eg with memcpy(destBuff, &circBufferRx[RxData.index], RxData.len);
    - Use USB_CDC_UserSend_Data(yourLen) function to send your data to your VCOM port host.
    - You can edit the descriptors on usb_cdc_fs.h file, to use different values than provided with STM HAL example
    - Slave-mode only (DMA not available on USB Full Speed with PA11 - PA12)
*
*  Tested at a solid 880KB/s (Rx) and 0.88MB/s (Tx, with Powershell script) - TinyUSB measured at 350KB/s (Rx)
***********************************************************************************************************************/
#include "stm32f4xx.h"
#include "stm32f469xx.h"
#include "usb_cdc_fs.h"
#include "timers.h" // for msTick
/****************************************************************
 * 		RX buffers for Endpoint structure
*****************************************************************/

#define RX_BUFFER_EP0_SIZE 64U // 8 is normally enough but 64 costs almost nothing in RAM and can prevent the most common USB crashes
#define RX_BUFFER_EP1_SIZE 64 // normally 64 is enough for FS

static uint8_t rxBufferEp0[RX_BUFFER_EP0_SIZE]; /* Received data is stored here after application reads DFIFO. RX FIFO is shared */
static uint8_t rxBufferEp1[RX_BUFFER_EP1_SIZE]; /* Received data is stored here after application reads DFIFO. RX FIFO is shared */

/**********************************************************************
 * 	The application will set linecoding according to the host request
 ***********************************************************************/
/* Since you are overwriting lineCoding[7], the static uint8_t lineCoding initialization can be simplified to just zeros,
   or the standard "115200, 8-N-1" default : 115200 (0x0001C200), 1 Stop bit (0), No Parity (0), 8 Data bits (8)
*/
static uint8_t lineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

/****************************************************************************/

volatile static uint32_t device_state = DEVICE_STATE_DEFAULT; /* Device state */
volatile uint8_t port_is_open; // this global is a "gate" to start transmit to the COM port

EndPointStruct EndPoint[EP_COUNT];	/* All the Enpoints are included in this array */

volatile static USB_setup_req_data setup_pkt_data; /* Setup Packet var */


/****************************************************************
 * 		static functions' declarations
*****************************************************************/
static void USB_CDC_ForceResetState(void);
static void USB_CDC_StopComPort(void);
static void USB_CDC_Config_Hardware(void);

/* Device state */
static inline void set_device_status(eDeviceState state);

/* Init EP */
static void initEndPoints(void);

/* FIFO handler */
static inline void set_FIFOs_sz(void);

/* EP routine */
static inline uint32_t is_tx_ep_fifo_ready(uint8_t EPnum, uint32_t param);
static inline void toggle_Rx_EP_Status(uint8_t EPnum, uint8_t param);

//static void SysTick_init(void);
static void Get_ID_To_String(uint8_t *dest); // Get STM32 serial number (UID)

/**********************************************************************
 * GetSysTick() is required for Timeout detection in several functions
 * you can add msTicks++ in your SysTick_Handler if you use it already
 * or you can provide your own Timeout function (eg with a timer)
 **********************************************************************/

//uint32_t SystemCoreClock = 180000000;
//volatile uint32_t msTicks = 0; // Volatile ensures the compiler doesn't optimize out reads of this value

//static void SysTick_init(void) {
//	// SysTick generates an interrupt every 1ms. ticks = SystemCoreClock / 1000
//	/* SysTick_Config() returns a value (typically 0 for success and 1 for failure)
//     1 = Force the whole chip to reboot and try again (Capture error, should not happen with valid clock) */
//	if (SysTick_Config(SystemCoreClock / 1000)) { NVIC_SystemReset(); }
//}

//void SysTick_Handler(void) { msTicks++; } // The SysTick handler is predefined in the vector table

uint32_t GetSysTick(void) { return msTicks; } // GetTick: this is a direct replacement for HAL_GetTick()

/****************************************************
* 		Circular buffer*
***************************************************/

__attribute__((aligned(4))) static uint8_t circBufferTx[CIRC_BUFFER_TX_SIZE];
static uint16_t writePtrTxCbuf = 0U;
static uint16_t readPtrTxCbuf = 0U;
uint8_t circBufferRx[CIRC_BUFFER_RX_SIZE];
static volatile uint16_t writePtrRxCbuf = 0U;  // make is static again if ISR functions included in this page
static volatile uint16_t readPtrRxCbuf = 0U; // make is static again if ISR functions included in this page


static inline uint32_t is_circBufferTx_empty(){
	return (readPtrTxCbuf == writePtrTxCbuf);
}

static inline uint32_t is_circBufferRx_empty(){ // make is static again if ISR functions included in this page
	return (readPtrRxCbuf == writePtrRxCbuf);
}

static inline uint32_t get_circBufferTx_freeSize(){
	/* Snapshot pointers to avoid interrupt race conditions - the pointers aren't modified, so no need to __disable_irq() here */
	uint16_t w = writePtrTxCbuf; // copying the values of volatile global variables into local variables at the beginning of a function.
	uint16_t r = readPtrTxCbuf; // creates a "frozen" version of the state that won't change while your math is running, even if an interrupt fires in the middle of your calculation.

	// We return SIZE - 1 to ensure we always leave one gap (the "N-1" Rule for full buffer)
	if (w == r) return CIRC_BUFFER_TX_SIZE - 1; // Empty
	if (w > r) {
		// Data is contiguous: free space is the two ends
		return CIRC_BUFFER_TX_SIZE - (w - r) - 1 ;
	} else {
		// Data is wrapped: free space is the gap in the middle
		return (r - w) - 1;
	}
}

static inline uint32_t get_circBufferRx_freeSize(){
	/* Snapshot pointers to avoid interrupt race conditions - the pointers aren't modified, so no need to __disable_irq() here */
	uint16_t w = writePtrRxCbuf; // copying the values of volatile global variables into local variables at the beginning of a function  creates a "frozen"
	uint16_t r = readPtrRxCbuf; // version of the state that won't change while your math is running, even if an interrupt fires in the middle of your calculation.

	// We return SIZE - 1 to ensure we always leave one gap (the "N-1" Rule for full buffer)
	if (w == r) return CIRC_BUFFER_RX_SIZE - 1; // Empty
	if (w > r) {
		// Data is contiguous: free space is the two ends
		return CIRC_BUFFER_RX_SIZE - (w - r) - 1;
	} else {
		// Data is wrapped: free space is the gap in the middle
		return (r - w) - 1;
	}
}


void write_to_circBufferTx(uint8_t *buf, uint16_t len){

	if (len == 0) return;

	// Check if we need to wrap around
	if((writePtrTxCbuf + len) >= CIRC_BUFFER_TX_SIZE){
		uint32_t write_tail = (uint32_t)((CIRC_BUFFER_TX_SIZE) - writePtrTxCbuf); // Calculate how much fits at the end of the array
		memcpy(&circBufferTx[writePtrTxCbuf], buf, write_tail); // Copy the tail part

		uint32_t remaining = (uint32_t)(len - write_tail); // Copy the remaining part to the beginning of the array
		if (remaining > 0) {
			memcpy(&circBufferTx[0], (buf + write_tail), remaining);
		}
		writePtrTxCbuf = (uint16_t)remaining; // Update pointer with modulo or manual wrap
	}
	else{
		memcpy(&circBufferTx[writePtrTxCbuf], buf, len); // Simple contiguous copy
		writePtrTxCbuf = (uint16_t)(writePtrTxCbuf + len); // Update pointer
	}
	// Safety check: if the pointer exactly hits the end of the array, wrap it to 0
	if (writePtrTxCbuf == CIRC_BUFFER_TX_SIZE) writePtrTxCbuf = 0;
}

void write_to_circBufferRx(uint8_t *buf, uint16_t len){

	/* The simplest way to make the code robust is to refuse to write if there is no room.
	   This prevents memory corruption and pointer de-sync, though you still lose data if your buffer is too small. */
	uint32_t freeSpace = get_circBufferRx_freeSize();

	// If we have 100 bytes free, we can only safely write 99 to keep the N-1 gap. However, get_circBufferRx_freeSize() should already return (ActualFree - 1)
	if (len > freeSpace) {
		len = (uint16_t)freeSpace;
	}
	if (len == 0) return; // Buffer is truly full, drop the packet

	// Wrap around calculations
	if((writePtrRxCbuf + len) >= CIRC_BUFFER_RX_SIZE){
		// Copy the first part to the end
		uint32_t write_tail = (uint32_t)((CIRC_BUFFER_RX_SIZE) - writePtrRxCbuf);
		memcpy(&circBufferRx[writePtrRxCbuf], buf, write_tail);
		// Copy the remainder to the beginning of the array
		uint16_t remaining = (uint16_t)(len - write_tail);
		if (remaining > 0) {
			memcpy(&circBufferRx[0], (buf + write_tail), remaining);
		}
		// New pointer is exactly the remainder
		writePtrRxCbuf = remaining;
	}
	else {
		memcpy(&circBufferRx[writePtrRxCbuf], buf, len); // Simple contiguous copy
		writePtrRxCbuf = (uint16_t)(writePtrRxCbuf + len); // update pointer
	}
	// FInal boundary check. If the pointer landed exactly on the Size, wrap it to 0.
	if (writePtrRxCbuf >= CIRC_BUFFER_RX_SIZE) {
		writePtrRxCbuf = 0;
	}
}


/**
* brief  Return  current index and length of datas from the circular buffer
* param  Maximum length to get. For Example, if len=256, the function returns 256 length string index, if used space > 256.
* param  If used space < x, the function returns used space.
* retval First character's index and length
*/
static circBufferAddress read_circBufferTx(uint16_t len) {

	circBufferAddress result = {0, 0};

	uint16_t w = writePtrTxCbuf; // Snapshot
	uint16_t r = readPtrTxCbuf;

	// 1. Check if truly empty
	if (w == r) return result;

	// 2. Calculate EXACT data count (Direct Math)
	uint16_t writtenSize;
	if (w > r) 	writtenSize = w - r;
	else writtenSize = (CIRC_BUFFER_TX_SIZE - r) + w;

	if (len > writtenSize) len = writtenSize;  // Constrain requested len to available data

	result.index = r;

	// 4. Wrap-around logic (note: Use '>' not '>=' because index starts at 0)
	// If r + len == SIZE, the last byte is at SIZE-1, which is perfectly valid.
	if ((r + len) > CIRC_BUFFER_TX_SIZE) {
		// Tell the USB driver to only send the bytes that actually exist at the end of the buffer.
		result.len = (uint16_t)(CIRC_BUFFER_TX_SIZE - r);
		readPtrTxCbuf = 0;  // Atomic update
	} else {
		result.len = len; // The data fits perfectly within the remaining space of the array, it returns the full requested len
		uint16_t nextReadPtr = r + len;
		if (nextReadPtr == CIRC_BUFFER_TX_SIZE) nextReadPtr = 0;
		readPtrTxCbuf = nextReadPtr; // Atomic update
	}

	return result;
}

circBufferAddress read_circBufferRx(uint16_t requested_Len) {

    circBufferAddress result = {0, 0};

    uint16_t w = writePtrRxCbuf;   //Snapshot pointers
    uint16_t r = readPtrRxCbuf;

    /* Before we fixed the math, your code was doing this: writtenSize = TotalSize - freeSize
	When the buffer was empty:  TotalSize = 1024 ->  freeSize = 1023 (because of the N-1 rule) -> writtenSize = 1
	The calculated that there was 1 byte of data sitting in the buffer, even though the pointers were both at zero.
	It then tried to "read" that non-existent byte, which is where those #NUL characters came from. */

    // Direct Data Count (N-1 compliant)
    uint16_t actualData;
    if (w == r) return result; // Truly empty, returns len 0
    else if (w > r) actualData = w - r;
    else actualData = (CIRC_BUFFER_RX_SIZE - r) + w;

    // Clamp length to available data
    uint16_t len = (requested_Len > actualData) ? actualData : requested_Len;
    result.index = r;

    // Wrap-around Logic :  Use '>' because index is 0 to SIZE-1
    if ((r + len) > CIRC_BUFFER_RX_SIZE) {
        result.len = (uint16_t)(CIRC_BUFFER_RX_SIZE - r);
        readPtrRxCbuf = 0;
    } else {
        result.len = len;
        uint16_t nextReadPtr = r + len;
        if (nextReadPtr >= CIRC_BUFFER_RX_SIZE) {
            nextReadPtr = 0;
        }
        readPtrRxCbuf = nextReadPtr;
    }

    return result;
}


/***************************************************
*
* 	Initialization functions
*
***************************************************/

/**
* brief  USB OTG HARDWARE bare-metal configuration RCC CLOCKS GPIO
* Note  Model-dependant - here for STM32F469
* param
* retval
* A11 and A12
*/
uint32_t USB_OTG_FS_Init(void) {

	NBdelay_ms(60); // mandatory to allow proper reboot after a firmware program - otherwise not essential
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12); //  Configure PA11 (DM) and PA12 (DP) as Alternate Function (AF10)
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);   // Mode: 10 (Alternate Function)
	GPIOA->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL11_Pos);
	GPIOA->AFR[1] |=  (10U << GPIO_AFRH_AFSEL11_Pos);
	GPIOA->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->AFR[1] |=  (10U << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->PUPDR &= ~(1<<22 | 1<<24); // No pull-up, no pull-down
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12); 	// Speed: 11 (Very High Speed)
	/* PA9 (VBUS) Configuration: the VBUS sensing on PA9 bypasses the standard AF multiplexer
	 and connects directly to the sensing block when enabled in the USB core (not needed in USB Device mode) */
	GPIOA->MODER &= ~(GPIO_MODER_MODER9); 	// Set to Input mode (00) - This is the "Default State"
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9); 	//Ensure No Pull-up/Pull-down to allow the 200 uA sensing circuit to work

	RCC->CR |= RCC_CR_HSEON; // Enable HSE (8MHz External Crystal)
	while(!(RCC->CR & RCC_CR_HSERDY));

	// Disable the PLL to modify the registers
	RCC->CR |= RCC_CR_HSION; 	// Enable HSI (Internal High Speed oscillator)
	while (!(RCC->CR & RCC_CR_HSIRDY));     //  Wait until HSI is ready
	RCC->CFGR &= ~RCC_CFGR_SW;   // Switch System Clock (SYSCLK) to HSI
	RCC->CFGR |= RCC_CFGR_SW_HSI;   // Clear SW bits and set them to 00 (HSI select)
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // 4. Wait until HSI is actually being used as the system clock,  SWS bits (System Clock Switch Status) should report 00
	RCC->CR &= ~RCC_CR_PLLON; 	// The PLL is now "free" and can be safely disabled, Disable the main PLL
	while (RCC->CR & RCC_CR_PLLRDY);     // Wait until PLL is fully stopped
	WRITE_REG(RCC->PLLCFGR, (0)); // !! NEEDED !! Why !?

	// Configure Main PLL (M=4, N=180, P=2), f_VCO = 8MHz*(180/4) = 360MHz,  f_SYS = 360MHz/2 = 180MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE 		//  main PLL clock source = HSE
			     |  RCC_PLLCFGR_PLLM_2
				 |  180 << RCC_PLLCFGR_PLLN_Pos
				 |  (6 << RCC_PLLCFGR_PLLQ_Pos)
				 |  (RCC_PLLCFGR_PLLR_1);
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_Msk);

	// Re-enable the PLL
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)); //  Wait for the PLL to lock

	//The Over-drive Sequence is strict. If you skip a step, the MCU will likely hang during the clock switch.
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable Power Control clock,
	PWR->CR |= PWR_CR_VOS; //Set Regulator Voltage Scaling to Scale 1
	PWR->CR |= PWR_CR_ODEN; // Enable Over-Drive Mode
	while(!(PWR->CSR & PWR_CSR_ODRDY));
	PWR->CR |= PWR_CR_ODSWEN; // Switch to Over-Drive
	while(!(PWR->CSR & PWR_CSR_ODSWRDY));

	// Configure Flash Latency and ART Accelerator, it's critical to do this before switching the clock to 180MHz
	FLASH->ACR = FLASH_ACR_ICEN           // Instruction Cache Enable
			   | FLASH_ACR_DCEN          // Data Cache Enable
			   | FLASH_ACR_PRFTEN       // Prefetch Enable
			   | FLASH_ACR_LATENCY_5WS;    // Implement the 5-cycle latency & the power scaling required for 180MHz

	while (!(RCC->CR & RCC_CR_PLLRDY)); //  Wait for the PLL to lock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;   // Switch System Clock to PLL, Flash Latency is set correctly for new speed
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	//Configure Prescalers (AHB=1, APB1=4, APB2=2), work without these instructions though
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1
			  |  RCC_CFGR_PPRE1_DIV4
			  |  RCC_CFGR_PPRE2_DIV2;

	// Configure PLLSAI for USB 48MHz
	RCC->PLLSAICFGR = (144 << RCC_PLLSAICFGR_PLLSAIN_Pos) | (RCC_PLLSAICFGR_PLLSAIP_1); // f_VCO_SAI = (f_HSE/M)*N = (8/4)*144 = 288MHz
	RCC->CR |= RCC_CR_PLLSAION; 	// PLLSAI enable
	while(!(RCC->CR & RCC_CR_PLLSAIRDY));
	RCC->DCKCFGR |= RCC_DCKCFGR_CK48MSEL; // select PLLSAI as the 48MHz source for USB

	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // USB OTG FS clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN; // OTGHSEN: USB OTG HS clock enable

	USB_OTG_FS_init_registers();

	return EP_OK;
}

/**
* brief  Init general settings of USB_OTG periph
* param
* param
* retval
*/
void USB_OTG_FS_init_registers(){

	device_state = DEVICE_STATE_DEFAULT;

	/* OTG general core configuration register */
	while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)); // Wait for AHB master IDLE state before resetting (not mandatory)
	USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;  // Apply Core Soft Reset (not mandatory)
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST);
	USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_VBDEN; // activate the FS PHY in transmission/reception & USB VBUS detection enable
	USB_OTG_FS->GAHBCFG = USB_OTG_GAHBCFG_GINT; // Enable Global Interrupt (GINTMSK: Global interrupt mask)

	/* OTG device control register */
	USB_OTG_DEVICE->DCTL = USB_OTG_DCTL_SDIS; // signal the USB OTG core to perform a soft disconnect (device disconnect event)
	/*OTG power and clock gating control register*/
	USB_OTG_PCGCCTL->PCGCCTL = 0;

	/* OTG USB configuration register */
	USB_OTG_FS->GUSBCFG =  USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL; // forces the core to device mode & Full Speed serial transceiver mode select
	//for(volatile int i = 0; i < 200000; i++); // Wait for PHY to stabilize
	USB_OTG_FS->GUSBCFG &= ~(uint32_t)(0x0FUL << 10UL) ;
	USB_OTG_FS->GUSBCFG |= (0x6 << 10); // USB (PHY clocks) turnaround time (according to AHB and Reference Manual)

	set_FIFOs_sz();

	// Initialize the structure of all EP (EP1, EP2 are hardware-enabled later in Activate_Composite_Endpoints()) and
	initEndPoints(); 					   // the Hardware for EP0: the STM32 can thus receive the first SETUP packet

	// Set TXFE level to 'Completely Empty' in the Global AHB Config Register (check space before loading bytes into Fifo)
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_TXFELVL; // (1U << 7)

	/* Enable Global Interrupt for Reset, IN, OUT, RX not empty */
	USB_OTG_FS->GINTMSK = USB_OTG_GINTMSK_USBRST // unmask USBRST: USB reset mask
	//		| USB_OTG_GINTMSK_USBSUSPM  // to activate for enabling Suspend interrupt
	//		| USB_OTG_GINTMSK_WUIM //   // to activate for enabling Wakeup interrupt
			| USB_OTG_GINTMSK_OEPINT    // unmask OUT endpoints interrupt
			| USB_OTG_GINTMSK_IEPINT    // unmask IN endpoints
			| USB_OTG_GINTMSK_RXFLVLM   // unmask Receive FIFO non-empty mask
			| USB_OTG_GINTMSK_ENUMDNEM; // Enumeration done mask // ?

	NVIC_SetPriority(OTG_FS_IRQn, 0); // Highest priority not to USB to freeze/crash
	NVIC_EnableIRQ(OTG_FS_IRQn);

	//SysTick_init(); // may be optional if you provide your own SysTick_Handler() / GetSysTick() functions
}

/**
* brief  fill endpoint structures with initial data
* param
* param
* retval
*/
static void initEndPoints(){
	for (uint32_t i = 0; i < EP_COUNT; i++) {
		/* Global defaults for all Endpoints */
		EndPoint[i].statusRx  = EP_READY;
		EndPoint[i].statusTx  = EP_READY;
		EndPoint[i].rxCounter = 0;
		EndPoint[i].txCounter = 0; // Decrements the remaining bytes to track the transfer progress
		EndPoint[i].setTxBuffer  = &USB_CDC_setTxBuffer;
		EndPoint[i].txCallBack   = &USB_CDC_transferTXCallback;
	}

	/* --- EP0: Control (Shared) --- */
	EndPoint[0].rxBuffer_ptr = rxBufferEp0;
	EndPoint[0].rxCallBack   = &USB_CDC_transferRXCallback_EP0; // Handles Setup packets

	/* --- EP1: CDC Data (Bulk IN/OUT) --- */
	EndPoint[1].rxBuffer_ptr = rxBufferEp1;
	EndPoint[1].rxCallBack   = &USB_CDC_transferRXCallback_EP1;

	/* Hardware: Setup EP0 to receive the first SETUP packet - 1 Packet, 3*8 bytes */
	USB_EP_OUT(0)->DOEPTSIZ = (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << 19)) // This field is decremented to zero after a packet is written into the RxFIFO
							| USB_CDC_MAX_PACKET_SIZE    	 // Set in descriptor
							| USB_OTG_DOEPTSIZ_STUPCNT;  	 // STUPCNT==0x11 means, EP can receive 3 packets. RM says to set STUPCNT = 3
	USB_EP_OUT(0)->DOEPCTL  |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);   	 // Clear NAK and enable EP0

	USB_OTG_DEVICE->DCFG |= USB_OTG_DCFG_DSPD_Msk;  // DSPD: Device speedDevice speed - FS
	USB_OTG_FS->GINTSTS = 0xFFFFFFFF; 			 	// Reset Global Interrupt status (core interrupt register OTG_GINTSTS)
	USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;     // Soft connect
}

/**
* brief  Set RX and TX FIFO size and offset for each EP
* param
* param
* retval
*/
static inline void set_FIFOs_sz(){
	USB_OTG_FS->GRXFSIZ = RX_FIFO_SIZE; // all EPs RX FIFO RAM size (GRXFSIZ) OTG receive FIFO size register)

	/* OTG Host Non-Periodic Transmit FIFO Size register - EP0 TX FIFO RAM size (Start: 128, Size: 64) */
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((TX_EP0_FIFO_SIZE) << 16) | RX_FIFO_SIZE; // EP0 TX FIFO RAM size

	/* OTG device IN endpoint transmit FIFO x size register - EP1 TX FIFO (CDC Data) (Start: 192, Size: 64)*/
	USB_OTG_FS->DIEPTXF[0] = ((TX_EP1_FIFO_SIZE) << 16) | (RX_FIFO_SIZE + TX_EP0_FIFO_SIZE); // EP1 TX FIFO RAM size

	/* Clear remaining FIFOs (Start loop at index 1 to protect EP1) */
	for(uint32_t i = 1; i < 0x10 ; i++){
		USB_OTG_FS->DIEPTXF[i] = 0;
	}
}


/***************************************************
*
* 	Miscellaneous service functions
*
***************************************************/

/**
* brief  Change EP OUT status
* param  EP number
* param  READY/BUSY
*/
static inline void toggle_Rx_EP_Status(uint8_t EPnum, uint8_t param){
	if(EndPoint[EPnum].statusRx == param) return;
	EndPoint[EPnum].statusRx = param; /* toggle status*/

		if(param==EP_READY){
			USB_EP_OUT(EPnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
		else{
			USB_EP_OUT(EPnum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
		}
}

static void Get_ID_To_String(uint8_t *dest) {
	// Pointer to the UID start address (Adjust for your specific STM32)
	uint32_t *id_ptr = (uint32_t *)0x1FFF7A10; // The 96-bit Unique ID (UID)
	uint32_t id[3];

	id[0] = id_ptr[0];
	id[1] = id_ptr[1];
	id[2] = id_ptr[2];
	dest[0] = 50; // Total length = 2 bytes header + (12 bytes * 2 chars/byte * 2 bytes/char) = 50
	dest[1] = 0x03; // // Descriptor Type: 3 always means "String"

	const char hex_table[] = "0123456789ABCDEF";
	uint8_t *pStr = &dest[2]; // Start writing at the 3rd byte (after the header)

	// Convert 12 bytes of raw ID to 24 Unicode characters
	for (int i = 0; i < 12; i++) {
		uint8_t byte = ((uint8_t *)id)[i]; // Take one byte, e.g., 0x48

		/* Hex chars only need 4 bits each so one byte holds exactly two hex chars. To send this as a string, we can't send the byte 0x4A directly.
         We have to send the character '4' and then the character 'A'. The >> 4 is the "separator" that lets us grab that first twin.
		 */
		// High nibble
		*pStr++ = hex_table[byte >> 4]; // // Look up '4' in our hex_table
		*pStr++ = 0x00; // Add the Unicode "0" (Unicode UTF-16LE padding)

		// Low nibble
		*pStr++ = hex_table[byte & 0x0F]; // Look up '8' in our hex_table - get the right-side twin (the A) using a Mask
		*pStr++ = 0x00; // // Add the Unicode "0"
	}
}

static void USB_CDC_ForceResetState(void) {
	/*	A "Force Reset" means manually clearing the hardware's status registers, flushing the FIFO buffers,
		and re-enabling the "listening" state for the next incoming packet. */

    // 1. Atomic lock, protect against the USB Interrupt firing while we are resetting its world.
    __disable_irq();

    // 2. Software state reset
    readPtrRxCbuf = 0;
    writePtrRxCbuf = 0;

    // 3. Harware FIFO flush : Flush TX FIFO 0 (or whichever FIFO your IN endpoint uses)
    USB_OTG_FS->GRSTCTL = (1 << 5) | (16 << 6); // 0x20: TxFIFO Flush + all Tx FIFO
    while (USB_OTG_FS->GRSTCTL & (1 << 5));    // Wait for hardware to finish flushing

    // Flush ALL RX FIFOs
    USB_OTG_FS->GRSTCTL = (1 << 4);            // 0x10: RxFIFO Flush
    while (USB_OTG_FS->GRSTCTL & (1 << 4));    // Wait for hardware to finish flushing

    // 4. Clear stuck interrupts: Clear any pending transfer complete or error flags for EP1 (your CDC data EP)
    // Writing 1 to these bits usually clears them in CMSIS/Bare-metal
    USB_EP_OUT(1)->DOEPINT = 0xFF;
    USB_EP_IN(1)->DIEPINT  = 0xFF;

    // 5. RE-PRIME THE RECEIVE ENDPOINT (The "Unstick" Step)
    // If a big packet caused a NAK, we manually tell the hardware to start listening again for a fresh packet.

    // Set expected transfer size to 1 Max Packet Size (e.g., 128 bytes)
    USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | (MAX_CDC_EP1_TX_SIZ << 0); // Bit 19-30: Packet Count (set to 1), Bit 0-18: Transfer Size

    // Enable the endpoint and Clear NAK (CNAK)
	USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA); //Bit 26: CNAK (Clear NAK), Bit 31: EPENA (Enable)

    // 6. Synchronize our software flag with the fresh hardware state and atomic unlock
    EndPoint[1].statusTx = EP_READY;

    __enable_irq();
}

void USB_CDC_StopComPort(void) {
	// Atomic lock, protect against the USB Interrupt firing while we are stopping
	__disable_irq();
	port_is_open = 0; // USB_CDC_UserSend_Data() will return immediately without executing
	// reset the state (DEVICE_STATE_LINECODED is added when the port is opened)
	device_state = DEVICE_STATE_ADDRESSED | DEVICE_STATE_CONFIGURED;
    // Kill the physical EP - tell the OTG Core to stop trying to use FIFO 1.
    if (USB_EP_IN(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        // To disable an active IN EP: Set SNAK and then EPDIS
        USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS;
        // Wait for hardware to flush its internal pipeline
        uint32_t timeout = 10000;
        while ((USB_EP_IN(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && --timeout);
    }
    // If the COM port is closed and reopened, the transmission cannot resume on EP(1) because DIEPTSIZ(PKTCNT) remains at 1
    // It cannot pass the safeguard "is_tx_ep_fifo_ready()" included in USB_CDC_transmit_scheduler()
    USB_EP_IN(1)->DIEPTSIZ = (0 << 19); // Therefore we reset PKTCNT
    __enable_irq();
}

static void USB_CDC_Config_Hardware(void) {
    // Re-allocate FIFO memory (This is lost during Bus Reset!)
    USB_OTG_FS->GRXFSIZ = 0x80; // 128 words for RX
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (0x40 << 16) | 0x80; // EP0 TX
    USB_OTG_FS->DIEPTXF[1] = (0x40 << 16) | 0xC0;          // EP1 TX (64 words)

    // Force Endpoint 1 Data Toggle to DATA0
    USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;

    // Ensure NAK is cleared so the hardware can talk
    USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;
}

 /**
 * brief  Flush TxFifo
 * param  Fifo number, 10 = all Tx Fifos,
 * param  timeout (default FLUSH_FIFO_TIMEOUT)
 * retval 1 = OK, 0 = Failed
 */
uint32_t USB_FlushTxFifo(uint32_t EPnum, uint32_t timeout){
	uint32_t count = 0;
	USB_OTG_FS->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (EPnum << 6));
	do{
		if (++count > timeout){
			return EP_FAILED;
		}
	}
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	 return EP_OK;
}

 /**
 * brief  Flush RxFifo
 * param  timeout (default FLUSH_FIFO_TIMEOUT)
 * retval 1 = OK, 0 = Failed
 */
uint32_t USB_FlushRxFifo(uint32_t timeout){
	uint32_t count = 0;
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	do{
		if (++count > timeout){
			return EP_FAILED;
		}
	}
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

	return EP_OK;
}


/**
* brief  Write data into DFIFO
* param  EP number, TX Buffer, length
* retval OK or FAILED (in case DFIFO overrun, for better details watch my youtube)
*/
uint32_t write_Fifo(uint8_t dfifo, uint8_t *src, uint16_t len){

	/*	The USB OTG FIFO is 32-bit wide. You cannot write a single byte to it; you must write full 32-bit words.
		This logic calculates how many 4-byte "blocks" are needed. If you have 5 bytes to send, block_cnt becomes 2.
		The hardware uses the DIEPTSIZ register (which you already set) to know exactly where the data ends;
		it will simply ignore the extra "padding" bytes in the last word you wrote to the FIFO.
	*/
	uint32_t word_count = (len + 3) >> 2; // //  Calculate blocks (Fastest way: (len + 3) / 4) (= number of 32-bit pushes needed)

	/*	The nested loop with an if condition inside is the safe but slow approach. At 180MHz,
		 if you're pushing a lot of data, that overhead adds up quickly. We can significantly
		 optimize this by using the "Hybrid Path" strategy. We check the alignment once at the start.
	*/
	// Check if the source pointer is 4-byte aligned
	if (((uint32_t)src & 0x03) == 0) {
		/* --- Aligned Path: use a direct 32-bit pointer cast (blazing fast) --- */
		uint32_t *src_32 = (uint32_t *)src;
		for (uint32_t i = 0; i < word_count; i++) {
			USB_OTG_DFIFO(dfifo) = *src_32++;
		}
	}
	else {
		/* --- Unaligned Path: Use a more efficient word-packing method that avoids the nested j loop --- */
		// We still perform 32-bit writes to the FIFO, but we use a temporary buffer to handle the unaligned source.
		for (uint32_t i = 0; i < word_count; i++) {
			// If it's the very last word, mask out the extra bytes
			// (though for USB TX, the hardware usually ignores extra bytes based on the BCNT value in DIEPTSIZ).
			uint32_t word;
			memcpy(&word, &src[i << 2], 4); // memcpy: most ARM compilers optimize this to unaligned LDR instructions.
			/*
				    If the src buffer is not 4-byte aligned in system RAM,this will cause a UsageFault on the Cortex-M4. In bare-metal STM32,
					It is often safer to use memcpy to a local temporary variable or use __attribute__((aligned(4))) on your buffers.
					You are now moving 4 bytes per clock cycle instead of 1 and the loop runs 4× fewer iterations.
					The USB OTG core internally prefers word-aligned writes to its FIFO RAM. */

			// USB_OTG_DFIFO(dfifo) is a macro pointing to a specific memory-mapped address (usually starting at offset 0x1000 from the USB base).
			USB_OTG_DFIFO(dfifo) = word;
		}
	}

	// UPDATE POINTERS HERE so the ISR knows where to start next
	EndPoint[dfifo].txBuffer_ptr += len;
	EndPoint[dfifo].txCounter -= len;
	return EP_OK;
}


void read_Fifo(uint8_t dfifo, uint16_t len) {

 	/* The STM USB peripheral is "word-aligned" (32-bit). This means you cannot just read 1 byte at a time from the hardware;
		you must read in 4-byte chunks. This check if the total byte count is a perfect multiple of 4.
		If there is a remainder (e.g., you received 5 bytes), you still need to perform an extra 32-bit read to get those trailing bytes*/

	uint32_t word_count = (len + 3) >> 2;  // represents the number of 32-bit read operations required to empty the FIFO for that specific packet.
	uint8_t *dest = EndPoint[dfifo].rxBuffer_ptr;

	// --- Safety Check (Wrap-around logic) - if unprocessed data length exceeds Max buffer length, it has to be rewritten
	if ((dfifo == 1) && ((EndPoint[dfifo].rxCounter + len) > RX_BUFFER_EP1_SIZE)) {
		dest = rxBufferEp1;
		EndPoint[dfifo].rxCounter = 0;
	}

	for (uint32_t i = 0; i < word_count; i++) {
		uint32_t temp_word = USB_OTG_DFIFO(0); // one global, single shared Rx FIFO

		// If this is NOT the last word, copy all 4 bytes
		if (i < (word_count - 1)) {
			memcpy(dest, &temp_word, 4);
			dest += 4;
		}
		else {
			// This is the LAST word. Only copy the remaining bytes (1 to 4)
			// This prevents overwriting memory outside your buffer!
			uint8_t bytes_left = len - (i * 4);
			memcpy(dest, &temp_word, bytes_left);
			dest += bytes_left;
		}
	}

	/* After you empty the hardware FIFO, the endpoint is "paused." To receive the next packet from the PC, you must:
	    CNAK (Clear NAK): Tell the hardware it’s okay to accept more data (stop saying "Busy" to the host).
	    EPENA (Endpoint Enable): Re-arm the endpoint for the next transfer.*/
	if (dfifo != 0) {
		// DOEPTSIZ = OUT endpoint x transfer size register. The application must modify this register before enabling the endpoint.
		USB_EP_OUT(dfifo)->DOEPTSIZ = (1 << 19) | 64; // PKTCNT=1, XFRSIZ=64
		USB_EP_OUT(dfifo)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);  // --- Re-arm Endpoint ---
		/*
		  Even though the loop copied data in 4-byte blocks (which might have been more than the actual data), these lines "correct" the pointer.
	   	It moves the pointer forward by exactly len (the actual number of bytes received), so the next packet starts exactly where this one ended.*/
		EndPoint[dfifo].rxBuffer_ptr = dest; // only for EP > 0 !
		EndPoint[dfifo].rxCounter += len;
	}
}

/****************************************************
* 		EndPoints' Callbacks*
***************************************************/

/**
* This Callback function is where the high-level logic meets the actual hardware registers.
* Take a buffer of any size, chop it into USB-compliant packets, handle FIFO hw constraints, and manage the ZLP logic required by the USB specification
* Atomic-Style Execution: Because you'll call this from the scheduler inside a __disable_irq() block, the registers are set,
* the FIFO is filled, and the hardware is ready before any other part of your code can interfere.
* brief  check Endpoint TX status
* param  EP number
* retval
*/
uint32_t USB_CDC_transferTXCallback(uint8_t EPnum){
    // 1. Wait for Hardware Readiness
    uint32_t start_tick = GetSysTick();
    while(USB_EP_IN(EPnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
    	if (GetSysTick() - start_tick > 5) return EP_FAILED;
    }

    uint16_t len = EndPoint[EPnum].txCounter;

    if (EPnum == 0) {
    	/* CONTROL HANDSHAKE / DESCRIPTORS */
    	// Calculate packets: (67 + 63) / 64 = 2 packets
    	uint32_t pktcnt = (len == 0) ? 1 : (len + 63) / 64;

    	// Hardware setup for exactly what is in the buffer
    	// DIEPTSIZ: [PKTCNT (bits 20:19)] | [XFRSIZ (bits 18:0)]
    	USB_EP_IN(0)->DIEPTSIZ = (pktcnt << 19) | len;
    	USB_EP_IN(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

    	// Fill FIFO (Only up to 64 bytes for the first packet)
    	uint16_t fill = (len > 64) ? 64 : len;
    	if (fill > 0) {
    		write_Fifo(0, EndPoint[0].txBuffer_ptr, fill);
    	}

    	// If more data remains (like our 3 extra bytes), unmask TXFE
    	if (len > 64) {
    		USB_OTG_DEVICE->DIEPEMPMSK |= (1 << 0);
    	}
    }
    else {
    	EndPoint[1].statusTx = EP_BUSY;
    	EndPoint[1].totXferLen = len;

    	/* DATA ENDPOINT 1 (Turbo Logic) - PKTCNT and XFRSIZ are set for up to 2048 bytes (originally was only for 64 bytes)
    	The hardware will automatically chop that 2048-byte block into 64-byte packets and send them one after another
    	There is only one XFRC (Transfer Complete) interrupt at the very end of the 2048 bytes, instead of 32 separate interrupts.
    	*/
    	uint32_t total = (len > 2048) ? 2048 : len;
    	uint32_t pktcnt = (total == 0) ? 1 : (total + 63) / 64;

    	USB_EP_IN(EPnum)->DIEPTSIZ = (pktcnt << 19) | total; // set nb of packets and total nb of bytes
    	USB_EP_IN(EPnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA); // re-arm EP for transmitting

    	uint16_t first_fill = (total > 128) ? 128 : total;
    	if (first_fill > 0) write_Fifo(EPnum, EndPoint[EPnum].txBuffer_ptr, first_fill); // load bytes into FIFO
    	if (total > 128) USB_OTG_DEVICE->DIEPEMPMSK |= (1 << EPnum); // Trigger Empty Fifo interrupt if there a more data
    }

    return EP_OK;
}


/*
 * brief  Set and start TX transaction
* param  EP number, TX Buffer, length
* retval OK/FAILED
*/
uint32_t USB_CDC_setTxBuffer(uint8_t EPnum, uint8_t *txBuff, uint16_t len){

	/************** Safety check - Previous transaction is not finished ****************/
	if((EndPoint[EPnum].txCounter != 0) || (EndPoint[EPnum].statusTx == EP_ZLP)
										|| (check_USB_device_status(DEVICE_STATE_TX_PR) == EP_OK)) {
		return EP_FAILED;
	}

	// 1. REMOVE the 128-byte cap check!
	// The hardware DIEPTSIZ register can handle up to 512KB. Let's allow len up to 4096 for now.
	if(len > 4096) return EP_FAILED;

	/* Set data to send */
	if(len != 0){
		EndPoint[EPnum].txBuffer_ptr = txBuff; 	// *txBuff points to the first index to read on the circular buffer
		EndPoint[EPnum].txCounter = len; //
		/* SEND DATA (calling USB_CDC_transferTXCallback) */
		set_device_status(DEVICE_STATE_TX_PR);
		EndPoint[EPnum].txCallBack(EPnum);

		/* TOGGLE INTERRUPT MASK HERE */
		/* If the callback (write_Fifo) didn't finish the whole 'len', we enable the Empty FIFO interrupt to handle the rest. */
		if (EPnum > 0 && EndPoint[EPnum].txCounter > 0) {
		/*	By enabling DIEPEMPMSK here, you are telling the STM32: "I just gave you the first 128 bytes of a large transfer.
			The moment you've sent those and the FIFO is empty, scream at me (interrupt) so I can give you the next 128."
			This keeps the USB data lines constantly toggling without the CPU having to "poll" or restart the whole transaction from scratch.*/
			// Unmask the Transmit FIFO Empty interrupt for this EP
			  USB_OTG_DEVICE->DIEPEMPMSK |= (1 << EPnum);
		}
		return EP_OK;
	}
	/* Zero-Length Packet (with EP0 after enumerate_Setup() send a zlp ACK) */
	else {
		// 1. Clear out any previous state (optional but safer in Turbo)
		USB_EP_IN(EPnum)->DIEPINT = 0xFF;
		// 2. Set SIZES FIRST (One single write)
		// This sets PKTCNT=1 and XFRSIZ=0 in one bus cycle
		USB_EP_IN(EPnum)->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
		// 3. ENABLE LAST
		// This is the only time you should touch EPENA in this block
		USB_EP_IN(EPnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		// 4. Arm the OUT endpoint
		USB_EP_OUT(EPnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

		return EP_OK;
	}
}

/**
* brief  Perform some action with recieved data (EP0) and refresh EP buffer counter
* brief  For EP0 - Set lineCoding
* param  command
* retval
*/
uint32_t USB_CDC_transferRXCallback_EP0(uint32_t param){
	uint16_t len = EndPoint[0].rxCounter;

	// Safety checks
	if (len == 0) return EP_OK;
	if (len > CDC_LINE_CODING_LENGTH) len = CDC_LINE_CODING_LENGTH; // Stay in bounds
	if (EndPoint[0].statusRx == EP_BUSY) return EP_FAILED;

	if (param == CDC_SET_LINE_CODING) {
		// Overwrite our variable lineCoding with whatever the PC sent (7 bytes).
		// Even if the PC asks for 9600 baud or 115200, we just store it.
		memcpy(lineCoding, rxBufferEp0, CDC_LINE_CODING_LENGTH);

		EndPoint[0].rxCounter = 0; // Clear the counter so we don't process the same data twice.

		/* Important: When you call this function, you are writing to the DOEPCTL (Device Out Endpoint Control) register.
		When Endpoint 0 is in the Status Stage of a Control Out transfer, setting it to VALID/READY tells the hardware:
		"I have finished processing the data in RAM. You are now authorized to send the ACK (Status Phase) back to the Host."
		The hardware then automatically generates the ZLP and sends it to the Host. */
		toggle_Rx_EP_Status(0, EP_READY); // Re-arm the endpoint to tell the Host we've accepted the settings.
	}
	return EP_OK;
}

/**
* brief  Perform some action with received data (EP1) and refresh EP buffer counter
* param  a command or a dummy param
*/
uint32_t USB_CDC_transferRXCallback_EP1(uint32_t param){

	if(EndPoint[1].statusRx == EP_BUSY) return EP_FAILED;

 	uint16_t len = EndPoint[1].rxCounter;

	// reset RX counter and buffer pointer
 	EndPoint[1].rxBuffer_ptr -= EndPoint[1].rxCounter; // rewinds the pointer back to the base address
	EndPoint[1].rxCounter = 0;

	write_to_circBufferRx(EndPoint[1].rxBuffer_ptr, len);  // Transferring Data to the Rx Circular Buffer

	USB_CDC_UserRxCallBack_EP1(len); // Weak function called upon interrupt XFRC (Transfer completed) - use then read_circBuffer() to process

	/* PKTCNT[9:0]: Packet count: Indicates the total number of USB packets that constitute the transfer size amount of data for this EP.
	This field is decremented every time a packet (maximum size or short packet) is written to the Rx FIFO. */
	USB_EP_OUT(1)->DOEPTSIZ = (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos))
							| DOEPT_TRANSFER_SIZE; // = 64, the core decrements XFRSIZ every time a pckt is read from the Rx FIFO and written to the external memory

	USB_EP_OUT(1)->DOEPCTL  |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

	return param;
}

/***************************************************
*			USB enumeration
***************************************************/

void enumerate_Reset(){

	/************************************************************/
	/* 1. CLEAN THE PIPES FIRST                                 */
	/************************************************************/
	USB_FlushRxFifo(2000);       // Clear the Global Receive FIFO
	USB_FlushTxFifo(0, 2048);    // Clear Control EP0 TX FIFO
	USB_FlushTxFifo(1, 2048);    // Clear CDC Data EP1 TX FIFO

	/************************************************************/
	/* 2. RESET SOFTWARE STATE                                  */
	/************************************************************/
	set_device_status(DEVICE_STATE_RESET);
	USB_OTG_FS->GINTSTS = 0xFFFFFFFF; // Clear interrupts

	/************************************************************/
	/* 3. RECONFIGURE ENDPOINTS 							    */
	/************************************************************/
	initEndPoints(); // Hardware-enable EP0 and reassert EP1/EP2 structure

	USB_OTG_FS->GINTSTS &= ~0xFFFFFFFF; // reset OTG core interrupt register

	/* OTG all endpoints interrupt mask register */
	USB_OTG_DEVICE->DAINTMSK = 0x30003; // IEPINT-> IN EP0 & IN EP1 interrupts unmasked, OEPINT: OUT endpoint 0 & 1 interrupts unmasked
	USB_OTG_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM; /* Unmask SETUP Phase done Mask,  TransfeR Completed interrupt for OUT */
	USB_OTG_DEVICE->DIEPMSK  =  /* USB_OTG_DIEPMSK_ITTXFEMSK | */ USB_OTG_DIEPMSK_XFRCM; /* TransfeR Completed interrupt for IN */

	USB_OTG_DEVICE->DCFG  &= ~USB_OTG_DCFG_DAD_Msk;  /* before Enumeration set address 0 */

	/* Endpoint 1 */
	USB_EP_IN(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
			USB_OTG_DIEPCTL_TXFNUM_0 |  /* TX Number 1 */
			USB_OTG_DIEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
			USB_OTG_DIEPCTL_USBAEP |  /* Set Endpoint active */
			USB_CDC_MAX_PACKET_SIZE;  /* Max Packet size (bytes) */

	USB_EP_OUT(1)->DOEPTSIZ = 0;
	USB_EP_OUT(1)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos)); /* RM quote: Indicates the total number of USB packets that constitute the Transfer Size amount of data for this endpoint. This field is decremented every time a packet (maximum size or short packet) is written to the RxFIFO */
	USB_EP_OUT(1)->DOEPTSIZ |= DOEPT_TRANSFER_SIZE; /* Transfer size. If you set transfer size = max. packet, the core will interrupt the application at the end of each packet */

	USB_EP_OUT(1)->DOEPCTL = USB_OTG_DOEPCTL_EPENA | 	/* Enable Endpoint after DOEPTSIZ programmed  */
			USB_OTG_DOEPCTL_CNAK |  /* Clear NAK */
			USB_OTG_DOEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
			USB_OTG_DOEPCTL_USBAEP | /* Set Endpoint active */
			USB_CDC_MAX_PACKET_SIZE; /* CHK MPSIZ The application must program this field with the maximum packet size for the current logical endpoint. This value is in bytes */
}


/**
 * brief  Handle all host requests, send all descriptors data
 * param
 * param
 * retval
 */
void enumerate_Setup(){
	// Combined request for your existing logic
	    uint16_t request = (setup_pkt_data.setup_pkt.bRequest << 8) | setup_pkt_data.setup_pkt.bmRequestType;
	    uint16_t len = setup_pkt_data.setup_pkt.wLength;
	    static uint8_t dest[128]; // Use static!
	switch(request){

	case REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR:
		switch(setup_pkt_data.setup_pkt.wValue){
		case DESCRIPTOR_TYPE_DEVICE: 				/* Request 0x0680  Value 0x0100 */
			if(DEVICE_DESCRIPTOR_LENGTH < len) len = DEVICE_DESCRIPTOR_LENGTH;
			memcpy(&dest, &deviceDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_CONFIGURATION: 			/* Request 0x0680  Value 0x0200 */
			if(CONFIGURATION_DESCRIPTOR_LENGTH < len) len = CONFIGURATION_DESCRIPTOR_LENGTH;
			memcpy(&dest, &configurationDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_DEVICE_QUALIFIER: 			/* Request 0x0680  Value 0x0600 */
			if(DEVICE_QUALIFIER_LENGTH < len) len = DEVICE_QUALIFIER_LENGTH;
			memcpy(&dest, &deviceQualifierDescriptor, len);
			//	return; /* CUBE MX CDC actually doesn't send any data here */
			break;
		case DESCRIPTOR_TYPE_LANG_STRING: 			/* Request 0x0680  Value 0x0300 */
			if(LANG_DESCRIPTOR_LENGTH < len) len = LANG_DESCRIPTOR_LENGTH;
			memcpy(&dest, &languageStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_MFC_STRING: 			/* Request 0x0680  Value 0x0301 */
			if(MFC_DESCRIPTOR_LENGTH < len) len = MFC_DESCRIPTOR_LENGTH;
			memcpy(&dest, &manufactorStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_PROD_STRING: 			/* Request 0x0680  Value 0x0302 */
			if(PRODUCT_DESCRIPTOR_LENGTH < len) len = PRODUCT_DESCRIPTOR_LENGTH;
			memcpy(&dest, &productStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_SERIAL_STRING: 			/* Request 0x0680  Value 0x0303 */
			Get_ID_To_String(dest); // fetch the UID
			len = SERIAL_DESCRIPTOR_LENGTH; // known fixed length
			break;
		case DESCRIPTOR_TYPE_CONFIGURATION_STRING: 		/* Request 0x0680  Value 0x0304 */
			if(CONFIG_STRING_LENGTH < len) len = CONFIG_STRING_LENGTH;
			memcpy(&dest, &configurationStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_INTERFACE_STRING: 			/* Request 0x0680  Value 0x0305 */
			if(INTERFACE_STRING_LENGTH < len) len = INTERFACE_STRING_LENGTH;
			memcpy(&dest, &stringInterface, len);
			break;
		default:
			return;
		}
		break;

	case REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS: 				/* Request 0x0500  */
		len=0; /* ZLP */
		USB_OTG_DEVICE->DCFG &= ~((uint32_t)0x7F << 4); // Clear the 7-bit DAD field (bits 4 to 10) first!
		USB_OTG_DEVICE->DCFG |= (uint32_t)(setup_pkt_data.setup_pkt.wValue << 4);
		device_state = DEVICE_STATE_ADDRESSED;
		break;
	case REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION: 			/* Request 0x0900  */
		len=0; /* ZLP */
		USB_CDC_ForceResetState(); // When the PC assigns an address or sets the configuration, it will start a fresh session with this hardware
		device_state |= DEVICE_STATE_CONFIGURED;
		break;
	case CDC_GET_LINE_CODING: 						/* Request 0x21A1  */
		if(CDC_LINE_CODING_LENGTH < len) len = CDC_LINE_CODING_LENGTH;
		memcpy(&dest, &lineCoding, len);
		break;
	case CDC_SET_LINE_CODING: 						/* Request 0x2021  */
		len=0;
		EndPoint[0].rxCallBack(CDC_SET_LINE_CODING);
		device_state |= DEVICE_STATE_LINECODED;
		break;
	case CDC_SET_CONTROL_LINE_STATE: /* Request 0x2221 - when click "Connect" in TeraTerm, the PC sends this request (Data Terminal Ready signal) */
		port_is_open = 1; // wValue bit 0 (Data Terminal Ready) is always at 0 with my Win10 so we don't know if port = Opened or Closed
		len=0;											// so we will use a timeout at the top of USB_CDC_UserSend_Data()
		USB_CDC_ForceResetState(); // Ensures that if there is no "garbage" left in the circular buffer from a previous session
		break;
	case CLEAR_FEATURE_ENDP: 						/* Request 0x0201  */
		uint8_t ep_num = setup_pkt_data.setup_pkt.wIndex & 0x7F;
		uint8_t is_in = setup_pkt_data.setup_pkt.wIndex & 0x80;
		if (is_in) {
			// 1. Clear the STALL bit in DIEPCTL
			USB_EP_IN(ep_num)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
			// 2. Reset Data Toggle to DATA0
			USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
		} else {
			// 1. Clear the STALL bit in DOEPCTL
			USB_EP_OUT(ep_num)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
			// 2. Reset Data Toggle to DATA0
			USB_EP_OUT(ep_num)->DOEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
		}
		len = 0; // Prepare ZLP for Status Phase
		break;   // Fall through to EndPoint[0].setTxBuffer
	default:
		break;
	}

	EndPoint[0].setTxBuffer(0, dest, len); // must be sent even if len == 0
}

/**
 * brief  Schedule a new transmission after a previous one finished
 * param
 * param
 * retval EP_FAILED if conditions are not met
 */
uint32_t USB_CDC_transmit_scheduler(){

	// if we are in the process of sending multiple packets or if Fifo is ready
	if(is_tx_ep_fifo_ready(1, 1) == EP_OK) {
		if(!is_circBufferTx_empty()) // exit if buffer is empty
		{
			// set the length of the string to send by reading data from ring buffer/ s
			circBufferAddress newMsgaddr = read_circBufferTx(MAX_CDC_EP1_TX_SIZ); // MAX_CDC_EP1_TX_SIZ = 128
			if (newMsgaddr.len == 0) return EP_READY ;
			EndPoint[1].setTxBuffer(1, &circBufferTx[newMsgaddr.index], newMsgaddr.len); // EndPoint[i].setTxBuffer = &USB_CDC_setTxBuffer;
			return EP_OK;
		}
	}
	return EP_FAILED;
}


/************************************************************/
/*************************** inline *************************/
/************************************************************/

/* Device status functions. Set/clear/check*/

static inline void set_device_status(eDeviceState state){
	device_state |= state;
}

void clear_USB_device_status(eDeviceState state){
	device_state &= ~state;
}

uint32_t check_USB_device_status(eDeviceState state){
	if(device_state & state){
		return EP_OK;
	}
	else return EP_FAILED;
}

/**
* brief  Check if TX FIFO is ready to push there data
* param  EP number
* param  param. If you use TX queue, "param" would be message count var pending in the queue
* param  if you have a var like message_counter, you use it here, otherwise use something > 0
* retval
*/

static inline uint32_t is_tx_ep_fifo_ready(uint8_t EPnum, uint32_t param){
	if((param > 0) & !(device_state & DEVICE_STATE_TX_PR) &
		!(USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) &
		((USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_HCTSIZ_PKTCNT) == 0) &
		!(USB_EP_IN(EPnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)) {
		return EP_OK;
	}
	else return EP_FAILED;
}

/*************************** End of inline functions *************************/


// TODO NON BLOCKING Version
/* User function for sending data */
uint32_t USB_CDC_UserSend_Data(uint8_t *txBuff, uint16_t len){

	// Gating check: If the port isn't open, don't even try
	if(!port_is_open) return EP_FAILED;
	if (len == 0) return EP_OK;

	/* - "get_circBufferTx_freeSize() < len", if the Main Loop is faster than the USB, the while loop slows it down, preventing accidental overwriting of the buffer
	   - "EndPoint[1].statusTx != EP_READY", when you use an if, you might miss the "opening" of a frame and have to wait for the next loop. With the while,
	   you are hitting the hardware the microsecond it becomes available. Most importantly, without this instruction, USB_CDC_send_data() simply won't work
	   in a loop, or only with slowing down the loop, because EndPoint[1].statusTx will most often remain on EP_BUSY (latency synchronization issue).
	*/
	uint32_t start_tick = GetSysTick(); // Custom timer

	while (EndPoint[1].statusTx != EP_READY || get_circBufferTx_freeSize() < len) {
		// "Gatekeeper" waits for the EP to be ready: this is the best place to catch a disconnect (Close port)
		// Auto-Disconnect: 30ms gives the Host 30 full chances to ask for data. If it doesn't ask for 30ms,
		if ((GetSysTick() - start_tick) > 30) { // it's not because it's busy - it's because the COM port is closed or the cable is gone
	        USB_CDC_StopComPort();       // Wipe the hardware state and set EP[1].statusTx = EP_READY to allow the next connection
	        GPIOD->ODR^=GPIO_ODR_OD4; // orange
	        return EP_FAILED;
	    }
	}
	uint16_t sentBytes = 0;

	while (sentBytes < len) {

		if (GetSysTick() - start_tick > 30) { // 30ms timeout
			USB_CDC_ForceResetState();
			return EP_FAILED;
		}
		uint16_t freeSize = get_circBufferTx_freeSize(); // if empty size = CIRC_BUFFER_TX_SIZE

		if (freeSize > 1) {
			// Note: if(freeSize > 0): if freeSize=1, chunk becomes 0 -> sentBytes += chunk does nothing, and the while() will spin forever until the timeout hits
			// Calculate how much we can actually fit right now
			uint16_t chunk = (len - sentBytes);
			if (chunk > freeSize) chunk = freeSize - 1; // - 1 bec if the buffer is full writePtrTxCbuf == readPtrTxCbuf = buffer empty
			write_to_circBufferTx(&txBuff[sentBytes], chunk);
			sentBytes += chunk;

			/* Since you are using the circular buffer to bridge the Main Loop (Producer) and the ISR (Consumer), you need to ensure they don't
			step on each other's toes. Whenever you call the scheduler from the "outside" (the Main Loop), wrap it in a brief interrupt disable.
			By checking both the hardware bit (EPENA) and your software flag (statusTx) inside the disabled-interrupt zone, you create a "Hard Lock."
			It is physically impossible for the packets to mix because the Main Loop will see that the hardware is busy and simply walk away,
			knowing the ISR will pick up the remaining data in the circular buffer as soon as the current packet finishes.
			*/
			__disable_irq();
			if (!(USB_EP_IN(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && (EndPoint[1].statusTx == EP_READY)) {
				USB_CDC_transmit_scheduler(); // will read circular buffer and prepare transfer
			}
			__enable_irq();
		}
	}
	return EP_OK;
}

/* it is recommended to store RX data in a buffer and process the data in a main loop or separated task */
__WEAK uint32_t USB_CDC_UserRxCallBack_EP1(uint16_t length){

	return length;
}

/*********************************************************************************/
/**************************** OTG FS ISR *****************************************/
/*********************************************************************************/

extern void OTG_FS_IRQHandler(void);

void OTG_FS_IRQHandler(){

/*	Typically handle the ISR these in order:
	-ENUMDNE (Enumeration Done): The hardware tells you the speed is set.
	-RXFLVL (RX FIFO Non-Empty): There is a packet from the PC. You read it to see if it's a SETUP, OUT, or DATA packet.
	-IEPINT (In Endpoint Interrupt): A previous "Send" (In) operation finished successfully; you can now send more data. */

	/***** No Start Of Frame event to prevent interruptions firing every ms in FS mode  ****/
	/* SOF generate a lot of CPU overhead for very little gain in a simple CDC application */

	// Identify only the interrupts that are both PENDING and ENABLED
	uint32_t active_irq = USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;

	/* SUSPEND DETECTED - to use uncomment in USB_OTG_FS_init_registers()
	 If USB bus has gone quiet (which happens about 3ms after you unplug the cable or the PC goes to sleep) */
	if (active_irq & USB_OTG_GINTSTS_USBSUSP){
		USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBSUSP; // clear flag
		USB_CDC_ForceResetState(); // Perform the robust cleanup
	}

	/* Wakeup DETECTED - to use uncomment in USB_OTG_FS_init_registers() */
	if (active_irq & USB_OTG_GINTSTS_WKUINT){
		USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_WKUINT; // clear flag
		USB_CDC_ForceResetState();
	}


	/**************************************************************/
	/****************** USBRST Reset event ************************/
	/**************************************************************/
	// The core sets this bit to indicate that a reset is detected on the USB.
	if(active_irq & USB_OTG_GINTSTS_USBRST){
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_USBRST);
		enumerate_Reset();
		return;
	}

	/**************************************************************/
	/****************** ENUMDNEM event ****************************/
	/**************************************************************/
	/* The core sets this bit to indicate that speed enumeration is complete: the hardware-level connection and speed negotiation (reset/chirp) are complete,
	   allowing software to begin handling USB control transfers (such as address assignment and descriptor setup).
	   The application must read the OTG_DSTS register to obtain the enumerated speed.
	   */
	if(active_irq & USB_OTG_GINTSTS_ENUMDNE){
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_ENUMDNE);
		USB_CDC_Config_Hardware(); 	// FIX: Re-configure the FIFOs and Toggles here!

		set_device_status(DEVICE_STATE_DEFAULT);
	}

	/**************************************************************/
	/*************** IN endpoint event ****************************/
	/**************************************************************/

	/* IN endpoint event */
	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT){

	    uint32_t epnums = USB_OTG_DEVICE->DAINT;

	    // --- Check EP0 IN ---
	    if(epnums & 0x0001){
	        uint32_t IN_interrupt = USB_EP_IN(0)->DIEPINT;

	        // Tx FIFO is empty
	        if (IN_interrupt & USB_OTG_DIEPINT_TXFE) {
	            // Only fill if we actually have data pending
	            if (EndPoint[0].txCounter > 0) {
	                // write_Fifo internally updates txBuffer_ptr and txCounter
	                write_Fifo(0, EndPoint[0].txBuffer_ptr, EndPoint[0].txCounter);
	            }
	            // Always mask EP0 TXFE after one fill or if empty to prevent interrupt storm
	            USB_OTG_DEVICE->DIEPEMPMSK &= ~(1 << 0);
	        }

	        if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {
	            // CRITICAL: Handshake for Control Status Phase -  Prepare for the next Setup Packet
	            USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	            clear_USB_device_status(DEVICE_STATE_TX_PR);
	            EndPoint[0].statusTx = EP_READY;
	            USB_EP_IN(0)->DIEPINT = USB_OTG_DIEPINT_XFRC;
	        }
	        CLEAR_IN_EP_INTERRUPT(0, IN_interrupt);
	    }

	    // --- Check EP1 IN ---
	    if (epnums & 0x0002) {
	        uint32_t IN_interrupt = USB_EP_IN(1)->DIEPINT;

	        if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {
	            CLEAR_IN_EP_INTERRUPT(1, USB_OTG_DIEPINT_XFRC);

	            // 1. Handle ZLP Logic First
	            if (EndPoint[1].statusTx == EP_BUSY) {
	            	if (EndPoint[1].totXferLen > 0 && (EndPoint[1].totXferLen % 64 == 0)) {

	            		/* Send ZLP: essential is the packet is a multiple of 64 */
	            		EndPoint[1].statusTx = EP_ZLP;  // Update status so the ISR cleanup logic knows what to do next
	            		// Set Transfer Size: 1 packet, 0 bytes. We do this in one write to ensure hardware atomic-like update
	            		USB_EP_IN(1)->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) & USB_OTG_DIEPTSIZ_PKTCNT_Msk;
	            		__DMB(); // Ensure the register write is finished before enabling EP
	            		// Enable the endpoint and clear NAK to allow the Host to read the 0-byte packet
	            		USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
	            		return; // Exit ISR, wait for ZLP's XFRC
	            	}
	            }

	            // 2. The transfer is officially DONE
	            EndPoint[1].statusTx = EP_READY;
	            clear_USB_device_status(DEVICE_STATE_TX_PR);

	            // 3. Re-prime the OUT endpoint (Always listen)
	            USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | 64;
	            USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

	            // 4. Trigger next transfer IF there is data in the buffer
	            if (is_circBufferTx_empty() == 0) {
	                USB_CDC_transmit_scheduler();
	            } else {
	                // Only SNAK if the buffer is truly empty and we aren't about to send more
	                USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
	            }
	        }

	        // Safety: Clear any other pending bits for this EP
	        USB_EP_IN(1)->DIEPINT = IN_interrupt;
	    }
	}

	// outside the individual endpoint checks. This ensures that if EP1 also had an interrupt, you don't clear the "Global" flag before you've had a chance to see EP1's flag.
	USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_IEPINT);

	/***************************************************************/
	/*************** OUT endpoint event - OEPINT ********************
	 ***************************************************************		*/

	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT){ /* OUT endpoint event */

		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_OEPINT);
		uint32_t epnums  = USB_OTG_DEVICE->DAINT;    /* Read out EndPoint INTerrupt bits */

		/************************	 EP 0  	OUT	  *******************************/
		if( epnums & 0x00010000){ /* EndPoint INTerrupt bits correspond to EP0 OUT */
			uint32_t epint = USB_EP_OUT(0)->DOEPINT; /* Read out Endpoint Interrupt register for EP0 */

			if(epint & USB_OTG_DOEPINT_STUP){		/*  SETUP phase done, Setup packet received */
				/*	Applies to control OUT endpoint only.Indicates that the SETUP phase for the control endpoint is complete and no more back-to-back
					SETUP packets were received for the	current control transfer. On this interrupt, the application can decode the received SETUP data packet.*/
				enumerate_Setup();
				}
			if(epint & USB_OTG_DOEPINT_XFRC){
				EndPoint[0].rxCallBack(0);
				/* CNAK and EPENA must be set again after every interrupt to let this EP receive upcoming data */
				USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
			}
			CLEAR_OUT_EP_INTERRUPT(0, epint);
		}

		/* Originally used the TXEF interrupt (that triggers whenever the FIFO is empty) and used USB_OTG_DIEPINT_XFRC interrupt only
		  for the last packet but it was required a blocking while(); to wait EPENA to deactivate from the previous transfer -> switched to all XFRC interrupts */

		/************************	 EP 1  	OUT	  *******************************/
		if( epnums & 0x00020000){ /* EndPoint INTerrupt bits correspond to EP1 OUT */

			// OTG_DOEPINTx : this register indicates the status of an endpoint with respect to USB- and AHB-related events.
			uint32_t epint = USB_EP_OUT(1)->DOEPINT; /* Read out Endpoint Interrupt register for EP1 */

			// XFRC: Transfer completed interrupt. Indicates that the programmed transfer is complete on the AHB as well as on the USB, for this endpoint.
			if(epint & USB_OTG_DOEPINT_XFRC){
				// Call function to handle the data coming from host, moving Data to a Circular Buffer
				EndPoint[1].rxCallBack(EP_OK); // EndPoint[i].rxCallBack = &USB_CDC_transferRXCallback_EP1

				// Tells the hardware we are ready for a fresh 64 bytes before setting EPENA. Otherwise, the byte counter stays at its old value.
				USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | 64; // PKTCNT = 1, XFRSIZ = 64

				// CNAK and EPENA must be set again after every interrupt to let this EP receive upcoming data.
				USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA); //  CNAK : clears the NAK bit for the endpoint ; EPENA: Endpoint enable. nsfer completed
			}
			CLEAR_OUT_EP_INTERRUPT(1, epint);
		}
		return;
	}

	/***************************************************************/
	/***************** RXFLVL event (Receive FIFO Level) ***********/
	/**************************************************************
	This is the very first interrupt to fire when receiving data. As soon as a packet is successfully received and validated, the hardware
	pushes it into the Global Receive FIF. The RXFLVL interrupt signals that there is at least one packet sitting in the FIFO waiting to be read.
	The CPU must read the status from the USB_OTG_GRXSTSP register, which tells it the byte count and the endpoint number.
	Then, it reads the data out of the FIFO. */

	// (Receive FIFO Level): This triggers every time a packet (or a status update) lands in the RX FIFO
	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {

	    // We must drain the queue until the interrupt flag clears
	    while(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
	 /* - Atomic Operations: By pulling the data directly inside the while loop instead of calling read_Setup_Fifo(),
		  you ensure that the Hardware Status (GRXSTSP) and the Data (DFIFO) are handled as one "transaction."
		- Zero Latency: In the original code, the jump between the interrupt and the helper functions could occasionally miss a timing window.
		  The inline version is much faster.
		- Proper popping: we now "pop" the zero-length status packets that the Host sends during the 19 cycles of Open/Close COM port.
		  This is exactly what was filling up the queue before (the host sends a "Setup" then a "Status" packet. The latter places a 0-byte packet entry into the FIFO)  */

	        // 1. POP the status (Mandatory every iteration).
	        uint32_t temp = USB_OTG_FS->GRXSTSP;
	        uint8_t EpNum  = (temp & USB_OTG_GRXSTSP_EPNUM);
	        uint16_t bcnt  = (temp & USB_OTG_GRXSTSP_BCNT) >> 4; 	 // Indicates the byte count of the received data packet.
	        uint8_t pktsts = (temp & USB_OTG_GRXSTSP_PKTSTS) >> 17;  /* Indicates the status of the received packet
																		0001: Global OUT NAK (triggers an interrupt)
			Pop: referring to the Hardware Receive Status Queue,    	0010: OUT data packet received
		 	not a buffer (rxBufferEp0).									0011: OUT transfer completed (triggers an interrupt)
			Inside the STM32 USB peripheral, there is a hidden 			0100: SETUP transaction completed (triggers an interrupt)
			internal list (the Queue). Every time something happens		0110: SETUP data packet received
	        on the USB wire, the hardware writes a Status Word into
	        this list. This word contains the pktsts (what happened), the bcnt (how many bytes), and the EpNum. The register
	        USB_OTG_FS->GRXSTSP is the "Portal" to this list. Reading GRXSTSP is the "Pop": the moment the code executes temp = USB_OTG_FS->GRXSTSP,
	        the hardware physically removes the top entry from that internal list and slides the next one up.	*/

	        // 2. Handle SETUP Data (pktsts 6)
	        if (pktsts == 0x06) {
	        	// Read exactly 8 bytes (2 words) for the SETUP packet
	        	// This is your original read_Setup_Fifo() logic, but inline
	        	uint32_t *dest = (uint32_t *)rxBufferEp0;
	        	dest[0] = USB_OTG_DFIFO(0);
	        	dest[1] = USB_OTG_DFIFO(0);

	        	// Immediately fill your setup union
	        	memcpy(setup_pkt_data.raw_data, rxBufferEp0, 8);
	        }
	        // 3. Handle OUT Data (pktsts 2)
	        else if (pktsts == 0x02) {
	        	if (bcnt > 0)
	        		read_Fifo(EpNum, bcnt);
	        }
	        // If pktsts is anything else (like Global OUT NAK), we've already popped it with the read of GRXSTSP, so we just move on.
	    }
	}
}


#ifndef STM32F469_USBDFS_CDC_H_INCLUDED
#define STM32F469_USBDFS_CDC_H_INCLUDED


/****************************************************************
* STM32F469
* USB OTG FS device (CDC) implementation
*
* Alex Svetlichnyy 2021 + Nicolas Prata 20256
*
****************************************************************/

#include <string.h>
#include "stm32f469xx.h"

/******************************************************************************
* This section contains some macros and defines for better compatibility
* with other libs. If you find any conflicting types, delete them from here
*******************************************************************************/

#define USB_CLEAR_INTERRUPT(IRQ)    ((USB_OTG_FS->GINTSTS) &= (IRQ))
#define USB_MASK_INTERRUPT(IRQ)     (USB_OTG_FS->GINTMSK &= ~(IRQ))
#define USB_UNMASK_INTERRUPT(IRQ)   (USB_OTG_FS->GINTMSK |= (IRQ))

#define CLEAR_IN_EP_INTERRUPT(NUM, IRQ)          (USB_EP_IN(NUM)->DIEPINT = (IRQ))
#define CLEAR_OUT_EP_INTERRUPT(NUM, IRQ)         (USB_EP_OUT(NUM)->DOEPINT = (IRQ))

#define USB_OTG_DEVICE      		((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

#define USB_EP_OUT(i) 			((USB_OTG_OUTEndpointTypeDef *) ((USB_OTG_FS_PERIPH_BASE +  USB_OTG_OUT_ENDPOINT_BASE) + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_EP_IN(i)    		((USB_OTG_INEndpointTypeDef *)	((USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE) + ((i) * USB_OTG_EP_REG_SIZE)))

#define USB_OTG_DFIFO(i)    *(__IO uint32_t *)((uint32_t)USB_OTG_FS_PERIPH_BASE  + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

#define USB_OTG_PCGCCTL      ((USB_OTG_PCGCCTLTypeDef *)( USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))

typedef struct{
	__IO uint32_t PCGCCTL;
}
USB_OTG_PCGCCTLTypeDef;



/***************************************************
 * 			User settings
***************************************************/
/*On the F469, the total dedicated USB FIFO RAM is only 1.25 KB (1280 bytes). This is shared between:
    Rx FIFO (All OUT endpoints)
    Tx FIFOs (Each IN endpoint)
*/

#define FLUSH_FIFO_TIMEOUT		2000
#define DTFXSTS_TIMEOUT 		1024 //TODO

/* In FS mode, the maximum packet size (MPS) for Bulk and Interrupt endpoints is fixed at 64 bytes. A 128-byte (32-word) FIFO allows
   for efficient double-buffering (2x 64bytes) to manage data transmission smoothly within 1ms frames */

#define RX_FIFO_SIZE			128				// 35 - minimum working / 128 - other values don't work
#define TX_EP0_FIFO_SIZE		64				// 16 - minimum working  64 - other values don't work
#define TX_EP1_FIFO_SIZE		128

/* FIFO total Memory = RX_FIFO_SIZE + TX_EP0_FIFO_SIZE + TX_EP1_FIFO_SIZE = 320 words of 4 bytes = 1280 bits = max allowed FIFO RAM */

#define EP1_DTFXSTS_SIZE    	TX_EP1_FIFO_SIZE	/* TX FIFO empty level */
#define EP1_MIN_DTFXSTS_LVL		16		/* Minimum TX FIFO empty level */

#define MAX_CDC_EP0_TX_SIZ  	64    /* Max TX transaction size for EP0. "64" means that you can send maximum one packet of max size
in TXCallback, then you send the rest bytes (or ZLP) in next function call. Max USB_OTG_DIEPTSIZ_XFRSIZ value.     */
#define MAX_CDC_EP1_TX_SIZ  	128   /* Max TX transaction size for EP1.  Max USB_OTG_DIEPTSIZ_XFRSIZ value.      */


#define DOEPT_TRANSFER_SIZE		0x40	// = 64 ; don't use higher values for transfer size/packets nb, Rx speed will be a few % slower !
#define DOEPT_TRANSFER_PCT 		0x01	// Value used in DOEPTSIZ for EP0 and EP1

#define USB_CDC_CIRC_BUFFER_SIZE 2048 // for std baudrate CLI, up to 4096 bytes buffer gives the CPU plenty of time to wake up and process the data before it overflows

/***************************************************
 * 			EP statuses
***************************************************/
#define EP_READY 				0U
#define EP_BUSY  				1U
#define EP_ZLP   				2U
#define EP_WAIT  				3U

/***************************************************
 * 			EP functions return values
***************************************************/

#define EP_OK				1U
#define EP_FAILED			0U

/***************************************************
 * 			Device states
 ***************************************************/
typedef enum {
	DEVICE_STATE_DEFAULT =			0,
	DEVICE_STATE_RESET =			1,
	DEVICE_STATE_ADDRESSED =		2,
	DEVICE_STATE_LINECODED =		4,
	DEVICE_STATE_CONFIGURED =		8,
	DEVICE_STATE_TX_PR =			16, /* TX in PRogress */
	DEVICE_STATE_TX_FIFO1_ERROR =	32,
} eDeviceState;


/***************************************************
 * 			SETUP stage request templates
***************************************************/

#define REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR	0x0680
#define REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS				0x0500
#define REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION		0x0900

#define DESCRIPTOR_TYPE_DEVICE							0x0100
#define DESCRIPTOR_TYPE_CONFIGURATION					0x0200
#define DESCRIPTOR_TYPE_LANG_STRING						0x0300
#define DESCRIPTOR_TYPE_MFC_STRING						0x0301
#define DESCRIPTOR_TYPE_PROD_STRING						0x0302
#define DESCRIPTOR_TYPE_SERIAL_STRING					0x0303
#define DESCRIPTOR_TYPE_CONFIGURATION_STRING			0x0304
#define DESCRIPTOR_TYPE_INTERFACE_STRING				0x0305
#define DESCRIPTOR_TYPE_DEVICE_QUALIFIER				0x0600

#define CDC_GET_LINE_CODING								0x21A1
#define CDC_SET_LINE_CODING								0x2021
#define CDC_SET_CONTROL_LINE_STATE						0x2221
#define CLEAR_FEATURE_ENDP								0x0102

/***************************************************
* 		Endpoint structure
***************************************************/

typedef struct EndPointStruct{
	volatile uint16_t statusRx; // Since it can be modified in the ISR, forces the CPU to read the actual memory location
	volatile uint16_t statusTx; // every single time instead of using a cached value in a register

	uint16_t rxCounter;
	uint16_t txCounter;

	uint8_t *rxBuffer_ptr;
	uint8_t *txBuffer_ptr;

	uint32_t (*txCallBack)(uint8_t EPnum);
	uint32_t (*rxCallBack)(uint32_t param);
	uint32_t (*setTxBuffer)(uint8_t EPnum, uint8_t *txBuff, uint16_t len);

	volatile uint16_t totXferLen;

} EndPointStruct;

extern EndPointStruct EndPoint[];

/****************************************************
* 	Setup packet structure
* 	is used in union to access data both
* 	as structure and as raw data*
***************************************************/

typedef struct __attribute__((packed)){
    uint8_t   bmRequestType;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
} USB_setup_req;	/* SETUP packet buffer. Always 8 bytes */


typedef union{
	USB_setup_req setup_pkt;
	uint32_t raw_data[2];
} USB_setup_req_data;


/*************************************************** *
 * 		Functions' declaration *
***************************************************/

/* init functions */
uint32_t USB_OTG_FS_Init(void);
void USB_OTG_FS_init_registers(void);
void enumerate_Reset(void);
void enumerate_Setup(void);
// void initEndPoints(void); -> static

/* FIFO access */
void read_Fifo(uint8_t dfifo, uint16_t len);
uint32_t write_Fifo(uint8_t dfifo, uint8_t *src, uint16_t len);

/* Endpoint functions */
uint32_t USB_CDC_setTxBuffer(uint8_t EPnum, uint8_t *txBuff, uint16_t len);
uint32_t USB_CDC_transferTXCallback(uint8_t EPnum);
uint32_t USB_CDC_transferRXCallback_EP0(uint32_t param);
uint32_t USB_CDC_transferRXCallback_EP1(uint32_t param);
// inline void toggle_Rx_EP_Status(uint8_t EPnum, uint8_t param);

/* misc */
uint32_t USB_CDC_transmit_scheduler(void); /* this function monitors if any data is pending in circ buffer or whatever */
uint32_t USB_FlushTxFifo(uint32_t EPnum, uint32_t timeout);
uint32_t USB_FlushRxFifo(uint32_t timeout);

uint32_t check_USB_device_status(eDeviceState state);
void clear_USB_device_status(eDeviceState state);

/* User code functions */
uint32_t USB_CDC_UserSend_Data(uint8_t *txBuff, uint16_t len); // Use this function in your Main to send data
uint32_t USB_CDC_UserRxCallBack_EP1(uint16_t length); // This __WEAK function called upon interrupt XFRC: Transfer completed

extern volatile uint32_t msTicks;

/****************************************************
* 			Circular buffer
****************************************************/

#define CIRC_BUFFER_TX_SIZE USB_CDC_CIRC_BUFFER_SIZE // 2048
#define CIRC_BUFFER_RX_SIZE USB_CDC_CIRC_BUFFER_SIZE

void write_to_circBufferTx(uint8_t *buf, uint16_t len);

typedef struct {
	uint16_t index;
	uint16_t len;
} circBufferAddress;

circBufferAddress read_circBufferRx(uint16_t len);
extern uint8_t circBufferRx[CIRC_BUFFER_RX_SIZE];


#endif /* STM32F469_USBDFS_CDC_H_INCLUDED */


#ifndef USB_DESC_H_INCLUDED
#define USB_DESC_H_INCLUDED

/******************************************************************************
* USB CDC device descriptors
* borrowed from STMicroelectronics for educational purposes*
*******************************************************************************/

#define LOBYTE(x) (uint8_t)(x & ~0xFF00)
#define HIBYTE(x) (uint8_t)((x >> 8) & ~0xFF00)

/*Why "64" is the most common choice
Even though the STM32F469 is capable of High-Speed, many developers use the built-in FS PHY. In this case:  You must use 64.
If you set it to 128 or 256 in FS mode, the hardware cannot physically fit that much data into a single USB frame transaction,
and the host controller will flag a "babble" error or a PID sequence error.*/
#define USB_CDC_MAX_PACKET_SIZE		64

#define CDC_CMD_PACKET_SIZE	8  /* Control Endpoint Packet size */
#define EP0_SIZE			64
#define EP_COUNT			2


#define USBD_VID				1155
#define USBD_LANGID_STRING			1033
#define USBD_MANUFACTURER_STRING		"STMicroelectronics"
#define USBD_PID_FS				22336
#define USBD_PRODUCT_STRING_FS			"STM32 Virtual ComPort"

#define DEVICE_DESCRIPTOR_LENGTH		18
#define CONFIGURATION_DESCRIPTOR_LENGTH 60

#define LANG_DESCRIPTOR_LENGTH			4
#define MFC_DESCRIPTOR_LENGTH			38
#define PRODUCT_DESCRIPTOR_LENGTH		44
#define SERIAL_DESCRIPTOR_LENGTH		26
#define DEVICE_QUALIFIER_LENGTH			10
#define INTERFACE_STRING_LENGTH			28
#define CONFIG_STRING_LENGTH			22

#define CDC_LINE_CODING_LENGTH			7

/* Device string descriptor */
static const uint8_t deviceDescriptor[DEVICE_DESCRIPTOR_LENGTH] = {
	DEVICE_DESCRIPTOR_LENGTH, //
	0x01, /* Descriptor type - device */
	0x00, /*  0x0110 = usb 1.1 ; 0x0200 = usb 2.0 */
	0x02,
	0x02, /* CDC */
	0x02, /*  Abstract Control Model subclass */
	0x00,  /* protocol */
	EP0_SIZE, /* EP0 size */
	LOBYTE(USBD_VID),
	HIBYTE(USBD_VID),
	LOBYTE(USBD_PID_FS),
	HIBYTE(USBD_PID_FS),
	0x00, /* ver. (BCD) */
	0x02, /* ver. (BCD) */
	0x01, /* Manufactor string index */
	0x02, /* Product string index */
	0x03, /* Serial number string index */
	1 /* configuration count */
};

/* Configuration descriptor */
static const uint8_t configurationDescriptor[CONFIGURATION_DESCRIPTOR_LENGTH] = {
	// EP0 being the mandatory "Default Control Pipe" for all USB devices, it is never declared in the Configuration Descriptor.
	/*Configuration Descriptor*/
	0x09,   /* bLength: Configuration Descriptor size */
	0x02,      /* bDescriptorType: Configuration */
	CONFIGURATION_DESCRIPTOR_LENGTH,                /* wTotalLength:no of returned bytes */
	0x00,
	0x02,   /* bNumInterfaces: 2 interface */
	0x01,   /* bConfigurationValue: Configuration value */
	0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
	0xC0,   /* bmAttributes: self powered */
	0x32,   /* MaxPower 0 mA */

	/*---------------------------------------------------------------------------*/

	/*Interface 0 Descriptor */
	0x09,   /* bLength: Interface Descriptor size */
	0x04,  /* bDescriptorType: Interface */
	/* Interface descriptor type */
	0x00,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x00,   /* bNumEndpoints: to 0, the Host will not look for an EP Descriptor following the functional descriptors */
	0x02,   /* bInterfaceClass: Communication Interface Class */
	0x02,   /* bInterfaceSubClass: Abstract Control Model */
	0x01,   /* bInterfaceProtocol: Common AT commands */
	0x00,   /* iInterface: */

	/*Header Functional Descriptor*/
	0x05,   /* bLength: Endpoint Descriptor size */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x00,   /* bDescriptorSubtype: Header Func Desc */
	0x10,   /* bcdCDC: spec release number */
	0x01,

	/*Call Management Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x01,   /* bDescriptorSubtype: Call Management Func Desc */
	0x00,   /* bmCapabilities: D0+D1 */
	0x01,   /* bDataInterface: 1 */

	/*ACM Functional Descriptor*/
	0x04,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
	0x02,   /* bmCapabilities */

	/*Union Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x06,   /* bDescriptorSubtype: Union func desc */
	0x00,   /* bMasterInterface: Communication class interface */
	0x01,   /* bSlaveInterface0: Data Class Interface */

	/*---------------------------------------------------------------------------*/

	/*Data class interface descriptor*/
	0x09,   /* bLength: Endpoint Descriptor size */
	0x04,  /* bDescriptorType: */
	0x01,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x02,   /* bNumEndpoints: Two endpoints used */
	0x0A,   /* bInterfaceClass: CDC */
	0x00,   /* bInterfaceSubClass: */
	0x00,   /* bInterfaceProtocol: */
	0x00,   /* iInterface: */

	/*Endpoint OUT Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	0x05,      /* bDescriptorType: Endpoint */
	0x01,                        /* bEndpointAddress */
	0x02,                              /* bmAttributes: Bulk */
	LOBYTE(USB_CDC_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
	HIBYTE(USB_CDC_MAX_PACKET_SIZE),
	0x00,                              /* bInterval: ignore for Bulk transfer */

	/*Endpoint IN Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	0x05,      /* bDescriptorType: Endpoint */
	0x81,                         /* bEndpointAddress */
	0x02,                              /* bmAttributes: Bulk */
	LOBYTE(USB_CDC_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
	HIBYTE(USB_CDC_MAX_PACKET_SIZE),
	0x00                               /* bInterval: ignore for Bulk transfer */
};


/* Language string descriptor */
static const uint8_t languageStringDescriptor[LANG_DESCRIPTOR_LENGTH] = {
	LANG_DESCRIPTOR_LENGTH,				 /* USB_LEN_LANGID_STR_DESC */
	0x03,    			/* USB_DESC_TYPE_STRING */
	LOBYTE(USBD_LANGID_STRING),
	HIBYTE(USBD_LANGID_STRING)
};
/* Manufactor string descriptor */
static const uint8_t manufactorStringDescriptor[MFC_DESCRIPTOR_LENGTH] = {
	MFC_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'i', 0x00,
	'c', 0x00,
	'r', 0x00,
	'o', 0x00,
	'e', 0x00,
	'l', 0x00,
	'e', 0x00,
	'c', 0x00,
	't', 0x00,
	'r', 0x00,
	'o', 0x00,
	'n', 0x00,
	'i', 0x00,
	'c', 0x00,
	's', 0x00
};
/* Product string descriptor */
static const uint8_t productStringDescriptor[PRODUCT_DESCRIPTOR_LENGTH] = {
	PRODUCT_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'3', 0x00,
	'2', 0x00,
	' ', 0x00,
	'V', 0x00,
	'i', 0x00,
	'r', 0x00,
	't', 0x00,
	'u', 0x00,
	'a', 0x00,
	'l', 0x00,
	' ', 0x00,
	'C', 0x00,
	'o', 0x00,
	'm', 0x00,
	'P', 0x00,
	'o', 0x00,
	'r', 0x00,
	't', 0x00
};
/* Serial number string descriptor */
static const uint8_t serialNumberStringDescriptor[SERIAL_DESCRIPTOR_LENGTH] = {
	SERIAL_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	0x34, 0x00,
	0x38, 0x00,
	0x08, 0x00,
	0x45, 0x00,
	0x37, 0x00,
	0x34, 0x00,
	0x46, 0x00,
	0x37, 0x00,
	0x36, 0x00,
	0x33, 0x00,
	0x30, 0x00,
	0x38, 0x00
};
/* Device qualifier string descriptor */
static const uint8_t deviceQualifierDescriptor[DEVICE_QUALIFIER_LENGTH] = {
	DEVICE_QUALIFIER_LENGTH,
	0x06,	/* Device Qualifier */
	0x00,
	0x02,
	0x00,
	0x00,
	0x00,
	0x40,
	0x01,
	0x00
};


static const uint8_t stringInterface[INTERFACE_STRING_LENGTH] = {
	INTERFACE_STRING_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'C', 0x00,
	'D', 0x00,
	'C', 0x00,
	' ', 0x00,
	'I', 0x00,
	'n', 0x00,
	't', 0x00,
	'e', 0x00,
	'r', 0x00,
	'f', 0x00,
	'a', 0x00,
	'c', 0x00,
	'e', 0x00
};

static const uint8_t configurationStringDescriptor[CONFIG_STRING_LENGTH] = {
	CONFIG_STRING_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'C', 0x00,
	'D', 0x00,
	'C', 0x00,
	' ', 0x00,
	'C', 0x00,
	'o', 0x00,
	'n', 0x00,
	'f', 0x00,
	'i', 0x00,
	'g', 0x00
};

#endif /* USB_DESC_H_INCLUDED */


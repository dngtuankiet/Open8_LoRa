#include <stdint.h>
#include <string.h>
#include "devices.h"
#include "print.h"
#include "lora.h"

#define TRANSMIT

//Pin assigment
#define SCK 14
#define CS 15 
#define MOSI 26
#define MISO 25
#define RST 19
#define DIO0 21


//SIFIVE SPI Register Offset
#define SPI_REG_SCKDIV 		0x00	//Serial clock divisor
#define SPI_REG_SCKMODE 	0x04	//Serial clock mode
#define SPI_REG_CSID		0x10	//Chip select ID
#define SPI_REG_CSDEF		0x14	//Chip select default
#define SPI_REG_CSMODE		0x18	//Chip select mode

#define SPI_REG_DELAY0		0x28	//Delay control 0
#define SPI_REG_DELAY1		0x2C	//Delay control 1


#define SPI_REG_FMT			0x40	//Frame format
#define SPI_REG_TXFIFO		0x48	//Tx FIFO data
#define SPI_REG_RXFIFO		0x4C	//RX FIFO data
#define SPI_REG_TXCTRL		0x50	//TX FIFO watermark
#define SPI_REG_RXCTRL		0x54	//RX FIFO watermark
#define SPI_REG_FCTRL		0x60	//SPI flash interface control
#define SPI_REG_FFMT			0x64	//SPI flash instruction format
#define SPI_REG_IE			0x70	//SPI interrupt enable
#define SPI_REG_IP			0x74	//SPI interrupt pending

#define _REG8(p, i) (*(volatile uint8_t *)((p) + (i)))
#define SPI_REG(offset) _REG8(SPI_CTRL_ADDR, offset)
#define LED_REG(offset) _REG8(LED_Address, offset)

//#define SPI_REG(offset) *((volatile uint8_t *)SPI_CTRL_ADDR + offset)

/* Fields */
#define SPI_SCK_PHA             0x1
#define SPI_SCK_POL             0x2

#define SPI_FMT_PROTO(x)        ((x) & 0x3)
#define SPI_FMT_ENDIAN(x)       (((x) & 0x1) << 2)
#define SPI_FMT_DIR(x)          (((x) & 0x1) << 3)
#define SPI_FMT_LEN(x)          (((x) & 0xf) << 16)

/* TXCTRL register */
#define SPI_TXWM(x)             ((x) & 0xffff)
/* RXCTRL register */
#define SPI_RXWM(x)             ((x) & 0xffff)

#define SPI_IP_TXWM             0x1
#define SPI_IP_RXWM             0x2

#define SPI_FCTRL_EN            0x1

#define SPI_INSN_CMD_EN         0x1
#define SPI_INSN_ADDR_LEN(x)    (((x) & 0x7) << 1)
#define SPI_INSN_PAD_CNT(x)     (((x) & 0xf) << 4)
#define SPI_INSN_CMD_PROTO(x)   (((x) & 0x3) << 8)
#define SPI_INSN_ADDR_PROTO(x)  (((x) & 0x3) << 10)
#define SPI_INSN_DATA_PROTO(x)  (((x) & 0x3) << 12)
#define SPI_INSN_CMD_CODE(x)    (((x) & 0xff) << 16)
#define SPI_INSN_PAD_CODE(x)    (((x) & 0xff) << 24)

#define SPI_TXFIFO_FULL  (1 << 31)   
#define SPI_RXFIFO_EMPTY (1 << 31)   

/* Values */

#define SPI_CSMODE_AUTO         0
#define SPI_CSMODE_HOLD         2
#define SPI_CSMODE_OFF          3

#define SPI_DIR_RX              0
#define SPI_DIR_TX              1

#define SPI_PROTO_S             0
#define SPI_PROTO_D             1
#define SPI_PROTO_Q             2

#define SPI_ENDIAN_MSB          0
#define SPI_ENDIAN_LSB          1


// LoRA Register Map
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255


// Pins of the ss (chip select) and reset
#define ss 0 // TODO: Not totally configurable
#define rst 1 // TODO: Not totally configurable

int packetIndex = 0;


/*Common functions*/
void wait_for_txfifo(void) {
  unsigned char val = 0x80;
  while(val == (unsigned char)0x80) {
    val = (unsigned char)SPI_REG(SPI_REG_TXFIFO+3);
    val = val & (unsigned char)0x80;
  }
}

uint8_t get_for_rxfifo(void) {
  unsigned char val = 0x80;
  uint8_t x;
  while(val == (unsigned char)0x80) {
    val = (unsigned char)SPI_REG(SPI_REG_RXFIFO+3); 
    val = val & (unsigned char)0x80; // if 80, is empty
    x = SPI_REG(SPI_REG_RXFIFO);
  }
}

uint8_t singleTransfer(uint8_t address, uint8_t value){
	uint8_t response;
	//Deassert CS
  LED_REG(0) = (0 << ss) | (1 << rst); //digitalWrite(ss, LOW) //TODO: Them vao day
  SPI_REG(SPI_REG_CSMODE) = (uint8_t)SPI_CSMODE_HOLD;
  
  // There is no way here to change the CS polarity.
  SPI_REG(SPI_REG_CSDEF)   = (uint8_t)0xFF;
  SPI_REG(SPI_REG_CSDEF+1) = (uint8_t)0xFF;
	
	
	//Send address
	SPI_REG(SPI_REG_TXFIFO) = address;
	wait_for_txfifo();
	response = get_for_rxfifo();
	SPI_REG(SPI_REG_TXFIFO) = value;
	wait_for_txfifo();
	get_for_rxfifo();
	
	//Assert CS
  LED_REG(0) = (1 << ss) | (1 << rst); //digitalWrite(ss, HIGH) //TODO: Them vao day
  SPI_REG(SPI_REG_CSMODE) = (uint8_t)SPI_CSMODE_AUTO;
	
	
	return response;
}

uint8_t readRegister(uint8_t address) {
	return singleTransfer(address & 0x7F, 0x00);
}

void writeRegister(uint8_t address, uint8_t value) {
	singleTransfer(address | 0x80, value);
}



////////////////////////////////////////////



/*Setup functions*/

void LoRa_setSyncWord(uint8_t sw) {
	writeRegister(REG_SYNC_WORD, sw);
}

void LoRa_setPower(int level){
	writeRegister(REG_PA_DAC, 0x84);
	
	uint8_t ocpTrim =27;
	ocpTrim = (100-45)/5;
	writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
	
	writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void LoRa_setFrequency(long frequency)
{
  //_frequency = frequency;

  unsigned long int frf = 0;// ((uint64_t)frequency << 19) / 32000000; // TODO: Change it back

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/*----------------------Thay doi theo Sifive SPI---------------------*/
void spi_begin() {
	//Clock divisor default
	SPI_REG(SPI_REG_SCKDIV) = (uint8_t)0x03; // input clock divide by 8
	
	//SPI MODE0 by default
	SPI_REG(SPI_REG_SCKMODE) = (uint8_t)0x00; // SPI MODE 0, pol = 0, pha = 0
	
	//encode the index of the CS pin to be toggle
	SPI_REG(SPI_REG_CSID) = (uint8_t)0x123; //--------------SHOULD CHANGE THIS WITH THE ID OF CS PIN
	
	//set inactive state of the CS pins by default
	SPI_REG(SPI_REG_CSDEF) = (uint8_t)0x01; //HIGH

	//CS mode
	SPI_REG(SPI_REG_CSMODE) = (uint8_t)SPI_CSMODE_AUTO;
	
	//set delay0 by default
	SPI_REG(SPI_REG_DELAY0) = (uint8_t)0x1; // CS to SCK delay 0x01 (half-period delay)
	SPI_REG(SPI_REG_DELAY0 + 0x2) = (uint8_t)0x1; // SCK to CS delay 0x01 (half-period delay)
	
	//set delay1 by defautl
	SPI_REG(SPI_REG_DELAY1) = (uint8_t)0x01; // Minimum CS inactive time
	SPI_REG(SPI_REG_DELAY1 + 0x2) = (uint8_t)0x00; // Maximum interframe delay
	
	//set frame format by default
	SPI_REG(SPI_REG_FMT) = (uint8_t)0x08; // SPI protocol: single - SPI endianess: MSB - SPI I/O direction: TX
	SPI_REG(SPI_REG_FMT + 0x2) = (uint8_t)0x08; // Number of bits per frame: 8

	//set transmit watermark by default
	//SPI_REG(SPI_REG_TXCTRL) = (uint8_t)0x1; // Transmit watermark: 1
	//SPI_REG(SPI_REG_RXCTRL) = (uint8_t)0x0; // TReceive watermark: 0 

	//SPI interrupt enable
	//SPI_REG(SPI_REG_IE) = (uint8_t)0x3; // Transmit and Receive watermark enable
	//Polling for this bit in the SPI_REG_IP register - raised when the number of f entrSPI_REG_IEs in the transmit FIFO is strictly less
	//than the count specifSPI_REG_IEd by the SPI_REG_TXCTRL register. The pending bit is cleared when sufficSPI_REG_IEnt entrSPI_REG_IEs
	//have been enqueued to exceed the watermark 
	
	//exit specialized memory-mapped SPI flash mode
	//SPI_REG(SPI_REG_FCTRL) = (uint8_t)0x0;
}
int LoRa_begin(long frequency) {
	
	/*----------------------Thay doi pin cua Open8---------------------*/
  // pinMode (ss, OUTPUT) // Not necessary. LED always output
  
  LED_REG(0) = (1 << ss) | (0 << rst); //digitalWrite(ss, HIGH)
                                       //digitalWrite(rst, LOW)

  //udelay(10000); //TODO: delay 10ms, uncomment this
  
  LED_REG(0) = (1 << ss) | (1 << rst); //digitalWrite(rst, HIGH)
	
  //udelay(10000); //TODO: delay 10ms, uncomment this
	/*-----------------------------------------------------------------*/
	
	
	// Setup for spi should be here
	spi_begin();
	
	//check version
	uint8_t version = readRegister(REG_VERSION);
	if (version != 0x12) {
		return 0;
	}
	
	//put in sleep mode
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);

	// set frequency
	LoRa_setFrequency(frequency);

	// set base addresses
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

	// set LNA boost
	writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

	// set auto AGC
	writeRegister(REG_MODEM_CONFIG_3, 0x04);

	// set output power to 17 dBm
	LoRa_setPower(17);

	// put in standby mode
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	
	return 1;
}


//----------------------------------------Transmit Functions--------------------------------------//
uint8_t write(const uint8_t *buffer, uint8_t size) {

	int currentLength = readRegister(REG_PAYLOAD_LENGTH);
			
	// write data to FIFO
	for (uint8_t i = 0; i < size; i++) {
		writeRegister(REG_FIFO, buffer[i]);
	}
	
	// update length
	writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
	return size;
}

uint8_t LoRa_write(char* str) {
	return write((const uint8_t *) str, strlen(str));
}

uint8_t LoRa_write_int(int val) {
  #define digits 4
	for (int i = (4*digits)-4; i >= 0; i -= 4) {
	  char dig = "0123456789ABCDEF"[(val >> i) % 16];
	  write((const uint8_t *)&dig, 1);
	}
  return digits;
  #undef digits
}

int isTransmitting() {
	if((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
		return 1;
	}
	if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 0;
}


int LoRa_beginPacket(){
	
	if (isTransmitting()) {
		return 0;
	}
  
	// put in standby mode
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

	// reset FIFO address and paload length
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	writeRegister(REG_PAYLOAD_LENGTH, 0);

	return 1;
}

int LoRa_endPacket() {
	
	//put in TX mode
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	
	//wait for TX done
	while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
		//yield(); //TODO: use delay instead
		udelay(10000);
	}
	
	//clear IRQ
	writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	
	return 1;
}

//----------------------------------------Receive Functions--------------------------------------//

/*Receiver functions*/

int LoRa_available()
{
  return (readRegister(REG_RX_NB_BYTES) - packetIndex);
}

int LoRa_read()
{
  if (!LoRa_available()) {
    return -1;
  }

  packetIndex++;

  return readRegister(REG_FIFO);
}


int LoRa_parsePacket() {
	int packetLength = 0;
	int irqFlags = readRegister(REG_IRQ_FLAGS);
	//explicitHeaderMode();
	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
	// clear IRQ's
	writeRegister(REG_IRQ_FLAGS, irqFlags);
	
	if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		// received a packet
		packetIndex = 0;

		// read packet length
		packetLength = readRegister(REG_RX_NB_BYTES);

		// set FIFO address to current RX address
		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

		// put in standby mode
		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	} else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
		// not currently in RX mode

		// reset FIFO address
		writeRegister(REG_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}
	
	return packetLength;
}

int main()
{
	
	print_string("LoRa DEMO\n");
	
	LoRa_begin(433E6); //TODO: should create a while loop to verify the connection to LoRa
	
	LoRa_setSyncWord(0xF3);
	print_string("Setup LoRa DONE \n");
	int counter = 0;
	
	#ifdef TRANSMIT
	while(1) {
		print_string("Sending packet \n");
		LoRa_beginPacket();
		LoRa_write("Hello");
		LoRa_write_int(counter);
		LoRa_endPacket();
		counter = counter + 1;
		
		//perform delay 1000ms - sendding packet every second
		udelay(1000000);
	}
	#else
	while(1) {
		int packetSize = LoRa.parsePacket();
		if (packetSize) {
		// received a packet
		print_string("Received packet: ");
		// read packet
		while (LoRa.available()) {
		  char* LoRaData = LoRa_read();
		  print_string(LoRaData); 
		}
	}			
	#endif //TRANSMIT
	
	return 0;
}
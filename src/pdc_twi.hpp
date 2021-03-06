/******************************************************************************
* PDC: Peripheral DMA Controller, while DMA stands for Direct Memory Access
* TWI: Two Wire Interface, same as I2C as far as our regards.
* Author: Ewing Kang
* Contact: f039281310@yahoo.com.tw
* References:
*   [I2C DMA] https://forum.arduino.cc/index.php?topic=605127.0
*   [I2C + DMA] https://forum.arduino.cc/index.php?topic=152643.0
*   [Interrupt misconception on Due (ATSAM3X)]
* 		https://forum.arduino.cc/index.php?topic=621506.0
*   [Hanging I2C on DUE, SDA Low SCL High permanent]
* 		https://forum.arduino.cc/index.php?topic=288573.0
* Liscense:
* Note:
  Tested with Arduino Due with Atmel SAM3X8E ARM Cortex-M3 CPU only
  For future multi-device suppor on the same Arduino Due:
  Definition of arduino interface  macro, in 
  .../arduino/hardware/sam/1.6.12/variants/arduino_due_x/variant.h
      #define WIRE_INTERFACE   TWI1   <- I'm using this default one.
      #define WIRE1_INTERFACE  TWI0   <0 @ 7pin 0 (SDA1), 71 (SCL1)
******************************************************************************/
#ifdef TwoWire_h
#error PDC_TWI is likely to conflict with Wire.h library
#endif 

#ifndef ARDUINO_ARCH_SAM
  #error “This library only supports boards with a SAM processor.”
#endif

#ifndef PDC_TWI_HPP_
#define PDC_TWI_HPP_

#include <Arduino.h>

//#define TWI_CLOCK    100000 	// 100K Hz
#define TWI_CLOCK    400000 	// 400K Hz

class PdcTwi {
public:
	enum class PdcTwoWireStatus {
		PDC_UNINIT,
		PDC_STDBY,
		PDC_SINGLE_TX,
		PDC_SINGLE_RX,
		PDC_MULTI_TX,
		PDC_MULTI_RX,
		PDC_TX_SUCCESS,
		PDC_RX_SUCCESS,
		PDC_TX_FAILED,
		PDC_RX_FAILED,
		PDC_UNKOWN_FAILED
	};
	/*
	const char* pdc_status [] = {
		"UNINIT",
		"STDBY",
		"SINGLE_TX",
		"SINGLE_RX",
		"MULTI_TX",
		"MULTI_RX",
		"TX_SUCCESS",
		"RX_SUCCESS",
		"TX_FAILED",
		"RX_FAILED",
		"UNKOWN_FAILED"
	};*/
	
	// Constructor
	PdcTwi()
	{
		_comm_st = PdcTwoWireStatus::PDC_UNINIT;
	}
		
	/****************************************************************
							Init, Reset, End
		Init(): return false if the bus is already initialized
		A hard reset.
	*****************************************************************/
	bool Init() 
	{
		if(_comm_st != PdcTwoWireStatus::PDC_UNINIT)
		{
			return false;
		}
		BusReset();
		pmc_enable_periph_clk(ID_TWI1);
		PIO_Configure(
					   g_APinDescription[PIN_WIRE_SDA].pPort,
					   g_APinDescription[PIN_WIRE_SDA].ulPinType,
					   g_APinDescription[PIN_WIRE_SDA].ulPin,
					   g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
		PIO_Configure(
					   g_APinDescription[PIN_WIRE_SCL].pPort,
					   g_APinDescription[PIN_WIRE_SCL].ulPinType,
					   g_APinDescription[PIN_WIRE_SCL].ulPin,
					   g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

		// setup vector interrupt controller
		NVIC_DisableIRQ(TWI1_IRQn);
		NVIC_ClearPendingIRQ(TWI1_IRQn);
		NVIC_SetPriority(TWI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 3));
		NVIC_EnableIRQ(TWI1_IRQn);
		
		TWI1->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;	// Disable PDC channel
		TWI_ConfigureMaster(TWI1, TWI_CLOCK, VARIANT_MCK);	// set to master mode
		TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
		
		_comm_st = PdcTwoWireStatus::PDC_STDBY;
	}
	
	//This function will blocked for ~15ms
	void Reset()
	{
		End();		//Disable TWI functions, blocked for 12ms
		
		BusReset();
		_comm_st = PdcTwoWireStatus::PDC_UNINIT;
		
		Init();
	}
		
	void End()
	{
		TWI1->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDCs
		TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
		
		NVIC_DisableIRQ(TWI1_IRQn);			// Disable interrupt control
		NVIC_ClearPendingIRQ(TWI1_IRQn);
		
		TWI_Disable(TWI1);		// This function will block for 10ms
		BusReset();
		_comm_st = PdcTwoWireStatus::PDC_UNINIT;
	}

	/****************************************************************
						Public read/write access
		[MMR] Master Mode register (p.719)
		[IADR] Internal ADdress Register (p.713) (p.716)
	*****************************************************************/
	void SendTo(uint8_t dev_addr, uint8_t *data_ptr, uint16_t len) 
	{
		if(len == 0) return;
		if( len == 1) {
			_comm_st = PdcTwoWireStatus::PDC_SINGLE_TX;	
			
			// P.719 Figure 33-15
			TWI1->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
			TWI1->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE;
			TWI1->TWI_THR = (*data_ptr);		// start the transmission
			TWI1->TWI_IER = TWI_IER_TXCOMP| TWI_IER_NACK| TWI_IER_TXRDY;
			TWI1->TWI_CR = TWI_CR_STOP;		// set stop for one byte
						
			// interrupt enable register, this *needs* to be set AFTER THR
			//TWI1->TWI_IER = TWI_IER_TXCOMP | TWI_IER_NACK;	
			return;
		}else {
			//p.718 start PDC procedure
			SetPdcWriteAddr(data_ptr, len);
			SetTwiMasterWrite(dev_addr);
			_comm_st = PdcTwoWireStatus::PDC_MULTI_TX;
			TWI1->TWI_PTCR = TWI_PTCR_TXTEN;					// enable TX
			TWI1->TWI_IER = TWI_IER_ENDTX | TWI_IER_NACK ;	// interrupt enable register
		}
	}

	void RecieveFrom(uint8_t dev_addr, uint8_t *data_ptr, uint16_t len)
	{
		if(len == 0) return;
		_rx_stop_set = false;
		if( len == 1) {
			// p.722 & p.715 procedure
			_rx_ptr = data_ptr;
			rx_ptr_rdy = false;
			_comm_st = PdcTwoWireStatus::PDC_SINGLE_RX;
			TWI1->TWI_CR = 0;
			TWI1->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
			TWI1->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD; // master read
			TWI1->TWI_CR = TWI_CR_START | TWI_CR_STOP;
			TWI1->TWI_IER = TWI_IER_RXRDY | TWI_IER_TXCOMP;
		}else {
			SetPdcReadAddr(data_ptr, len);
			SetTwiMasterRead(dev_addr);
			_comm_st = PdcTwoWireStatus::PDC_MULTI_RX;
			TWI1->TWI_PTCR = TWI_PTCR_RXTEN;
			TWI1->TWI_CR = TWI_CR_START;
			TWI1->TWI_IER = TWI_IER_RXRDY | TWI_IER_NACK;
		}
	}
	
	// I2C write operation with 1-Byte internal address
	// Future: multibyte, be very notice on byte order
	// Flow chart: p.720/721
	void WriteTo(const uint8_t dev_addr, const uint8_t reg_addr, uint8_t *data_ptr, uint16_t len) 
	{
		if(len == 0) return;
		
		SetTwiMasterWriteTo(dev_addr, reg_addr);
		if( len == 1) {
			_comm_st = PdcTwoWireStatus::PDC_SINGLE_TX;	
			
			// load transmit register
			TWI1->TWI_THR = (*data_ptr);		
			
			TWI1->TWI_IER = TWI_IER_TXCOMP| TWI_IER_NACK| TWI_IER_TXRDY;
			// write stop command
			TWI1->TWI_CR = TWI_CR_STOP;
			// procedure: TXRDY -> TXCOMP
			return;
		}else {
			_comm_st = PdcTwoWireStatus::PDC_MULTI_TX;
			//p.718 start PDC procedure
			SetPdcWriteAddr(data_ptr, len);
			// enable TX
			TWI1->TWI_PTCR = TWI_PTCR_TXTEN;
			// interrupt enable register			
			TWI1->TWI_IER = TWI_IER_ENDTX | TWI_IER_NACK ;
		}
	}
	
	// I2C read operation with 1-Byte internal address
	// Flow chart: p.723, figure33-12 @p.717
	void ReadFrom(uint8_t dev_addr, const uint8_t reg_addr, 
				  uint8_t *data_ptr, uint16_t len)
	{
		if(len == 0) return;
		_rx_stop_set = false;
		
		SetTwiMasterReadFrom(dev_addr, reg_addr);
		if( len == 1) {
			_rx_ptr = data_ptr;
			rx_ptr_rdy = false;
			_comm_st = PdcTwoWireStatus::PDC_SINGLE_RX;
			
			TWI1->TWI_CR = TWI_CR_START | TWI_CR_STOP;
			TWI1->TWI_IER = TWI_IER_RXRDY | TWI_IER_TXCOMP;
			//Procedure: RXRDY -> TXCOMP
		}else {
			SetPdcReadAddr(data_ptr, len);
			_comm_st = PdcTwoWireStatus::PDC_MULTI_RX;
			TWI1->TWI_PTCR = TWI_PTCR_RXTEN;
			TWI1->TWI_CR = TWI_CR_START;
			TWI1->TWI_IER = TWI_IER_RXRDY | TWI_IER_NACK;
		}
	}
	
	bool TxComplete() 
	{
		return _comm_st == PdcTwoWireStatus::PDC_TX_SUCCESS;
	}
	bool RxComplete() 
	{
		return _comm_st == PdcTwoWireStatus::PDC_RX_SUCCESS;
	}
	bool ResetStatus()
	{
		switch(_comm_st) {
			case(PdcTwoWireStatus::PDC_UNINIT):
			case(PdcTwoWireStatus::PDC_SINGLE_TX):
			case(PdcTwoWireStatus::PDC_SINGLE_RX):
			case(PdcTwoWireStatus::PDC_MULTI_TX):
			case(PdcTwoWireStatus::PDC_MULTI_RX):
				return false;
				break;
			case(PdcTwoWireStatus::PDC_STDBY):
			case(PdcTwoWireStatus::PDC_TX_SUCCESS):
			case(PdcTwoWireStatus::PDC_RX_SUCCESS):
			case(PdcTwoWireStatus::PDC_TX_FAILED):
			case(PdcTwoWireStatus::PDC_RX_FAILED):
			case(PdcTwoWireStatus::PDC_UNKOWN_FAILED):
				_comm_st = PdcTwoWireStatus::PDC_STDBY;
				return true;
				break;
		}
		return false; // uncatched case
	}
	

	/****************************************************************
							ISR handling
		Handles IRQ. Should be called in TWI1_Handler() in global 
		scope.
		Note:
	*****************************************************************/
	inline void IsrHandler() 
	{
		int sr = TWI1->TWI_SR;
		int rcr = TWI1->TWI_RCR; // Receive Counter Register
		//Serial.print("isr: ");
		//Serial.println(sr, BIN);
		
	  switch(_comm_st) {
			
		case(PdcTwoWireStatus::PDC_SINGLE_TX):  	//===== SINGLE transmitting =======//
			if (sr & TWI_SR_NACK)
			{
				// failed, no ack on I2C bus
				_comm_st = PdcTwoWireStatus::PDC_TX_FAILED;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;	// disable all interrupts
				Serial.println("txSglNAck");
				return;
			} else if( sr & TWI_SR_TXCOMP) 
			{
				_comm_st = PdcTwoWireStatus::PDC_TX_SUCCESS;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;	// disable all interrupts
				//Serial.println("txSglCmp");
				return;
			} else if( sr & TWI_SR_TXRDY) 
			{
				//TWI1->TWI_IER = TWI_IER_TXCOMP |  TWI_IER_NACK;
				//Serial.println("txSglRdy");
				return;
			} else 
			{
				_comm_st = PdcTwoWireStatus::PDC_TX_FAILED;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				Serial.print("txSglUkn: ");
				Serial.println(sr);
				return;
			}
		  break;
	
		case(PdcTwoWireStatus::PDC_SINGLE_RX):		//===== SINGLE recieving =====//
			if (sr & TWI_SR_NACK) 
			{
				// failed, no ack on I2C bus
				_comm_st = PdcTwoWireStatus::PDC_RX_FAILED;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				Serial.println("rxTxNAck");
				return;
			} 
			if( sr & TWI_SR_RXRDY) 
			{
				//_comm_st = PdcTwoWireStatus::PDC_RX_SUCCESS;
				//TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				(*_rx_ptr) = TWI1->TWI_RHR;
				rx_ptr_rdy = true;
				//Serial.println("rxSglData");
				return;
			}
			if( rx_ptr_rdy && (sr & TWI_SR_TXCOMP) )
			{
				_comm_st = PdcTwoWireStatus::PDC_RX_SUCCESS;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				(*_rx_ptr) = TWI1->TWI_RHR;
				rx_ptr_rdy = true;
				//Serial.println("rxSglComp");
				return;
			} else
			{
				_comm_st =  PdcTwoWireStatus::PDC_RX_FAILED;
				Serial.println("rxSglNotTxComp");
				return;
			}
		  break;

		case(PdcTwoWireStatus::PDC_MULTI_TX):		//===== MULTI transmitting =====//
			if (sr & TWI_SR_NACK)
			{							
				// No ack in addr, probably something is disconnected
				_comm_st = PdcTwoWireStatus::PDC_TX_FAILED;
				TWI1->TWI_CR = TWI_CR_STOP;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				TWI1->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				Serial.println("txNAck");
				return;
			}
			if( sr & TWI_SR_TXCOMP)
			{
				_comm_st = PdcTwoWireStatus::PDC_TX_SUCCESS;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				TWI1->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				//Serial.println("txComp");
				return;
			}
			if( sr & TWI_SR_ENDTX ) 
			{
				// p.744: The Receive Counter Register has reached 0 since the last write in TWI_RCR or TWI_RNCR.
				// p.507 [Transmit Transfer End] This flag is set when PERIPH_TCR register reaches zero and the last
				//       data has been written into peripheral THR. It is reset by writing a non zero value in 
				//       PERIPH_TCR or PERIPH_TNCR.
				TWI1->TWI_CR = TWI_CR_STOP;		// This is necessary or SCL will hang low
				TWI1->TWI_IDR = TWI_SR_ENDTX;		// Disable PDC end isr
				TWI1->TWI_IER = TWI_IER_TXCOMP;	// so it'll trigger comp next time for multi-byte write
				//Serial.println("txLast");
				return;	
			}
		  break;
		
		case(PdcTwoWireStatus::PDC_MULTI_RX):		//===== MULTI receiving =====//
			if( sr & TWI_SR_NACK ) 
			{	// no ack, when slave not response to initial address call
				_comm_st = PdcTwoWireStatus::PDC_RX_FAILED;
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				TWI1->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable pdc
				Serial.println("rxNAck");
				return;
			}
			
			//Serial.print("RCR: ");
			//Serial.println(rcr);
			if( sr & TWI_IER_ENDRX ) 
			{
				_comm_st = PdcTwoWireStatus::PDC_RX_SUCCESS;
				if( !_rx_stop_set)	TWI1->TWI_CR = TWI_CR_STOP;		// Stop case missed
				TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
				TWI1->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				//Serial.println("rxEnd");
				return;
			} else 
			{
				// This is triggered by RXRDY, but since the PDC has already taken the data, the SR
				// will show RXRDY as 0.
				// p.716 This should be fired when ever RHR is ready, i.e evertime a new data is ready.
				//       check if we're ending the RX
				// p.506 Receive Counter Register is counting down
				if( rcr <= 1) 
				{
					TWI1->TWI_CR = TWI_CR_STOP;
					_rx_stop_set = true;
					//Serial.println("rxLastOne");
				}
				return;
			}
		  break;
		default:	  break;
		}
		//============= Uncatched interrupt =============//
		Serial.print("Unknown ISR, SR: ");
		Serial.println(sr, BIN);
		Serial.print("status: ");
		Serial.print( (unsigned int)_comm_st );
		Serial.print(", PTSR: ");		// PDC Transfer Status Register
		Serial.println(TWI1->TWI_PTSR, BIN);
		
		TWI1->TWI_CR = TWI_CR_STOP;							// make sure things is stooping
		TWI1->TWI_IDR = TWI1 -> TWI_IMR;			// disable all interrupts
		TWI1->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
		_comm_st = PdcTwoWireStatus::PDC_UNKOWN_FAILED;
		return;
	}	// IsrHandler()
	
	
private:
	bool _rx_stop_set;
	volatile PdcTwoWireStatus _comm_st;  ///TODO: static is dirty 
	byte *_rx_ptr;
	bool rx_ptr_rdy;
	/****************************************************************
							Set PDC addresses
		Set relative address and counter into PDC register
		Note:
			[PTCR] Peripheral Transfer Control Register
			[TPR]/[RPR] Transmit/Receive Pointer Register
			[TCR]/[RCR] Transmit/Receive Counter Register
			[TNPR]/[RNPR] Transmit/Receive Next Pointer Register
			[TNCR]/[RNCR] Transmit Next Pointer Counter
	*****************************************************************/
	static inline int SetPdcWriteAddr(uint8_t *data, uint16_t count) 
	{
		TWI1->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
		TWI1->TWI_TPR = (RwReg)data;		
		TWI1->TWI_TCR = count;
		TWI1->TWI_TNPR = 0;		
		TWI1->TWI_TNCR = 0;				
	}
	static inline int SetPdcReadAddr(uint8_t *data, uint16_t count) 
	{
		TWI1->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
		TWI1->TWI_RPR = (RwReg)data;		
		TWI1->TWI_RCR = count;
		TWI1->TWI_RNPR = 0;
		TWI1->TWI_RNCR = 0;
	}
	
	/***************************************************************
							Set TWI control mode
		Note:
			MMR-Master Mode register
	****************************************************************/
	static inline int SetTwiMasterWrite(const uint8_t dev_addr) 
	{
		TWI1->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE;
		TWI1->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
	}
	static inline int SetTwiMasterRead(const uint8_t dev_addr) 
	{
		TWI1->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD; // master read
		TWI1->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
	}
	
	static inline int SetTwiMasterWriteTo(const uint8_t dev_addr, const uint8_t reg_addr) 
	{
		TWI1->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_1_BYTE;
		TWI1->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
		TWI1->TWI_IADR = TWI_IADR_IADR(reg_addr);
		
	}
	static inline int SetTwiMasterReadFrom(const uint8_t dev_addr, const uint8_t reg_addr) 
	{
		TWI1->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD; // master read
		TWI1->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
		TWI1->TWI_IADR = TWI_IADR_IADR(reg_addr);
	}
	
	
	/***************************************************************
	  manual reset by BusReset() to clear slave pull down issue.
	****************************************************************/
	void BusReset()
	{
		// Sent 9 pulses over CLK pin, should free-up any slave SDA hangs
		// I2C 100K Hz is 10us per clock cycle. The delay value gives ~84us/8 clk cycle
		pinMode(21, OUTPUT);
		for (int i = 0; i < 9; i++) {
			
			digitalWrite(21, HIGH);
			delayMicroseconds(3);	
			digitalWrite(21, LOW);
			delayMicroseconds(3);
		}
		pinMode(21, INPUT);
		pinMode(21, INPUT_PULLUP);
	}
	
};	//class PdcTwi

#endif
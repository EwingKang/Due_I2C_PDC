/******************************************************************************
 * TODO
 *  Contact: f039281310@yahoo.com.tw
 * References:
 *   [TurboSPI by AndyDream](https://github.com/anydream/TurboSPI)
 * Liscense:
 * Note:
 *   DMAC is defined in sam3x8e.h
 ******************************************************************************/
#ifdef _SPI_H_INCLUDED
#error This library conflicts with SPI.h library
#endif 

#if !defined(__SAM3X8E__) && !defined(__SAM3X8H__)
  #error “This library only supports boards with a SAM processor.”
#endif

#ifndef dma_spi_hpp__
#define dma_spi_hpp__

#include <Arduino.h>
//#include <sam3x8e.h>

#define SPI_DMAC_RX_CH  1 // DMAC receive channel
#define SPI_DMAC_TX_CH  0 // DMAC transmit channel

#define SPI_CHIP_SEL 0 // pin 10
//#define SPI_CHIP_SEL 1 // pin 4

//Table 22-2. DMA Controller
#define SPI_TX_IDX  1  // DMAC Channel HW Interface Number for SPI TX.
#define SPI_RX_IDX  2  // DMAC Channel HW Interface Number for SPI RX.

class DmaSpi
{
public:
	enum class SpiDmaStatus {
		UNINIT,
		STDBY,
		TXING,
		RXADDRTX,
		RXING,
		TX_SUCCESS,
		RX_SUCCESS,
		TX_FAILED,
		RX_FAILED,
		UNKOWN_FAILED
	};


	// Constructor
	DmaSpi()
	{
		_comm_st = SpiDmaStatus::UNINIT;
	}
		
	bool CommAvailable(const SpiDmaStatus st) const
	{
		return st == SpiDmaStatus::STDBY	| 
			   st == SpiDmaStatus::TX_SUCCESS |  st == SpiDmaStatus::RX_SUCCESS | 
		       st == SpiDmaStatus::TX_FAILED | st == SpiDmaStatus::RX_FAILED;
	}
	bool IsTransfering() const
	{
		volatile SpiDmaStatus st = _comm_st;
		return st == SpiDmaStatus::TXING | st == SpiDmaStatus::RXING |
			   st == SpiDmaStatus::RXADDRTX;
	}
	bool TransferDone()
	{
		volatile SpiDmaStatus st = _comm_st;
		//Serial.print("TR done? ");
		//Serial.println((int)st);
		volatile bool done = st == SpiDmaStatus::TX_SUCCESS | st == SpiDmaStatus::RX_SUCCESS;
		if(done)
		{
			_comm_st = SpiDmaStatus::STDBY;
		}
		return done;
	}
	

	/*********************************************************************
	 * Target baudrate. Actual baudrate is always no-larger-than the 
	 * target. It is an integer division of Master clock (Due=84MHz)
	 *
	 *********************************************************************/
	bool Init(uint32_t baudrate)
	{
		if(_comm_st != SpiDmaStatus::UNINIT)
		{
			return false;
		}
		uint8_t divisor = uint8_t(VARIANT_MCK/baudrate + (VARIANT_MCK%baudrate!=0) );
		SetIo();
		SetNvic();
		SetDmac();
		SetSpiRegister(divisor);
		_comm_st = SpiDmaStatus::STDBY;
		return true;
	}

	bool Send(const uint8_t * buf, size_t n)
	{
		if( !CommAvailable(_comm_st) ) return false;
		if( n>4095 ) return false;
		
		// TODO: Multi-buffer with Linked Address for Source and Destination Buffers are Contiguous
		//static const size_t DMABufSize = 4095;
		// while (n > DMABufSize)
		// {
			// SPI_DMA_Transfer(buf, DMABufSize);
			// while (!DMAC_ChannelTransferDone(SPI_DMAC_TX_CH))
				// ;
			// n -= DMABufSize;
			// buf += DMABufSize;
		// }
		
		_comm_st = SpiDmaStatus::TXING;
		SPI_DMA_Transfer(buf, n, true);
		return true;
	}
	
	bool SlowSend(const uint8_t * buf, size_t n)
	{
		if( !CommAvailable(_comm_st) ) return false;
		
		_comm_st = SpiDmaStatus::TXING;
		
		pSpi->SPI_CSR[SPI_CHIP_SEL] = pSpi->SPI_CSR[SPI_CHIP_SEL] | SPI_CSR_CSAAT;

		int i=0; 
		while( i<n )
		{
			uint32_t reg = SPI_TDR_TD(*(buf+i)) | SPI_TDR_PCS(SPI_CHIP_SEL);
			
			pSpi->SPI_TDR = reg;
			if( i==(n-1) ) {
				pSpi->SPI_CR = SPI_CR_LASTXFER;
			}
			
			while((pSpi->SPI_SR & SPI_SR_TXEMPTY) == 0) {} // Wait for TDR 
			i++;
		}
		// leave RDR empty
		uint8_t b = pSpi->SPI_RDR;
		_comm_st = SpiDmaStatus::TX_SUCCESS;
		
		// Disable CSAAT
		pSpi->SPI_CSR[SPI_CHIP_SEL] = pSpi->SPI_CSR[SPI_CHIP_SEL] & ~SPI_CSR_CSAAT;
		return true;
	}
	bool SlowRead(const uint8_t * addr, const size_t &len, const bool &incr, uint8_t * dest)
	{
		if( !CommAvailable(_comm_st) ) return false;
		if( len<1) return false;
		
		_comm_st = SpiDmaStatus::RXING;
		pSpi->SPI_CSR[SPI_CHIP_SEL] = pSpi->SPI_CSR[SPI_CHIP_SEL] | SPI_CSR_CSAAT;
		int i = 0;
		while( i<len )
		{
			uint16_t data = 0x00;
			if(incr || i==0)
			{
				data = *(addr+i);
			}
			pSpi->SPI_TDR = SPI_TDR_TD(data) | SPI_TDR_PCS(SPI_CHIP_SEL);
			if( i==(len-1) ) {
				pSpi->SPI_CR = SPI_CR_LASTXFER;
			}
			while((pSpi->SPI_SR & SPI_SR_RDRF) == 0) {} // Wait for RDRF
			uint32_t rdata = pSpi->SPI_RDR;
			*(dest+i) = rdata;
			i++;
		}
		// Disable CSAAT
		pSpi->SPI_CSR[SPI_CHIP_SEL] = pSpi->SPI_CSR[SPI_CHIP_SEL] & ~SPI_CSR_CSAAT;
		_comm_st = SpiDmaStatus::RX_SUCCESS;
		return true;
	}
	
	/*********************************************************************
	 * Target baudrate. Actual baudrate is always no-larger-than the 
	 * target. It is an integer division of Master clock (Due=84MHz)
	 * incr: If true, the length of [addr] must match [len]. During the 
	 *       SPI transaction, everything from addr to addr+len is 
	 *       transferred to the device.
	 *       If false, only the first byte pointed by the addr is 
	 *       transferred. The transaction continue by sending 0x00 
	 *       len-1 times.
	 *********************************************************************/
	bool Recv(const uint8_t * addr, const size_t &len, const bool &incr, uint8_t * dest)
	{
		if( !CommAvailable(_comm_st) ) return false;
		if( len>4095 || len<1) return false;
		
		// TODO: Multi-buffer with Linked Address for Source and Destination Buffers are Contiguous
		//static const size_t DMABufSize = 4095;
		// while (n > DMABufSize)
		// {
			// SPI_DMA_Transfer(buf, DMABufSize);
			// while (!DMAC_ChannelTransferDone(SPI_DMAC_TX_CH))
				// ;
			// n -= DMABufSize;
			// buf += DMABufSize;
		// }
		
		uint8_t b = pSpi->SPI_RDR;  // Clear RDR
		if(incr)
		{
			_comm_st = SpiDmaStatus::RXING;
			SPI_DMA_Receive(dest, len, true);
			SPI_DMA_Transfer(addr, len, false);
		}
		else
		{
			// For Receive, it is more complicate. We have to first write an target address, 
			// than burst receive the length we want, with consecutive 0x00 as TX payload.
			// This operation requires mode change and will take longer to tranfer (~500 ns)
			_comm_st = SpiDmaStatus::RXADDRTX;
			
			// Enable CSAAT in SPI so the CS (chip select) won't rise in-between mode change,
			// which occures in interrupt service routine.
			pSpi->SPI_CSR[SPI_CHIP_SEL] = pSpi->SPI_CSR[SPI_CHIP_SEL] | SPI_CSR_CSAAT;
			
			_rx_len = len;
			SPI_DMA_Receive(dest, len, false);
			SPI_DMA_Transfer(addr, 1, true);
		}
		return true;
	}
	
	bool TxDone()
	{
		return DMAC_ChannelTransferDone(SPI_DMAC_TX_CH) && (pSpi->SPI_SR & SPI_SR_TXEMPTY);
	}

	/****************************************************************
	 *							ISR handling
	 * IRQ Handles for DMAC_Handler() and SPI0_Handler() respectively
	 *  
	 * Note:
	 ****************************************************************/
	bool DmacIsrHandler()
	{
		int spi_sr = pSpi->SPI_SR;
		if( !DMAC_ChannelTransferDone(SPI_DMAC_TX_CH) )
		{
			Serial.println("[DMAC-ISR]ERR: not done");
			return false;
		}
		if( spi_sr & SPI_SR_TXEMPTY == 0)
		{
			Serial.println("[DMAC-ISR]ERR: not empty");
			return false;
		}
		
		int ebcisr;
		switch (_comm_st)
		{
			case(SpiDmaStatus::TXING):
				//Serial.println("[DMAC-ISR]TX suc");
				DMAC->DMAC_EBCIDR = DMAC->DMAC_EBCIMR; // Disable interrupts
				_comm_st = SpiDmaStatus::TX_SUCCESS;
				break;
			case(SpiDmaStatus::RXADDRTX):
				//Serial.println("[DMAC-ISR]RXADDRTX");
				SPI_DMA_Transfer(nullptr, _rx_len-1, false);
				// Disable CSAAT so the CS (pin select) can rise
				pSpi->SPI_CSR[SPI_CHIP_SEL] = pSpi->SPI_CSR[SPI_CHIP_SEL] & ~SPI_CSR_CSAAT;
				SetupRxInt( SPI_DMAC_RX_CH );  // Enable RX interrupt
				_comm_st = SpiDmaStatus::RXING;
				break;
			case(SpiDmaStatus::RXING):
				//Serial.println("[DMAC-ISR]RX suc");
				DMAC->DMAC_EBCIDR = DMAC->DMAC_EBCIMR; 	// Disable interrupts
				_comm_st = SpiDmaStatus::RX_SUCCESS;
				break;
			default:
				Serial.print("[DMAC-ISR]ERR: unknown status: ");
				Serial.println((int)_comm_st);
				return false;
		}
		return true;
	}
	bool SpiIsrHandler()
	{
		int spi_sr = pSpi->SPI_SR;
		Serial.print("_comm_st: ");	Serial.println((int)_comm_st); //Debug
		if( !(spi_sr & SPI_SR_TDRE) )
		{
			Serial.println("[SPI-ISR]ERR: not ready");
			return false;
		}
		if(_comm_st != SpiDmaStatus::RXADDRTX && _comm_st != SpiDmaStatus::RXING)
		{
			Serial.print("[SPI-ISR]ERR: unknown status: ");
			Serial.println((int)_comm_st);
			return false;
		}
		pSpi->SPI_IDR = pSpi->SPI_IMR; // Disable interrupts
		_comm_st = SpiDmaStatus::RXING;
		
		// Start Dmac receive
		//DMAC->DMAC_EBCIDR = DMAC->DMAC_EBCIMR;	// Disable interrupts
		//DMAC_ChannelEnable(SPI_DMAC_RX_CH);
		// Start Dmac Transmit empty data
		StartDmacTx(true);
		
		return true;
	}

	void (DmaSpi::*pisr)();
private:
	volatile SpiDmaStatus _comm_st;
	Spi* pSpi = SPI0;
	
	size_t _rx_len;

	/* Disable DMA Controller. */
	static void DMAC_Disable()
	{
		DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
	}
	/* Enable DMA Controller. */
	static void DMAC_Enable()
	{
		DMAC->DMAC_EN = DMAC_EN_ENABLE;
	}
	/* Disable DMA Channel. */
	static void DMAC_ChannelDisable(uint32_t ul_num)
	{
		DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
	}
	/* Enable DMA Channel. */
	static void DMAC_ChannelEnable(uint32_t ul_num)
	{
		DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
	}
	/* Poll for transfer complete. */
	static bool DMAC_ChannelTransferDone(uint32_t ul_num)
	{
		return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;
	}


	void SetIo()
	{
		PIO_Configure(
			g_APinDescription[PIN_SPI_MOSI].pPort,
			g_APinDescription[PIN_SPI_MOSI].ulPinType,
			g_APinDescription[PIN_SPI_MOSI].ulPin,
			g_APinDescription[PIN_SPI_MOSI].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PIN_SPI_MISO].pPort,
			g_APinDescription[PIN_SPI_MISO].ulPinType,
			g_APinDescription[PIN_SPI_MISO].ulPin,
			g_APinDescription[PIN_SPI_MISO].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PIN_SPI_SCK].pPort,
			g_APinDescription[PIN_SPI_SCK].ulPinType,
			g_APinDescription[PIN_SPI_SCK].ulPin,
			g_APinDescription[PIN_SPI_SCK].ulPinConfiguration);
			
		// Chip select pins
		PIO_Configure(
			g_APinDescription[PIN_SPI_SS0].pPort,
			g_APinDescription[PIN_SPI_SS0].ulPinType,
			g_APinDescription[PIN_SPI_SS0].ulPin,
			g_APinDescription[PIN_SPI_SS0].ulPinConfiguration);		
		PIO_Configure(
			g_APinDescription[PIN_SPI_SS1].pPort,
			g_APinDescription[PIN_SPI_SS1].ulPinType,
			g_APinDescription[PIN_SPI_SS1].ulPin,
			g_APinDescription[PIN_SPI_SS1].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PIN_SPI_SS2].pPort,
			g_APinDescription[PIN_SPI_SS2].ulPinType,
			g_APinDescription[PIN_SPI_SS2].ulPin,
			g_APinDescription[PIN_SPI_SS2].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PIN_SPI_SS3].pPort,
			g_APinDescription[PIN_SPI_SS3].ulPinType,
			g_APinDescription[PIN_SPI_SS3].ulPin,
			g_APinDescription[PIN_SPI_SS3].ulPinConfiguration);
			
		pmc_enable_periph_clk(ID_SPI0);
		pmc_enable_periph_clk(ID_DMAC);
	}
	void SetNvic()
	{
		// setup vector interrupt controller
		NVIC_DisableIRQ(SPI0_IRQn);
		NVIC_ClearPendingIRQ(SPI0_IRQn);
		NVIC_SetPriority(SPI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 1));
		NVIC_EnableIRQ(SPI0_IRQn);
		
		NVIC_DisableIRQ(DMAC_IRQn);
		NVIC_ClearPendingIRQ(DMAC_IRQn);
		NVIC_SetPriority(DMAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 2));
		NVIC_EnableIRQ(DMAC_IRQn);
	}
	void SetDmac()
	{
		DMAC_Disable();
		DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
		DMAC_Enable();
	}
	/******************************************************************************
	 * Initialize SPI controller
	 * SCBR = Serial Clock Baud Rate, where SPCK Baudrate = MCK/SCBR, 
	 * default VARIANT_MCK = 84,000,000
	 ******************************************************************************/
	void SetSpiRegister(uint8_t sckDivisor)
	{
		uint8_t scbr = sckDivisor;
		Spi* pSpi = SPI0;
		//  disable SPI
		pSpi->SPI_CR = SPI_CR_SPIDIS;
		// reset SPI
		pSpi->SPI_CR = SPI_CR_SWRST;
		
		// master mode, Fixed Peripheral Select, no chip select decode,
		// no mode fault detection, WDRBT=0, LLB=0
		// set Chip select  (without decode), no DLYBCS
		pSpi->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS | 
					   SPI_PCS(SPI_CHIP_SEL) ; // | SPI_MR_DLYBCS(84);
		// mode 0, 8-bit, delay
		const int delay_ns = 500;
		pSpi->SPI_CSR[SPI_CHIP_SEL] = SPI_CSR_SCBR(scbr) | SPI_CSR_NCPHA | 
									//SPI_CSR_CSNAAT |
									//SPI_CSR_CSAAT |
									SPI_DLYBS(delay_ns, VARIANT_MCK) | SPI_DLYBCT(delay_ns, VARIANT_MCK);
		// disable all interrupts
		pSpi->SPI_IDR = pSpi->SPI_IMR;
		
		// enable SPI
		pSpi->SPI_CR |= SPI_CR_SPIEN;
	}
	
	/******************************************************************************
	 * start TX DMA
	 * Set src to null makes the function send FF for count times
	 ******************************************************************************/
	static void SetupDmacTx(const int32_t & dmac_ch, const uint8_t * src, uint16_t count)
	{
		static uint8_t ff = 0X00;
		uint32_t src_incr = DMAC_CTRLB_SRC_INCR_INCREMENTING;
		if (!src)
		{
			src = &ff;
			src_incr = DMAC_CTRLB_SRC_INCR_FIXED;
		}
		DMAC_ChannelDisable(dmac_ch);
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_SADDR = (uint32_t)src;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_DADDR = (uint32_t)&SPI0->SPI_TDR;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_DSCR = 0;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_CTRLA = count |
					DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR |
					DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC |
					src_incr | DMAC_CTRLB_DST_INCR_FIXED;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_CFG = DMAC_CFG_DST_PER(SPI_TX_IDX) |
					DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ALAP_CFG;
	}
	static void StartDmacTx(const bool en_interrupt)
	{
		if(en_interrupt) 	// Enable buffer tx complete interrupt
		{
			DMAC->DMAC_EBCIDR = DMAC->DMAC_EBCIMR;			// Disable interrupts
			int ebcisr = DMAC->DMAC_EBCISR;  				// Clear pending interrupts
			DMAC->DMAC_EBCIER = DMAC_EBCIER_BTC0  << SPI_DMAC_TX_CH;
		}
		
		DMAC_ChannelEnable(SPI_DMAC_TX_CH);
	}
	static void SPI_DMA_Transfer(const uint8_t * src, uint16_t count, const bool en_interrupt)
	{
		SetupDmacTx(SPI_DMAC_TX_CH, src, count);
		StartDmacTx(en_interrupt);
	}
	/******************************************************************************
	 * start RX DMA
	 ******************************************************************************/
	static void SetupDmacRx(const int32_t & dmac_ch, uint8_t * dst, uint16_t count)
	{
		DMAC_ChannelDisable(dmac_ch);
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_SADDR = (uint32_t)&SPI0->SPI_RDR;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_DADDR = (uint32_t)dst;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_DSCR = 0;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_CTRLA = count |
					DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR |
					DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC |
					DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING;
		DMAC->DMAC_CH_NUM[dmac_ch].DMAC_CFG = DMAC_CFG_SRC_PER(SPI_RX_IDX) |
					DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG;
	}
	static void SetupRxInt(const int32_t & dmac_ch)
	{
		DMAC->DMAC_EBCIDR = DMAC->DMAC_EBCIMR;	// Disable interrupts
		int ebcisr = DMAC->DMAC_EBCISR;  		// Clear pending interrupts
		DMAC->DMAC_EBCIER = DMAC_EBCIER_BTC0 << dmac_ch;  // Enable interrupt
	}
	static void StartDmacRx(const bool en_interrupt)
	{
		if(en_interrupt)
		{
			SetupRxInt(SPI_DMAC_RX_CH);
		}
		DMAC_ChannelEnable(SPI_DMAC_RX_CH);
	}
	static void SPI_DMA_Receive(uint8_t * dst, uint16_t count, const bool en_interrupt)
	{
		SetupDmacRx(SPI_DMAC_RX_CH, dst, count);	
		StartDmacRx(en_interrupt);
	}

}; // class DmaSpi
#endif


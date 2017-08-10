#include "stm32f0xx.h"

#define BSRR_VAL1 0x0300

#define SPIx                             SPI1
#define SPIx_CLK                         RCC_APB2Periph_SPI1
#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler

#define SPIx_NSS_PIN                     GPIO_Pin_4
#define SPIx_NSS_GPIO_PORT               GPIOA
#define SPIx_NSS_GPIO_CLK                RCC_AHBPeriph_GPIOA
#define SPIx_NSS_SOURCE                  GPIO_PinSource4
#define SPIx_NSS_AF                      GPIO_AF_0


#define SPIx_SCK_PIN                     GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_GPIO_CLK                RCC_AHBPeriph_GPIOA
#define SPIx_SCK_SOURCE                  GPIO_PinSource5
#define SPIx_SCK_AF                      GPIO_AF_0

#define SPIx_MISO_PIN                    GPIO_Pin_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define SPIx_MISO_SOURCE                 GPIO_PinSource6
#define SPIx_MISO_AF                     GPIO_AF_0

#define SPIx_MOSI_PIN                    GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define SPIx_MOSI_SOURCE                 GPIO_PinSource7
#define SPIx_MOSI_AF                     GPIO_AF_0

#define EVAL_COM1                        USART2
#define EVAL_COM1_CLK                    RCC_APB1Periph_USART2

#define EVAL_COM1_TX_PIN                 GPIO_Pin_2
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define EVAL_COM1_TX_SOURCE              GPIO_PinSource2
#define EVAL_COM1_TX_AF                  GPIO_AF_1

#define EVAL_COM1_RX_PIN                 GPIO_Pin_3
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define EVAL_COM1_RX_SOURCE              GPIO_PinSource3
#define EVAL_COM1_RX_AF                  GPIO_AF_1

#define EVAL_COM1_CTS_PIN                GPIO_Pin_11
#define EVAL_COM1_CTS_GPIO_PORT          GPIOA
#define EVAL_COM1_CTS_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define EVAL_COM1_CTS_SOURCE             GPIO_PinSource11
#define EVAL_COM1_CTS_AF                 GPIO_AF_1

#define EVAL_COM1_RTS_PIN                GPIO_Pin_12
#define EVAL_COM1_RTS_GPIO_PORT          GPIOA
#define EVAL_COM1_RTS_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define EVAL_COM1_RTS_SOURCE             GPIO_PinSource12
#define EVAL_COM1_RTS_AF                 GPIO_AF_1
   
#define EVAL_COM1_IRQn                   USART2_IRQn

#define SPI_DATASIZE                     SPI_DataSize_16b
#define SPI_DATAMASK                     (uint8_t)0xFF
 
#define SYSTEMCLOCK_REG   	0x00
#define PLL1_REG 			      0x01
#define PLL2_REG 			      0x02
#define PLL3_REG 			      0x03
#define PLL4_REG		    	  0x04
#define PLL5_REG	    	  	0x05
#define PLL6_REG	    	  	0x06
#define CRYSTAL_REG		    	0x07
#define PAGEA_REG	      		0x08
#define PAGEB_REG		      	0x09
#define RX1_REG  	    	  	0x0A
#define RX2_REG     	  		0x0B
#define ADC_REG     		  	0x0C
#define PIN_REG 	    	  	0x0D
#define CALIBRATION_REG    	0x0E
#define MODE_REG  	    		0x0F

#define TX1_PAGEA           0x00
#define WOR1_PAGEA          0x01
#define WOR2_PAGEA          0x02
#define RFI_PAGEA           0x03
#define PM_PAGEA            0x04
#define RTH_PAGEA           0x05
#define AGC1_PAGEA          0x06
#define AGC2_PAGEA          0x07
#define GIO_PAGEA           0x08
#define CKO_PAGEA           0x09
#define VCB_PAGEA           0x0A
#define CHG1_PAGEA          0x0B
#define CHG2_PAGEA          0x0C
#define FIFO_PAGEA		    	0x0D
#define CODE_PAGEA	    		0x0E
#define WCAL_PAGEA	    		0x0F

#define TX2_PAGEB	      		0x00
#define	IF1_PAGEB	      		0x01
#define IF2_PAGEB	      		0x02
#define	ACK_PAGEB		      	0x03
#define	ART_PAGEB		      	0x04
#define SYN_PAGEB		      	0x05
#define ACKFIFO_PAGEB	    	0x06
#define DSSS_PAGEB			    0x07
#define CCM1_PAGEB			    0x08
#define CCM2_PAGEB			    0x09
#define CCM3_PAGEB			    0x0A
#define CCM4_PAGEB			    0x0B
#define CCM5_PAGEB			    0x0C
#define RF_Cmd_PAGEB		    0x0D
#define Btn_Cmd_PAGEB		    0x0E
#define DSSS2_PAGEB			    0x0F
#define DSSS3_PAGEB			    0x10
#define CCM6_PAGEB			    0x11
#define CDET_PAGEB			    0x12
#define DC_SHIFT_PAGEB		  0x13
#define RCOSC_PAGEB			    0x14


#define CMD_Reg_W			      0x00	//000x,xxxx	control register write
#define CMD_Reg_R			      0x80	//100x,xxxx	control register read
#define CMD_ID_W			      0x20	//001x,xxxx	ID write
#define CMD_ID_R			      0xA0	//101x,xxxx	ID Read
#define CMD_FIFO_W			    0x40	//010x,xxxx	TX FIFO Write
#define CMD_FIFO_R			    0xC0	//110x,xxxx	RX FIFO Read
#define CMD_RF_RST			    0xFF	//x111,xxxx	RF reset
#define CMD_TFR		      		0x60	//0110,xxxx	TX FIFO address pointrt reset
#define CMD_RFR				      0xE0	//1110,xxxx	RX FIFO address pointer reset

#define CMD_SLEEP			      0x10	//0001,0000	SLEEP mode
#define CMD_IDLE			      0x12	//0001,0010	IDLE mode
#define CMD_STBY			      0x14	//0001,0100	Standby mode
#define CMD_PLL				      0x16	//0001,0110	PLL mode
#define CMD_RX	      			0x18	//0001,1000	RX mode
#define CMD_TX				      0x1A	//0001,1010	TX mode
//#define CMD_DEEP_SLEEP	  0x1C	//0001,1100 Deep Sleep mode(tri-state)
#define CMD_DEEP_SLEEP		  0x1F	//0001,1111 Deep Sleep mode(pull-high)


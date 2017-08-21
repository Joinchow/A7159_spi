/*********************************************************************
 ** Device: A7159
 ** File: main.c
 ** Target: Winbond W77LE58
 ** Tools: ICE
 ** Updated: 2016-04-08
 ** Description:
 ** This file is a sample code for your reference.
 **
 ** Copyright (C) 2016 AMICCOM Corp.
 **
 *********************************************************************/
#include "main.h"


void SPI_Config(void);
void USART_Config(void);
void TimeOut_UserCallback(void);
void TimingDelay_Decrement(void);
void SysTick_Handler(void);
void Delay(__IO uint32_t nTime);
void A7159_Config(void);
void A7159_ReadReg(uint8_t address);
void A7159_WriteReg(uint8_t address ,uint16_t dataWord);
void A7159_WritePageA(uint8_t address, uint16_t dataWord);
void A7159_WritePageB(uint8_t address, uint16_t dataWord);
void A7159_WriteID(void);
void A7159_WriteFIFO(void);
void entry_deep_sleep_mode(void);
void wake_up_from_deep_sleep_mode(void);
void GPIO2_Config(void);
void GPIOA4_Config(void);
void SPICSOn(void);
void SPICSOff(void);
void Rxpacket(void);
uint8_t A7159_Cal(void);
uint8_t InitRF(void);
extern __IO uint16_t bb[];
extern __IO uint16_t RxBuffer[];
extern __IO uint16_t RxCnt;
extern __IO bool getSPI ;
SPI_InitTypeDef         SPI_InitStructure;
GPIO_InitTypeDef        GPIO_InitStructure;
static __IO uint32_t TimingDelay;
__IO uint8_t i = 0 ;
const uint16_t A7159Config[]=        //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x1221,     //SYSTEM CLOCK register,
    0x0A21,     //PLL1 register,
    0xDA05,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0A20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0015,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D4,     //RX1 register,     IFBW=100KHz
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t A7159Config_PageA[]=   //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706,     //TX1 register,     Fdev = 37.5kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1107,     //RFI register,     Enable Tx Ramp up/down  
    0x8170,     //PM register,      CST=1
    0x0706,     //RTH register,
    0x400F,     //AGC1 register,    
    0x2AC0,     //AGC2 register, 
    0x0059,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD981,     //CKO register
    0x0004,     //VCB register,
    0x1A21,     //CHG1 register,    430MHz
    0x0022,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x8000      //WCAL register,
};

const uint16_t A7159Config_PageB[]=   //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
  0x0337,	  //TX2 register, 	
	0x8400,		//IF1 register, 	Enable Auto-IF, IF=200KHz
	0x0000,		//IF2 register,
	0x0000,		//ACK register,
	0x0000,		//ART register,
	0x0000,		//SYN register,
	0x0000,		//ACKFIFO register,
	0x0000,		//DSSS register,
	0x0000,		//CCM1 register,
	0x0000,		//CCM2 register,
	0x0000,		//CCM3 register,
	0x0000,		//CCM4 register,
	0x0000,		//CCM5 register,
	0x0000,		//RF_Cmd register,
	0x0000,		//Btn_Cmd register,
	0x0000,		//DSSS2 register,
	0x0000,		//DSSS3 register,
	0x0000,		//CCM6 register,
	0x0C00,		//CDET register,
	0x0000,		//DC_SHIFT register,
	0x0000		//RCOSC register,
};

const uint8_t ID_Tab[8]={0x34,0x75,0xC5,0x8C,0xC7,0x33,0x45,0xE7};
const uint8_t PN9_Tab[64]=
{
  0xFF, 0x83, 0xDF, 0x17, 0x32, 0x09, 0x4E, 0xD1,
  0xE7, 0xCD, 0x8A, 0x91, 0xC6, 0xD5, 0xC4, 0xC4,
  0x40, 0x21, 0x18, 0x4E, 0x55, 0x86, 0xF4, 0xDC,
  0x8A, 0x15, 0xA7, 0xEC, 0x92, 0xDF, 0x93, 0x53,
  0x30, 0x18, 0xCA, 0x34, 0xBF, 0xA2, 0xC7, 0x59,
  0x67, 0x8F, 0xBA, 0x0D, 0x6D, 0xD8, 0x2D, 0x7D,
  0x54, 0x0A, 0x57, 0x97, 0x70, 0x39, 0xD2, 0x7A,
  0xEA, 0x24, 0x33, 0x85, 0xED, 0x9A, 0x1D, 0xE0
};
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  

int main(void)
{

  /* Output a message on Hyperterminal using printf function */

  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
  /* SPI configuration ------------------------------------------------------*/
  SPI_Config();
  
  USART_Config();
  
  GPIO2_Config();
  
  GPIOA4_Config();
  /* Loop until the end of transmission */
  /* The software must wait until TC=1. The TC flag remains cleared during all data
     transfers and it is set by hardware at the last frame’s end of transmission*/
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}
  
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Configure PC10 and PC11 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Initialize the FIFO threshold */
  SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
  
  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);
  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
  /* Enable the SPI Error interrupt */
  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_ERR, ENABLE);
  /* Data transfer is performed in the SPI interrupt routine */
  
  /* Enable the SPI peripheral */
  SPI_Cmd(SPIx, ENABLE);
  /* Waiting until TX FIFO is empty */
  while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty)
  {}    
  
  /* Wait busy flag */
  while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET)
  {}
    
  while(1)
  {
    if(InitRF())
    {
      entry_deep_sleep_mode();
      Delay(2);
      wake_up_from_deep_sleep_mode();
    }
    else
    {
      break;
    }
  }
  #if defined (SPI_MASTER)
  
  A7159_WriteFIFO();
  SPI_SendData8(SPIx,CMD_TX);
  Delay(1);
  while(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_1) == Bit_RESET);
  SPI_SendData8(SPIx,CMD_RX);
  Delay(1);
  
  while((GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_1) == Bit_SET) && SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
  if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY))
  {
    SPI_SendData8(SPIx,CMD_STBY);
  }
  else
  {
    Rxpacket();
    Delay(10);
  }
  #endif /* SPI_MASTER*/
  
  #if defined (SPI_SLAVE)
  
  RxCnt = 0;
  
  while(1)
  {
    SPI_SendData8(SPIx,CMD_RX);
    Delay(1);
    while(GPIO_Pin_1);
    Rxpacket();
    
    A7159_WriteFIFO();
    SPI_SendData8(SPIx,CMD_TX);
    Delay(1);
    while(GPIO_Pin_1);
    
    Delay(100);
  }
  #endif /* SPI_SLAVE*/
  
}

void GPIO2_Config(void)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void GPIOA4_Config(void)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET)
  {}

  return ch;
}

void A7159_Config(void)
{
  uint8_t i;
  
  printf("WriteReg:\r\n");
  for(i=0;i<8;i++)
  {
    A7159_WriteReg(i,A7159Config[i]);
  }
  Delay(10);
  for(i=0;i<1;i++)
  {
    A7159_ReadReg(i);
    while(!getSPI);
  }
  for(i=10;i<16;i++)
  {
    A7159_WriteReg(i,A7159Config[i]);
  }
  printf("\r\nWritePageA:\r\n");
  for(i=0;i<16;i++)
  {
    A7159_WritePageA(i,A7159Config_PageA[i]);
  }
  printf("\r\nWritePageB:\r\n");
  for(i=0;i<21;i++)
  {
    A7159_WritePageB(i,A7159Config_PageB[i]);
  }
}
void A7159_WritePageA(uint8_t address, uint16_t dataWord)
{
  SPICSOn();
  uint16_t tmp;
  tmp = address;
  tmp = ((tmp << 12) | A7159Config[CRYSTAL_REG]);
  A7159_WriteReg(CRYSTAL_REG, tmp);
  A7159_WriteReg(PAGEA_REG, dataWord);
  SPICSOff();
}

void A7159_WritePageB(uint8_t address, uint16_t dataWord)
{
  SPICSOn();
  uint16_t tmp;
  tmp = address;
  tmp = ((tmp << 7) | A7159Config[CRYSTAL_REG]);
  A7159_WriteReg(CRYSTAL_REG, tmp);
  A7159_WriteReg(PAGEB_REG, dataWord);
  SPICSOff();
}
void A7159_WriteID(void)
{
  uint8_t i;
  
  SPI_SendData8(SPIx,CMD_ID_W);
  for(i=0;i<4;i++)
  {
    SPI_SendData8(SPIx,ID_Tab[i]);
  }
}
void A7159_WriteReg(uint8_t address ,uint16_t dataWord)
{
  SPICSOn();
  address |= CMD_Reg_W ;
  SPI_SendData8(SPIx,address);
  SPI_I2S_SendData16(SPIx,dataWord);
  printf("(%02X)(%04X)\r\n",address,dataWord);
  SPICSOff();
}
void A7159_ReadReg(uint8_t address)
{
  SPICSOn();
  address |= CMD_Reg_R ;
  SPI_I2S_SendData16(SPIx,address);
  printf("(%02X)(%04X)\r\n",address-0x80,bb[address]);
  SPICSOff();
}
void A7159_WriteFIFO(void)
{
  uint8_t i;
  
  SPI_SendData8(SPIx,CMD_TFR);
  for(i=0;i<64;i++)
  {
    SPI_SendData8(SPIx,PN9_Tab[i]);
  }
}
uint8_t A7159_Cal(void)
{
  uint8_t fb,fcd,fbcf;
  uint8_t vb,vbcf;
  uint8_t vcb,vccf;
  uint16_t tmp;
  
  SPI_SendData8(SPIx,CMD_STBY);
  
  A7159_WriteReg(MODE_REG,A7159Config[MODE_REG]|0x0802);
  do
  {
    tmp=SPI_I2S_ReceiveData16(SPIx);
  }while(tmp & 0x0802);
  
  tmp = SPI_I2S_ReceiveData16(SPIx);
  fb = tmp & 0x0F;
  fcd = (tmp>>11) & 0x1F;
  fbcf = (tmp>>4) & 0x01;
  if(fbcf)
  {
    return 1;
  }
  
  tmp = SPI_I2S_ReceiveData16(SPIx);
  vcb = tmp & 0x0F;
  vccf = (tmp>>4) & 0x01;
  if(vccf)
  {
    return 1;
  }
  
  A7159_WriteReg(ADC_REG, 0x4C00);
  A7159_WriteReg(MODE_REG, A7159Config[MODE_REG] | 0x1000);
  do
  {
    tmp = SPI_I2S_ReceiveData16(SPIx);
  }while(tmp & 0x1000);
  A7159_WriteReg(ADC_REG, A7159Config[ADC_REG]);
  
  A7159_WriteReg(PLL1_REG, A7159Config[PLL1_REG]);
  A7159_WriteReg(PLL2_REG, A7159Config[PLL2_REG]);
  A7159_WriteReg(MODE_REG, A7159Config[MODE_REG] | 0x0004);
  do
  {
    tmp = SPI_I2S_ReceiveData16(SPIx);
  }while(tmp & 0004);
  
  tmp=SPI_I2S_ReceiveData16(SPIx);
  vb = (tmp >>5) & 0x07;
  vbcf = (tmp >>8) & 0x01;
  if(vbcf)
  {
    return 1;
  }
  
  return 0;
}
uint8_t InitRF(void)
{
  SPICSOn();
  SPI_SendData8(SPIx,CMD_RF_RST);
  SPICSOff();
  Delay(10);
  A7159_Config();
  Delay(1);
  A7159_WriteID();
  Delay(1);
  if(A7159_Cal())
  {
    return 1;
  }
  
  return 0;
}
void entry_deep_sleep_mode(void)
{
  SPICSOn();
  SPI_SendData8(SPIx,CMD_RF_RST);
  A7159_WriteReg(PIN_REG, A7159Config[PIN_REG] | 0x0800);
  A7159_WritePageA(PM_PAGEA, A7159Config_PageA[PM_PAGEA] | 0x0010);
  A7159_WritePageA(WCAL_PAGEA, A7159Config_PageA[WCAL_PAGEA] & 0x7FFF | 0x4000);
  SPI_SendData8(SPIx,CMD_SLEEP);
  Delay(6);
  SPI_SendData8(SPIx,CMD_DEEP_SLEEP);
  Delay(2);
  SPICSOff();
}
void Rxpacket(void)
{
  uint8_t i;
  uint8_t recv;
  uint8_t tmp;
  
  RxCnt++;
  
  SPI_SendData8(SPIx,CMD_RFR);
  SPI_SendData8(SPIx,CMD_FIFO_R);
  
  for(i=0;i<64;i++)
  {
    recv = bb[i];
    tmp = recv ^ PN9_Tab[i];
    if(tmp != 0)
    {
      printf("ERROR");
    }
  }
}
void wake_up_from_deep_sleep_mode(void)
{
  SPICSOn();
  SPI_SendData8(SPIx,CMD_STBY);
  Delay(10);
  SPICSOff();
}
void SPI_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);  
  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK , ENABLE);
  
  GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
  GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);
  GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);
//  GPIO_PinAFConfig(SPIx_NSS_GPIO_PORT, SPIx_NSS_SOURCE, SPIx_NSS_AF);
  
  GPIO_InitStructure.GPIO_Mode                 = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType                = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd                 = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed                = GPIO_Speed_Level_3;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin                  = SPIx_SCK_PIN | SPIx_MOSI_PIN | SPIx_MISO_PIN;
  GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPIx);
  SPI_InitStructure.SPI_Direction              = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize               = SPI_DATASIZE;
  SPI_InitStructure.SPI_CPOL                   = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA                   = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS                    = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler      = SPI_BaudRatePrescaler_64;
  /* Initializes the SPI communication */
  SPI_InitStructure.SPI_Mode                   = SPI_Mode_Master;
  SPI_Init(SPIx, &SPI_InitStructure);
  
   
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel           = SPIx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority   = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd        = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void SPICSOn(void)
{
  GPIOA->BRR = GPIO_Pin_4 ;
}
void SPICSOff(void)
{
  GPIOA->BSRR = GPIO_Pin_4 ;
}
static void USART_Config(void)
{ 
  USART_InitTypeDef USART_InitStructure;
  
  /* USARTx configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - Stop Bit = 1 Stop Bit
  - Parity = No Parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  STM_EVAL_COMInit(COM1, &USART_InitStructure);
}


void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

 void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif



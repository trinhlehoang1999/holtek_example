
/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32.h"
#include "ht32_board.h"
#include "ht32_board_config.h"
#include "system_ht32f5xxxx_01.h"
#include "pwm.h"
#include "delay.h"

/* Private function prototypes -----------------------------------------------------------------------------*/
void CKCU_Configuration(void);
void GPIO_IN_Configuration(void);
void GPIO_OUT_Configuration(void);

void EXTI_WAKE_Configuration(void);

void TM_Configuration(void);

void BFTM_Configuration(void);
void BFTM_Configuration_1(void);

void UxART_Configuration(void);
void UxART_TxTest(void);
void UxART_RxTest(void);

void ADC_Configuration(void);

void EXTI_WAKE_Configuration(void);
void EXTI_KEY1_Configuration(void);
void EXTI_KEY2_Configuration(void);

void chay_led(void);
void led_button(void);

void Key_Process(void);

void TimingDelay(void);
void Delay(u32 nTime);

u32 get_tick(void);

/* Private variables ---------------------------------------------------------------------------------------*/
#define TM_FREQ_HZ           (1000)
#define TM_TIME_DELAY        (500)

FlagStatus tmp;
////////////////////////////Adc
int i;
int check_key0;
u32 cnt=0;
u32 last_cnt=0;
u32 current_cnt=0;

u32 tick=0;
u32 last_tick=0;
u32 current_tick=0;
int check_tick =0;
/////////////////////////// uart
uc8  *gURTx_Ptr;
vu32 gURTx_Length = 0;
u8  *gURRx_Ptr;
vu32 gURRx_Length = 0;
vu32 gIsTxFinished = FALSE;

u8 gTx_Buffer[128];
u8 gRx_Buffer[128];
uc8 gHelloString[] = "my name: trinh le hoang -- 17146023--holtek\n";
///////////////////////////// button

///////////////////////////////
volatile uint32_t msTicks = 0;
uint8_t signal = 0;
uint32_t x=0;
uint8_t check=0;
vu32 gIsUpdateEvent = FALSE;
s32 gTimeCount;
vu32 DelayTime;
////////////////////////////////
u8 flag_echo[4];
u8 flag_button=0;
float value_distance=0;
////////////////////////////////

//void Delay(u32 nTime)
//{
//  /* Enable the SysTick Counter */
//  SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);
//  SYSTICK_CounterCmd(SYSTICK_COUNTER_ENABLE);

//  DelayTime = nTime;

//  while(DelayTime != 0);

//  /* Disable SysTick Counter */
//  SYSTICK_CounterCmd(SYSTICK_COUNTER_DISABLE);
//  /* Clear SysTick Counter */
//  SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);
//}

//void TimingDelay(void)
//{
//  if(DelayTime != 0)
//  {
//    DelayTime--;
//  }
//}
void led_run(void)
{
	value_distance = current_cnt / 6.0;
	if(value_distance <= 15 )
	{
		GPIO_WriteOutBits(HTCFG_LED0, HTCFG_OUTPUT_LED0_GPIO_PIN, RESET);
		GPIO_WriteOutBits(HTCFG_LED1, HTCFG_OUTPUT_LED1_GPIO_PIN, SET);
		GPIO_WriteOutBits(HTCFG_LED2, HTCFG_OUTPUT_LED2_GPIO_PIN, SET);
	} 
	else if (value_distance <= 50)
	{
		GPIO_WriteOutBits(HTCFG_LED0, HTCFG_OUTPUT_LED0_GPIO_PIN, SET);
		GPIO_WriteOutBits(HTCFG_LED1, HTCFG_OUTPUT_LED1_GPIO_PIN, RESET);
		GPIO_WriteOutBits(HTCFG_LED2, HTCFG_OUTPUT_LED2_GPIO_PIN, SET);
	}
	else
	{
		GPIO_WriteOutBits(HTCFG_LED0, HTCFG_OUTPUT_LED0_GPIO_PIN, SET);
		GPIO_WriteOutBits(HTCFG_LED1, HTCFG_OUTPUT_LED1_GPIO_PIN, SET);
		GPIO_WriteOutBits(HTCFG_LED2, HTCFG_OUTPUT_LED2_GPIO_PIN, RESET);
	}
}

void UxART_Configuration(void)
{
  #if 0 // Use following function to configure the IP clock speed.
  // The UxART IP clock speed must be faster 16x then the baudrate.
  CKCU_SetPeripPrescaler(CKCU_PCLK_UxARTn, CKCU_APBCLKPRE_DIV2);
  #endif

  { /* Enable peripheral clock of AFIO, UxART                                                               */
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
    CKCUClock.Bit.AFIO                   = 1;
    CKCUClock.Bit.HTCFG_UART_RX_GPIO_CLK = 1;
    CKCUClock.Bit.HTCFG_UART_IPN         = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
  }

  /* Turn on UxART Rx internal pull up resistor to prevent unknow state                                     */
  GPIO_PullResistorConfig(HTCFG_UART_RX_GPIO_PORT, HTCFG_UART_RX_GPIO_PIN, GPIO_PR_UP);

  /* Config AFIO mode as UxART function.                                                                    */
  AFIO_GPxConfig(HTCFG_UART_TX_GPIO_ID, HTCFG_UART_TX_AFIO_PIN, AFIO_FUN_USART_UART);
  AFIO_GPxConfig(HTCFG_UART_RX_GPIO_ID, HTCFG_UART_RX_AFIO_PIN, AFIO_FUN_USART_UART);

  {
    /* UxART configured as follow:
          - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - None parity bit
    */

    /* !!! NOTICE !!!
       Notice that the local variable (structure) did not have an initial value.
       Please confirm that there are no missing members in the parameter settings below in this function.
    */
    USART_InitTypeDef USART_InitStructure = {0};
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WORDLENGTH_8B;
    USART_InitStructure.USART_StopBits = USART_STOPBITS_1;
    USART_InitStructure.USART_Parity = USART_PARITY_NO;
    USART_InitStructure.USART_Mode = USART_MODE_NORMAL;
    USART_Init(HTCFG_UART_PORT, &USART_InitStructure);
  }

  /* Enable UxART interrupt of NVIC                                                                         */
  NVIC_EnableIRQ(HTCFG_UART_IRQn);

  /* Enable UxART Rx interrupt                                                                              */
  USART_IntConfig(HTCFG_UART_PORT, USART_INT_RXDR|USART_INT_TXC, ENABLE);


  /* Enable UxART Tx and Rx function                                                                        */
  USART_TxCmd(HTCFG_UART_PORT, ENABLE);
  USART_RxCmd(HTCFG_UART_PORT, ENABLE);
}




void UxART_TxTest(void)
{
  gIsTxFinished = FALSE;
  gURTx_Ptr = (uc8 *)gHelloString;
  gURTx_Length = sizeof(gHelloString) - 1;
  USART_IntConfig(HTCFG_UART_PORT, USART_INT_TXDE | USART_INT_TXC, ENABLE);

  while (gURTx_Length != 0);      // Latest byte move to UxART shift register, but the transmission may be on going.
  while (gIsTxFinished == FALSE); // Set by TXC interrupt, transmission is finished.
}

void UxART_RxTest(void)
{
	
  u32 i;
  u32 uLength;

  /* Waiting for receive 5 data                                                                             */
  if (gURRx_Length >= 1)
  {
    // Process Rx data by gRx_Buffer[] and gURRx_Length here
    // .....

    uLength = gURRx_Length;
    for (i = 0; i < uLength; i++)
    {
      gTx_Buffer[i] = gRx_Buffer[i];
    }

    #if 1 // Loop back Rx data to Tx for test
    gIsTxFinished = FALSE;
    gURTx_Ptr = gTx_Buffer;
    gURTx_Length = uLength;
    USART_IntConfig(HTCFG_UART_PORT, USART_INT_TXDE | USART_INT_TXC, ENABLE);
    #endif

    gURRx_Length = 0;
  }
}



int main(void)
{
  CKCU_Configuration();               /* System Related configuration                                       */
	RETARGET_Configuration();
  /* Configure WAKEUP, KEY1, KEY2 pins as the input function                                                */
  GPIO_IN_Configuration();
	UxART_Configuration();
  /* Configure LED1, LED2, LED3 pins as output function                                                     */
  GPIO_OUT_Configuration();
	
	gTimeCount = TM_TIME_DELAY;
	EXTI_WAKE_Configuration();
	EXTI_KEY1_Configuration();
	
	BFTM_Configuration();
	BFTM_Configuration_1();
	current_cnt = 0;
	cnt = 0;
	delay_ms(1000);
	HT32F_DVB_LEDInit(HT_LED1);
  HT32F_DVB_LEDInit(HT_LED2);
  HT32F_DVB_LEDInit(HT_LED3);
	gURRx_Ptr = gRx_Buffer;
	printf("start\n");
	UxART_TxTest();

	//SysTick_Config(SystemCoreClock/8);

//	SYSTICK_ClockSourceConfig(SYSTICK_SRC_STCLK);       // Default : CK_AHB/8
//  SYSTICK_SetReloadValue(SystemCoreClock / 8/1000);     // (CK_AHB/8/1000) = 1ms on chip
//  SYSTICK_IntConfig(ENABLE);  
		SysTick->CTRL =0;
		SysTick->LOAD = 47;
		SysTick->VAL = 0;
		SysTick->CTRL =2;


	
                             
  while (1)
  {
//		GPIO_WriteOutBits(HT_GPIOB, GPIO_PIN_15, RESET);
//		delay_us(1);
//		GPIO_WriteOutBits(HT_GPIOB, GPIO_PIN_15, SET);
//		delay_us(1);
//		GPIO_WriteOutBits(HT_GPIOB, GPIO_PIN_15, RESET);
		led_run();

//	if (flag_echo[2] == 1)
//	{

//	}
//	else
//	{

//	}
}

  }

void EXTI_KEY1_Configuration(void)
{
  { /* Enable peripheral clock                                                                              */
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
    CKCUClock.Bit.AFIO       = 1;
    CKCUClock.Bit.EXTI       = 1;
    CKCUClock.Bit.HTCFG_KEY1_GPIO_CK = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
  }

  /* Configure AFIO mode of input pins                                                                      */
  AFIO_GPxConfig(HTCFG_KEY1_GPIO_ID, HTCFG_KEY1_AFIO_PIN, AFIO_FUN_GPIO);

  /* Enable GPIO Input Function                                                                             */
  GPIO_InputConfig(HTCFG_KEY1_GPIO_PORT, HTCFG_KEY1_GPIO_PIN, ENABLE);

  /* Configure GPIO pull resistor of input pins                                                             */
  GPIO_PullResistorConfig(HTCFG_KEY1_GPIO_PORT, HTCFG_KEY1_GPIO_PIN, GPIO_PR_DISABLE);

  /* Select Port as EXTI Trigger Source                                                                     */
  AFIO_EXTISourceConfig(HTCFG_KEY1_AFIO_EXTI_CH, HTCFG_KEY1_AFIO_ESS);

  { /* Configure EXTI Channel n as falling edge trigger                                                     */

    /* !!! NOTICE !!!
       Notice that the local variable (structure) did not have an initial value.
       Please confirm that there are no missing members in the parameter settings below in this function.
    */
    EXTI_InitTypeDef EXTI_InitStruct;

    EXTI_InitStruct.EXTI_Channel = HTCFG_KEY1_EXTI_CH;
    EXTI_InitStruct.EXTI_Debounce = EXTI_DEBOUNCE_DISABLE;
    EXTI_InitStruct.EXTI_DebounceCnt = 0;
    EXTI_InitStruct.EXTI_IntType = EXTI_NEGATIVE_EDGE;
    EXTI_Init(&EXTI_InitStruct);
  }

  /* Enable EXTI & NVIC line Interrupt                                                                      */
  EXTI_IntConfig(HTCFG_KEY1_EXTI_CH, ENABLE);
  NVIC_EnableIRQ(HTCFG_KEY1_EXTI_IRQn);
}

void EXTI_WAKE_Configuration(void)
{
  { /* Enable peripheral clock                                                                              */
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
    CKCUClock.Bit.AFIO       = 1;
    CKCUClock.Bit.EXTI       = 1;
    CKCUClock.Bit.HTCFG_WAKE_GPIO_CK = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
  }

  /* Configure AFIO mode of input pins                                                                      */
  AFIO_GPxConfig(HTCFG_WAKE_GPIO_ID, HTCFG_WAKE_AFIO_PIN, AFIO_FUN_GPIO);

  /* Enable GPIO Input Function                                                                             */
  GPIO_InputConfig(HTCFG_WAKE_GPIO_PORT, HTCFG_WAKE_GPIO_PIN, ENABLE);

  /* Configure GPIO pull resistor of input pins                                                             */
  GPIO_PullResistorConfig(HTCFG_WAKE_GPIO_PORT, HTCFG_WAKE_GPIO_PIN, GPIO_PR_DISABLE);

  /* Select Port as EXTI Trigger Source                                                                     */
  AFIO_EXTISourceConfig(HTCFG_WAKE_AFIO_EXTI_CH, HTCFG_WAKE_AFIO_ESS);

  { /* Configure EXTI Channel n as rising edge trigger                                                      */

    /* !!! NOTICE !!!
       Notice that the local variable (structure) did not have an initial value.
       Please confirm that there are no missing members in the parameter settings below in this function.
    */
    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Channel = HTCFG_WAKE_EXTI_CH;
    EXTI_InitStruct.EXTI_Debounce = EXTI_DEBOUNCE_DISABLE;
    EXTI_InitStruct.EXTI_DebounceCnt = 0;
    EXTI_InitStruct.EXTI_IntType = EXTI_BOTH_EDGE;
    EXTI_Init(&EXTI_InitStruct);
  }

  /* Enable EXTI & NVIC line Interrupt                                                                      */
  EXTI_IntConfig(HTCFG_WAKE_EXTI_CH, ENABLE);
  NVIC_EnableIRQ(HTCFG_WAKE_EXTI_IRQn);
}

void BFTM_Configuration(void)
{
  #if 0 // Use following function to configure the IP clock speed.
  CKCU_SetPeripPrescaler(CKCU_PCLK_BFTM0, CKCU_APBCLKPRE_DIV2);
  #endif

  { /* Enable peripheral clock                                                                              */
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
    CKCUClock.Bit.BFTM0 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
  }

  /* BFTM as Repetitive mode, every 0.5 second to trigger the match interrupt                               */
  BFTM_SetCompare(HT_BFTM0, SystemCoreClock / 100000);
  BFTM_SetCounter(HT_BFTM0, 0);
  BFTM_IntConfig(HT_BFTM0, ENABLE);
  BFTM_EnaCmd(HT_BFTM0, ENABLE);

  NVIC_EnableIRQ(BFTM0_IRQn);
}

void BFTM_Configuration_1(void)
{
  #if 0 // Use following function to configure the IP clock speed.
  CKCU_SetPeripPrescaler(CKCU_PCLK_BFTM1, CKCU_APBCLKPRE_DIV2);
  #endif

  { /* Enable peripheral clock                                                                              */
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
    CKCUClock.Bit.BFTM1 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
  }

  /* BFTM as Repetitive mode, every 0.5 second to trigger the match interrupt                               */
  BFTM_SetCompare(HT_BFTM1, SystemCoreClock / 10);
  BFTM_SetCounter(HT_BFTM1, 0);
  BFTM_IntConfig(HT_BFTM1, ENABLE);
  BFTM_EnaCmd(HT_BFTM1, ENABLE);

  NVIC_EnableIRQ(BFTM1_IRQn);
}
	
void CKCU_Configuration(void)
{
#if 1
  CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
  HTCFG_OUTPUT_LED0_CLK(CKCUClock) = 1;
  HTCFG_OUTPUT_LED1_CLK(CKCUClock) = 1;
  HTCFG_OUTPUT_LED2_CLK(CKCUClock) = 1;
  HTCFG_INPUT_WAKE_CLK(CKCUClock)  = 1;
  HTCFG_INPUT_KEY1_CLK(CKCUClock)  = 1;
  HTCFG_INPUT_KEY2_CLK(CKCUClock)  = 1;
	CKCUClock.Bit.PA = 1;
  CKCUClock.Bit.AFIO       = 1;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);
#endif
}

/*********************************************************************************************************//**
  * @brief  Configure the GPIO ports for input.
  * @retval None
  ***********************************************************************************************************/
void GPIO_IN_Configuration(void)
{
  /* Configure WAKEUP, KEY1, KEY2 pins as the input function                                                */
  /* Configure AFIO mode of input pins                                                                      */
  AFIO_GPxConfig(GPIO_PA,AFIO_PIN_0,AFIO_FUN_GPIO);
	AFIO_GPxConfig(HTCFG_INPUT_WAKE_ID, HTCFG_INPUT_WAKE_AFIO_PIN, AFIO_FUN_GPIO);
  AFIO_GPxConfig(HTCFG_INPUT_KEY1_ID, HTCFG_INPUT_KEY1_AFIO_PIN, AFIO_FUN_GPIO);
  AFIO_GPxConfig(HTCFG_INPUT_KEY2_ID, HTCFG_INPUT_KEY2_AFIO_PIN, AFIO_FUN_GPIO);

  /* Configure GPIO direction of input pins                                                                 */
  GPIO_DirectionConfig(HTCFG_WAKE, HTCFG_INPUT_WAKE_GPIO_PIN, GPIO_DIR_IN);
  GPIO_DirectionConfig(HTCFG_KEY1, HTCFG_INPUT_KEY1_GPIO_PIN, GPIO_DIR_IN);
  GPIO_DirectionConfig(HTCFG_KEY2, HTCFG_INPUT_KEY2_GPIO_PIN, GPIO_DIR_IN);
	GPIO_DirectionConfig(HT_GPIOA,GPIO_PIN_0,GPIO_DIR_IN);

  /* Configure GPIO pull resistor of input pins                                                             */
  GPIO_PullResistorConfig(HTCFG_WAKE, HTCFG_INPUT_WAKE_GPIO_PIN, GPIO_PR_DISABLE);
  GPIO_PullResistorConfig(HTCFG_KEY1, HTCFG_INPUT_KEY1_GPIO_PIN, GPIO_PR_DISABLE);
  GPIO_PullResistorConfig(HTCFG_KEY2, HTCFG_INPUT_KEY2_GPIO_PIN, GPIO_PR_DISABLE);
	GPIO_PullResistorConfig(HT_GPIOA, GPIO_PIN_0, GPIO_PR_DISABLE);
  
	GPIO_InputConfig(HT_GPIOA, GPIO_PIN_0, ENABLE);
	GPIO_InputConfig(HTCFG_WAKE, HTCFG_INPUT_WAKE_GPIO_PIN, ENABLE);
  GPIO_InputConfig(HTCFG_KEY1, HTCFG_INPUT_KEY1_GPIO_PIN, ENABLE);
  GPIO_InputConfig(HTCFG_KEY2, HTCFG_INPUT_KEY2_GPIO_PIN, ENABLE);
}

/*********************************************************************************************************//**
  * @brief  Configure the GPIO ports for output.
  * @retval None
	
  ***********************************************************************************************************/
void GPIO_OUT_Configuration(void)
{
  /* Configure LED1, LED2, LED3 pins as output function                                                     */
  /* Configure AFIO mode of output pins    
	*/
	AFIO_GPxConfig(GPIO_PB,AFIO_PIN_15,AFIO_FUN_GPIO);
  AFIO_GPxConfig(HTCFG_OUTPUT_LED0_ID, HTCFG_OUTPUT_LED0_AFIO_PIN, AFIO_FUN_GPIO);
  AFIO_GPxConfig(HTCFG_OUTPUT_LED1_ID, HTCFG_OUTPUT_LED1_AFIO_PIN, AFIO_FUN_GPIO);
  AFIO_GPxConfig(HTCFG_OUTPUT_LED2_ID, HTCFG_OUTPUT_LED2_AFIO_PIN, AFIO_FUN_GPIO);

  /* Configure GPIO direction of output pins 
	*/
	GPIO_DirectionConfig(HT_GPIOB,GPIO_PIN_15,GPIO_DIR_OUT);
  GPIO_DirectionConfig(HTCFG_LED0, HTCFG_OUTPUT_LED0_GPIO_PIN, GPIO_DIR_OUT);
  GPIO_DirectionConfig(HTCFG_LED1, HTCFG_OUTPUT_LED1_GPIO_PIN, GPIO_DIR_OUT);
  GPIO_DirectionConfig(HTCFG_LED2, HTCFG_OUTPUT_LED2_GPIO_PIN, GPIO_DIR_OUT);
}

void HTCFG_UART_IRQHandler(void)
{


//#if (DEBUG_IO == 1)
//  #define DBG_IO1_LO()    HT32F_DVB_LEDOn(HT_LED1)
//  #define DBG_IO1_HI()    HT32F_DVB_LEDOff(HT_LED1)
//  #define DBG_IO2_LO()    HT32F_DVB_LEDOn(HT_LED2)
//  #define DBG_IO2_HI()    HT32F_DVB_LEDOff(HT_LED2)
//#else
//  #define DBG_IO1_LO(...)
//  #define DBG_IO1_HI(...)
//  #define DBG_IO2_LO(...)
//  #define DBG_IO2_HI(...)
//#endif

  /* Rx: Move data from UART to buffer                                                                      */
  if (USART_GetFlagStatus(HTCFG_UART_PORT, USART_FLAG_RXDR))
  {
//    DBG_IO1_LO();
    gURRx_Ptr[gURRx_Length++] = USART_ReceiveData(HTCFG_UART_PORT);
		
//    DBG_IO1_HI();

    #if 0
    if (gURRx_Length == 128)
    {
      while (1) {}; // Rx Buffer full
    }
    #endif
  }

  /* Tx, move data from buffer to UART                                                                      */
  if (USART_GetIntStatus(HTCFG_UART_PORT, USART_INT_TXDE) &&
      USART_GetFlagStatus(HTCFG_UART_PORT, USART_FLAG_TXDE))
  {
//    DBG_IO2_LO();
    if (gURTx_Length > 0)
    {
      USART_SendData(HTCFG_UART_PORT, *gURTx_Ptr++);
      gURTx_Length--;
      if (gURTx_Length == 0)
      {
        USART_IntConfig(HTCFG_UART_PORT, USART_INT_TXDE, DISABLE);
      }
    }
//    DBG_IO2_HI();
  }

  if (USART_GetIntStatus(HTCFG_UART_PORT, USART_INT_TXC) &&
      USART_GetFlagStatus(HTCFG_UART_PORT, USART_FLAG_TXC))
  {
    USART_IntConfig(HTCFG_UART_PORT, USART_INT_TXC, DISABLE);
    gIsTxFinished = TRUE;
  }
}

u32 get_tick(void)
	{
		tick++;
		return tick;
	}
void SysTick_Handler(void)
{
	
  //void TimingDelay(void);
  //TimingDelay();
	tick++;
}

void EXTI4_15_IRQHandler(void)
{
	  if (EXTI_GetEdgeStatus(HTCFG_WAKE_EXTI_CH,EXTI_EDGE_POSITIVE))
  {
    EXTI_ClearEdgeFlag(HTCFG_WAKE_EXTI_CH);

		flag_echo[0] = 1;
  } 
	else
		{
			EXTI_ClearEdgeFlag(HTCFG_WAKE_EXTI_CH);
			flag_echo[0] = 0;
		}

//	else if (EXTI_GetEdgeFlag(HTCFG_WAKE_EXTI_CH))
//		{
//			EXTI_ClearEdgeFlag(HTCFG_WAKE_EXTI_CH);
//		}
}

void BFTM0_IRQHandler(void)
{
	
  BFTM_ClearFlag(HT_BFTM0);
	//HT32F_DVB_LEDToggle(HT_LED2);
		if(flag_echo [0] == 1)
		{
			cnt++;     
			current_cnt = cnt;   // 1 cm = 6 xung
		}
	else if(flag_echo [0] == 0)
		{
			cnt=0;

		}
}
	
void BFTM1_IRQHandler(void)
	{
		BFTM_ClearFlag(HT_BFTM1);
		printf("-------------------------\n");
		printf("check:%d\n",flag_echo[0]);
		printf("value_cnt:%d\n",cnt);
		printf("value_current_cnt:%d\n",current_cnt);
		printf("value_distance:%f\n",value_distance);
		printf("-------------------------\n");
		//HT32F_DVB_LEDToggle(HT_LED2);
	}
	

void EXTI0_1_IRQHandler(void)
{
	  if (EXTI_GetEdgeFlag(HTCFG_KEY1_EXTI_CH))
  {
			EXTI_ClearEdgeFlag(HTCFG_KEY1_EXTI_CH);
			flag_echo[2] = 1;
			GPIO_WriteOutBits(HT_GPIOB, GPIO_PIN_15, RESET);
			GPIO_WriteOutBits(HT_GPIOB, GPIO_PIN_15, SET);
			delay_us(1);
			GPIO_WriteOutBits(HT_GPIOB, GPIO_PIN_15, RESET);	
  }
	else 
	{
		flag_echo[2] = 0;
	}
}



#if (HT32_LIB_DEBUG == 1)
/*********************************************************************************************************//**
  * @brief  Report both the error name of the source file and the source line number.
  * @param  filename: pointer to the source file name.
  * @param  uline: error line source number.
  * @retval None
  ***********************************************************************************************************/
void assert_error(u8* filename, u32 uline)
{


  while (1)
  {
  }
}
#endif



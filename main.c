
/* Includes ------------------------------------------------------------------*/

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "LCD_STM32F4.h"
#include "stm32f4xx_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t PrescalerValue = 0;

__IO uint32_t TimingDelay;
__IO uint8_t UserButtonPressed = 0x00;


/* Private function prototypes -----------------------------------------------*/
static void LED_task(void *pvParameters);
static void button_task(void *pvParameters);
static void LCD_task(void *pvParameters);

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  UserButtonPressed = 0x01;

  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize LEDs to be managed by GPIO */
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  /* Turn OFF all LEDs */
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);

  /* Initialization */
  Init_SysTick();
  Init_GPIO();
  Init_FSMC();
  Init_LCD();

   STM_EVAL_LEDToggle(LED5);
  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00;

  /* Create a task to display on LCD. */
  xTaskCreate(LCD_task,
             (signed portCHAR *) "LCD",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 4, NULL);

  /* Create a task to flash the LED. */
  xTaskCreate(LED_task,
             (signed portCHAR *) "LED Flash",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 4, NULL);

  /* Create a task to button check. */
  xTaskCreate(button_task,
             (signed portCHAR *) "User Button",
             512 /* stack size */, NULL,
             tskIDLE_PRIORITY + 4, NULL);

  /* Start running the tasks. */
  vTaskStartScheduler();

  return 0;
}

static void LED_task(void *pvParameters)
{
  RCC_ClocksTypeDef RCC_Clocks;
  uint8_t togglecounter = 0x00;

  while(1)
  {
      /* Toggle LED5 */
      STM_EVAL_LEDToggle(LED5);
      vTaskDelay(100);
      /* Toggle LED6 */
      STM_EVAL_LEDToggle(LED6);
      vTaskDelay(100);
  }
}

static void button_task(void *pvParameters)
{
	while (1)
	{
		    /* Waiting User Button is pressed */
    		    if (UserButtonPressed == 0x01)
    		    {
      		    	/* Toggle LED4 */
      			STM_EVAL_LEDToggle(LED4);
      			vTaskDelay(100);
      			/* Toggle LED3 */
      			STM_EVAL_LEDToggle(LED3);
      			vTaskDelay(100);
    		    }
		    /* Waiting User Button is Released */
    		    while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET);
		    UserButtonPressed = 0x00;
	}
}

static void LCD_task(void *pvParameters)
{

  STM_EVAL_LEDToggle(LED4);
  STM_EVAL_LEDToggle(LED3);
  STM_EVAL_LEDToggle(LED5);
  STM_EVAL_LEDToggle(LED6);
  vTaskDelay(3000);

  /* Demo */
  while(1)
  {
    Clear_Screen(0x0000);

     Set_Font(&Font16x24);
     Display_String(14, 295, "MMIA PROJEKT 2012", LCD_WHITE);
     Display_String(72, 287, "DEMONSTRACNY KIT", LCD_WHITE);
     Set_Font(&Font12x12);
     Display_String(97, 285, "pre STM32F4-Discovery", LCD_WHITE);
    Draw_Image(120, 206, 54, 93, logo_urel);

      Set_Font(&Font8x8);
      Display_String(220, 259, "Jakub Lanik, Jakub Nedoma", LCD_WHITE);
      Display_String(230, 259, "Martin Tajc, Martin Serik", LCD_WHITE);
      vTaskDelay(1000);

    Draw_Image(0, 319, 240, 320, mandelbrot1);
      vTaskDelay(7000);

      Clear_Screen(0x0000);
      Set_Font(&Font16x24);
      Display_String(107, 199, "Image", LCD_WHITE);

     vTaskDelay(3000);
  }
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);

  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    vTaskDelay(5);
  }
}


void vApplicationTickHook()
{
}



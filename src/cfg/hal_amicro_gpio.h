#ifndef HAL_AMICRO_R10_H
#define HAL_AMICRO_R10_H
#include "gpio_cfg.h"
#include "am_gpio.h"

typedef struct
{
  U8          name;
  U8          start;
} IO_PIN_CFG;


#define AM_IO_IR_REC_BACK_LEFT    GPIOD(4)
#define AM_IO_IR_REC_BACK_RIGHT   GPIOD(4)
#define AM_IO_BUMP_RIGHT          GPIOB(10)
#define AM_IO_BUMP_LEFT           GPIOB(9)
#define AM_IO_MC_KEY_WAKE_ON      GPIOB(15)
#define AM_I0_GARBAGE_BOX_DETECT  GPIOC(4)
#define AM_IO_WHEEL_DROP_RIGHT    GPIOD(3)
#define AM_IO_WHEEL_DROP_LEFT     GPIOC(12)
#define AM_I0_CLEAN_LED           GPIOC(15)
#define AM_I0_SPOT_LED            GPIOD(0)
#define AM_I0_DOCK_LED            GPIOC(14)
#define HAL_IO_AUDIO_PWR_EN       GPIOD(1)
#define AM_I0_LED_DIG1            GPIOC(8)
#define AM_IO_SWITCH_DET          GPIOD(5)
#define AM_I0_WIRELESS_SPIEN      GPIOC(9)
#define AM_IO_VAC_POWER           GPIOD(6)
#define ARF2496_IRQ_PIN           GPIOD(8)
#define AM_I0_LED_DIG4            GPIOC(13)
#define AM_IO_SIDE_BRUSH_DIR      GPIOE(3)
#define AM_IO_MAIN_BRUSH_DIR      GPIOE(5)
#define IO_CHARGER_LOW            GPIOE(9)
#define IO_CHARGER_HIGH           GPIOE(11)
#define AM_IO_IR_REC_MIDDLE_RIGHT GPIOD(4)

#define AM_IO_TOUCH_LED_FAR_EN        GPIOE(10)
#define AM_IO_TOUCH_LED_NEAR_EN       GPIOE(13)
#define AM_IO_TOUCH_CLIFF_LED_FAR_EN  GPIOE(12)

#define AM_IO_LED_STB             GPIOE(13)

#define AM_IO_IR_REC_MIDDLE_LEFT  GPIOD(4)
#define AM_IO_IR_REC_RIGHT        GPIOD(4)
#define AM_IO_IR_REC_LEFT         GPIOD(4)
#define IO_VCC_KILL               GPIOF(11)
#define AM_IO_LED_CLK             GPIOF(13)
#define AM_IO_LED_DIN             GPIOF(14)
#define AM_IO_ARF2498_IRQ         AM_I0_LED_DIG4
#define AM_IO_ARF2498_SPI_EN      AM_I0_LED_DIG1
#define AM_I0_FLASH_DATA_O        GPIOF(15)
#define AM_I0_FLASH_DATA_I        GPIOF(14)
#define AM_I0_FLASH_CLOCK         GPIOF(13)

#define AM_IO_WIFI_POWER          GPIOD(2)
#define AM_IO_RESET_WIFI          GPIOD(9)
#define AM_IO_WIFI_WAKE_UP        GPIOD(12)
#define AM_I0_WATER_TANK_DETECT   GPIOD(4)

#define AM_IO_BK2425_SPI_EN       GPIOC(8)
#define AM_IO_BK2425_SPI_CE       GPIOC(9)
#define IR_TX_BACK_LEFT_WEAK      GPIOC(1)
#define IR_TX_BACK_LEFT_STRONG    GPIOC(2)
#define IR_TX_BACK_RIGHT_WEAK     GPIOC(3)
#define IR_TX_BACK_RIGHT_STRONG   GPIOC(11)
#define IR_TX_LEFT_WEAK           GPIOF(3)
#define IR_TX_LEFT_STRONG         GPIOF(2)
#define IR_TX_RIGHT_WEAK          GPIOE(15)
#define IR_TX_RIGHT_STRONG        GPIOE(6)
#define IR_TX_MIDDLE_WEAK         GPIOF(1)
#define IR_TX_MIDDLE_STRONG       GPIOF(0)

/*下面中不使用的IO 口配置为0xff，lyy*/
#define ROBOT_IODEF(m)                                             \
	/* Port                          Strt    */\
        m( AM_IO_FLASH_HOLD ,           GPIOB(11)) \
	m( AM_IO_MOTOR_DISABLE ,        GPIOB(13))  \
	m( AM_IO_MOTOR_L_PWM   ,        GPIOC(5) )  \
	m( AM_IO_LEFT_MOTOR_DIR ,       GPIOC(6) )  \
	m( AM_IO_RIGHT_MOTOR_DIR,       GPIOF(12))  \
	m( AM_IO_MOTOR_R_PWM,           GPIOG(1) )   \
	m( AM_IO_AUDIO_PWR_EN    ,      GPIOD(1) )   \
 	m( AM_IO_MOTOR_WHEEL_SLEEP ,    GPIOE(0) )   \
	m( AM_IO_MOTOR_WHEEL_FAULT,     GPIOE(4) )   \
	m( AM_IO_MOTOR_WHEEL_RESET ,    0xff )  \
	m( AM_IO_CHARGER_ENABLE,        GPIOF(8) )  \
	m( AM_IO_BATTERY_I_CALIBRATE,   GPIOF(9) )  \
	m( AM_IO_WATCHDOG_CLEAR  ,      GPIOF(10))  \
	m( AM_IO_VCC_KILL,              GPIOF(11))  \
        m( AM_IO_SWITCHED_PWR_5V,       GPIOG(2) )   \
        m( AM_IO_SWITCHED_PWR_3V,       GPIOG(3) )\
        m( AM_IO_REV17,       0xff )\
        m( AM_IO_REV18,       0xff )\
        m( AM_IO_REV19,       0xff )



#define AM380S_IO_CFG_INIT(name,start) \
  {name,start} ,
  
#endif

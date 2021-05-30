/*********************************************************************************************************//**
 * @file    I2C/TouchKey/ht32_board_config.h
 * @version $Rev:: 5067         $
 * @date    $Date:: 2020-11-07 #$
 * @brief   The header file of board configuration.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __HT32_BOARD_CONFIG_H
#define __HT32_BOARD_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Settings ------------------------------------------------------------------------------------------------*/
#if defined(USE_HT32XXXXXX_DVB)
  /* !!! NOTICE !!!
     This example requires external component on the expansion board but the development board can not use
     with it directly. The extra jumper/wired connections may required to use this example.
  */
#endif

#if defined(USE_HT32F52230_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_7)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_8)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F52241_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F52253_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_1)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F52341_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F52352_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_1)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F0008_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F50230_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F50241_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F52354_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F57341_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_1)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F57352_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_1)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F52367_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F50343_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C1)
  #define TOUCHKEY_I2C                  (HT_I2C1)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PC)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_0)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F0006_DVB)
  #error "This example code does not apply to the chip you selected."
#endif

#if defined(USE_HT32F65240_DVB)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_6)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_15)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F65240_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_10)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_11)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F65232_SK)  //PRELIMINARY_NOT_TEST
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_10)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PB)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_11)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#if defined(USE_HT32F61355_SK) || defined(USE_HT32F61356_SK) || defined(USE_HT32F61357_SK)
  #define TOUCHKEY_I2C_CLK(CK)          (CK.Bit.I2C0)
  #define TOUCHKEY_I2C                  (HT_I2C0)

  #define TOUCHKEY_I2C_SCL_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SCL_AFIO_PIN     (AFIO_PIN_4)
  #define TOUCHKEY_I2C_SCL_AFIO_MODE    (AFIO_FUN_I2C)

  #define TOUCHKEY_I2C_SDA_GPIO_ID      (GPIO_PA)
  #define TOUCHKEY_I2C_SDA_AFIO_PIN     (AFIO_PIN_5)
  #define TOUCHKEY_I2C_SDA_AFIO_MODE    (AFIO_FUN_I2C)
#endif

#ifdef __cplusplus
}
#endif

#endif

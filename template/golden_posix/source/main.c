/*********************************************************************************
* File Name       : main.c
*
* Description     : This is the source code for freertos-posix example for
*                   ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*********************************************************************************
 * Header Files
*********************************************************************************/
/* demo include */
#include "posix_demo.h"


/*********************************************************************************
 * Macros
*********************************************************************************/
/* Demo task priority */
#define POSIX_DEMO_TASK_PRIORITY    (tskIDLE_PRIORITY + 4)


/*********************************************************************************
* Function Name: main
**********************************************************************************
* Summary:
*  This is the main function for CPU. It...
*    1. Initializes the board bsp.
*    2. Initializes retarget-io to use the debug UART port.
*    3. Initializes the User RGB LED.
*    4. Create and start the POSIX demo rtos task.
*
* Parameters:
*  void
*
* Return:
*  int
*
*********************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t posix_demo_task_status;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User RGB LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("******************"
            " FreeRTOS-POSIX: Demo "
            "******************\r\n\n");

    /* Start the task to run POSIX demo */
    posix_demo_task_status = xTaskCreate(posix_demo, "posix", configMINIMAL_STACK_SIZE, NULL, POSIX_DEMO_TASK_PRIORITY, NULL);

    if (pdPASS != posix_demo_task_status)
    {
        CY_ASSERT(0);
    }

    /* Start the RTOS Scheduler */
    vTaskStartScheduler();

    /* Should never get there */
    printf("APP_LOG: Error: FreeRTOS doesn't start\r\n");

    return 0;
}

/* [] END OF FILE */

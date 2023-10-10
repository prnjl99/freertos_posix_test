/*********************************************************************************
* File Name   : posix_demo.c
*
* Description : This file provides the source code to implement the freertos-posix
*               threads, mutex, and timers.
*
* Note        : See README.md
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
#include "FreeRTOS_POSIX.h"
#include "FreeRTOS_POSIX/unistd.h"
#include "FreeRTOS_POSIX/pthread.h"
#include "FreeRTOS_POSIX/fcntl.h"
#include "FreeRTOS_POSIX/mqueue.h"
#include "FreeRTOS_POSIX/errno.h"

#include "posix_demo.h"

/*********************************************************************************
 * Macros
*********************************************************************************/
#define THREAD_PRIORITY                       ((configMAX_PRIORITIES) - 3)
#define THREAD_STACK_SZ                       (4 * (configMINIMAL_STACK_SIZE))

#define NUM_TX_THREADS                        (2U)
#define NUM_CONTROL_MSGS_PER_TX_THREAD        (50U)

/* The number of items the mqueue can hold at once. */
#define MQUEUE_MAX_NUMBER_OF_MSGS             (1UL)           /* Maximum number of messages in a queue */
#define MQUEUE_MSG_SIZE                       sizeof(uint8_t) /* Message size */

#define THREAD_QUEUE_NAME                      "/threadqueue"


/*********************************************************************************
 * Global Variables
*********************************************************************************/
/* mqueue handle. This mqueue is used by all the threads. */
static mqd_t thread_queue;

/* mutex handle. This mutex makes sure that only one TX thread at a time can access the mqueue. */
static pthread_mutex_t mutex_lock;

/* TX thread attributes and ID. */
typedef struct tx_thread_resources_t
{
    pthread_attr_t tx_thread_attrib;
    pthread_t tx_thread_id;
} tx_thread_resources;

static tx_thread_resources tx_threads[NUM_TX_THREADS];

/* messages used by the TX threads to send to the mqueue. */
typedef enum control_message
{
    CONTROL_MSG_TX_THREAD_1 = 1,
    CONTROL_MSG_TX_THREAD_2 = 2,
    CONTROL_MSG_EXIT_TX_THREAD_1 = 3,
    CONTROL_MSG_EXIT_TX_THREAD_2 = 4,
} control_msg;


/*********************************************************************************
 * Function Definitions
*********************************************************************************/

/*********************************************************************************
* Function Name: queue_send_thread
**********************************************************************************
* Summary:
*  This is the POSIX TX thread function. It sends a CONTROL_MSG_TX_THREAD_n message
*  NUM_CONTROL_MSGS_PER_TX_THREAD times to the mqueue and ultimately one
*  CONTROL_MSG_EXIT_TX_THREAD_n message before terminating.
*
* Parameters:
*  arg: Not used
*
* Return:
*  void*
*
*********************************************************************************/
void* queue_send_thread(void *arg)
{
    int ret_val;
    int tx_thread = -1;
    char send_buffer[MQUEUE_MSG_SIZE] = {0};
    static int control_msg_count[NUM_TX_THREADS] = {0, 0};
    pthread_t thread_id = NULL;

    for (;;)
    {
        ret_val = pthread_mutex_trylock(&mutex_lock);
        if (0 == ret_val)
        {
            thread_id = pthread_self();

            if (pthread_equal(thread_id, tx_threads[0].tx_thread_id))
            {
                /* TX thread 1 */
                tx_thread = 0;

                if (control_msg_count[tx_thread] < NUM_CONTROL_MSGS_PER_TX_THREAD)
                {
                    send_buffer[0] = (char)CONTROL_MSG_TX_THREAD_1;
                }
                else
                {
                    send_buffer[0] = (char)CONTROL_MSG_EXIT_TX_THREAD_1;
                }
            }
            else if (pthread_equal(thread_id, tx_threads[1].tx_thread_id))
            {
                /* TX thread 2 */
                tx_thread = 1;

                if (control_msg_count[tx_thread] < NUM_CONTROL_MSGS_PER_TX_THREAD)
                {
                    send_buffer[0] = (char)CONTROL_MSG_TX_THREAD_2;
                }
                else
                {
                    send_buffer[0] = (char)CONTROL_MSG_EXIT_TX_THREAD_2;
                }
            }
            else
            {
                continue;
            }

            ret_val = mq_send(thread_queue, send_buffer, MQUEUE_MSG_SIZE, 0);
            if (0 == ret_val)
            {
                printf("TX thread %d iteration #[%d]: Sent CONTROL MESSAGE: %d to the mqueue\r\n",
                        tx_thread + 1, control_msg_count[tx_thread] + 1, (int)send_buffer[0]);
                control_msg_count[tx_thread]++;
            }
            else /* error while sending msg to mqueue */
            {
                printf("TX thread %d iteration #[%d]: mqueue error = %s\r\n",
                        tx_thread + 1, control_msg_count[tx_thread] + 1, strerror(errno));
            }

            /* unlock the mutex so that the other TX thread can use the mqueue */
            pthread_mutex_unlock(&mutex_lock);

            if (control_msg_count[tx_thread] > NUM_CONTROL_MSGS_PER_TX_THREAD)
            {
                pthread_exit(NULL);
            }
        }

        usleep(200000);
    }
}

/*********************************************************************************
* Function Name: queue_receive_thread
**********************************************************************************
* Summary:
*  This is the POSIX RX thread function. It retrieves control messages from the
*  mqueue and identifies which message belong to which TX thread. It also exits
*  after receiving the CONTROL_MSG_EXIT_TX_THREAD_n message from both the TX
*  threads.
*
* Parameters:
*  arg: Not used
*
* Return:
*  void*
*
*********************************************************************************/
void* queue_receive_thread(void *arg)
{
    int ret_val;
    char recv_buffer[MQUEUE_MSG_SIZE] = {0};
    static uint8_t exit_status = 0;

    for (;;)
    {
        ret_val = mq_receive(thread_queue, recv_buffer, MQUEUE_MSG_SIZE, NULL);
        if (0 <= ret_val)
        {
            /* Parse received messages from the mqueue */
            switch((int) recv_buffer[0])
            {
                case CONTROL_MSG_TX_THREAD_1:
                    printf("RX thread: Received CONTROL MESSAGE: %d from TX thread 1\r\n", CONTROL_MSG_TX_THREAD_1);
                    break;

                case CONTROL_MSG_TX_THREAD_2:
                    printf("RX thread: Received CONTROL MESSAGE: %d from TX thread 2\r\n", CONTROL_MSG_TX_THREAD_2);
                    break;

                case CONTROL_MSG_EXIT_TX_THREAD_1:
                    printf("RX thread: Received CONTROL EXIT MESSAGE: %d from TX thread 1\r\n", CONTROL_MSG_EXIT_TX_THREAD_1);
                    exit_status ^= 1 << 0;
                    break;

                case CONTROL_MSG_EXIT_TX_THREAD_2:
                    printf("RX thread: Received CONTROL EXIT MESSAGE: %d from TX thread 2\r\n", CONTROL_MSG_EXIT_TX_THREAD_2);
                    exit_status ^= 1 << 1;
                    break;

                default:
                    /* Received a message that we don't care or not defined. */
                    break;
            }
        }
        else
        {
            /* Invalid message / error */
            printf("RX thread: mqueue error = %s\r\n", strerror(errno));
        }

        if ((exit_status & 3) == 3 )
        {
            pthread_exit(NULL);
        }

        usleep(200000);
    }
}

/*********************************************************************************
* Function Name: queue_send_timer_callback
**********************************************************************************
* Summary:
*  This is the POSIX timer function. It toggles the User LED @ every 1 second
*  until the timer is deleted by the posix_demo rtos task.
*
* Parameters:
*  sigval arg: Not used
*
* Return:
*  void
*
*********************************************************************************/
static void queue_send_timer_callback(union sigval arg)
{
    CY_UNUSED_PARAMETER(arg);

    /* Toggle the User RGB LED every 1 second */
    cyhal_gpio_toggle(CYBSP_USER_LED);

}

/*********************************************************************************
* Function Name: posix_demo
**********************************************************************************
* Summary:
*  This is the POSIX demo rtos task. It creates/initializes 1 mqueue, 1 mutex,
*  and 1 timer. It spins up 2 TX and 1 RX threads. TX threads share a mqueue using
*  a mutex to exchange messages with the RX thread. RX thread retrieves the
*  messages from the mqueue and identify which TX thread sent the message. On
*  the other hand user RGB LED is toggled by 1 second timer. Ultimately posix_demo
*  task joins all the threads and release the resources.
*
* Parameters:
*  sigval arg: Not used
*
* Return:
*  void
*
*********************************************************************************/
void posix_demo(void *arg)
{
    int ret_val = 0;

    pthread_t rx_thread_id;
    pthread_attr_t rx_thread_attrib;

    static timer_t timer_id;

    static struct mq_attr queue_attrib;

    /* Create the timer structures */
    static struct sigevent signal_event;
    static struct itimerspec timer_attrib;

    const struct sched_param schedular_param =
    {
        .sched_priority = THREAD_PRIORITY
    };

    CY_UNUSED_PARAMETER(arg);

    /* Timer */
    /* clear everything in the timer structures. */
    memset(&signal_event, 0, sizeof(struct sigevent));
    memset(&timer_attrib, 0, sizeof(struct itimerspec));

    /* Set attributes for signal event structure */
    /*
     * Set the notification method as SIGEV_THREAD:
     *
     * Upon timer expiration, `sigev_notify_function` (queue_send_timer_callback()),
     * will be invoked as if it were the start function of a new thread.
     *
     */
    signal_event.sigev_notify = SIGEV_THREAD;
    signal_event.sigev_notify_function = &queue_send_timer_callback;
    signal_event.sigev_value.sival_ptr = (void *) timer_id;

    /* Set timeout attributes */
    timer_attrib.it_value.tv_sec = 1;    /* Timer expiration */
    timer_attrib.it_interval.tv_sec = 1; /* Timer period */

    /* Message Queue */
    /* Initialize message queue attributes */
    queue_attrib.mq_flags   = 0;
    queue_attrib.mq_maxmsg  = MQUEUE_MAX_NUMBER_OF_MSGS;
    queue_attrib.mq_msgsize = MQUEUE_MSG_SIZE ;
    queue_attrib.mq_curmsgs = 0;

    /* Open a mqueue with --
     * O_CREAT -- create a message queue.
     * O_RDWR  -- both receiving and sending messages.
     * O_EXCL  -- queue with the given name already exists, then fail with the error
     *            EEXIST.
     */
    thread_queue = mq_open(THREAD_QUEUE_NAME, O_CREAT | O_RDWR | O_EXCL, (mode_t)0, &queue_attrib);

    if ((mqd_t) -1 != thread_queue)
    {
        /* Create the mutex used by TX threads */
        ret_val = pthread_mutex_init(&mutex_lock, NULL);
        if (0 != ret_val)
        {
            CY_ASSERT(0);
        }

        /* Create the software timer */
        ret_val = timer_create(CLOCK_REALTIME, &signal_event, &timer_id);
        if (0 != ret_val)
        {
            CY_ASSERT(0);
        }

        /* Arm the timer. No flags are set and no old_value will be retrieved */
        ret_val = timer_settime(timer_id, 0, &timer_attrib, NULL);
        if (0 != ret_val)
        {
            printf("Failed to set timer timeout, errno = %s\r\n", strerror(ret_val));
            CY_ASSERT(0);
        }

        /* Initialize TX thread attributes and create the TX threads */
        for (uint8_t i = 0; i < NUM_TX_THREADS; i++)
        {
            ret_val = pthread_attr_init(&tx_threads[i].tx_thread_attrib);
            if (0 != ret_val)
            {
                CY_ASSERT(0);
            }

            ret_val = pthread_attr_setstacksize(&tx_threads[i].tx_thread_attrib, THREAD_STACK_SZ);
            if (0 != ret_val)
            {
                CY_ASSERT(0);
            }

            ret_val = pthread_attr_setschedparam(&tx_threads[i].tx_thread_attrib, &schedular_param);
            if (0 != ret_val)
            {
                CY_ASSERT(0);
            }

            ret_val = pthread_create(&tx_threads[i].tx_thread_id, &tx_threads[i].tx_thread_attrib, queue_send_thread, NULL);
            if (0 != ret_val)
            {
                CY_ASSERT(0);
            }
        }

        /* Initialize RX thread attributes */
        ret_val = pthread_attr_init(&rx_thread_attrib);
        if (0 != ret_val)
        {
            CY_ASSERT(0);
        }

        /* Set RX thread stack size */
        ret_val = pthread_attr_setstacksize(&rx_thread_attrib, THREAD_STACK_SZ);
        if (0 != ret_val)
        {
            CY_ASSERT(0);
        }

        /* Set RX thread priority */
        ret_val = pthread_attr_setschedparam(&rx_thread_attrib, &schedular_param);
        if (0 != ret_val)
        {
            CY_ASSERT(0);
        }

        /* Create the RX thread */
        ret_val = pthread_create(&rx_thread_id, &rx_thread_attrib, queue_receive_thread, NULL);
        if (0 != ret_val)
        {
            CY_ASSERT(0);
        }

        /* Wait for TX threads to join */
        for (uint8_t i = 0; i < NUM_TX_THREADS; i++)
        {
            (void) pthread_join(tx_threads[i].tx_thread_id, NULL);
        }

        /* Wait for RX thread to join */
        (void) pthread_join(rx_thread_id, NULL);

        /* delete the mutex */
        (void) pthread_mutex_destroy(&mutex_lock);

        /* delete the timer */
        (void) timer_delete(timer_id);

        /* Close and unlink message queue */
        if (thread_queue != NULL)
        {
            (void) mq_close(thread_queue);
            (void) mq_unlink(THREAD_QUEUE_NAME);
        }

        printf("All POSIX resources (threads, mutex, timer, and mqueue) are finished/deleted!\r\n");
    }
    else
    {
        printf("mqueue did not get initialized properly. Did not run demo.\r\n");
    }

    /* This task was created with the native xTaskCreate() API function, so
    must not run off the end of its implementing thread. */
    vTaskDelete( NULL );
}

/* [] END OF FILE */

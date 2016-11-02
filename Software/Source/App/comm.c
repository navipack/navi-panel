/**
******************************************************************************
* @file    comm.c
* @author  Jalon
* @date    2016/02/01
* @brief   通讯协议传输层解包封包等相关函数
* @attention Copyright (C) 2016 Inmotion
******************************************************************************
*/

#include "comm.h"
#include "comm_usart.h"
#include "global_defines.h"
#include "queue.h"
#include "tim_user.h"
#include "gpio_user.h"

#define TALK_MAX 1
#define TALK_MSG_BOX_SIZE 10
#define UART_BUFF_SIZE  128

#define NAVI_MAX(a, b) (a>b?a:b)
#define NAVI_MAX_BLOCK (u32)(NAVI_MAX(NAVI_MAX( sizeof(NaviPack_CtrlType), sizeof(NaviPack_StatusType)), sizeof(NaviPack_ConfigType)))
#define NAVIPACK_COMM_SIZE (NAVI_MAX_BLOCK + sizeof(NaviPack_HeadType) + 1)

NavipackComm_Type NavipackComm;
NavipackUserType UserReg;

static CommUsartType CommUsart;
static u8 UartBuffer[UART_BUFF_SIZE];

static QueueType TxQueue;
static NaviPack_HeadType TxQueuePool[10];

static u8 CommTxBuffer[NAVIPACK_COMM_SIZE*2+6];
static u8 CommRxBuffer[NAVIPACK_COMM_SIZE];

/**
* @brief  通讯传输层初始化
* @param  None
* @retval 是否成功
*/
bool Comm_Init(void)
{
    NavipackComm_Type *comm = &NavipackComm;
    comm->rxBuffer = CommRxBuffer;
    comm->txBuffer = CommTxBuffer;
    comm->rxSize = sizeof(CommRxBuffer);
    comm->txSize = sizeof(CommTxBuffer);
    comm->rxDataLen = 0;
    comm->txDataLen = 0;

    Queue_Init(&TxQueue, TxQueuePool, sizeof(TxQueuePool), sizeof(TxQueuePool[0]));
    
    NaviPack_Init();
    
    if(GetCommMode())
    {
        GlobalParams.commMode = COMM_UART;
        CommUsart.buffer_size = UART_BUFF_SIZE;
        CommUsart.dma_rx_buffer = UartBuffer;
    }
    else
    {
        GlobalParams.commMode = COMM_USB;
    }

    return true;
}

/**
* @brief  获得串口句柄
* @retval 串口句柄
*/
void* GetCommUartHandle(void)
{
    return &CommUsart;
}

/**
* @brief  数据接收
* @param  data : 数据
* @retval None
*/
void Comm_RecvPackage(u8 data)
{
    NaviPack_RxProcessor(&NavipackComm, data);
}

/**
* @brief  通讯发送处理 Task
* @param  p_arg: 参数
* @retval None
*/
void Comm_TxTask(void)
{
    NaviPack_HeadType head;
    
    if(Queue_Query(&TxQueue, &head))
    {
        if(NaviPack_TxProcessor(&NavipackComm, &head))
        {
            Queue_Get(&TxQueue, NULL);
        }
    }
}

/**
* @brief  通讯接收处理 Task
* @param  None
* @retval None
*/
void Comm_RxTask(void)
{
    u8 *data;
    u32 len, i;
    
    if(GlobalParams.commMode == COMM_UART)
    {
        if(CommUsart_RecvData(&CommUsart, &data, &len))
        {
            for(i=0; i<len; i++)
            {
                Comm_RecvPackage(data[i]);
            }
        }
    }
    else if(CDC_ReceiveData(&data, &len) == USBD_OK)
    {
        for(i=0; i<len; i++)
        {
            Comm_RecvPackage(data[i]);
        }
    
        CDC_StartReceiveData();
    }
}

/**
* @brief  推送一个发送事件
* @param  handle  : 对象编号
* @param  head    : 数据头指针
* @retval 是否成功
*/
bool Comm_PostTxEvent(NaviPack_HeadType *head)
{
    Queue_Put(&TxQueue, head);
    return true;
};

/**
* @brief  通讯定时发送 Task
* @param  p_arg: 参数
* @retval None
*/
void Comm_BoostTask(void)
{
    static NaviPack_HeadType head = {
            NAVIPACK_SLAVE_ID,
            FUNC_ID_READ_STATUS,
            0,
            sizeof(NaviPack_StatusType),
        };
    
    if(RunFlag.boost)
    {
        RunFlag.boost = 0;
            
        Comm_PostTxEvent(&head);
    }
}

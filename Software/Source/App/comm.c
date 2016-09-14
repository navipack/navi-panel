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

#define TALK_MAX 1
#define TALK_MSG_BOX_SIZE 10

#define NAVI_MAX(a, b) (a>b?a:b)
#define NAVI_MAX_BLOCK (u32)(NAVI_MAX(NAVI_MAX( sizeof(NaviPack_CtrlType), sizeof(NaviPack_StatusType)), sizeof(NaviPack_ConfigType)))
#define NAVIPACK_COMM_SIZE (NAVI_MAX_BLOCK + sizeof(NaviPack_HeadType) + 1)

NavipackComm_Type NavipackComm;
NavipackUserType UserReg;

//static TalkFuncType TalkObj[TALK_MAX];

//static OS_EVENT *CommTxEvent;
//static void *TxEventBox[TALK_MSG_BOX_SIZE];

//static OS_MEM *CommTxMemo;
//static NaviPack_HeadType PackHeadPool[TALK_MSG_BOX_SIZE];

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

    return true;
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
    
    if(Queue_Query(&TxQueue, &head) && CommUsart_CanSendData())
    {
        if(NaviPack_TxProcessor(&NavipackComm, &head))
        {
            Queue_Get(&TxQueue, NULL);
        }
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
    CommUsart_EnableIT(false);
    Queue_Put(&TxQueue, head);
    CommUsart_EnableIT(true);
    return true;
};

/**
* @brief  通讯定时发送 Task
* @param  p_arg: 参数
* @retval None
*/
void Comm_BoostTask(void)
{
    u16 cnt = 0;
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

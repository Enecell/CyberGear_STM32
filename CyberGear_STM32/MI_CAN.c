#include "MI_CAN.h"                  // Device header

/*----------------小米电机CAN初始化函数-----------------*/



/*———————————————————————————————————————————————————
Name:MI_CAN_Init   CAN 初始化函数
Param：No
Init Pin：GPIOA P11   CAN_RX，输入引脚是A11
          GPIOA P12   CAN_TX，输出引脚是A12         */
////注意A板使用的can1引脚为：CAN1 RX-PD0 与 TX-PD1
					
void MI_CAN_Init(void)
{
	GPIO_InitTypeDef gpio_can1; 
	CAN_InitTypeDef        can1_1;//can控制器
  	CAN_FilterInitTypeDef  can1_2;//can过滤器
	
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/**********************开启GPIOA和CAN1时钟************************************/
	
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORTA时钟	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	
	/***********************初始化GPIOA的11,12号引脚，CAN 输出/输入***********************/
	
	
//  	gpio_can1.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	gpio_can1.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
    gpio_can1.GPIO_Mode = GPIO_Mode_AF;//复用功能
    gpio_can1.GPIO_OType = GPIO_OType_PP;//推挽输出
    gpio_can1.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    gpio_can1.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//    GPIO_Init(GPIOA, &gpio_can1);//初始化GPIOPA11,PA12
	GPIO_Init(GPIOD, &gpio_can1);//初始化GPIOD0,GPIOD1
	
	
//	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
//	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOD0复用为CAN1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOD1复用为CAN1
	

	
	
	
	/*************************初始化CAN1控制器************************************/
	
	/*CAN工作模式选择
    CAN_Mode_Normal             ((uint8_t)0x00)  正常模式
    CAN_Mode_LoopBack           ((uint8_t)0x01)  环回模式
	CAN_Mode_Silent             ((uint8_t)0x02)  静默模式，只听不发
    CAN_Mode_Silent_LoopBack    ((uint8_t)0x03)  环回静默模式*/
	//CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;//CAN工作模式为环回模式，用于自收自发测试用,实际通信时可以改成正常模式
	
//	can_1.CAN_Mode =  CAN_Mode_LoopBack ;
	can1_1.CAN_Mode =  CAN_Mode_Normal ;
	
	/*波特率计算
	fi03c8t6;
	波特率 = 36M（时钟频率） / 48 （预分频器的值）/ (1 + 2（BS1）+ 3（BS2）) = 125K    
	波特率为1Mb，小米电机的波特率就是1Mb=36M/6/（1+3+2）
	
	f407vet6;
	APB1max时钟频率42MHz,APB2max时钟频率84MHz.
	42M/6/(3+3+1)=1M
	
	f427hiih6:
	45M/6/(1+2+3)=15/12M
	45M/3/(10+4+1)=1M
	
	*/
	can1_1.CAN_Prescaler = 3; //预分频器的值为12
	can1_1.CAN_BS1 = CAN_BS1_10tq; //BS1的时间长度，1-16tq
	can1_1.CAN_BS2 = CAN_BS2_4tq; //BS2的时间长度，1-8tq
	can1_1.CAN_SJW = CAN_SJW_1tq; //SJW的时间长度，1-4tq
  //*/
	
	can1_1.CAN_NART = DISABLE;//DISABLE，表示寄存器置0，表示自动重传，这个功能叫做不自动重传，一般系统默认是自动重传
	can1_1.CAN_TXFP = DISABLE;//发送邮箱优先级，DISABLE置零，ID小的先发送，如果是ENABLE置一，先进先出
	can1_1.CAN_RFLM = DISABLE;//禁用FIOF锁定，溢出后，新报文覆盖最后一个报文，如果是ENABLE，则溢出时新报文丢弃
	can1_1.CAN_AWUM = DISABLE;//DISABLE 手动唤醒，ENABLE 自动唤醒
	can1_1.CAN_TTCM = DISABLE;//关闭时间触发通信
	can1_1.CAN_ABOM = DISABLE;//DISABLE 手动恢复，ENABLE 自动恢复（离线自动恢复）
	CAN_Init(CAN1, &can1_1);//初始化CAN
	/*****************************初始化CAN过滤器*********************************/
	
	can1_2.CAN_FilterNumber = 0;//过滤器编号，0-13
	/*几种不同的
	16位列表模式，四个参数分别存入一组ID即可，共四个16位ID列表
	屏蔽模式：IDHIGH存入第一组ID，MaskIDHIGH存入对应的屏蔽位，共两组16位ID和两组屏蔽位
	32位列表模式：IDHIGH和IDLOW组合成一个32位ID，MaskIDHIGH和MaskIDLOW组成第二组32位ID
	32位频闭模式：IDHIGH和IDLOW组合成一个32位ID，MaskIDHIGH和MaskIDLOW组成第二组32位屏蔽位
	*/
	can1_2.CAN_FilterIdHigh = 0x0000;//高16位
	can1_2.CAN_FilterIdLow = 0x0000;//低16位
	can1_2.CAN_FilterMaskIdHigh = 0x0000;
	can1_2.CAN_FilterMaskIdLow = 0x0000;
	//选则相应模式为32位屏蔽模式
	can1_2.CAN_FilterScale = CAN_FilterScale_32bit;//过滤器位宽，32位或者16位，这里是32位
	can1_2.CAN_FilterMode = CAN_FilterMode_IdMask;//过滤器模式，CAN_FilterMode_IdMask 屏蔽模式 CAN_FilterMode_IdList  列表模式
	can1_2.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//配置过滤器关联，这里有两个CAN_Filter_FIFO0，CAN_Filter_FIFO1
	can1_2.CAN_FilterActivation = ENABLE;//激活过滤器
	CAN_FilterInit(&can1_2);//初始化过滤器
	
	#if CAN1_RX0_INT_ENABLE
	
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
	
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	#endif
	
}

/***********************************************************
Name：MI_CAN_Transmit CAN 发送报文
Param： ID       ID是32位，便于后面使用扩展帧
        Length   数据长度
        *Data    数据指针                                     
		******************************************************/
void MI_CAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
	CanTxMsg TxMessage; //定义CanTxMsg结构体变量，表示待发送的报文
	TxMessage.StdId = ID; //标准ID
	TxMessage.ExtId = ID; //扩展ID
	//TxMessage.IDE = CAN_Id_Standard; //扩展标志位，CAN_Id_Standard 标准ID ，CAN_Id_Extended扩展ID
	TxMessage.IDE = CAN_Id_Extended; //扩展标志位，CAN_Id_Standard 标准ID ，CAN_Id_Extended扩展ID
	TxMessage.RTR = CAN_RTR_Data; //遥控标志位，CAN_RTR_Remote 遥控帧，	CAN_RTR_Data数据帧
	TxMessage.DLC = Length; //数据段长度，传入的参数
	//把形参DATA传过来的数组赋值给TxMessage.Data
	for(uint8_t i=0;i<Length;i++)
	{
		TxMessage.Data[i] = Data[i];//将传入的Data数组的值赋值给结构体的Data，他们都是8字节的数组
	}
	//请求发送报文函数
	//CAN_Transmit的原理：选择空发送邮箱——如果邮箱有空位，则将报文写入指定寄存器——TXRQ置1，请求发送
	uint8_t TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);//请求发送结构体指向的报文，返回值是邮箱编号
	uint32_t Timeout = 0;
	//CAN_TransmitStatus表示返回传输状态函数，返回请求发送邮箱的邮箱状态，CAN_TxStatus_OK表示发送成功
	//等待函数返回OK，当CAN1的邮箱状态为CAN_TxStatus_Ok表示发送成功，如果不成功则进入循环
	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok)
	{
		Timeout ++;
		//如果大于超时时间，则跳出循环
		if(Timeout > 2047)
		{
			break;
		}
	}
}

/************************************************************************
* @name: MI_CAN_ReceiveFlag
* @brief 函数用于判断接收FIFO里是否有报文，返回值为表示有报文，返回值为0表示没有报文  
*************************************************************************/
uint8_t MI_CAN_ReceiveFlag(void)
{
	if(CAN_MessagePending(CAN1, CAN_FIFO0) > 0)//如果大于零，表明FIFO0里面有报文，此处的FIFO0与前面的设置一致
	{
		return 1;
	}
	return 0;
}

/*****************************************
@brief: 接收 CAN message
@param： *ID        the ID，用于做返回值
         *Length    the length of DATA，用于做返回值
         *DATA      the Data of CAN massage
注意，接收信息是需要输出参数，但C语言不支持多个值输出，
所以这里用指针表示，可以通过函数修改相应的值
*****************************************/
void MI_CAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
	CanRxMsg RxMessage;//定义一个CanRxMsg结构体，用于存放接收报文
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//接收报文
	//判断接收的报文是标准ID还是扩展ID
	if(RxMessage.IDE == CAN_Id_Standard)
	{
		*ID = RxMessage.StdId;//标准ID
	}
	else
	{
		*ID = RxMessage.ExtId; //扩展ID
	}
	//判断接收报文是否为数据帧还是遥控帧
	if(RxMessage.RTR == CAN_RTR_Data)//是否为数据帧
	{
		//数据帧
		*Length = RxMessage.DLC;//数据长度
		//数据内容
		for (uint8_t i = 0; i < *Length; i ++)
		{
			Data[i] = RxMessage.Data[i];
		}
	}
	else
	{
		//遥控帧，暂时不做处理
	}
}

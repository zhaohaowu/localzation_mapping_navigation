#include <cstdlib>
#include <sstream>
#include <iostream>

#include <time.h>
#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "ros/ros.h"

#include "std_msgs/String.h"

#include "OsDeclarations.h"
#include "EMCB.h"

#include "can_com/CanCtrlInfo.h"

//#define SHOW_DETAIL_INFO
using namespace std;

void * RxThread(void *pContext);
void * TxThread(void *pContext);

int f_EID = 0;
int f_IDPrecep  = 0x31;
int f_IDControl = 0x21;
int f_ID  = 0;                //frame ID
int f_RTR = 0;
int f_baudrate  = 500;
int f_chdir = 0;              //ch0 dir
int f_singledir = 0;          //ch0 dir
unsigned int f_mask0 = 0, f_mask1 = 0;
unsigned int f_mask0_EID = 0, f_mask1_EID = 0;
unsigned int f_filter0 = 0, f_filter1 = 0,f_filter2 = 0, f_filter3 = 0,f_filter4 = 0, f_filter5 = 0;
unsigned int f_filter0_EID = 0, f_filter1_EID = 0,f_filter2_EID = 0, f_filter3_EID = 0,f_filter4_EID = 0, f_filter5_EID = 0;
unsigned int f_busmode    = 0;
unsigned int f_rx0bufmode = 0;
unsigned int f_rx1bufmode = 0;

typedef struct _CommOption
{
	BYTE Ch  ;            //CAN channel choice
	unsigned int ID;
	BYTE EID ;
	BYTE RTR ;            //Select this message type.
	BYTE Len ;            //The maximum value is eight. 8 bit
	unsigned char Data[8];//Pointer to a buffer that the received data.
	                      //Pointer to a buffer that the data will be sent.
} CommOption, *pCommOption;

void sigint_handler(int sig){
        ROS_INFO("shutting down!");
        EMCBLibUninitialize();	
        ros::shutdown();
}

int main(int argc, char** argv)
{  
	ros::init(argc, argv, "can_capture_emcb");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<can_com::CanCtrlInfo>("emcb_info");
	can_com::CanCtrlInfo srv;

	signal(SIGINT, sigint_handler);

   	unsigned int nRet = 0;
	int i = 0;

	CommOption rxOp;      //read_class
	CommOption txOp;      //trans_class
    /* check */
	{
	if (EMCBLibInitialize() == (unsigned int)EMCB_STATUS_NOT_INITIALIZED) // find device
	{
		printf("Device not found!\n");
	}

	unsigned int libVer = 0;
	if(EMCBDeviceGetValue(EMCB_IID_LIB_VER, &libVer) != EMCB_STATUS_SUCCESS)
	{
		printf("Get device value failed!\n");
	}
	else
		printf("Lib version is %d.%d.%d\r\n",(libVer>>24)&0xff, (libVer>>16)&0xff, libVer&0xffff);

	unsigned int chNum = 0;
	if(EMCBDeviceGetValue(EMCB_IID_CHANNEL_NUM, &chNum) != EMCB_STATUS_SUCCESS)
	{
		printf("Get device value failed!\n");
	}
	else
		printf("Device channel is %d\r\n",chNum);

	for(i = 0; i<chNum; i++)
	{
		nRet = EMCBReset(i);
		if( nRet!= EMCB_STATUS_SUCCESS)
		{
			printf("Reset channel failed!\n");
		}
	}

	for(i = 0; i<chNum; i++)
	{
		nRet = EMCBSetChannelValue(i, EMCB_IID_TERMINAL_RES, 1);
		if( nRet!= EMCB_STATUS_SUCCESS)
		{
			printf("Set channel %d terminal resistor value failed!\n",i);
		}
		nRet = EMCBSetChannelValue(i, EMCB_IID_BUS_SPEED, f_baudrate);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d speed failed!\n",i);
		}
		nRet = EMCBSetChannelValue(i, EMCB_IID_BUS_MODE, f_busmode);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d bus mode failed!\n",i);
		}
	}
	
	}
    /* channel double or single */
	{
	if(f_chdir)
	{
		rxOp.Ch = 1;  //channel 1 is rx
		txOp.Ch = 1;  //
	}
	else
	{	
		rxOp.Ch = 0;
		txOp.Ch = 0;
	}
	}	
	
	txOp.EID = f_EID;     //f_EID = 0; ????
	txOp.ID  = f_ID ;     //f_ID = 0;  ?ID
	txOp.RTR = f_RTR;     //f_RTR = 0; ????
	
	txOp.Len = 8;
	rxOp.EID = f_EID;
	rxOp.Len = 8;

	//filter settings
	{	
	//set(f_mask0 || f_mask0_EID)
	{
		nRet = EMCBSetMask(rxOp.Ch, 0, f_mask0,f_mask0_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d mask0 failed!\n",rxOp.Ch);
		}
	}
	//set(f_mask1 || f_mask1_EID)
	{
		nRet = EMCBSetMask(rxOp.Ch, 1, f_mask1,f_mask1_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d mask1 failed!\n",rxOp.Ch);
		}
	}
	//set(f_filter0 || f_filter0_EID)
	{
		nRet = EMCBSetFilter(rxOp.Ch, 0, f_filter0,f_filter0_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d filter0 failed!\n",rxOp.Ch);
		}
	}
	//set(f_filter1 || f_filter1_EID)
	{
		nRet = EMCBSetFilter(rxOp.Ch, 1, f_filter1,f_filter1_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d filter1 failed!\n",rxOp.Ch);
		}
	}
	//set(f_filter2 || f_filter2_EID)
	{
		nRet = EMCBSetFilter(rxOp.Ch, 2, f_filter2,f_filter2_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d filter2 failed!\n",rxOp.Ch);
		}
	}
	//set(f_filter3 || f_filter3_EID)
	{
		nRet = EMCBSetFilter(rxOp.Ch, 3, f_filter3,f_filter3_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d filter3 failed!\n",rxOp.Ch);
		}
	}
	//set(f_filter4 || f_filter4_EID)
	{
		nRet = EMCBSetFilter(rxOp.Ch, 4, f_filter4,f_filter4_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d filter4 failed!\n",rxOp.Ch);
		}
	}
	//set(f_filter5 || f_filter5_EID)
	{
		nRet = EMCBSetFilter(rxOp.Ch, 5, f_filter5,f_filter5_EID);
		if( nRet!= EMCB_STATUS_SUCCESS)	
		{
			printf("Set channel %d filter5 failed!\n",rxOp.Ch);
		}
	}
	}
	
	unsigned int tResistorState = 0;
    /* check */
	{
	unsigned int chNum = 0;	  
	for(i = 0; i<chNum; i++)
	{
		nRet = EMCBGetChannelValue(i, EMCB_IID_TERMINAL_RES, &tResistorState);
		if( nRet!= EMCB_STATUS_SUCCESS)
		{
			
			if(nRet == EMCB_STATUS_NOT_INITIALIZED)
				printf("not init!\n");
			else if(nRet == EMCB_STATUS_UNSUPPORTED)
				printf("unsupported!\n");
			else if(nRet == EMCB_STATUS_READ_ERROR)
				printf("read error!\n");
			else if(nRet == EMCB_STATUS_ERROR)
				printf("status error!\n");
			else
				printf("Get channel value failed!\n");
		}
		else
		{
			if(tResistorState == EMCB_TR_DISABLE)
				printf("Device channel %d status is false\r\n", i);
			else
				printf("Device channel %d status is true\r\n", i);
		}
		unsigned int bMode = 0;
		nRet = EMCBGetChannelValue(i, EMCB_IID_BUS_STATUS, &bMode);
		if( nRet!= EMCB_STATUS_SUCCESS)
		{
			if(nRet == EMCB_STATUS_NOT_INITIALIZED)
				printf("not init!\n");
			else if(nRet == EMCB_STATUS_UNSUPPORTED)
				printf("unsupported!\n");
			else if(nRet == EMCB_STATUS_READ_ERROR)
				printf("read error!\n");
			else if(nRet == EMCB_STATUS_ERROR)
				printf("status error!\n");
			else
				printf("Get channel value failed!\n");
		}
		else
		{
			printf("Channel %d bus mode is %d\r\n",i,bMode);
		}
	}
	
	}
			
	//pthread_t rxTid;
	//pthread_t txTid;
	
	CommOption *txData = (CommOption *)&txOp;
	CommOption *rxData = (CommOption *)&rxOp;
	
	//set 120 oumu
 	EMCB_API EMCBReset(EMCB_TR_ENABLE);
	memset(txData->Data, 0, sizeof(txData->Data)); //memset
	
	ros::Rate loop_rate(100);
		      
        while(nh.ok())
        {
  
#ifdef SHOW_DETAIL_INFO
		printf("RRRRRx CH %d, ID %d, EID %d, RTR %d\n",rxData->Ch,rxData->ID,rxData->EID,rxData->RTR);
#endif
		// rxData->ID = 0x31;
		nRet = EMCBMsgRx(rxData->Ch, &rxData->ID, &rxData->EID, &rxData->RTR, rxData->Data, &rxData->Len);
		//std::cout<<"nRet: "<<nRet<<std::endl;		
		if( nRet!= EMCB_STATUS_SUCCESS){
			switch(nRet){
				case EMCB_STATUS_TIMEOUT:
				printf("Read data timeout !");
				break;
				case EMCB_STATUS_READ_ERROR:
				printf("Read data timeout !");
				break;
				case EMCB_STATUS_NO_DATA:
				printf("Read no data !");
				break;int main(int argc, char **argv);
				default:
				//printf("Read data failed 0x%x",nRet);
				break;
			}
		}
		/* printf any data received */
		/*
		else
		{
			int j = 0;
			for(j = 0; j<rxData->Len; j++)
			{
				if(f_singledir == 0)
				{
					printf("Data[%d] 0x%x; ", j, rxData->Data[j]);
				}
				else
				{
					printf("RxData[%d] 0x%x; ", j, rxData->Data[j]);
				}
			}
		}
		*/
		rxData->Len = 8;
		printf("\r\n");
				
		//request service
		if(rxOp.ID==f_IDPrecep)
		{
			for(int p = 0; p<8; p++)
			{
				srv.request.cRate[p] = rxData->Data[p];
				//printf("crRight[%d] 0x%x; ", i, srv.request.crRightRate[i]);
			}				
		}
                //wait service
		if (client.call(srv))
		{
			for(i = 0; i<8; i++)
			{			  
				txData->Data[i]=srv.response.sControl[i];
				//ROS_INFO("Actsf:0x %x", srv.response.sControl[i]);
				txOp.ID=f_IDControl;
				
#ifdef SHOW_DETAIL_INFO
		printf("TTTTTx CH %d, ID %d, EID %d, RTR %d\n",txData->Ch,txData->ID,txData->EID,txData->RTR);
#endif    
				nRet = EMCBMsgTx(txData->Ch,txData->ID,txData->EID,txData->RTR,(BYTE *)txData->Data,txData->Len);	
				//std::cout<<"nRet: "<<nRet<<std::endl;
				if( nRet!= EMCB_STATUS_SUCCESS) 
				{
					printf("Send data failed!");
				}
				if(f_singledir == 2)
				{
					printf("\r\n");
				}
				txOp.ID=f_ID;		
			}
		}
		else
		{
		        ROS_ERROR("Failed to call service act_motor_raid");
		}
		rxData->Len = 8;
		printf("\r\n");    			
		usleep(1);  			
	}
	ros::spin();	 			
	EMCBLibUninitialize();
	return 0;
}

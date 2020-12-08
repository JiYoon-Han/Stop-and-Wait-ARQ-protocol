#include "mbed.h"
#include "string.h"

#include "ARQ_FSMevent.h"
#include "ARQ_msg.h"
#include "ARQ_timer.h"
#include "ARQ_LLinterface.h"
#include "ARQ_parameters.h"

//FSM state -------------------------------------------------
#define MAINSTATE_IDLE              0
#define MAINSTATE_TX                1
#define MAINSTATE_waitACK			2

//GLOBAL variables (DO NOT TOUCH!) ------------------------------------------
//serial port interface
Serial pc(USBTX, USBRX);

//state variables
uint8_t main_state = MAINSTATE_IDLE; //protocol state

//source/destination ID
uint8_t endNode_ID=1;
uint8_t dest_ID=0;

//PDU context/size
uint8_t arqPdu[200];
uint8_t pduSize;
uint8_t AckSize; 

//SDU (input)
uint8_t originalWord[200];
uint8_t wordLen=0;



//ARQ parameters -------------------------------------------------------------
uint8_t seqNum = 0;     //ARQ sequence number
uint8_t retxCnt = 0;    //ARQ retransmission counter
uint8_t arqAck[5];      //ARQ ACK PDU


//application event handler : generating SDU from keyboard input
void arqMain_processInputWord(void)
{
    char c = pc.getc();
    if (main_state == MAINSTATE_IDLE &&
        !arqEvent_checkEventFlag(arqEvent_dataToSend))
    {
        if (c == '\n' || c == '\r')
        {
            originalWord[wordLen++] = '\0';
            arqEvent_setEventFlag(arqEvent_dataToSend);
            pc.printf("word is ready! ::: %s\n", originalWord);
        }
        else
        {
            originalWord[wordLen++] = c;
            if (wordLen >= ARQMSG_MAXDATASIZE-1)
            {
                originalWord[wordLen++] = '\0';
                arqEvent_setEventFlag(arqEvent_dataToSend);
                pc.printf("\n max reached! word forced to be ready :::: %s\n", originalWord);
            }
        }
    }
}




//FSM operation implementation ------------------------------------------------
int main(void){
    uint8_t flag_needPrint=1;
    uint8_t prev_state = 0;

    //initialization
    pc.printf("------------------ ARQ protocol starts! --------------------------\n");
    arqEvent_clearAllEventFlag();
    
    //source & destination ID setting
    pc.printf(":: ID for this node : ");
    pc.scanf("%d", &endNode_ID);
    pc.printf(":: ID for the destination : ");
    pc.scanf("%d", &dest_ID);
    pc.getc();

    pc.printf("endnode : %i, dest : %i\n", endNode_ID, dest_ID);

    arqLLI_initLowLayer(endNode_ID);
    pc.attach(&arqMain_processInputWord, Serial::RxIrq);


    while(1)
    {
        //debug message
        if (prev_state != main_state)
        {
            debug_if(DBGMSG_ARQ, "[ARQ] State transition from %i to %i\n", prev_state, main_state);
            prev_state = main_state;
        }


        //FSM should be implemented here! ---->>>>
        switch (main_state)
        {
            case MAINSTATE_IDLE: //IDLE state description
                
                if (arqEvent_checkEventFlag(arqEvent_dataRcvd)) //if data reception event happens, RX PDU가 들어오면 
                {
                    //Retrieving data info.
                    uint8_t srcId = arqLLI_getSrcId(); 
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    uint8_t size = arqLLI_getSize();

                    pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                                srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));
					//상대편에서 PDU 받은 것에 대한 정보를 출력해 준다. 

					//ACK PDU gen
					AckSize = arqMsg_encodeAck(arqAck, arqMsg_getSeq(dataPtr));//그냥 바로 dataPtr[1] 해도 되는지 ? 

					//DATA_REQ
					arqLLI_sendData(arqAck, AckSize, dest_ID);

					//state transition
                    main_state = MAINSTATE_TX; //곧바로 ACK을 보내야 함. 
                    
                    flag_needPrint = 1;

                    arqEvent_clearEventFlag(arqEvent_dataRcvd);
                }
				
                else if (arqEvent_checkEventFlag(arqEvent_dataToSend)) //if data needs to be sent (keyboard input) //keyboard에서 input이 들어오면, 즉 SDU가 내려오면, 
                {
                    //msg header setting
                   	//seqNum++; 
                    pduSize = arqMsg_encodeData(arqPdu, originalWord, seqNum, wordLen); //encode 해서, PDU 생성하는 부분 
                    arqLLI_sendData(arqPdu, pduSize, dest_ID); //lower layer와의 상호작용 역할 해서 PDU 전송 하라는 함수
					retxCnt=0;
					
                    seqNum++; 
                    
                    pc.printf("[MAIN] sending to %i (seqNum:%i)\n", dest_ID, (seqNum-1)%ARQMSSG_MAX_SEQNUM); //보내는 node의 ID, PDU의 seqNum 출력 

                    main_state = MAINSTATE_TX; //SDU 들어오면 PDU생성 한 뒤 tx state로 바꿔주는 것  
                    flag_needPrint = 1;

                    wordLen = 0;
                    arqEvent_clearEventFlag(arqEvent_dataToSend); 
                }
				
                else if (flag_needPrint == 1)
                {
                    pc.printf("Give a word to send : ");
                    flag_needPrint = 0;
                }     

                break;

            case MAINSTATE_TX: //IDLE state description

                if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) //data TX finished
                {
					pc.printf("[MAIN] ACK received!!!!\n"); //ACK message에는 수신한 PDU의 번호 명시

					if(arqTimer_getTimerStatus()==0) //timer off이면 idle로 돌아론다. 
					{
						main_state = MAINSTATE_IDLE; //ACK 다 보냈으면 tx는 다시 idle상태로 돌아간다. 
                    	arqEvent_clearEventFlag(arqEvent_ackTxDone); 
					}
					
					else if(arqTimer_getTimerStatus()||arqEvent_checkEventFlag(arqEvent_arqTimeout)) //timer on이면 다시 ack wait상태로 돌아간다. 
					{
						//arqTimer_stopTimer();
						main_state = MAINSTATE_waitACK;
						arqEvent_clearEventFlag(arqEvent_ackTxDone); 
					}
                }

				else if(arqEvent_checkEventFlag(arqEvent_dataTxDone))
				{
					//timer만 시작하고 state만 바꾸면 okay
					arqTimer_startTimer(); //PDU 보냈으니까 timer 시작 
					//retxCnt=0; 

					main_state = MAINSTATE_waitACK; 
					arqEvent_clearEventFlag(arqEvent_dataTxDone); 
				}
				
                break;

			case MAINSTATE_waitACK: //waiting for ACK 

				if(arqEvent_checkEventFlag(arqEvent_dataRcvd)) //rx PDU 받은 경우라서 ACK를 만들어서 다시 보내주어야함. 
				{
					uint8_t srcId = arqLLI_getSrcId(); 
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    uint8_t size = arqLLI_getSize();

                    pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                                srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));
					//상대편에서 PDU 받은 것에 대한 정보를 출력해 준다. 

					//ack 생성하는 부분 
					AckSize = arqMsg_encodeAck(arqAck, arqMsg_getSeq(dataPtr));
					arqLLI_sendData(arqAck, AckSize, dest_ID);
					
					main_state = MAINSTATE_TX; 
					arqEvent_clearEventFlag(arqEvent_dataRcvd);
				}

				else if(arqEvent_checkEventFlag(arqEvent_ackRcvd)) //ack를 받은 경우 
				{
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
					if(arqMsg_getSeq(dataPtr)==arqMsg_getSeq(arqPdu))
					{
						pc.printf("[MAIN] ACK received for the seq %d\n", arqMsg_getSeq(dataPtr));
						arqTimer_stopTimer();
						
						main_state = MAINSTATE_IDLE;
					}
					
					arqEvent_clearEventFlag(arqEvent_ackRcvd);
				}

				else if(arqEvent_checkEventFlag(arqEvent_arqTimeout))
				{
					pc.printf("time out!!!\n");
					
					if(retxCnt<ARQ_MAXRETRANSMISSION)
					{ 
						pc.printf("retransmission! %d\n", seqNum);
						arqLLI_sendData(arqPdu, pduSize, dest_ID); //lower layer와의 상호작용 역할 해서 PDU 전송 하라는 함수 
						retxCnt++; 
						//pc.printf("[MAIN] sending to %i (seqNum:%i)\n", dest_ID, (seqNum-1)%ARQMSSG_MAX_SEQNUM); //보내는 node의 ID, PDU의 seqNum 출력 
						
						main_state = MAINSTATE_TX;
					}
					else
					{
						pc.printf("max retransmission! give up!!\n");
						main_state = MAINSTATE_IDLE; 
					}

					arqEvent_clearEventFlag(arqEvent_arqTimeout);
				}
				
            default :
                break;
        }
    }
}

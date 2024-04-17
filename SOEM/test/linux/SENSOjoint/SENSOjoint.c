/** \file
 * \brief Example code for Simple Open EtherCAT master with Synapticon SOMANET servo drive
 *
 * Usage : CSV_test_SOMANET_v42 [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test, programmed based on the simple_test of SOEM, driving a motor with SOMANET (v4.2 firmware) in CSV mode at 100RPM.
 *
 * Chencheng Tang 2019
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

/* define pointer structure */
typedef struct PACKED
{
  int16 Statusword;
  int8  OpModeDisplay;
  int32 PositionValue;
  int32 VelocityValue;
  int16 TorqueValue;
  int32 OutputTorque;
  int32 errorReport[2];
  int32 SecPositionValue;	//<< Antriebsposition
  int32 SecVelocityValue;	//<< Abtriebsgeschwindigkeit
  int8 safetyStatus1;		//<< STO status 1
  int8 safetyStatus2;		//<< STO status 2
} in_somanet_42t;

typedef struct PACKED
{
  int16 Controlword;
  int8  OpMode;
  int16 TargetTorque;
  int32 TargetPosition;
  int32 TargetVelocity;
  int32 outputTorque;
} out_somanet_42t;

int configureSensoJoint(uint16 slave) {

        int retval;
        uint8 u8val;
        uint16 u16val;
        uint32 u32val;

        retval = 0;

        //clear sm pdos 0x1c12 and 0x1c13
        u8val = 0;
        retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
        u8val = 0;
        retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

        //clear pdo 0x1a00 entries
        u32val = (uint32) 0x656c3f00;
        retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //download pdo 0x1a00 entries
        //statusword
        u32val = (uint32) 0x60410010;
        retval += ec_SDOwrite(slave, 0x1a00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //modes of operation
        u32val = (uint32) 0x60610008;
        retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //position actual value
        u32val = (uint32) 0x60640020;
        retval += ec_SDOwrite(slave, 0x1a00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //velocity actual value
        u32val = (uint32) 0x606c0020;
        retval += ec_SDOwrite(slave, 0x1a00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //torque actual value
        u32val = (uint32) 0x60770010;
        retval += ec_SDOwrite(slave, 0x1a00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //output torque actual value
        u32val = (uint32) 0x27f00920;
        retval += ec_SDOwrite(slave, 0x1a00, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //error report
        u32val = (uint32) 0x203f0140;
        retval += ec_SDOwrite(slave, 0x1a00, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);



        //download pdo 0x1a00 entry count
        u32val = (uint32) 0x776f4007;
        retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //clear pdo 0x1a01 entries
        u32val = (uint32) 0x656c3f00;
        retval += ec_SDOwrite(slave, 0x1a01, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //download pdo 0x1a01 entries
        //sec position value
        u32val = (uint32) 0x230a0020;
        retval += ec_SDOwrite(slave, 0x1a01, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //sec velocity value
        u32val = (uint32) 0x230b0020;
        retval += ec_SDOwrite(slave, 0x1a01, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	//sto status 1
	u32val = (uint32) 0x66210108;
	retval += ec_SDOwrite(slave, 0x1a01, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	//sto status 2
	u32val = (uint32) 0x66210208;
	retval += ec_SDOwrite(slave, 0x1a01, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //download pdo 0x1a01 entry count
        u32val = (uint32) 0x776f4004;
        retval += ec_SDOwrite(slave, 0x1a01, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //clear pdo 0x1600 entries
        u32val = (uint32) 0x656c3f00;
        retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //download pdo 0x1600 entries
        //controlword
        u32val = (uint32) 0x60400010;
        retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //modes of operation
        u32val = (uint32) 0x60600008;
        retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //target torque
        u32val = (uint32) 0x60710010;
        retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //target position
        u32val = (uint32) 0x607a0020;
        retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
        //target velocity
        u32val = (uint32) 0x60ff0020;
        retval += ec_SDOwrite(slave, 0x1600, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	//target output torque
        u32val = (uint32) 0x27710020;
        retval += ec_SDOwrite(slave, 0x1600, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //download pdo 0x1600 entry count
        u32val = (uint32) 0x776f4006;
        retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

        //download pdo 0x1c12:01 index
        u16val = (uint16) 0x1600;
        retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

        //download pdo 0x1c12 count
        u8val = (uint8) 0x01;
        retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

        //download pdo 0x1c13 index
        u16val = (uint16) 0x1a00;
        retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
        u16val = (uint16) 0x1a01;
        retval += ec_SDOwrite(slave, 0x1c13, 0x02, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

        //download pdo 0x1c13 count
        u8val = (uint8) 0x02;
        retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);



//        while(EcatError) ROS_ERROR("%s", ec_elist2string());
//
        printf("SensoJoint slave %d set, retval = %d\n", slave, retval);


	    int sizeArray = 50;
	    unsigned char result[50] = {0};
	    ec_SDOread(slave, 0x100A, 0x00, FALSE, &sizeArray, &result, EC_TIMEOUTRXM);
	    printf("VERSION (0x100A): %s\n", result);
	    uint32 version = 0;
	    sizeArray = 4;
	    ec_SDOread(slave, 0x1018, 0x03, FALSE, &sizeArray, &version, EC_TIMEOUTRXM);
	    printf("VERSION (0x1018): %i\n", version);


        return 1;

}


void simpletest(char *ifname)
{
    int i, j, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */

       if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

	 for(i = 1; i<=ec_slavecount ; i++)
	 {
	   ec_slave[i].PO2SOconfig = &configureSensoJoint;
	 }

         ec_config_map(&IOmap);

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;

            // initialize counter j
            j = 0;
                /* create and connect struture pointers to I/O */
            in_somanet_42t* in_somanet_1;
            in_somanet_1 = (in_somanet_42t*) ec_slave[0].inputs;
            out_somanet_42t* out_somanet_1;
            out_somanet_1 = (out_somanet_42t*) ec_slave[0].outputs;

	    int32 zeroPos = in_somanet_1->SecPositionValue;

                /* cyclic loop */
            for(i = 1; i <= 10000; i++)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

                   if(wkc >= expectedWKC)
                    {
                        j++;

                          /* Example code simplest */
                        /*
                        if (j == 100) out_somanet_1->OpMode = 9; // Set Opmode, 9 = CSV
                        if (j == 200) out_somanet_1->Controlword = -128; // Fault reset: Fault -> Swith on disabled, if the drive is in fault state
                        if (j == 300) out_somanet_1->Controlword = 0; // Resetting the Controlword, can be skipped theoretically
                        if (j == 400) out_somanet_1->Controlword = 6; // Shutdown: Switch on disabled -> Ready to switch on
                        if (j == 500) out_somanet_1->Controlword = 7; // Switch on: Ready to switch on -> Switched on
                        if (j == 600) out_somanet_1->Controlword = 15; // Enable operation: Switched on -> Operation enabled
                        if (j >= 1000) out_somanet_1->TargetVelocity = 100; // Sending velocity command
                        */
                        
                           /* Example code more proper for the same functioning, for refernce*/
                        
                           // Set Opmode, 9 = CSV
                        if (j == 1) out_somanet_1->OpMode = -110;//9;//-110;

                           // Fault reset: Fault -> Swith on disabled, if the drive is in fault state
                        if ((in_somanet_1->Statusword & 0b0000000001001111) == 0b0000000000001000)
			                  out_somanet_1->Controlword = 0b10000000;

                           // Shutdown: Switch on disabled -> Ready to switch on
		                  else if ((in_somanet_1->Statusword & 0b0000000001001111) == 0b0000000001000000)
			                  out_somanet_1->Controlword = 0b00000110;

                           // Switch on: Ready to switch on -> Switched on
		                  else if ((in_somanet_1->Statusword & 0b0000000001101111) == 0b0000000000100001)
			                  out_somanet_1->Controlword = 0b00000111;
                           
                           // Enable operation: Switched on -> Operation enabled
                        else if ((in_somanet_1->Statusword & 0b0000000001101111) == 0b0000000000100011)
			                  out_somanet_1->Controlword = 0b00001111;
                           
                           // Sending velocity command
                        else if ((in_somanet_1->Statusword & 0b0000000001101111) == 0b0000000000100111)
//			                  out_somanet_1->TargetVelocity = 400000;
//			                  out_somanet_1->TargetPosition = 50454;
			{
				double diffPosition = (double)(zeroPos - in_somanet_1->SecPositionValue) / 524288.0 / 81.0;
				out_somanet_1->outputTorque = (int32)(diffPosition * 2400.0);
//			                  out_somanet_1->outputTorque = 300;
			}			  

                        printf("Processdata cycle %4d , WKC %d ,", i, wkc);
                          // print statusword
                        printf(" Statusword: %X ,", in_somanet_1->Statusword);
                          // print opmode display
                        printf(" Op Mode Display: %d ,", in_somanet_1->OpModeDisplay);
                          // print actual position value
                        printf(" ActualPos: %" PRId32 " ,", in_somanet_1->PositionValue);
                          // print actual velocity value
                        printf(" ActualVel: %" PRId32 " ,", in_somanet_1->VelocityValue);
			printf(" STO0: %" PRId8 " ,", in_somanet_1->safetyStatus1);
			printf(" STO1: %" PRId8 " ,", in_somanet_1->safetyStatus2);
                          // print demand velocity value
//                        printf(" DemandVel: %" PRId32 " ,", in_somanet_1->VelocityDemandValue);
//			printf(" SecPos value: %" PRId32 " ,", in_somanet_1->SecPositionValue);
//			printf(" SecVel value: %" PRId32 " ,", in_somanet_1->SecVelocityValue);
//                        printf(" Time: %" PRId64 " ,", ec_DCtime);
//			printf(" Torque value: %" PRId16 " ,", in_somanet_1->TorqueValue);
//			printf(" Output torque: %" PRId32 " ,", in_somanet_1->OutputTorque);

                        printf(" T:%" PRId64 "\r",ec_DCtime);
			printf("\n");
                        needlf = TRUE;
                    }
                    osal_usleep(5000);
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}

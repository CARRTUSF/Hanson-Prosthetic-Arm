#include "Motors.h"


Motors::Motors() {
    // Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
        isOpen = true;
    }
    else
    {
        printf("Failed to change the baudrate!\n");
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 1, 100, 30, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 1, 36, 600, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }


}

void Motors::backToNomial(vector<int> orgPos) {
    moveTogether(orgPos[0], orgPos[1], orgPos[2], orgPos[3], orgPos[4], orgPos[5], orgPos[6]);
}

void Motors::moveOne(int position, int dxl_id) {

    if (!isOpen) {
        portHandler->openPort();
        isOpen = true;
    }

    int32_t dxl_present_position = 0, oldSpot = 0;               // Present position
    int count = 0;
    // Enable Dynamixel Torque
    connect(dxl_id);


    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_PRO_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    do
    {
        oldSpot = dxl_present_position;

        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", dxl_id, position, dxl_present_position);

        if (oldSpot == dxl_present_position - 1 || oldSpot == dxl_present_position + 1)
            count++;
        if (count > 5)
            break;

    } while ((abs(position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    disconnect(dxl_id);

    portHandler->closePort();
    isOpen = false;

}

void Motors::connect(int dxl_id) {
    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", dxl_id);
    }
}

void Motors::disconnect(int dxl_id) {
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void Motors::setParamGoalArray(int position) {
    
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position));

}

void Motors::moveTogether(int P1, int P2, int P3, int P4, int P5, int P6, int P7) {
    
    if (!isOpen) {
        portHandler->openPort();
        isOpen = true;
    }

    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
    // Initialize Groupsyncread instance for Present Position
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

   
    int32_t dxl1_present_position = 0, dxl2_present_position = 0;
    int32_t dxl3_present_position = 0, dxl4_present_position = 0;
    int32_t dxl5_present_position = 0, dxl6_present_positoin = 0;
    int32_t dxl7_present_position = 0;
    
    int32_t oldDx1, oldDx2, oldDx3, oldDx4, oldDx5, oldDx6, oldDx7;

    oldDx1 = oldDx2 = oldDx3 = oldDx4 = oldDx5 = oldDx6 = oldDx7 = 0;

    // Add parameter storage for Dynamixel#1 present position value
    /*dxl_addparam_result = groupSyncRead.addParam(DXL_ID1);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID1);
        return;
    }*/

    // Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID2);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID2);
        return;
    }

    dxl_addparam_result = groupSyncRead.addParam(DXL_ID3);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID3);
        return;
    }

    dxl_addparam_result = groupSyncRead.addParam(DXL_ID4);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID4);
        return;
    }

    dxl_addparam_result = groupSyncRead.addParam(DXL_ID5);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID5);
        return;
    }

    
        dxl_addparam_result = groupSyncRead.addParam(DXL_ID6);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID6);
        return;
    }
        dxl_addparam_result = groupSyncRead.addParam(DXL_ID7);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID7);
        return;
    }
    
    
    //connect(DXL_ID1);
    connect(DXL_ID2);
    connect(DXL_ID3);
    connect(DXL_ID4);
    connect(DXL_ID5);
    connect(DXL_ID6);
    connect(DXL_ID7);

    //write logic
        // Add Dynamixel#1 goal position value to the Syncwrite storage
    /*etParamGoalArray(P1);
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID1, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID1);
        return;
    }*/
    
    setParamGoalArray(P2);
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID2, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID2);
        return;
    }
   
    setParamGoalArray(P3);
    // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID3, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID3);
        return;
    }
    
    setParamGoalArray(P4);
    // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID4, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID4);
        return;
    }
    
    setParamGoalArray(P5);
    // Add Dynamixel#5 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID5, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID5);
        return;
    }

    
        setParamGoalArray(P6);
    // Add Dynamixel#6 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID6, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID6);
        return;
    }
        setParamGoalArray(P7);
    // Add Dynamixel#7 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID7, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID7);
        return;
    }
    
    
    

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    int count1 = 0, count2 = 0, count3 = 9, count4 = 0, count5 = 0;
    do
    {
        //condition to break out of the loop if motors are stuck
        oldDx1 = dxl1_present_position;
        oldDx2 = dxl2_present_position;
        oldDx3 = dxl3_present_position;
        oldDx4 = dxl4_present_position;
        oldDx5 = dxl5_present_position;

        // Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (groupSyncRead.getError(DXL_ID4, &dxl_error))
        {
            printf("[ID:%03d] %s\n", DXL_ID4, packetHandler->getRxPacketError(dxl_error));
        }
        else if (groupSyncRead.getError(DXL_ID2, &dxl_error))
        {
            printf("[ID:%03d] %s\n", DXL_ID2, packetHandler->getRxPacketError(dxl_error));
        }
        else if (groupSyncRead.getError(DXL_ID3, &dxl_error))
        {
            printf("[ID:%03d] %s\n", DXL_ID3, packetHandler->getRxPacketError(dxl_error));
        }

        else if (groupSyncRead.getError(DXL_ID5, &dxl_error))
        {
            printf("[ID:%03d] %s\n", DXL_ID5, packetHandler->getRxPacketError(dxl_error));
        }

//        else if (groupSyncRead.getError(DXL_ID1, &dxl_error))
  //      {
    //        printf("[ID:%03d] %s\n", DXL_ID1, packetHandler->getRxPacketError(dxl_error));
      //  }

                else if (groupSyncRead.getError(DXL_ID6, &dxl_error))
        {
            printf("[ID:%03d] %s\n", DXL_ID6, packetHandler->getRxPacketError(dxl_error));
        }
                else if (groupSyncRead.getError(DXL_ID7, &dxl_error))
        {
            printf("[ID:%03d] %s\n", DXL_ID7, packetHandler->getRxPacketError(dxl_error));
        }
        

        // Check if groupsyncread data of Dynamixel#1 is available
        
        //dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        //if (dxl_getdata_result != true)
        //{
         //   fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID1);
          //  return;
        //}
        

        // Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID2);
            return;
        }
        // Check if groupsyncread data of Dynamixel#3 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID3, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID3);
            return;
        }
        // Check if groupsyncread data of Dynamixel#4 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID4, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID4);
            return;
        }
        // Check if groupsyncread data of Dynamixel#5 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID5, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID5);
            return;
        }

        
                // Check if groupsyncread data of Dynamixel#6 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID6, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID6);
            return;
        }
                // Check if groupsyncread data of Dynamixel#7 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID7, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID7);
            return;
        }
        
        


        // Get Dynamixel#1 present position value
      //  dxl1_present_position = groupSyncRead.getData(DXL_ID1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

        // Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(DXL_ID2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        
        // Get Dynamixel#3 present position value
        dxl3_present_position = groupSyncRead.getData(DXL_ID3, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

        // Get Dynamixel#2 present position value
        dxl4_present_position = groupSyncRead.getData(DXL_ID4, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

        // Get Dynamixel#5 present position value
        dxl5_present_position = groupSyncRead.getData(DXL_ID5, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

       
                // Get Dynamixel#6 present position value
        dxl6_present_positoin = groupSyncRead.getData(DXL_ID6, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

                // Get Dynamixel#7 present position value
        dxl7_present_position = groupSyncRead.getData(DXL_ID7, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        
        


        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 
            DXL_ID1, P1, dxl1_present_position, DXL_ID2, P2, dxl2_present_position, DXL_ID3, P3, dxl3_present_position, DXL_ID4, P4, dxl4_present_position, DXL_ID5, P5, dxl5_present_position);

       // if (oldDx1 == dxl1_present_position + 1 || oldDx1 == dxl1_present_position - 1 || oldDx1 + 1 == dxl1_present_position || oldDx1 - 1 == dxl1_present_position || oldDx1 == dxl1_present_position)
         //   count1++;
        if (oldDx2 == dxl2_present_position + 1 || oldDx2 == dxl2_present_position - 1 || oldDx2 + 1 == dxl2_present_position || oldDx2 - 1 == dxl2_present_position || oldDx2 == dxl2_present_position)
            count2++;
        if (oldDx3 == dxl3_present_position + 1 || oldDx3 == dxl3_present_position - 1 || oldDx3 + 1 == dxl3_present_position || oldDx3 - 1 == dxl3_present_position || oldDx3 == dxl3_present_position)
            count3++;
        if (oldDx4 == dxl4_present_position + 1 || oldDx4 == dxl4_present_position - 1 || oldDx4 + 1 == dxl4_present_position || oldDx4 - 1 == dxl4_present_position || oldDx4 == dxl4_present_position)
            count4++;
        if (oldDx5 == dxl5_present_position + 1 || oldDx5 == dxl5_present_position - 1 || oldDx5 + 1 == dxl5_present_position || oldDx5 - 1 == dxl5_present_position || oldDx5 == dxl5_present_position)
            count5++;
        if ( count2 > 5 && count3 > 5 && count4 > 5 && count5 > 5)
            break;
       // printf("\n%d\t%d\t%d\t%d\t%d\n", count1, count2, count3, count4, count5);
    } while ( abs(P2 - dxl2_present_position) > 1  || abs(P4 - dxl4_present_position) > 1 || abs(P3 - dxl3_present_position) > 1 || abs(P5 - dxl5_present_position) > 1 || abs(P6 - dxl6_present_positoin) > 1 || abs(P7 - dxl7_present_position) > 1);
    //(
    
    //disconnect(DXL_ID1);
    disconnect(DXL_ID2);
    disconnect(DXL_ID3);
    disconnect(DXL_ID4);
    disconnect(DXL_ID5);
    disconnect(DXL_ID6);
    disconnect(DXL_ID7);

    portHandler->closePort();
    isOpen = false;


}

int Motors::getOneMotorPosition(int dxl_id) {

    if (!isOpen) {
        portHandler->openPort();
        isOpen = true;
    }

    int32_t dxl_present_position;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
  //  printf("CurrPos:%03d", dxl_present_position);
    disconnect(dxl_id);
    return dxl_present_position;

    
}

vector<int> Motors::getAllMotorPosition() {
    vector<int> pos;
    
    if (!isOpen) {
        portHandler->openPort();
        isOpen = true;
    }


    int pos1 = getOneMotorPosition(DXL_ID1);
    int pos2 = getOneMotorPosition(DXL_ID2);
    int pos3 = getOneMotorPosition(DXL_ID3);
    int pos4 = getOneMotorPosition(DXL_ID4);
    int pos5 = getOneMotorPosition(DXL_ID5);
    int pos6 = getOneMotorPosition(DXL_ID6);
    int pos7 = getOneMotorPosition(DXL_ID7);
    
    pos.push_back(pos1);
    pos.push_back(pos2);
    pos.push_back(pos3);
    pos.push_back(pos4);
    pos.push_back(pos5);
    pos.push_back(pos6);
    pos.push_back(pos7);

    portHandler->closePort();
    isOpen = false;

    return pos;
}
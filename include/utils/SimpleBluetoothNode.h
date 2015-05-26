/* 
 * File:   
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */



#ifndef BLUETOOTHNODE_H
#define	BLUETOOTHNODE_H

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <unistd.h>

using namespace std;

namespace lar_visionsystem{
    class SimpleBluetoothNode {
        
    public:
        SimpleBluetoothNode(std::string address);
        virtual ~SimpleBluetoothNode();
        std::string address;
        int status;
        int handle;
        void writeMessage(std::string message);
    private:

    };
}
#endif	/* BLUETOOTHNODE_H */


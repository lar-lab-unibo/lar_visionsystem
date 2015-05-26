/* 
 * File:   MathUtils.cpp
 * Author: daniele
 * 
 * Created on 22 gennaio 2014, 11.43
 */

#include "SimpleBluetoothNode.h"

namespace lar_visionsystem{
    
SimpleBluetoothNode::SimpleBluetoothNode(std::string address) {
    
    this->address = address;
    
    struct sockaddr_rc addr = { 0 };
   
    this->handle = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
 
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( address.c_str(), &addr.rc_bdaddr );
    
    this->status = connect(this->handle, (struct sockaddr *)&addr, sizeof(addr));
}

SimpleBluetoothNode::~SimpleBluetoothNode() {
}

void SimpleBluetoothNode::writeMessage(std::string message) {
    int size= message.length();
    write(this->handle, message.c_str(), size);
}

}
/**
 * CANSerial
 * Arduino Library to emulate a uart over a CAN bus.
 * Designed to interface with https://github.com/bondus/CanSerial/
 *
 * Copyright (C) 2022  Jacob Geigle
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/
#include "CANSerial.h"

//
// Constructor
//
CANSerial::CANSerial(uint16_t baseID, MCP_CAN &CAN_Object, bool inverse_logic /* = false */) : 
  _baseID(baseID),
  _CAN_Object(&CAN_Object),
  _Buffered_CAN_Object(NULL),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic),
  _heartbeat(millis()),
  _validID(false),
  _usingBufferedCAN(false)
{
}

//
// Constructor
// (Buffered_MCP_CAN)
//
CANSerial::CANSerial(uint16_t baseID, Buffered_MCP_CAN &CAN_Object, bool inverse_logic /* = false */) : 
  _baseID(baseID),
  _CAN_Object(&CAN_Object),
  _Buffered_CAN_Object(&CAN_Object),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic),
  _heartbeat(millis()),
  _validID(false),
  _usingBufferedCAN(true)
{
}

//
// Destructor
//
CANSerial::~CANSerial()
{
  end();
}

void CANSerial::begin(unsigned char *UUID, size_t len)
{
  for (size_t i=0; i < (len < _CS_UUID_SIZE ? len:_CS_UUID_SIZE); i++) {
    _UUID[i] = UUID[i];
  }
  recv();
}

void CANSerial::end()
{
}

void CANSerial::recv()
{
  unsigned char len = 0;
  unsigned char buf[8];
  if (millis() - _heartbeat > 30000) {
    _validID = false;
  }
  if ( _usingBufferedCAN ) {
    if (CAN_MSGAVAIL == _Buffered_CAN_Object->checkReceive(_lCANBufHead)) {         // check if data coming
      byte ret = _Buffered_CAN_Object->readMsgBuf(&len, buf, _lCANBufHead);    // read data,  len: data length, buf: data buf
      if ( ret == CAN_OK ) {
        _lCANBufHead = (_lCANBufHead + 1) % _CAN_MAX_RX_BUF;
      }
      processFrame(buf,len);
    }
  }
  else {
    if (CAN_MSGAVAIL == _CAN_Object->checkReceive()) {         // check if data coming
      _CAN_Object->readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
      processFrame(buf,len);
    }
  }
  send();
}

/*
 * processFrame()
 *
 * process incoming frames
 */
void CANSerial::processFrame(unsigned char *buf, unsigned char len)
{
  unsigned long canId = _CAN_Object->getCanId();
  if (canId == _baseID+1 ) {
    _heartbeat = millis();
    if (!_validID || _nodeID == buf[0] + (buf[1] * 0x100u)) {
      _CAN_Object->sendMsgBuf(_baseID+3,0,0,_CS_UUID_SIZE,_UUID);
    }
  }
  else if ( canId == _baseID+2 ) {
    bool lContinue = true;
    for ( int i = 0;i<_CS_UUID_SIZE;i++ ) {
      if (_UUID[i] != buf[i+2]) {
        lContinue = false;
      }
    }
    if (lContinue) {
      _nodeID = buf[0] + (buf[1] * 0x100);
      _validID = true;
      _heartbeat = millis();
    }
  }
  else if (canId == _nodeID) {
    if (len == 0) {
      _CAN_Object->sendMsgBuf(_nodeID+1,0,0,0,NULL);
    }
    for (int i=0;i<len;i++) {
      if ( (_receive_buffer_head - _receive_buffer_tail) == 1 ){
        _buffer_overflow = true;
        return;
      }
      _receive_buffer[_receive_buffer_tail] = buf[i];
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _CS_MAX_RX_BUFF;
    }
    _heartbeat = millis();
  }
}


// Read data from buffer
int CANSerial::read()
{
  recv();

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _CS_MAX_RX_BUFF;
  return d;
}

int CANSerial::available()
{
  recv();

  return (_receive_buffer_tail + _CS_MAX_RX_BUFF - _receive_buffer_head) % _CS_MAX_RX_BUFF;
}

int CANSerial::send()
{
  int count = 0;
  if ( _validID ) {
    while ( _send_buffer_tail != _send_buffer_head ) {
      _CAN_Object->sendMsgBuf(_nodeID+1,0,0,1,&_send_buffer[_send_buffer_head]);
      _send_buffer_head = (_send_buffer_head + 1) % _CS_MAX_TX_BUFF;
      count++;
    }
  }
  return count;
}

size_t CANSerial::write(uint8_t b)
{
  recv();
  if ( (_send_buffer_head - _send_buffer_tail) == 1 ){
    _buffer_overflow = true;
    return -1;
  }
  _send_buffer[_send_buffer_tail] = b;
  _send_buffer_tail = (_send_buffer_tail + 1) % _CS_MAX_TX_BUFF;
  send();
  return 1;
}

void CANSerial::flush()
{
  // There is no tx buffering, simply return
}

int CANSerial::peek()
{

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

/*
 * Buffered CAN_MCP
 */

/*
 * Constructor
 */
Buffered_MCP_CAN::Buffered_MCP_CAN(MCP_CAN &CAN_Object) :
  MCP_CAN(0),
  _CAN_Object(&CAN_Object)
{
}
/*
 * Destructor
 */
Buffered_MCP_CAN::~Buffered_MCP_CAN(){}

/*
 * readMsgBufID
 * the real workorse
 */
byte Buffered_MCP_CAN::readMsgBufID(byte status, volatile unsigned long *id, volatile byte *ext, volatile byte *rtr, volatile byte *len, volatile byte *buf, uint8_t extHead)
{
  //TODO - Add check to extHead to make sure not overflowing (eg, bad implementation in client)
  byte ret = CAN_NOMSG;
  // Check if underlying device has received anything
  if ( _CAN_Object->checkReceive() == CAN_MSGAVAIL ) {
      // add to FIFO
      unsigned long newID;
      byte newExt;
      byte newRTR;
      byte newLen;
      byte newBuf[8];
      byte res = _CAN_Object->readMsgBufID(status,&newID,&newExt,&newRTR,&newLen,(byte*)&newBuf);
      if ( res == CAN_OK ) {
        _canBuf[_RX_buf_tail].canid = newID;
        _canBuf[_RX_buf_tail].canid |= newExt ? CAN_EFF_FLAG : 0;
        _canBuf[_RX_buf_tail].canid |= newRTR ? CAN_RTR_FLAG : 0;
    _canBuf[_RX_buf_tail].len = newLen;
        for ( int i=0;i<newLen;i++ ) {
          _canBuf[_RX_buf_tail].data[i] = newBuf[i];
        }
        _RX_buf_tail = (_RX_buf_tail + 1) % _CAN_MAX_RX_BUF;
      }
  }

  // return next item from FIFO
  if ( checkReceive() == CAN_MSGAVAIL) {
    *id = _canBuf[extHead].canid & CAN_EFF_FLAG ? _canBuf[extHead].canid & CAN_EFF_MASK : _canBuf[extHead].canid & CAN_SFF_MASK;
    *ext = _canBuf[extHead].canid & CAN_EFF_FLAG ? 1 : 0;
    *rtr = _canBuf[extHead].canid & CAN_RTR_FLAG ? 1 : 0;
    *len = _canBuf[extHead].len;
    for ( int i=0;i<*len;i++ ) {
      buf[i] = _canBuf[extHead].data[i];
    }
    ret = CAN_OK;
  }
        
  return ret;
}

/*
 * readMsgBufID
 * wrapper for normal operation
 */
byte Buffered_MCP_CAN::readMsgBufID(byte status, volatile unsigned long *id, volatile byte *ext, volatile byte *rtr, volatile byte *len, volatile byte *buf)
{
  byte ret = readMsgBufID(status,id,ext,rtr,len,buf,_RX_buf_head);
  if (ret == CAN_OK) {
    _RX_buf_head = (_RX_buf_head + 1) % _CAN_MAX_RX_BUF;
  }
  return ret;
}

/*
 * checkReceive
 * checks underlying controller and internal buffer
 * @param in extHead External head pointer to the FIFO
 */
byte Buffered_MCP_CAN::checkReceive(uint8_t extHead)
{
  byte res;
  res = _CAN_Object->checkReceive();
  if ( res == CAN_MSGAVAIL ) {
    return CAN_MSGAVAIL;
  }
  if ( ((_RX_buf_tail + _CAN_MAX_RX_BUF - extHead) % _CAN_MAX_RX_BUF) > 0 ) {
    return CAN_MSGAVAIL;
  }
  return CAN_NOMSG;
}

/*
 * checkReceive
 * checks underlying controller and internal buffer
 * wrapper for normal operation
 */
byte Buffered_MCP_CAN::checkReceive()
{
  return checkReceive(_RX_buf_head);
}

/*
 * readRxTxStatus
 * checks underlying chip and adjusts based on items in buffer
 */
byte Buffered_MCP_CAN::readRxTxStatus(void)
{
  //TODO
  // Maybe doesn't matter
  return _CAN_Object->readRxTxStatus();
}

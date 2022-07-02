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


/**
 * Functions to Implement from regular serial API
 */
#ifndef CANSerial_h
#define CANSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <mcp_can.h>

/******************************************************************************
* Definitions
******************************************************************************/

#ifndef _CS_UUID_SIZE
#define _CS_UUID_SIZE 6 // UUID size
#endif

#ifndef _CS_MAX_RX_BUFF
#define _CS_MAX_RX_BUFF 64 // RX buffer size
#endif

#ifndef _CS_MAX_TX_BUFF
#define _CS_MAX_TX_BUFF 8 // TX buffer size
#endif

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#ifndef _CAN_MAX_RX_BUF
#define _CAN_MAX_RX_BUF 16
#endif

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

typedef struct can_frame {
  unsigned long canid=0;
  int len=0;
  byte data[8]={0};
}can_frame_t;

class Buffered_MCP_CAN : public MCP_CAN
{
  private:
    MCP_CAN *_CAN_Object;
    can_frame_t _canBuf[_CAN_MAX_RX_BUF];
    volatile uint8_t _RX_buf_head;
    volatile uint8_t _RX_buf_tail;
  public:
    Buffered_MCP_CAN(MCP_CAN &CAN_Object);
    ~Buffered_MCP_CAN();
    byte checkReceive(uint8_t extHead);
    byte readMsgBufID(byte status, volatile unsigned long *id, volatile byte *ext, volatile byte *rtr, volatile byte *len, volatile byte *buf, uint8_t extHead);
  public:
    virtual void enableTxInterrupt(bool enable = true) // enable transmit interrupt
    {
      _CAN_Object->enableTxInterrupt(enable);
    }
    virtual void reserveTxBuffers(byte nTxBuf = 0)
    {
        _CAN_Object->reserveTxBuffers(nTxBuf);
    }
    virtual byte getLastTxBuffer()
    {
      return _CAN_Object->getLastTxBuffer();
    }
    virtual byte begin(uint32_t speedset, const byte clockset = MCP_16MHz)
    {
      return _CAN_Object->begin(speedset,clockset);
    }
    virtual byte init_Mask(byte num, byte ext, unsigned long ulData)
    {
      return _CAN_Object->init_Mask(num,ext,ulData);
    }
    virtual byte init_Filt(byte num, byte ext, unsigned long ulData)
    {
      return _CAN_Object->init_Filt(num,ext,ulData);
    }
    virtual void setSleepWakeup(byte enable)
    {
      _CAN_Object->setSleepWakeup(enable);
    }
    virtual byte sleep()
    {
      return _CAN_Object->sleep();
    }      
    virtual byte wake()
    {
      return _CAN_Object->wake();
    }
    virtual byte setMode(byte opMode)
    {
      return _CAN_Object->setMode(opMode);
    }
    virtual byte getMode()
    {
      return _CAN_Object->getMode();
    }
    virtual byte checkError(uint8_t* err_ptr = NULL)
    {
      return _CAN_Object->checkError(err_ptr);
    }

    virtual byte checkReceive();
    virtual byte readMsgBufID(byte status, volatile unsigned long *id, volatile byte *ext, volatile byte *rtr, volatile byte *len, volatile byte *buf);
    /* wrapper */
    byte readMsgBufID(unsigned long *ID, byte *len, byte *buf) {
        return readMsgBufID(readRxTxStatus(), ID, &ext_flg, &rtr, len, buf);
    }
    byte readMsgBuf(byte *len, byte *buf) {
        return readMsgBufID(readRxTxStatus(), &can_id, &ext_flg, &rtr, len, buf);
    }
    byte readMsgBuf(byte *len, byte *buf, uint8_t extHead) {
        return readMsgBufID(readRxTxStatus(), &can_id, &ext_flg, &rtr, len, buf, extHead);
    }

    virtual byte trySendMsgBuf(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, byte iTxBuf = 0xff)
    {
      return _CAN_Object->trySendMsgBuf(id,ext,rtrBit,len,buf,iTxBuf);
    }
    virtual byte sendMsgBuf(byte status, unsigned long id, byte ext, byte rtrBit, byte len, volatile const byte *buf)
    {
      return _CAN_Object->sendMsgBuf(status,id,ext,rtrBit,len,buf);
    }
    virtual byte sendMsgBuf(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, bool wait_sent = true)
    {
      return _CAN_Object->sendMsgBuf(id,ext,rtrBit,len,buf,wait_sent);
    }
    /* wrapper */
    inline byte sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf) {
        return sendMsgBuf(id, ext, 0, len, buf, true);
    }

    virtual void clearBufferTransmitIfFlags(byte flags = 0)
    {
      _CAN_Object->clearBufferTransmitIfFlags(flags);
    }
    virtual byte readRxTxStatus(void);
    virtual byte checkClearRxStatus(byte *status)
    {
      return _CAN_Object->checkClearRxStatus(status);
    }
    virtual byte checkClearTxStatus(byte *status, byte iTxBuf = 0xff)
    {
      return _CAN_Object->checkClearTxStatus(status,iTxBuf);
    }
    virtual bool mcpPinMode(const byte pin, const byte mode)
    {
      return _CAN_Object->mcpPinMode(pin,mode);
    }
    virtual bool mcpDigitalWrite(const byte pin, const byte mode)
    {
      return _CAN_Object->mcpDigitalWrite(pin,mode);
    }
    virtual byte mcpDigitalRead(const byte pin)
    {
      return _CAN_Object->mcpDigitalRead(pin);
    }

};

/*
 * CANSerial Class
 * transport serial communications over CAN bus
*/

class CANSerial : public Stream
{
private:
  // per object data
  uint16_t _baseID;
  MCP_CAN *_CAN_Object; 
  Buffered_MCP_CAN *_Buffered_CAN_Object; 
  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;
  unsigned long _heartbeat;
  unsigned char _UUID[_CS_UUID_SIZE];
  uint16_t _nodeID;
  bool _validID;
  uint8_t _receive_buffer[_CS_MAX_RX_BUFF]; 
  volatile uint8_t _receive_buffer_tail;
  volatile uint8_t _receive_buffer_head;
  uint8_t _send_buffer[_CS_MAX_TX_BUFF]; 
  volatile uint8_t _send_buffer_tail;
  volatile uint8_t _send_buffer_head;
  const bool _usingBufferedCAN;
  uint8_t _lCANBufHead = 0;

  void recv();
  int send();
  void processFrame(unsigned char *buf, unsigned char len);

public:
  // public methods
  CANSerial(uint16_t baseID, MCP_CAN &CAN_Object, bool inverse_logic = false);
  CANSerial(uint16_t baseID, Buffered_MCP_CAN &CAN_Object, bool inverse_logic = false);
  ~CANSerial();
  void begin(unsigned char *UUID, size_t len);
  void end();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();

  bool isBuffered() { return _usingBufferedCAN; }

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }
  
  using Print::write;
};

#endif

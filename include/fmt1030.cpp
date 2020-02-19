#include "fmt1030.h"
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <util/delay.h>



// Contructor of the class, do nothing
Fmt1030::Fmt1030()
{
}

// Destructor
Fmt1030::~Fmt1030()
{
}

// Initialization of the SPI
uint8_t Fmt1030::fmt1030_init() {
    _spi.spi_init_master(STD_DIRECTION);
    return 1;
}

//---------------------------------------//
//          LOW LEVEL FUNCTIONS          //
//---------------------------------------//

// Compute and return the checksum
uint8_t Fmt1030::cs_calculator() {
    uint8_t tmp = 0xFF + _messageXbus.mid + _messageXbus.dataLenght;
    if (_messageXbus.dataLenght != 0) {
        for (int i=0; i<_messageXbus.dataLenght; i++) {
            tmp += _messageXbus.data[i];
        }
    }
    tmp = 0x100 - tmp;
    return tmp;
}

// Send an OPCode on the SPI
void Fmt1030::sendOpCode(uint8_t opCode) {
    _spi.spi_tranceiver(opCode);
    for (int filler=0; filler<3; filler++) {
        _spi.spi_tranceiver(filler);
    }
}

// Send a Xbus message
uint8_t Fmt1030::sendXbusRawMessage(uint8_t opCode) 
{
    _messageXbus.cs = this->cs_calculator();
    this->sendOpCode(opCode);
    _spi.spi_tranceiver(_messageXbus.mid);
    _spi.spi_tranceiver(_messageXbus.dataLenght);
    if (_messageXbus.dataLenght != 0)
    {
        for (int i=0; i<_messageXbus.dataLenght; i++) {
            _spi.spi_tranceiver(_messageXbus.data[i]);
        }
    }
    _spi.spi_tranceiver(_messageXbus.cs);
    return 1;
}

const MessageXbus& Fmt1030::getMessage()
{
	return _messageXbus;
}

//---------------------------------------//
//          HIGH LEVEL FUNCTIONS         //
//---------------------------------------//

const MtsspConfiguration& Fmt1030::read_pipe_status() {
    uint8_t tabrecep[4];
    _spi.spi_begin_transceive();
    this->sendOpCode(0x04); // PipeStatus OpCode
    for (int i=0; i<4; i++)
    {
        tabrecep[i] = _spi.spi_tranceiver(0);
    }
    _spi.spi_stop_transceive();
    _notification.notificationMessageSize = (tabrecep[1] << 8) + tabrecep[0];
    _notification.measurementMessageSize = (tabrecep[3] << 8) + tabrecep[2];
    return _notification;
}

uint8_t Fmt1030::read_notification_pipe() {
    _spi.spi_begin_transceive();
    this->sendOpCode(0x05); // NotificationPipe OpCode
    uint8_t result = _spi.spi_tranceiver(0);
    _spi.spi_stop_transceive();
    return result;
}


uint8_t Fmt1030::goToConfig()
{
    _messageXbus.mid = XMID_GOTOCONFIG;
    _messageXbus.dataLenght = 0;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do 
    {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_GOTOCONFIGACK) & (escape != 0xFF));
    if (ack == XMID_GOTOCONFIGACK){
        return XMID_GOTOCONFIGACK;
    }
    else
        return 0x00;
}

uint8_t Fmt1030::gotoMeasurement()
{
    _messageXbus.mid = XMID_GOTOMEASUREMENT;
    _messageXbus.dataLenght = 0;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do
    {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_GOTOMEASUREMENTACK) & (escape != 0xFF));
    if (ack == XMID_GOTOMEASUREMENTACK){
        return XMID_GOTOMEASUREMENTACK;
    }
    else
        return 0x00;
}

uint8_t Fmt1030::readMesure(uint16_t nbOctToRead)
{
    uint8_t tabrecep[nbOctToRead];
    _spi.spi_begin_transceive();
    this->sendOpCode(0x06);
    for (uint16_t k=0; k<nbOctToRead; k++) {
        tabrecep[k] = _spi.spi_tranceiver(0);
    }
    _spi.spi_stop_transceive();

    // The received frame : tabrecep (reduced XBus message)):
    //  1byte : MID = 0x36 => MTData2                                   (0)
    //  1byte : LEN = 0x22 => 34 data bytes                             (1)
    //  Data :
    //      2 bytes : DID = 0x2010 => identifier (quaternion)           (2)
    //      1 byte  : DLEN = 0x10  => 16 bytes (4 bytes for each data)  (4)
    //      4x4 bytes : w, x, y, z                                      (5, 9, 13, 17)
    //      2 bytes : DID = 0x8020 => identifier (rate of turn)         (21)
    //      1 byte  : DLEN = 0x0C  => 12 bytes                          (23)
    //      3x4 bytes : x, y, z                                         (24, 28, 32)
    //  1 byte : Checksum                                               (36)

    // Note that bytes are given from the lower weight to the upper weight

 
    // We record those data into float values
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_quaternion.w)) + 3 - i, &tabrecep[i + 5], 1);
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_quaternion.x)) + 3 - i, &tabrecep[i + 9], 1);
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_quaternion.y)) + 3 - i, &tabrecep[i + 13], 1);
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_quaternion.z)) + 3 - i, &tabrecep[i + 17], 1);
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_rateofturn.x)) + 3 - i, &tabrecep[i + 24], 1);
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_rateofturn.y)) + 3 - i, &tabrecep[i + 28], 1);
    for (uint8_t i = 0; i<4; i++) memcpy(((uint8_t*)(&_rateofturn.z)) + 3 - i, &tabrecep[i + 32], 1);

    return 1; // affectation_done;
}

const Quaternion& Fmt1030::getQuaternion() { return _quaternion; }
const Vector& Fmt1030::getRateOfTurn() { return _rateofturn; }

uint8_t Fmt1030::setOutputConfiguration(uint8_t nb_Arg, ...) {
    va_list listImput;
    va_start (listImput, nb_Arg);
    _messageXbus.dataLenght = 2 * nb_Arg;
    for (int i = 0; i < nb_Arg; i++) {
        int currentImput = va_arg (listImput, int);
        _messageXbus.data[i*2] = ((currentImput >> 8) & 0x00FF);
        _messageXbus.data[(i*2)+1] = (currentImput & 0x00FF);
    }
    va_end (listImput);

    _messageXbus.mid = XMID_SETOUTPUTCONFIG;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do
    {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_OUTPUTCONFIG) & (escape != 0xFF));
    if (ack == XMID_OUTPUTCONFIG){
        return XMID_OUTPUTCONFIG;
    }
    else
        return 0x00;
}

uint8_t Fmt1030::setMyOutputConfiguration() {
    _messageXbus.dataLenght = 8;
    _messageXbus.data[0] = ((XDI_QUATERNION >> 8) & 0x00FF);
    _messageXbus.data[1] = (XDI_QUATERNION & 0x00FF);
    _messageXbus.data[2] = ((FREQ_100Hz >> 8) & 0x00FF);
    _messageXbus.data[3] = (FREQ_100Hz & 0x00FF);
    _messageXbus.data[4] = ((XDI_RATEOFTURN >> 8) & 0x00FF);
    _messageXbus.data[5] = (XDI_RATEOFTURN & 0x00FF);
    _messageXbus.data[6] = ((FREQ_100Hz >> 8) & 0x00FF);
    _messageXbus.data[7] = (FREQ_100Hz & 0x00FF);

    _messageXbus.mid = XMID_SETOUTPUTCONFIG;
    _spi.spi_begin_transceive();
    this->sendXbusRawMessage(0x03);
    _spi.spi_stop_transceive();

    // check the Ack
    uint8_t ack, escape=0;
    do {
        ack = this->read_notification_pipe();
        escape++;
    } while ((ack != XMID_OUTPUTCONFIG) & (escape != 0xFF));
    
    if (ack == XMID_OUTPUTCONFIG) return XMID_OUTPUTCONFIG;
    else return 0x00;
}
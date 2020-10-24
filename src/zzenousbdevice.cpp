/*
 *             Copyright 2020 by Morgan
 *
 * This software BSD-new. See the included COPYING file for details.
 *
 * License: BSD-new
 * ==============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "zzenousbdevice.h"
#include "zzenocandriver.h"
#include "zusbcontext.h"
#include "zdebug.h"

ZZenoUSBDevice::ZZenoUSBDevice(VxZenoCANDriver* _driver,
                                 int _device_no, libusb_device* _device,
                                 const QString& _display_name)
: driver(_driver), usb_context(new VxUSBContext(driver->getUSBContext(),this)),
  device_gone_or_disconnected(false),
  device_no(_device_no), open_ref_count(0),
  device(_device), handle(nullptr),
  in_end_point_address(0), in_end_point_interrupt_address(0), in_max_packet_size(0),
  in_bulk_transfer_complete(0), in_interrupt_transfer_complete(0),
  in_bulk_transfer(nullptr), in_buffer(new quint8[ZENO_USB_MAX_PACKET_IN]),
  out_end_point_address(0), display_name(_display_name),
  reply_timeout_in_ms(1000), reply_cmd_id(0), reply_received(0), reply_command(nullptr),
  next_transaction_id(0),
  zeno_clock_resolution(1),
  serial_number(0),
  fw_version(0),
  t2_clock_start_ref_in_us(0),
  init_calibrate_count(0),
  drift_time_in_us(0),
  read_reply_received(0)
{
    libusb_ref_device(device);
    int res;
    libusb_config_descriptor* config = nullptr;

    res = libusb_get_config_descriptor(device,0,&config);
    if ( res ) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qCritical() << "ERROR(ZenoUSB) get active configuration failed: " << last_error_text;
        return;
    }

    if ( config->bNumInterfaces < 2 ) {
        qCritical() << "ERROR(ZenoUSB): bNumInterfaces " << config->bNumInterfaces;
        libusb_free_config_descriptor(config);
        return;
    }

    if ( config->interface[0].num_altsetting < 1 ) {
        qCritical() << "ERROR(ZenoUSB): interface 0 -> num_altsetting " << config->interface[0].num_altsetting;
        libusb_free_config_descriptor(config);
        return;
    }

    const libusb_interface_descriptor* interface = config->interface[0].altsetting;

    /* Interface 0 */
    for ( int i = 0; i < interface->bNumEndpoints; ++i ) {
        const libusb_endpoint_descriptor* end_point = interface->endpoint + i;
        qDebug() << "end_point->bEndpointAddress" << hex << end_point->bEndpointAddress;
        qDebug() << "end_point->bmAttributes" << hex << end_point->bmAttributes;

        if ( !in_end_point_address &&
             (end_point->bEndpointAddress & LIBUSB_ENDPOINT_IN) &&
             (end_point->bmAttributes & 0x3) == LIBUSB_TRANSFER_TYPE_BULK) {
            in_end_point_address = end_point->bEndpointAddress;
            in_max_packet_size = end_point->wMaxPacketSize;
        }

        if ( !out_end_point_address &&
             !(end_point->bEndpointAddress & LIBUSB_ENDPOINT_IN) &&
             (end_point->bmAttributes & 0x3) == LIBUSB_TRANSFER_TYPE_BULK) {
            out_end_point_address = end_point->bEndpointAddress;
            out_max_packet_size = end_point->wMaxPacketSize;
        }
    }

    qDebug() << " -- in address: " << in_end_point_address << " -- out address: " << out_end_point_address;
    qDebug() << " -- Max in: " << in_max_packet_size << " Max out: " << out_max_packet_size;

    /* Interface 1 */
    interface = config->interface[1].altsetting;
    qDebug() << "Interface 1 bNumEndpoints" << interface->bNumEndpoints;
    if ( interface->bNumEndpoints > 0 ) {
        /* Assume only 1 IN interrupt end-point */
        in_end_point_interrupt_address = interface->endpoint->bEndpointAddress;
        qDebug() << "Intterrupt end-point address:" << interface->endpoint->bEndpointAddress;
    }

    libusb_free_config_descriptor(config);

    out_buffer[0] = new quint8[ZENO_USB_MAX_PACKET_OUT];
    out_buffer[1] = new quint8[ZENO_USB_MAX_PACKET_OUT];
    out_bulk_transfer[0] = nullptr;
    out_bulk_transfer[1] = nullptr;
    bulk_out_transfer_completed = 0;
    next_transfer_index = -1; /* No pending TX bulk transfer */

    retrieveDeviceInfo();
}

VxZenoUSBDevice::~VxZenoUSBDevice()
{
    if ( handle != nullptr ) close();
    libusb_unref_device(device);
    delete[] in_buffer;
    delete[] out_buffer[0];
    delete[] out_buffer[1];
}

void VxZenoUSBDevice::retrieveDeviceInfo()
{
    if (!open()) return;

    ZenoCmd cmd;
    ZenoResponse reply;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZENO_CMD_RESET;

    qDebug() << "Sending reset";
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        qCritical() << "ERROR(ZenoUSB): failed to reset device " << last_error_text;
        close();
        return;
    }

    cmd.h.cmd_id = ZENO_CMD_INFO;
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        qCritical() << "ERROR(ZenoUSB): failed to retrieve device info" << last_error_text;
        close();
        return;
    }

    ZenoInfoResponse* info_response = reinterpret_cast<ZenoInfoResponse*>(&reply);
    qDebug() << "Zeno Capabilities:" << hex << info_response->capabilities;
    qDebug() << "       fw_version:" << hex << info_response->fw_version;
    qDebug() << "        serial-nr:" << hex << info_response->serial_number;
    qDebug() << "          clock @:" << (float(info_response->clock_resolution) / 1000.0f) << "Mhz";
    qDebug() << "CAN channel count:" << info_response->can_channel_count;
    qDebug() << "LIN channel count:" << info_response->lin_channel_count;

    zeno_clock_resolution = info_response->clock_resolution;
    serial_number = info_response->serial_number;
    fw_version = info_response->serial_number;

    can_channel_list.clear();
    for( int i = 0; i < info_response->can_channel_count; ++i ) {
        can_channel_list.append(new VxZenoCANChannel(i, this));
    }

    lin_channel_list.clear();
    int display_index = 1;
    for( int i = info_response->lin_channel_count-1; i >=0 ; --i ) {
        lin_channel_list.append(new VxZenoLINChannel(i, display_index++, this));
    }

    close();
}

const QString VxZenoUSBDevice::getLastErrorText()
{
    return last_error_text;
}

const QString VxZenoUSBDevice::getObjectText() const
{
    return display_name;
}

bool VxZenoUSBDevice::open()
{
    QMutexLocker lock(&device_mutex);
    if ( open_ref_count > 0 ) {
        open_ref_count ++;
        return true;
    }

    Q_ASSERT(open_ref_count == 0);
    Q_ASSERT(handle == NULL);
    Q_ASSERT(in_bulk_transfer == NULL);

    int res = libusb_open(device, &handle);
    if ( res ) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) open failed: " << last_error_text;
        return false;
    }

    res = libusb_claim_interface(handle,0);
    if ( res ) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) claim interface[0] failed: " << last_error_text;

        libusb_close(handle);
        handle = nullptr;

        return false;
    }


    res = libusb_claim_interface(handle,1);
    if ( res ) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) claim interface[1] failed: " << last_error_text;

        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }

    in_bulk_transfer = libusb_alloc_transfer(0);
    in_interrupt_transfer = libusb_alloc_transfer(0);
    out_bulk_transfer[0] = libusb_alloc_transfer(0);
    out_bulk_transfer[1] = libusb_alloc_transfer(0);
    if ( in_bulk_transfer == nullptr ||
         in_interrupt_transfer == nullptr ||
         out_bulk_transfer[0] == nullptr ||
         out_bulk_transfer[1] == nullptr) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(LIBUSB_ERROR_NO_MEM);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) failed to allocate bulk in/out transfers";

        freeTransfers();
        libusb_release_interface(handle, 1);
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }

    libusb_fill_bulk_transfer(in_bulk_transfer, handle, in_end_point_address,
                              in_buffer, ZENO_USB_MAX_PACKET_IN,
                              &__inBulkTransferCallback, this, ZENO_USB_BULK_TRANSFER_TIMEOUT);

    in_bulk_transfer_complete = 0;


    libusb_fill_interrupt_transfer(in_interrupt_transfer, handle, in_end_point_interrupt_address,
                              in_interrupt_buffer, sizeof(in_interrupt_buffer),
                              &__inInterruptTransferCallback, this, ZENO_USB_INTERRUPT_TRANSFER_TIMEOUT);

    in_interrupt_transfer_complete = 0;

    res = libusb_submit_transfer(in_bulk_transfer);
    if ( res ) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) failed to submit bulk transfer: " << last_error_text;

        freeTransfers();

        libusb_release_interface(handle, 1);
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }


    res = libusb_submit_transfer(in_interrupt_transfer);
    if ( res ) {
        last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) failed to submit interrupt transfer: " << last_error_text;

        freeTransfers();

        libusb_release_interface(handle, 1);
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }

    open_ref_count ++;
    usb_context->startRef();

    startClockInt();

    return true;
}

void VxZenoUSBDevice::freeTransfers()
{
    if ( in_bulk_transfer != nullptr ) libusb_free_transfer(in_bulk_transfer);
    if ( in_interrupt_transfer != nullptr ) libusb_free_transfer(in_interrupt_transfer);
    if ( out_bulk_transfer[0] != nullptr ) libusb_free_transfer(out_bulk_transfer[0]);
    if ( out_bulk_transfer[1] != nullptr ) libusb_free_transfer(out_bulk_transfer[1]);

    in_bulk_transfer = nullptr;
    in_interrupt_transfer = nullptr;
    out_bulk_transfer[0] = nullptr;
    out_bulk_transfer[1] = nullptr;
}

int VxZenoUSBDevice::getNextTransferIndex()
{
    int buffer_index;
    if ( next_transfer_index == -1 ) {
        libusb_fill_bulk_transfer(out_bulk_transfer[0], handle, out_end_point_address,
                                  out_buffer[0], 0,
                                  &__outBulkTransferCallback, this, ZENO_USB_BULK_TRANSFER_TIMEOUT);

        libusb_fill_bulk_transfer(out_bulk_transfer[1], handle, out_end_point_address,
                                  out_buffer[1], 0,
                                  &__outBulkTransferCallback, this, ZENO_USB_BULK_TRANSFER_TIMEOUT);

        buffer_index = 0;
    } else {
        buffer_index = next_transfer_index;
    }

    return buffer_index;
}

bool VxZenoUSBDevice::close()
{
    QMutexLocker lock(&device_mutex);
    Q_ASSERT(handle != NULL);

    if ( open_ref_count == 1 ) stopClockInt();

    open_ref_count--;
    Q_ASSERT(open_ref_count >= 0);

    if ( open_ref_count == 0 ) {
        libusb_cancel_transfer(in_bulk_transfer);
        libusb_cancel_transfer(in_interrupt_transfer);

        if ( out_bulk_transfer[0]->length > 0 ) libusb_cancel_transfer(out_bulk_transfer[0]);
        if ( out_bulk_transfer[1]->length > 0 ) libusb_cancel_transfer(out_bulk_transfer[1]);

        while (!in_bulk_transfer_complete || next_transfer_index != -1) {
            if (!usb_context->handleEvents(in_bulk_transfer_complete))
                break;
        }

        while (!in_interrupt_transfer_complete) {
            if (!usb_context->handleEvents(in_interrupt_transfer_complete))
                break;
        }

        freeTransfers();

        // int res = libusb_reset_device(handle);
        // if ( res ) {
        //   translateLibUSBErrorCode(res);
        //   qDebug() << "ERROR(ZenoUSB) reset device failed: " << last_error_text;
        // }
        libusb_release_interface(handle, 0);
        libusb_release_interface(handle, 1);
        libusb_close(handle);
        handle = nullptr;

        usb_context->stopUnRef();
    }

    return true;
}

bool VxZenoUSBDevice::isOpen() const
{
    QMutexLocker lock(&device_mutex);
    return open_ref_count > 0;
}

int VxZenoUSBDevice::getCANChannelCount() const
{
    return can_channel_list.size();
}

VxReference<VxCANChannel> VxZenoUSBDevice::getCANChannel(int channel_index)
{
    if ( channel_index < 0 || channel_index >= can_channel_list.size()) return nullptr;
    return can_channel_list[channel_index].get();
}

int VxZenoUSBDevice::getLINChannelCount() const
{
    return lin_channel_list.size();
}

VxReference<VxLINChannel> VxZenoUSBDevice::getLINChannel(int channel_index)
{
    if ( channel_index < 0 || channel_index >= lin_channel_list.size()) return nullptr;
    return lin_channel_list[channel_index].get();
}

quint32 VxZenoUSBDevice::getSerialNumber() const
{
    return serial_number;
}

quint32 VxZenoUSBDevice::getFWVersion() const
{
    return fw_version;
}

bool VxZenoUSBDevice::sendAndWhaitReply(ZenoCmd* request, ZenoResponse* reply)
{
    Q_ASSERT(handle != nullptr);
    QMutexLocker lock(&send_reply_mutex);

    if (!queueRequest(request)) return false;

    QMutexLocker command_lock(&command_mutex);
    reply_received = 0;
    reply_command = reply;
    reply_cmd_id = request->h.cmd_id;

    // qDebug() << "Queue reqeust";

    while (!reply_received) {
        if (!command_cond.wait(&command_mutex, reply_timeout_in_ms)) {
            last_error_text = "Timeout waiting for reply";
            break;
        }
    }

    reply_command = nullptr;
    // qDebug() << "last_error_text" << last_error_text << reply_received;

    return (reply_received != 0);
}

bool VxZenoUSBDevice::___queueRequestUnlocked(ZenoCmd* request, int timeout_in_ms) {
    int buffer_index = getNextTransferIndex();
    if ( buffer_index == -1) return false;

    int offset = out_bulk_transfer[buffer_index]->length;
    // if ( (offset & -out_max_packet_size) != ((offset+HYDRA_CMD_SIZE) & -out_max_packet_size) ) {
    //    out_bulk_transfer[buffer_index]->buffer[offset] = 0;
    //    offset += out_max_packet_size;
    //    offset &= - out_max_packet_size;
    // }

    if ( (offset + ZENO_CMD_SIZE) >= ZENO_USB_MAX_PACKET_OUT ) {
        if ( next_transfer_index == -1 ) {
            int res = libusb_submit_transfer(out_bulk_transfer[buffer_index]);
            if ( res ) {
                last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
                device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
                qDebug() << "ERROR(ZenoUSB) failed to submit bulk transfer: " << last_error_text;
                return false;
            }
        }

        qDebug() << "ZenoUSB: Wait for next bulk transfer";
        next_transfer_index = (buffer_index + 1) % 2;
        if (!waitForBulkTransfer(timeout_in_ms)) return false;
        buffer_index = next_transfer_index;

        offset = 0;
    }

    memcpy(out_bulk_transfer[buffer_index]->buffer + offset, request, ZENO_CMD_SIZE);
    out_bulk_transfer[buffer_index]->length += ZENO_CMD_SIZE;

    if ( next_transfer_index == -1 ) {
        // qDebug() << "-- submit buffer_index: " << buffer_index;
        int res = libusb_submit_transfer(out_bulk_transfer[buffer_index]);
        if ( res ) {
            last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
            device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
            qDebug() << "ERROR(ZenoUSB) failed to submit bulk transfer: " << last_error_text;
            return false;
        }
        next_transfer_index = (buffer_index + 1) % 2;
    }

    // qDebug() << "-- nt: " << next_transfer_index;
    // qDebug() << "b0 length: " << out_bulk_transfer[0]->length;
    // qDebug() << "b1 length: " << out_bulk_transfer[1]->length;

    return true;
}

bool VxZenoUSBDevice::queueRequest(ZenoCmd* request, int timeout_in_ms)
{
    QMutexLocker lock(&out_transfer_mutex);
    request->h.transaction_id = quint8(next_transaction_id ++);

    return ___queueRequestUnlocked(request,timeout_in_ms);
}

bool VxZenoUSBDevice::queueTxRequest(ZenoCmd* request, int timeout_in_ms)
{
    QMutexLocker lock(&out_transfer_mutex);
    return ___queueRequestUnlocked(request,timeout_in_ms);
}

bool VxZenoUSBDevice::startFlash()
{
    ZenoFlashStart cmd;
    ZenoResponse reply;

    if (!isOpen()) return false;

    memset(&cmd,0,sizeof(ZenoFlashStart));
    cmd.h.cmd_id = ZENO_CMD_FLASH_START;
    cmd.start_code = 0x1442u;

    if (!sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        qCritical() << "ERROR(KvUSB) failed to start FW udpate: " << last_error_text;
        return false;
    }

    stopClockInt();

    return true;
}

bool VxZenoUSBDevice::readFlashPage(quint16 page, quint16 offset, quint8* data)
{
    ZenoFlashReadRowToBuffer cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoFlashStart));
    cmd.h.cmd_id = ZENO_CMD_FLASH_READ_ROW_TO_BUFFER;
    cmd.page = page;
    cmd.offset = offset;

    if (!sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        qCritical() << "ERROR(KvUSB) failed to read flash-page: " << last_error_text;
        return false;
    }

    quint16 data_offset = 0;
    do {
        QMutexLocker lock(&read_flash_buffer_mutex);
        ZenoFlashReadFromBuffer read_cmd;

        memset(&read_cmd,0,sizeof(ZenoFlashReadFromBuffer));
        read_cmd.h.cmd_id = ZENO_CMD_FLASH_READ_ROW_FROM_BUFFER;
        read_cmd.offset = data_offset;

        read_reply_received = 0;
        uint read_size;
        if ( data_offset < 4080 ) read_size = 30;
        else read_size = 16;

        if (!queueRequest(zenoRequest(read_cmd))) return false;

        while (!read_reply_received) {
            if (!read_flash_buffer_cond.wait(&read_flash_buffer_mutex, reply_timeout_in_ms)) {
                last_error_text = "Timeout waiting for reply";
                return false;
            }
        }

        memcpy(data + data_offset, read_flash_buffer_response.data, read_size);
        data_offset += read_size;

        // qDebug() << "data_offset" << data_offset << read_size << read_cmd.offset;

        // QByteArray b = QByteArray::fromRawData(reinterpret_cast<const char*>(read_flash_buffer_response.data), read_size);
        // qDebug() << "data" << b.toHex(' ');
    } while ( data_offset < 4096);

    return true;
}

bool VxZenoUSBDevice::eraseFlashPage(quint16 page, quint16 offset)
{
    ZenoFlashEraseRow cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoFlashStart));
    cmd.h.cmd_id = ZENO_CMD_FLASH_ERASE_ROW;
    cmd.page = page;
    cmd.offset = offset;

    if (!sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        qCritical() << "ERROR(KvUSB) failed to erase flash-page: " << last_error_text;
        return false;
    }

    return true;
}

bool VxZenoUSBDevice::writeFlashPage(quint16 page, quint16 offset, const quint8* data)
{
    quint16 data_offset = 0;
    ZenoResponse reply;

    do {
        ZenoFlashWriteToBuffer write_cmd;

        memset(&write_cmd,0,sizeof(ZenoFlashWriteToBuffer));
        write_cmd.h.cmd_id = ZENO_CMD_FLASH_WRITE_TO_BUFFER;
        write_cmd.offset = data_offset;

        uint write_size;
        if ( data_offset < 4088 ) write_size = 28;
        else write_size = 8;

        memcpy(write_cmd.data, data + data_offset, write_size);

        if (!sendAndWhaitReply(zenoRequest(write_cmd), zenoReply(reply))) {
            qCritical() << "ERROR(KvUSB) failed to write to page buffer: " << last_error_text;
            return false;
        }
        // qDebug() << "data_offset" << data_offset << write_size;
        data_offset += write_size;
    } while ( data_offset < 4096);

#if 0
    data_offset = 0;
    do {
        QMutexLocker lock(&read_flash_buffer_mutex);
        ZenoFlashReadFromBuffer read_cmd;

        memset(&read_cmd,0,sizeof(ZenoFlashReadFromBuffer));
        read_cmd.h.cmd_id = ZENO_CMD_FLASH_READ_ROW_FROM_BUFFER;
        read_cmd.offset = data_offset;

        read_reply_received = 0;
        uint read_size;
        if ( data_offset < 4080 ) read_size = 30;
        else read_size = 16;

        if (!queueRequest(zenoRequest(read_cmd))) return false;

        while (!read_reply_received) {
            if (!read_flash_buffer_cond.wait(&read_flash_buffer_mutex, reply_timeout_in_ms)) {
                last_error_text = "Timeout waiting for reply";
                return false;
            }
        }

        // memcpy(data + data_offset, read_flash_buffer_response.data, read_size);
        data_offset += read_size;

        qDebug() << "data_offset" << data_offset << read_size << read_cmd.offset;

        QByteArray b = QByteArray::fromRawData(reinterpret_cast<const char*>(read_flash_buffer_response.data), read_size);
        qDebug() << "data" << b.toHex(' ');
    } while ( data_offset < 4096);
#endif

    ZenoFlashWriteRowFromBuffer write_flash_cmd;
    memset(&write_flash_cmd,0,sizeof(ZenoFlashWriteRowFromBuffer));
    write_flash_cmd.h.cmd_id = ZENO_CMD_FLASH_WRITE_ROW_FROM_BUFFER;
    write_flash_cmd.page = page;
    write_flash_cmd.offset = offset;

    if (!sendAndWhaitReply(zenoRequest(write_flash_cmd), zenoReply(reply))) {
        qCritical() << "ERROR(KvUSB) failed to write to flash page: " << last_error_text;
        return false;
    }

    return true;
}

bool VxZenoUSBDevice::endFlash(quint16 finish_code, quint16 from_table, quint16 page_count)
{
    ZenoFlashFinish cmd;
    ZenoResponse reply;

    if (!isOpen()) return false;

    memset(&cmd,0,sizeof(ZenoFlashFinish));
    cmd.h.cmd_id = ZENO_CMD_FLASH_FINISH;
    cmd.finish_code = finish_code;
    cmd.from_table = from_table;
    cmd.page_count = page_count;

    if (!sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        qCritical() << "ERROR(KvUSB) failed to finish FW udpate: " << last_error_text;
        return false;
    }

    if ( finish_code != 0x1445 ) {
        startClockInt();
    }

    return true;
}

VxZenoLINDriver* VxZenoUSBDevice::getZenoLINDriver() const
{
    VxZenoLINDriver* lin_driver = driver->getZenoLINDriver();
    Q_ASSERT(lin_driver != nullptr);
    return lin_driver;
}

bool VxZenoUSBDevice::waitForBulkTransfer(int timeout_in_ms)
{
    while (next_transfer_index != -1 &&
           out_bulk_transfer[next_transfer_index]->length > 0 ) {
//        qDebug() << "nt: " << next_transfer_index;
//        qDebug() << "length: " << out_bulk_transfer[next_transfer_index]->length;

        bulk_out_transfer_completed = 0;
        while (!bulk_out_transfer_completed) {
            if (!out_transfer_cond.wait(&out_transfer_mutex, timeout_in_ms)) {
                last_error_text = "Timeout, TX buffer overflow";
                return false;
            }
        }

        /* Device closed or handleEvents() failed */
        if ( open_ref_count == 0 ) return false;
    }

    return true;
}

void VxZenoUSBDevice::handleIncomingData(int bytes_transferred)
{
    int offset = 0;
    ZenoCmd* zeno_cmd;


    // if (bytes_transferred > 32) qDebug() << " R bytes_transfered " << bytes_transferred;
    while ( offset < bytes_transferred ) {
        zeno_cmd = reinterpret_cast<ZenoCmd*>(in_buffer + offset);

        // qDebug() << " R bytes_transfered " << bytes_transferred << zeno_cmd->h.cmd_id;
        // if ( bytes_transferred != 32 ) qDebug() << " R cmdNo: " << command_ptr->cmdNo << " trans id " << command_ptr->cmdIOPSeq.transId << " srcHE " << command_ptr->cmdIOPSeq.srcHE << " dst " << command_ptr->cmdIOP.dstAddr << "srcChannel" <<  command_ptr->cmdIOP.srcChannel;

        offset += ZENO_CMD_SIZE;
        handleCommand(zeno_cmd);

        if ( zeno_cmd->h.cmd_id == ZENO_CMD_RESPONSE) {
            QMutexLocker lock(&command_mutex);
            if ( reply_command != nullptr ) {

                ZenoResponse* response = reinterpret_cast<ZenoResponse*>(zeno_cmd);
                if ( response->response_cmd_id == reply_cmd_id ) {
                    *reply_command = *response;
                    reply_received = 1;
                    command_cond.wakeOne();
                }
            }
        }
    }
    // qDebug() << "URB done";
}

void VxZenoUSBDevice::handleInterruptData()
{
    ZenoIntCmd* zeno_int_cmd = reinterpret_cast<ZenoIntCmd*>(in_interrupt_buffer);
    switch(zeno_int_cmd->h.cmd_id) {
    case ZENO_CLOCK_INFO_INT:  {
        quint64 host_t0 = VxTimerBase::getCurrentTimeInUs() - t2_clock_start_ref_in_us;
        ZenoIntClockInfoCmd* clock_info = reinterpret_cast<ZenoIntClockInfoCmd*>(zeno_int_cmd);

        qint64 zeno_clock_in_us = clock_info->clock_value_t1 / clock_info->clock_divisor;
        qint64 drift_in_us = zeno_clock_in_us - host_t0;
        // qint64 t0_d = qAbs((clock_info->clock_value_t1 & 0xffffffff) - clock_info->clock_value_t0) / clock_info->clock_divisor;

        // qDebug() << "Clock-info:" << zeno_clock_in_us << host_t0 << drift_in_us;
        // qDebug() << "        t0:" << clock_info->clock_value_t0/70 << t0_d << drift_in_us - t0_d << drift_in_us + t0_d;

        if ( init_calibrate_count > 0 ) {
            init_calibrate_count--;
            drift_time_in_us += drift_in_us;
            if ( init_calibrate_count == 0 ) {
                drift_time_in_us /= 5;
                qDebug() << "Avg drift:" << drift_time_in_us;
                t2_clock_start_ref_in_us -= drift_time_in_us;
            }
        }
        else {
            foreach(auto channel, can_channel_list) {
                if (!channel->is_open) continue;
                if (!channel->initial_timer_adjustment_done) continue;

                qint64 device_clock_in_us = zeno_clock_in_us - channel->open_start_ref_timestamp_in_us;
                device_clock_in_us -= channel->getInitialTimeAdjustInUs();
                channel->ajdustDeviceTimeDrift(device_clock_in_us,
                                               drift_in_us);
            }
        }
        break;
    }
    default:
        break;
    }
}

void VxZenoUSBDevice::handleCommand(ZenoCmd* zeno_cmd)
{
    // qDebug() << zeno_cmd->h.cmd_id;
    switch(zeno_cmd->h.cmd_id) {
    case ZENO_CMD_CAN_TX_ACK: {
        ZenoTxCANRequestAck* zeno_tx_ack = reinterpret_cast<ZenoTxCANRequestAck*>(zeno_cmd);
        // qDebug() << "TX ack " << zeno_tx_ack->trans_id << zeno_tx_ack->dlc << zeno_tx_ack->id << zeno_tx_ack->timestamp;

        if ( zeno_tx_ack->channel < can_channel_list.size()) {
            static_cast<VxZenoCANChannel*>(can_channel_list[zeno_tx_ack->channel].get())->txAck(*zeno_tx_ack);
        }
        break;
    }
    case ZENO_CMD_CAN_RX: {
        ZenoCAN20Message* zeno_can_msg = reinterpret_cast<ZenoCAN20Message*>(zeno_cmd);
        // qDebug() << "RX ch:" << zeno_can_msg->channel << zeno_can_msg->dlc << zeno_can_msg->id << zeno_can_msg->timestamp;

        if ( zeno_can_msg->channel < can_channel_list.size()) {
            can_channel_list[zeno_can_msg->channel]->queueMessage(*zeno_can_msg);
        }

        break;
    }
    case ZENO_CMD_CANFD_P1_RX: {
        ZenoCANFDMessageP1* zeno_canfd_msg_p1 = reinterpret_cast<ZenoCANFDMessageP1*>(zeno_cmd);
        // qDebug() << "RX CANFD P1 ch:" << zeno_canfd_msg_p1->channel << zeno_canfd_msg_p1->dlc << zeno_canfd_msg_p1->id << zeno_canfd_msg_p1->timestamp;

        if ( zeno_canfd_msg_p1->channel < can_channel_list.size()) {
            can_channel_list[zeno_canfd_msg_p1->channel]->queueMessageCANFDP1(*zeno_canfd_msg_p1);
        }

        break;
    }
    case ZENO_CMD_CANFD_P2_RX: {
        ZenoCANFDMessageP2* zeno_canfd_msg_p2 = reinterpret_cast<ZenoCANFDMessageP2*>(zeno_cmd);
        // qDebug() << "RX CANFD P2 ch:" << zeno_canfd_msg_p2->channel;

        if ( zeno_canfd_msg_p2->channel < can_channel_list.size()) {
            can_channel_list[zeno_canfd_msg_p2->channel]->queueMessageCANFDP2(*zeno_canfd_msg_p2);
        }

        break;
    }
    case ZENO_CMD_CANFD_P3_RX: {
        ZenoCANFDMessageP3* zeno_canfd_msg_p3 = reinterpret_cast<ZenoCANFDMessageP3*>(zeno_cmd);
        // qDebug() << "RX CANFD P3 ch:" << zeno_canfd_msg_p3->channel;

        if ( zeno_canfd_msg_p3->channel < can_channel_list.size()) {
            can_channel_list[zeno_canfd_msg_p3->channel]->queueMessageCANFDP3(*zeno_canfd_msg_p3);
        }

        break;
    }
    case ZENO_CMD_LIN_RX_MESSAGE: {
        ZenoLINMessage* zeno_lin_msg = reinterpret_cast<ZenoLINMessage*>(zeno_cmd);
        // qDebug() << "ZenoLIN-RX" << zeno_lin_msg->channel;

        if ( lin_channel_list.size() == 2) {
            switch(zeno_lin_msg->channel) {
            case 0:
                lin_channel_list[1]->queueMessage(*zeno_lin_msg);
                break;
            case 1:
                lin_channel_list[0]->queueMessage(*zeno_lin_msg);
                break;
            }
        }

        break;
    }
    case ZENO_CMD_LIN_TX_ACK: {
        ZenoTxLINRequestAck* zeno_lin_tx_ack = reinterpret_cast<ZenoTxLINRequestAck*>(zeno_cmd);

        /* Assume Zeno CANquatro - with 2 LIN channels */
        if ( lin_channel_list.size() == 2) {
            switch(zeno_lin_tx_ack->channel) {
            case 0:
                lin_channel_list[1]->txAck(*zeno_lin_tx_ack);
                break;
            case 1:
                lin_channel_list[0]->txAck(*zeno_lin_tx_ack);
                break;
            }
        }

        break;
    }
    case ZENO_DEBUG: {
        ZenoDebug* zeno_debug = reinterpret_cast<ZenoDebug*>(zeno_cmd);
        int debug_msg_len = 30;
        for ( int i = 0; i < 30; ++i ) {
            if ( zeno_debug->debug_msg[i] == 0 ) {
                debug_msg_len = i;
                break;
            }
        }
        QString debug_msg = QString::fromLatin1(reinterpret_cast<const char*>(zeno_debug->debug_msg), debug_msg_len);
        qDebug() << "ZenoDBG: " << debug_msg;
        break;
    }
    case ZENO_CMD_FLASH_READ_ROW_FROM_BUFFER: {
        ZenoFlashReadFromBufferResponse* zeno_read_flash_response = reinterpret_cast<ZenoFlashReadFromBufferResponse*>(zeno_cmd);

        QMutexLocker lock(&read_flash_buffer_mutex);
        read_flash_buffer_response = *zeno_read_flash_response;
        read_reply_received = 1;
        read_flash_buffer_cond.wakeOne();
        break;
    }
    default:
        break;
    }
}

void VxZenoUSBDevice::startClockInt()
{
    ZenoCmd cmd;
    ZenoResponse reply;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZEMO_CMD_START_CLOCK_INT;

    qDebug() << "Sending Start-Clock INT cmd";
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        qCritical() << "ERROR(ZenoUSB): failed to start clock INT" << last_error_text;
        return;
    }
    t2_clock_start_ref_in_us = VxTimerBase::getCurrentTimeInUs();
    init_calibrate_count = 5;
    drift_time_in_us = 0;
}

void VxZenoUSBDevice::stopClockInt()
{
    ZenoCmd cmd;
    ZenoResponse reply;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZEMO_CMD_STOP_CLOCK_INT;

    qDebug() << "Sending Stop-Clock INT cmd";
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        qCritical() << "ERROR(ZenoUSB): failed to stop clock INT" << last_error_text;
        return;
    }
}

void VxZenoUSBDevice::__inBulkTransferCallback(libusb_transfer* in_bulk_transfer)
{
    VxZenoUSBDevice* _this = static_cast<VxZenoUSBDevice*>(in_bulk_transfer->user_data);

    // qDebug() << " bulk complete status: " << in_bulk_transfer->status << " transferrred: " << in_bulk_transfer->actual_length;

    if ( _this->open_ref_count == 0) {
        /* Device closed do not re-submit transfer */
        _this->in_bulk_transfer_complete = 1;
        return;
    }

    if ( in_bulk_transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
         in_bulk_transfer->status != LIBUSB_TRANSFER_CANCELLED &&
         in_bulk_transfer->status != LIBUSB_TRANSFER_COMPLETED ) {
        qCritical() << "ERROR(ZenoUSB) unexcepted bulk transfer status: " << in_bulk_transfer->status;
        _this->in_bulk_transfer_complete = 1;
        return;
    }

    if ( in_bulk_transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        qDebug() << " --- in bulk transfer was canceled";
    }

    if ( in_bulk_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        _this->handleIncomingData(in_bulk_transfer->actual_length);
    }

    /* Re-submit bulk transfer */
    int res;
    Q_ASSERT(_this->in_bulk_transfer_complete == 0);
    res = libusb_submit_transfer(_this->in_bulk_transfer);
    if ( res ) {
        _this->last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        _this->device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) failed to submit bulk transfer: " << _this->last_error_text;
        _this->in_bulk_transfer_complete = 1;
    }
}

void VxZenoUSBDevice::__inInterruptTransferCallback(libusb_transfer *in_interrupt_transfer)
{
    VxZenoUSBDevice* _this = static_cast<VxZenoUSBDevice*>(in_interrupt_transfer->user_data);

    // qDebug() << __PRETTY_FUNCTION__ << "in" << in_interrupt_transfer->status <<  _this->open_ref_count;

    if ( _this->open_ref_count == 0) {
        /* Device closed do not re-submit transfer */
        _this->in_interrupt_transfer_complete = 1;
        return;
    }

    if ( in_interrupt_transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
         in_interrupt_transfer->status != LIBUSB_TRANSFER_CANCELLED &&
         in_interrupt_transfer->status != LIBUSB_TRANSFER_COMPLETED ) {
        qCritical() << "ERROR(ZenoUSB) unexcepted interrupt transfer status: " << in_interrupt_transfer->status;
        _this->in_interrupt_transfer_complete = 1;
        return;
    }


    if ( in_interrupt_transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        qDebug() << " --- in interrupt transfer was canceled";
    }

    if ( in_interrupt_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        // qDebug() << "Interrupt data ready:" << in_interrupt_transfer->length;
        _this->handleInterruptData();
    }

    /* Re-submit bulk transfer */
    int res;
    Q_ASSERT(_this->in_interrupt_transfer_complete == 0);
    res = libusb_submit_transfer(_this->in_interrupt_transfer);
    if ( res ) {
        _this->last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
        _this->device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        qDebug() << "ERROR(ZenoUSB) failed to submit bulk transfer: " << _this->last_error_text;
        _this->in_interrupt_transfer_complete = 1;
    }
}

void VxZenoUSBDevice::__outBulkTransferCallback(libusb_transfer* out_bulk_transfer)
{
    VxZenoUSBDevice* _this = static_cast<VxZenoUSBDevice*>(out_bulk_transfer->user_data);
    QMutexLocker lock(&_this->out_transfer_mutex);

    // qDebug() << " TX bulk complete status: " << out_bulk_transfer->status << " transferrred: " << out_bulk_transfer->actual_length;
    _this->bulk_out_transfer_completed = 1;
    _this->out_transfer_cond.wakeOne();

    if ( _this->open_ref_count == 0) {
        /* Device closed do not re-submit transfer */
        _this->next_transfer_index = -1;
        return;
    }

    if ( out_bulk_transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
         out_bulk_transfer->status != LIBUSB_TRANSFER_CANCELLED &&
         out_bulk_transfer->status != LIBUSB_TRANSFER_COMPLETED ) {
        qCritical() << "ERROR(ZenoUSB) unexcepted transfer status: " << out_bulk_transfer->status;
        _this->next_transfer_index = -1;
        return;
    }

    Q_ASSERT(_this->next_transfer_index != -1);
    int res;
    out_bulk_transfer->length = 0;

    libusb_transfer* next_transfer;
    if ( out_bulk_transfer == _this->out_bulk_transfer[0] ) {
        _this->next_transfer_index = 0;
        next_transfer = _this->out_bulk_transfer[1];
    } else {
        Q_ASSERT(out_bulk_transfer == _this->out_bulk_transfer[1]);
        _this->next_transfer_index = 1;
        next_transfer = _this->out_bulk_transfer[0];
    }

    if ( next_transfer->length > 0 ) {
        res = libusb_submit_transfer(next_transfer);
        if ( res ) {
            _this->last_error_text = VxUSBContext::translateLibUSBErrorCode(res);
            _this->device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
            qDebug() << "ERROR(ZenoUSB) failed to submit bulk transfer: " << _this->last_error_text;
            _this->next_transfer_index = -1;
        }
        // _this->next_transfer_index = (_this->next_transfer_index + 1) % 2;
    } else {
        _this->next_transfer_index = -1;
    }
}

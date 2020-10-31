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

#include "canlib.h"

#pragma GCC diagnostic ignored "-Wunused-parameter"

void CANLIBAPI canInitializeLibrary (void)
{
}

canStatus CANLIBAPI canClose (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canBusOn (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canBusOff (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetBusParams (const CanHandle handle,
                                     long freq,
                                     unsigned int tseg1,
                                     unsigned int tseg2,
                                     unsigned int sjw,
                                     unsigned int noSamp,
                                     unsigned int syncmode)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetBusParamsFd(const CanHandle handle,
                                      long freq_brs,
                                      unsigned int tseg1_brs,
                                      unsigned int tseg2_brs,
                                      unsigned int sjw_brs)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetBusParams (const CanHandle handle,
                                     long  *freq,
                                     unsigned int *tseg1,
                                     unsigned int *tseg2,
                                     unsigned int *sjw,
                                     unsigned int *noSamp,
                                     unsigned int *syncmode)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetBusParamsFd(const CanHandle handle,
                                      long  *freq_brs,
                                      unsigned int *tseg1_brs,
                                      unsigned int *tseg2_brs,
                                      unsigned int *sjw_brs)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetBusOutputControl (const CanHandle handle,
                                            const unsigned int drivertype)
{
    return canERR_NOT_IMPLEMENTED;
}


canStatus CANLIBAPI canGetBusOutputControl (const CanHandle handle,
                                            unsigned int *drivertype)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canAccept (const CanHandle handle,
                               const long envelope,
                               const unsigned int flag)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadStatus (const CanHandle handle,
                                   unsigned long *const flags)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadErrorCounters (const CanHandle handle,
                                          unsigned int *txErr,
                                          unsigned int *rxErr,
                                          unsigned int *ovErr)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canWrite (const CanHandle handle,
                              long id,
                              void *msg,
                              unsigned int dlc,
                              unsigned int flag)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canWriteSync (const CanHandle handle, unsigned long timeout)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canRead (const CanHandle handle,
                             long *id,
                             void *msg,
                             unsigned int *dlc,
                             unsigned int *flag,
                             unsigned long *time)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadWait (const CanHandle handle,
                                 long *id,
                                 void *msg,
                                 unsigned int  *dlc,
                                 unsigned int  *flag,
                                 unsigned long *time,
                                 unsigned long timeout)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSpecific (const CanHandle handle, long id, void * msg,
                                     unsigned int * dlc, unsigned int * flag,
                                     unsigned long * time)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSync (const CanHandle handle, unsigned long timeout)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSyncSpecific (const CanHandle handle,
                                         long id,
                                         unsigned long timeout)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSpecificSkip (const CanHandle handle,
                                         long id,
                                         void * msg,
                                         unsigned int * dlc,
                                         unsigned int * flag,
                                         unsigned long * time)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetNotify (const CanHandle handle,
                                  void (*callback)(canNotifyData *),
                                  unsigned int notif_fFlags,
                                  void *tag)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetRawHandle (const CanHandle handle, void *pvFd)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canTranslateBaud (long *const freq,
                                      unsigned int *const tseg1,
                                      unsigned int *const tseg2,
                                      unsigned int *const sjw,
                                      unsigned int *const nosamp,
                                      unsigned int *const syncMode)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetErrorText (canStatus err, char *buf, unsigned int bufsiz)
{
    return canERR_NOT_IMPLEMENTED;
}

unsigned short CANLIBAPI canGetVersion (void)
{
    return 14;
}

canStatus CANLIBAPI canIoCtl (const CanHandle handle,
                              unsigned int func,
                              void *buf,
                              unsigned int buflen)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadTimer (const CanHandle handle, unsigned long *time)
{
    return canERR_NOT_IMPLEMENTED;
}

CanHandle CANLIBAPI canOpenChannel (int channel, int flags)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetNumberOfChannels (int *channelCount)
{
    return canERR_NOT_IMPLEMENTED;
}


canStatus CANLIBAPI canGetChannelData (int channel,
                                       int item,
                                       void *buffer,
                                       size_t bufsize)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetBusParamsC200 (const CanHandle handle, unsigned char btr0, unsigned char btr1)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetDriverMode (const CanHandle handle, int lineMode, int resNet)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetDriverMode (const CanHandle handle, int *lineMode, int *resNet)
{
    return canERR_NOT_IMPLEMENTED;
}

unsigned int CANLIBAPI canGetVersionEx (unsigned int itemCode)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufFreeAll (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufAllocate (const CanHandle handle, int type)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufFree (const CanHandle handle, int idx)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufWrite (const CanHandle handle,
                                    int idx,
                                    int id,
                                    void* msg,
                                    unsigned int dlc,
                                    unsigned int flags)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetFilter (const CanHandle handle,
                                        int idx,
                                        unsigned int code,
                                        unsigned int mask)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetFlags (const CanHandle handle,
                                       int idx,
                                       unsigned int flags)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetPeriod (const CanHandle handle,
                                        int idx,
                                        unsigned int period)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetMsgCount (const CanHandle handle,
                                          int idx,
                                          unsigned int count)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufEnable (const CanHandle handle, int idx)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufDisable (const CanHandle handle, int idx)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSendBurst (const CanHandle handle,
                                        int idx,
                                        unsigned int burstlen)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canResetBus (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canWriteWait (const CanHandle handle,
                                  long id,
                                  void *msg,
                                  unsigned int dlc,
                                  unsigned int flag,
                                  unsigned long timeout)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canUnloadLibrary (void)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetAcceptanceFilter (const CanHandle handle,
                                            unsigned int code,
                                            unsigned int mask,
                                            int is_extended)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canFlushReceiveQueue (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canFlushTransmitQueue (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvFlashLeds (const CanHandle handle, int action, int timeout)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canRequestChipStatus (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canRequestBusStatistics (const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetBusStatistics (const CanHandle handle,
                                         canBusStatistics *stat,
                                         size_t bufsiz)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetHandleData (const CanHandle handle,
                                      int item,
                                      void *buffer,
                                      size_t bufsize)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainCreate (kvTimeDomain *domain)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainDelete (kvTimeDomain domain)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainResetTime (kvTimeDomain domain)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainGetData (kvTimeDomain domain,
                                        kvTimeDomainData *data,
                                        size_t bufsiz)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainAddHandle(kvTimeDomain domain,
                                         const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainRemoveHandle (kvTimeDomain domain,
                                             const CanHandle handle)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvSetNotifyCallback (const CanHandle handle,
                                        kvCallback_t callback,
                                        void* context,
                                        unsigned int notif_fFlags)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvGetSupportedInterfaceInfo (int index,
                                                char *hwName,
                                                size_t nameLen,
                                                int *hwType,
                                                int *hwBusType)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvReadDeviceCustomerData (const CanHandle handle,
                                             int userNumber,
                                             int itemNumber,
                                             void *data,
                                             size_t bufsiz)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptStart (const CanHandle handle, int slotNo)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptStop (const CanHandle handle, int slotNo, int mode)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptUnload (const CanHandle handle, int slotNo)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptSendEvent (const CanHandle handle,
                                      int slotNo,
                                      int eventType,
                                      int eventNo,
                                      unsigned int data)
{
    return canERR_NOT_IMPLEMENTED;
}

kvEnvHandle CANLIBAPI kvScriptEnvvarOpen (const CanHandle handle,
                                          char* envvarName,
                                          int *envvarType,
                                          int *envvarSize)
{
    return -1;
}

kvStatus CANLIBAPI kvScriptEnvvarClose (kvEnvHandle e_handle)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarSetInt (kvEnvHandle e_handle, int val)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarGetInt (kvEnvHandle e_handle, int *val)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarSetFloat (kvEnvHandle e_handle, float val)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarGetFloat (kvEnvHandle e_handle, float *val)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarSetData (kvEnvHandle e_handle,
                                          void *buf,
                                          int start_index,
                                          int data_len)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarGetData (kvEnvHandle e_handle,
                                          void *buf,
                                          int start_index,
                                          int data_len)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptLoadFileOnDevice (const CanHandle handle,
                                             int slotNo,
                                             char *localFile);

kvStatus CANLIBAPI kvScriptLoadFile (const CanHandle handle,
                                     int slotNo,
                                     char *filePathOnPC)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptStatus(const CanHandle handle,
                                  int  slot,
                                  unsigned int *status)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptGetMaxEnvvarSize(int hnd, int *envvarSize)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptTxeGetData(const char *filePathOnPC,
                                      int item,
                                      void *buffer,
                                      unsigned int *bufsize)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileCopyToDevice (const CanHandle handle,
                                       char *hostFileName,
                                       char *deviceFileName)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileCopyFromDevice (const CanHandle handle,
                                         char *deviceFileName,
                                         char *hostFileName)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileDelete (const CanHandle handle, char *deviceFileName)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileGetName (const CanHandle handle,
                                  int fileNo,
                                  char *name,
                                  int namelen)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileGetCount (const CanHandle handle, int *count)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileGetSystemData (const CanHandle handle,
                                        int itemCode,
                                        int *result)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvDeviceSetMode (const CanHandle handle, int mode)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvDeviceGetMode (const CanHandle handle, int *result)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvReadTimer (const CanHandle handle, unsigned int *time)
{
    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvReadTimer64 (const CanHandle handle, uint64_t *time)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoGetNumberOfPins (const CanHandle handle, unsigned int *pinCount)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetInfo (const CanHandle handle, unsigned int pin, int item, void *buffer, const unsigned int bufsize)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetInfo (const CanHandle handle, unsigned int pin, int item, const void *buffer, const unsigned int bufsize)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetDigital (const CanHandle handle, unsigned int pin, unsigned int value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetDigital (const CanHandle handle, unsigned int pin, unsigned int *value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetOutputDigital (const CanHandle handle, unsigned int pin, unsigned int *value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetRelay (const CanHandle handle, unsigned int pin, unsigned int value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetOutputRelay (const CanHandle handle, unsigned int pin, unsigned int *value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetAnalog (const CanHandle handle, unsigned int pin, float value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetAnalog (const CanHandle handle, unsigned int pin, float* value)
{
    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetOutputAnalog (const CanHandle handle, unsigned int pin, float* value)
{
    return canERR_NOT_IMPLEMENTED;
}

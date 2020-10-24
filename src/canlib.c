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

void CANLIBAPI canInitializeLibrary (void)
{
}

canStatus CANLIBAPI canClose (const CanHandle hnd)
{
}

canStatus CANLIBAPI canBusOn (const CanHandle hnd)
{
}

canStatus CANLIBAPI canBusOff (const CanHandle hnd)
{
}

canStatus CANLIBAPI canSetBusParams (const CanHandle hnd,
                                     long freq,
                                     unsigned int tseg1,
                                     unsigned int tseg2,
                                     unsigned int sjw,
                                     unsigned int noSamp,
                                     unsigned int syncmode)
{
}

canStatus CANLIBAPI canSetBusParamsFd(const CanHandle hnd,
                                      long freq_brs,
                                      unsigned int tseg1_brs,
                                      unsigned int tseg2_brs,
                                      unsigned int sjw_brs);

canStatus CANLIBAPI canGetBusParams (const CanHandle hnd,
                                     long  *freq,
                                     unsigned int *tseg1,
                                     unsigned int *tseg2,
                                     unsigned int *sjw,
                                     unsigned int *noSamp,
                                     unsigned int *syncmode);

canStatus CANLIBAPI canGetBusParamsFd(const CanHandle hnd,
                                      long  *freq_brs,
                                      unsigned int *tseg1_brs,
                                      unsigned int *tseg2_brs,
                                      unsigned int *sjw_brs);

canStatus CANLIBAPI canSetBusOutputControl (const CanHandle hnd,
                                            const unsigned int drivertype);


canStatus CANLIBAPI canGetBusOutputControl (const CanHandle hnd,
                                            unsigned int *drivertype);

canStatus CANLIBAPI canAccept (const CanHandle hnd,
                               const long envelope,
                               const unsigned int flag);

canStatus CANLIBAPI canReadStatus (const CanHandle hnd,
                                   unsigned long *const flags);

canStatus CANLIBAPI canReadErrorCounters (const CanHandle hnd,
                                          unsigned int *txErr,
                                          unsigned int *rxErr,
                                          unsigned int *ovErr);

canStatus CANLIBAPI canWrite (const CanHandle hnd,
                              long id,
                              void *msg,
                              unsigned int dlc,
                              unsigned int flag);

canStatus CANLIBAPI canWriteSync (const CanHandle hnd, unsigned long timeout);

canStatus CANLIBAPI canRead (const CanHandle hnd,
                             long *id,
                             void *msg,
                             unsigned int *dlc,
                             unsigned int *flag,
                             unsigned long *time);

canStatus CANLIBAPI canReadWait (const CanHandle hnd,
                                 long *id,
                                 void *msg,
                                 unsigned int  *dlc,
                                 unsigned int  *flag,
                                 unsigned long *time,
                                 unsigned long timeout);

canStatus CANLIBAPI canReadSpecific (const CanHandle hnd, long id, void * msg,
                                     unsigned int * dlc, unsigned int * flag,
                                     unsigned long * time);

canStatus CANLIBAPI canReadSync (const CanHandle hnd, unsigned long timeout);

canStatus CANLIBAPI canReadSyncSpecific (const CanHandle hnd,
                                         long id,
                                         unsigned long timeout);

canStatus CANLIBAPI canReadSpecificSkip (const CanHandle hnd,
                                         long id,
                                         void * msg,
                                         unsigned int * dlc,
                                         unsigned int * flag,
                                         unsigned long * time);

canStatus CANLIBAPI canSetNotify (const CanHandle hnd,
                                  void (*callback)(canNotifyData *),
                                  unsigned int notifyFlags,
                                  void *tag);

canStatus CANLIBAPI canGetRawHandle (const CanHandle hnd, void *pvFd);

canStatus CANLIBAPI canTranslateBaud (long *const freq,
                                      unsigned int *const tseg1,
                                      unsigned int *const tseg2,
                                      unsigned int *const sjw,
                                      unsigned int *const nosamp,
                                      unsigned int *const syncMode);

canStatus CANLIBAPI canGetErrorText (canStatus err, char *buf, unsigned int bufsiz);

unsigned short CANLIBAPI canGetVersion (void);

canStatus CANLIBAPI canIoCtl (const CanHandle hnd,
                              unsigned int func,
                              void *buf,
                              unsigned int buflen);

canStatus CANLIBAPI canReadTimer (const CanHandle hnd, unsigned long *time);

CanHandle CANLIBAPI canOpenChannel (int channel, int flags);

canStatus CANLIBAPI canGetNumberOfChannels (int *channelCount);


canStatus CANLIBAPI canGetChannelData (int channel,
                                       int item,
                                       void *buffer,
                                       size_t bufsize);

canStatus CANLIBAPI canSetBusParamsC200 (const CanHandle hnd, unsigned char btr0, unsigned char btr1);

canStatus CANLIBAPI canSetDriverMode (const CanHandle hnd, int lineMode, int resNet);

canStatus CANLIBAPI canGetDriverMode (const CanHandle hnd, int *lineMode, int *resNet);

unsigned int CANLIBAPI canGetVersionEx (unsigned int itemCode);

canStatus CANLIBAPI canObjBufFreeAll (const CanHandle hnd);

canStatus CANLIBAPI canObjBufAllocate (const CanHandle hnd, int type);

canStatus CANLIBAPI canObjBufFree (const CanHandle hnd, int idx);

canStatus CANLIBAPI canObjBufWrite (const CanHandle hnd,
                                    int idx,
                                    int id,
                                    void* msg,
                                    unsigned int dlc,
                                    unsigned int flags);

canStatus CANLIBAPI canObjBufSetFilter (const CanHandle hnd,
                                        int idx,
                                        unsigned int code,
                                        unsigned int mask);

canStatus CANLIBAPI canObjBufSetFlags (const CanHandle hnd,
                                       int idx,
                                       unsigned int flags);

canStatus CANLIBAPI canObjBufSetPeriod (const CanHandle hnd,
                                        int idx,
                                        unsigned int period);

canStatus CANLIBAPI canObjBufSetMsgCount (const CanHandle hnd,
                                          int idx,
                                          unsigned int count);

canStatus CANLIBAPI canObjBufEnable (const CanHandle hnd, int idx);

canStatus CANLIBAPI canObjBufDisable (const CanHandle hnd, int idx);

canStatus CANLIBAPI canObjBufSendBurst (const CanHandle hnd,
                                        int idx,
                                        unsigned int burstlen);

canStatus CANLIBAPI canResetBus (const CanHandle hnd);

canStatus CANLIBAPI canWriteWait (const CanHandle hnd,
                                  long id,
                                  void *msg,
                                  unsigned int dlc,
                                  unsigned int flag,
                                  unsigned long timeout);

canStatus CANLIBAPI canUnloadLibrary (void);

canStatus CANLIBAPI canSetAcceptanceFilter (const CanHandle hnd,
                                            unsigned int code,
                                            unsigned int mask,
                                            int is_extended);
canStatus CANLIBAPI canFlushReceiveQueue (const CanHandle hnd);

canStatus CANLIBAPI canFlushTransmitQueue (const CanHandle hnd);

canStatus CANLIBAPI kvFlashLeds (const CanHandle hnd, int action, int timeout);

canStatus CANLIBAPI canRequestChipStatus (const CanHandle hnd);

canStatus CANLIBAPI canRequestBusStatistics (const CanHandle hnd);

canStatus CANLIBAPI canGetBusStatistics (const CanHandle hnd,
                                         canBusStatistics *stat,
                                         size_t bufsiz);

canStatus CANLIBAPI canGetHandleData (const CanHandle hnd,
                                      int item,
                                      void *buffer,
                                      size_t bufsize);

kvStatus CANLIBAPI kvTimeDomainCreate (kvTimeDomain *domain);

kvStatus CANLIBAPI kvTimeDomainDelete (kvTimeDomain domain);

kvStatus CANLIBAPI kvTimeDomainResetTime (kvTimeDomain domain);

kvStatus CANLIBAPI kvTimeDomainGetData (kvTimeDomain domain,
                                        kvTimeDomainData *data,
                                        size_t bufsiz);

kvStatus CANLIBAPI kvTimeDomainAddHandle(kvTimeDomain domain,
                                         const CanHandle hnd);

kvStatus CANLIBAPI kvTimeDomainRemoveHandle (kvTimeDomain domain,
                                             const CanHandle hnd);

kvStatus CANLIBAPI kvSetNotifyCallback (const CanHandle hnd,
                                        kvCallback_t callback,
                                        void* context,
                                        unsigned int notifyFlags);

kvStatus CANLIBAPI kvGetSupportedInterfaceInfo (int index,
                                                char *hwName,
                                                size_t nameLen,
                                                int *hwType,
                                                int *hwBusType);

kvStatus CANLIBAPI kvReadDeviceCustomerData (const CanHandle hnd,
                                             int userNumber,
                                             int itemNumber,
                                             void *data,
                                             size_t bufsiz);

kvStatus CANLIBAPI kvScriptStart (const CanHandle hnd, int slotNo);

kvStatus CANLIBAPI kvScriptStop (const CanHandle hnd, int slotNo, int mode);

kvStatus CANLIBAPI kvScriptUnload (const CanHandle hnd, int slotNo);

kvStatus CANLIBAPI kvScriptSendEvent (const CanHandle hnd,
                                      int slotNo,
                                      int eventType,
                                      int eventNo,
                                      unsigned int data);

kvEnvHandle CANLIBAPI kvScriptEnvvarOpen (const CanHandle hnd,
                                          char* envvarName,
                                          int *envvarType,
                                          int *envvarSize); // returns scriptHandle

kvStatus CANLIBAPI kvScriptEnvvarClose (kvEnvHandle eHnd);

kvStatus CANLIBAPI kvScriptEnvvarSetInt (kvEnvHandle eHnd, int val);

kvStatus CANLIBAPI kvScriptEnvvarGetInt (kvEnvHandle eHnd, int *val);

kvStatus CANLIBAPI kvScriptEnvvarSetFloat (kvEnvHandle eHnd, float val);

kvStatus CANLIBAPI kvScriptEnvvarGetFloat (kvEnvHandle eHnd, float *val);

kvStatus CANLIBAPI kvScriptEnvvarSetData (kvEnvHandle eHnd,
                                          void *buf,
                                          int start_index,
                                          int data_len);

kvStatus CANLIBAPI kvScriptEnvvarGetData (kvEnvHandle eHnd,
                                          void *buf,
                                          int start_index,
                                          int data_len);

kvStatus CANLIBAPI kvScriptLoadFileOnDevice (const CanHandle hnd,
                                             int slotNo,
                                             char *localFile);

kvStatus CANLIBAPI kvScriptLoadFile (const CanHandle hnd,
                                     int slotNo,
                                     char *filePathOnPC);

kvStatus CANLIBAPI kvScriptStatus(const CanHandle hnd,
                                  int  slot,
                                  unsigned int *status);

kvStatus CANLIBAPI kvScriptGetMaxEnvvarSize(int hnd, int *envvarSize);

kvStatus CANLIBAPI kvScriptTxeGetData(const char *filePathOnPC,
                                      int item,
                                      void *buffer,
                                      unsigned int *bufsize);

kvStatus CANLIBAPI kvFileCopyToDevice (const CanHandle hnd,
                                       char *hostFileName,
                                       char *deviceFileName);

kvStatus CANLIBAPI kvFileCopyFromDevice (const CanHandle hnd,
                                         char *deviceFileName,
                                         char *hostFileName);

kvStatus CANLIBAPI kvFileDelete (const CanHandle hnd, char *deviceFileName);

kvStatus CANLIBAPI kvFileGetName (const CanHandle hnd,
                                  int fileNo,
                                  char *name,
                                  int namelen);

kvStatus CANLIBAPI kvFileGetCount (const CanHandle hnd, int *count);

kvStatus CANLIBAPI kvFileGetSystemData (const CanHandle hnd,
                                        int itemCode,
                                        int *result);

kvStatus CANLIBAPI kvDeviceSetMode (const CanHandle hnd, int mode);

kvStatus CANLIBAPI kvDeviceGetMode (const CanHandle hnd, int *result);

kvStatus CANLIBAPI kvReadTimer (const CanHandle hnd, unsigned int *time);

kvStatus CANLIBAPI kvReadTimer64 (const CanHandle hnd, uint64_t *time);

canStatus CANLIBAPI kvIoGetNumberOfPins (const CanHandle hnd, unsigned int *pinCount);

canStatus CANLIBAPI kvIoPinGetInfo (const CanHandle hnd, unsigned int pin, int item, void *buffer, const unsigned int bufsize);

canStatus CANLIBAPI kvIoPinSetInfo (const CanHandle hnd, unsigned int pin, int item, const void *buffer, const unsigned int bufsize);

canStatus CANLIBAPI kvIoPinSetDigital (const CanHandle hnd, unsigned int pin, unsigned int value);

canStatus CANLIBAPI kvIoPinGetDigital (const CanHandle hnd, unsigned int pin, unsigned int *value);

canStatus CANLIBAPI kvIoPinGetOutputDigital (const CanHandle hnd, unsigned int pin, unsigned int *value);

canStatus CANLIBAPI kvIoPinSetRelay (const CanHandle hnd, unsigned int pin, unsigned int value);

canStatus CANLIBAPI kvIoPinGetOutputRelay (const CanHandle hnd, unsigned int pin, unsigned int *value);

canStatus CANLIBAPI kvIoPinSetAnalog (const CanHandle hnd, unsigned int pin, float value);

canStatus CANLIBAPI kvIoPinGetAnalog (const CanHandle hnd, unsigned int pin, float* value);

canStatus CANLIBAPI kvIoPinGetOutputAnalog (const CanHandle hnd, unsigned int pin, float* value);

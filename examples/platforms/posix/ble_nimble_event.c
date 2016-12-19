/*
 *  Copyright (c) 2018, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This defines the eventing mechanism between NimBLE host task
 *   and the main openthread task.
 */

#include "platform-posix.h"

#include <openthread/platform/ble.h>
#include <openthread/platform/ble_hci.h>

#include <common/logging.hpp>

#include <nimble/nimble_npl.h>

#include "ble_nimble_event.h"

#define PIPE_READ 0
#define PIPE_WRITE 1

static int      sPipeFd[2]; ///< file descriptors for waking ot
extern uint32_t gNodeId;

void ble_hci_sock_set_device(int dev);

void platformBleInit()
{
    int err = pipe2(sPipeFd, O_NONBLOCK);
    if (err)
    {
        fprintf(stderr, "ERROR: platformBleInit unable to open pipe.\n");
        exit(EXIT_FAILURE);
    }
}

void platformBleSignal()
{
    uint8_t val = 1;
    write(sPipeFd[PIPE_WRITE], &val, sizeof(val));
    otPlatBleHciTick(NULL);
}

void platformBleUpdateFdSet(fd_set *aReadFdSet, int *aMaxFd)
{
    if (aReadFdSet != NULL && sPipeFd[PIPE_READ])
    {
        FD_SET(sPipeFd[PIPE_READ], aReadFdSet);

        if (aMaxFd != NULL && *aMaxFd < sPipeFd[PIPE_READ])
        {
            *aMaxFd = sPipeFd[PIPE_READ];
        }
    }
}

void platformBleProcess(otInstance *aInstance)
{
    otPlatBleHciTick(aInstance);
}

//=============================================================================
//                        HCI
//=============================================================================

int otPlatBleHciGetDeviceId(otInstance *aInstance)
{
    (void)aInstance;

    return gNodeId;
}

void otPlatBleHciSetDeviceId(otInstance *aInstance, int aDeviceId)
{
    (void)aInstance;

    if (aDeviceId >= 0)
    {
        otLogDebgBle("otPlatBleHciSetDeviceId: %d", aDeviceId);

        gNodeId = aDeviceId;

        ble_hci_sock_set_device(aDeviceId);
    }
}

void otPlatBleHciTick(otInstance *aInstance)
{
    (void)aInstance;

    // Relinquish time on app thread to let the BLE host thread run a bit.
    ble_npl_task_yield();
}

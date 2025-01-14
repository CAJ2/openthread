#!/usr/bin/env python
#
#  Copyright (c) 2018, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

import os
import sys
import time
import pexpect
import unittest
import subprocess

from node_cli import Node

CENTRAL = 1
PERIPHERAL = 2
NODE_COUNT = 2

class test_conn(unittest.TestCase):
    def setUp(self):
        self.nodes = Node.setUp(NODE_COUNT)

    def tearDown(self):
        del self.nodes
        Node.tearDown()

    def test_connection(self):

        self.nodes[CENTRAL].ble_start()

        self.nodes[PERIPHERAL].ble_start()
        self.nodes[PERIPHERAL].ble_adv_data("0201060302affe")
        self.nodes[PERIPHERAL].ble_adv_start(500)
        dst_addr = self.nodes[PERIPHERAL].ble_get_bdaddr()

        self.nodes[CENTRAL].ble_conn_start(dst_addr, 1)

        self.nodes[CENTRAL].pexpect.expect("connected: @id=")
        self.nodes[PERIPHERAL].pexpect.expect("connected: @id=")

        self.nodes[CENTRAL].ble_conn_stop()
        self.nodes[PERIPHERAL].pexpect.expect("disconnected: @id=")
        self.nodes[CENTRAL].pexpect.expect("disconnected: @id=")

        # Connect second time for robust testing

        self.nodes[PERIPHERAL].ble_adv_start(500)
        self.nodes[CENTRAL].ble_scan_start()
        self.nodes[CENTRAL].ble_scan_stop()
        self.nodes[CENTRAL].ble_conn_start(dst_addr, 1)

        self.nodes[PERIPHERAL].pexpect.expect("connected: @id=")
        self.nodes[CENTRAL].pexpect.expect("connected: @id=")


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3

import asyncio
import asyncudp
import ipaddress
import struct


# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.

class Pkt:
    def __init__(self, msg:str, msg_type: int = 0):
        self.msg_type = msg_type
        self.msg = msg

    def __repr__(self):
        return f"Pkt('{self.msg}', {self.msg_type})"

    def pack(self):
        # I: unsigned integer
        # p: pascal string - first byte is length <= 255, the remaining bytes are the chars
        encoded_msg = self.msg.encode('utf-8')
        msg_len = len(encoded_msg)
        #return struct.pack("!Ip", self.msg_type, encoded_msg)
        return struct.pack(f"!II{msg_len}s", self.msg_type, msg_len, encoded_msg)

    @classmethod
    def unpack(cls, buffer: bytes):
        msg_type, msg_size = struct.unpack_from("!II", buffer, 0)
        msg, = struct.unpack_from(f"!{msg_size}s", buffer, 8)
        #msg_type, msg = struct.unpack("!Ip", buffer)
        return Pkt(msg.decode('utf-8'), msg_type)


async def producer(queue: asyncio.Queue):
    sendcount = 0
    while True:
        p = Pkt(f"Packet#{sendcount}")
        #print(f"Created {p}")
        sendcount += 1
        await queue.put(p)


async def sender(queue: asyncio.Queue, ipaddr: ipaddress.ip_address, port: int):
    sock = await asyncudp.create_socket(remote_addr=(format(ipaddr), port))

    while True:
        pkt = await queue.get()
        #print(f"Pulled {pkt} from queue")
        try:
            sock.sendto(pkt.pack())
        except ConnectionRefusedError:
            # ignore dropped packets
            pass
        #print(f"Sent {pkt.pack()}")
        queue.task_done()
        await asyncio.sleep(.020)


async def consumer(ipaddr: ipaddress.ip_address, port: int):
    sock = await asyncudp.create_socket(local_addr=(format(ipaddr), port))

    while True:
        datagram, addr = await sock.recvfrom()
        #print(f"Got {len(datagram)} UDP packet ({datagram}) from {addr}")
        p = Pkt.unpack(datagram)

        print(str(p))


async def main(ipaddr: ipaddress.ip_address, port: int, client: bool = False):
    if client:
        await asyncio.gather(consumer(ipaddr, port))
    else:
        pktQ = asyncio.Queue(1)
        await asyncio.gather(producer(pktQ), sender(pktQ, ipaddr, port))


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    import argparse

    DEF_IP_ADDR = ipaddress.ip_address('127.0.0.1')
    DEF_PORT = 5999

    parser = argparse.ArgumentParser("ex-client-server", description="Example UDP client/server")
    parser.add_argument('--ip-addr', '-i', default=DEF_IP_ADDR, type=ipaddress.ip_address)
    parser.add_argument('--port', '-p', default=DEF_PORT, type=int)
    parser.add_argument('--client', '-c', default=False, action='store_true',)
    args = parser.parse_args()

    asyncio.run(main(args.ip_addr, args.port, args.client))
#! /usr/bin/env python3

import sys

from itertools import cycle

from scapy.all import IP, TCP, UDP, wrpcap, Ether, Raw


def generate_pcap(nb_pkts, out_pcap, pkt_size, nb_dest):
    sample_pkts = []
    for i in range(nb_dest):
        pkt = (
            Ether() /
            IP(dst=f'192.168.1.{i}', src='192.168.0.1') /
            TCP(dport=80, sport=8080, flags='S')
        )

        # payload = '0' * missing_bytes
        # pkt = pkt/Raw(load=payload)
        sample_pkts.append(pkt)

    pkts = []
    for i, pkt in zip(range(nb_pkts), cycle(sample_pkts)):
        missing_bytes = pkt_size - len(pkt) - 4  # does not include CRC
        payload = str(i).zfill(missing_bytes)
        pkt = pkt/Raw(load=payload)
        pkts.append(pkt)

    wrpcap(out_pcap, pkts)


def main():
    if len(sys.argv) != 5:
        print('Usage:', sys.argv[0], 'nb_pkts output_pcap pkt_size nb_dest')
        sys.exit(1)

    nb_pkts = int(sys.argv[1])
    out_pcap = sys.argv[2]
    pkt_size = int(sys.argv[3])
    nb_dest = int(sys.argv[4])

    if nb_dest > 256:
        print('Can only support up to 256 destinations')

    generate_pcap(nb_pkts, out_pcap, pkt_size, nb_dest)


if __name__ == "__main__":
    main()

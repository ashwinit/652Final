#!/bin/bash

echo "Hello, this is a patch to the location based objective function." 
echo "Run the patch in the contiki folder, otherwise it will through an error!"

sudo cp uip6.c ../core/net/uip6.c
sudo cp rpl-dag.c ../core/net/rpl/rpl-dag.c
sudo cp collect-common.h ../examples/ipv6/rpl-collect/collect-common.h
sudo cp collect-common.c ../examples/ipv6/rpl-collect/collect-common.c
sudo cp udp-sink.c ../examples/ipv6/rpl-collect/udp-sink.c
sudo cp udp-sender.c ../examples/ipv6/rpl-collect/udp-sender.c

echo "Patch completed, follow the report or presentation for more details.."



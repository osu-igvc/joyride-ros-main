#!/bin/bash

interfaces=("nicEthernet1" "nicEthernet2" "nicEthernet3" "nicEthernet4" "nicEthernet5" "nicEthernet6" "nicEthernet7" "nicEthernet8")

for nic in ${interfaces[@]}; do
    echo "Checking interface: $nic" $'\n'

    sudo arp-scan --interface=$nic --localnet

    echo $'\n\n'

done

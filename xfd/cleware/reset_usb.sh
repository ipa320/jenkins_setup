#!/bin/bash
for i in $(ls /sys/bus/pci/drivers/ehci_hcd/|grep :)
 do echo $i >/sys/bus/pci/drivers/ehci_hcd/unbind
echo $i >/sys/bus/pci/drivers/ehci_hcd/bind
done

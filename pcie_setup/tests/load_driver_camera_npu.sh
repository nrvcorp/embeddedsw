#!/bin/bash

# Remove the existing xdma kernel module
lsmod | grep xdma_zcu106
if [ $? -eq 0 ]; then
   rmmod xdma_zcu106
fi

echo "====================================================================="
#################################
# device 0
#################################
# Make sure only root can run our script
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

# Remove the existing xdma kernel module
lsmod | grep xdma_zcu106
if [ $? -eq 0 ]; then
   rmmod xdma_zcu106
fi
echo -n "Loading xdma zcu106 driver..."
# Use the following command to Load the driver in the default 
# or interrupt drive mode. This will allow the driver to use 
# interrupts to signal when DMA transfers are completed.
insmod ../xdma_zcu106/xdma_zcu106.ko
# Use the following command to Load the driver in Polling
# mode rather than than interrupt mode. This will allow the
# driver to use polling to determ when DMA transfers are 
# completed.
#insmod ../xdma/xdma.ko poll_mode=1

if [ ! $? == 0 ]; then
  echo "Error: Kernel module did not load properly."
  echo " FAILED"
  exit 1
fi

# Check to see if the xdma devices were recognized
echo ""
cat /proc/devices | grep xdma_zcu106 > /dev/null
returnVal=$?
if [ $returnVal == 0 ]; then
  # Installed devices were recognized.
  echo "The Kernel module installed correctly and the xmda devices were recognized."
else
  # No devices were installed.
  echo "Error: The Kernel module installed correctly, but no devices were recognized."
  echo " FAILED"
  exit 1
fi
echo "====================================================================="
#################################
# device 1
#################################
# Make sure only root can run our script
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

# Remove the existing xdma kernel module
lsmod | grep xdma_dvs
if [ $? -eq 0 ]; then
   rmmod xdma_dvs
fi
echo -n "Loading xdma_dvs driver..."
# Use the following command to Load the driver in the default 
# or interrupt drive mode. This will allow the driver to use 
# interrupts to signal when DMA transfers are completed.
insmod ../xdma_dvs/xdma_dvs.ko
# Use the following command to Load the driver in Polling
# mode rather than than interrupt mode. This will allow the
# driver to use polling to determ when DMA transfers are 
# completed.
#insmod ../xdma1/xdma1.ko poll_mode=1

if [ ! $? == 0 ]; then
  echo "Error: Kernel module did not load properly."
  echo " FAILED"
  exit 1
fi

# Check to see if the xdma devices were recognized
echo ""
cat /proc/devices | grep xdma_dvs > /dev/null
returnVal=$?
if [ $returnVal == 0 ]; then
  # Installed devices were recognized.
  echo "The Kernel module installed correctly and the xmda devices were recognized."
else
  # No devices were installed.
  echo "Error: The Kernel module installed correctly, but no devices were recognized."
  echo " FAILED"
  exit 1
fi

echo " DONE"

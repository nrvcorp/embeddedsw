# DVS HOST CODE

******************************************************************************

**Vendor**                      : NRV\
**Current README Version**      : 1.0

**Date Last Modified**          : 10APR2024  
**Date Created**                : 10APR2024

**Supported Device(s)**         : XCZU7EV  
**Supported Board(s)**          : ZCU106 rev-c and above  

******************************************************************************

## This README file contains these sections

1. **OVERVIEW**  
2. **SOFTWARE TOOLS AND SYSTEM REQUIREMENTS**  
3. **DESIGN FILE HIERARCHY**  
4. **INSTALLATION AND OPERATING INSTRUCTIONS**  
5. **REVISION HISTORY**  

---

### 1. OVERVIEW

NRV DVS Streaming Host PC code

### 2. SOFTWARE TOOLS AND SYSTEM REQUIREMENTS

- C++

### 3. DESIGN FILE HIERARCHY

```
host/
├── app/                    # DVS host code application
|   ├── Stream              # Frame Receiving Application
|   |   ├── config.h
|   |   ├── dma_utils.c
|   |   ├── dma_utils.h
|   |   ├── main.cpp
|   |   └── Makefile
|   └── Stream_Parser       # Discontinued
|   |   ├── config.h
|   |   ├── dma_utils.c
|   |   ├── dma_utils.h
|   |   ├── main.cpp
|   |   └── Makefile
├── include/
|   └── libxdma_api.h
├── tests/                  # sample test scripts
|   ├── data/               # data for sample tests
|   |   ├── datafile_(8K~256K).bin
|   |   ├── datafile(0~3)_4K.bin
|   |   └── output_datafile(0~3)_4K.bin
|   ├── dma_memory_mapped_test.sh
|   ├── dma_streaming_test.sh
|   ├── load_driver.sh
|   ├── perform_hwcount.sh 
|   └── run_test.sh
├── tools/              # functions (main)
|   ├── dma_from_device.c
|   ├── dma_to_device.c
|   ├── dma_utils.c
|   ├── mem_polling.c   # memory polling test code
|   ├── NPU_host.c      # NPU host code (main)
|   ├── pcie_host_app.c
|   ├── performance.c
|   └── reg_rw.c        # (reg read write test)
└── xdma/               # Xilinx PCIe DMA kernel module
       driver files.
...
```

### 4. INSTALLATION AND OPERATING INSTRUCTIONS

#### 1. Check if PCIE driver is shown Host PC

go to `/dev` folder  
check if you can see `xdma0_c2h_(0~3)`, `xdma0_control`, etc  
if you can see those drivers, Host PC can connect to ZCU106 via PCIE

#### 2. PCIE init

If PCIE init script is not installed, you need to do the following steps to connect PCIE

```
cd xdma
make install
cd ../tools
make all
modprobe xdma
cd ../tests
./load_driver.sh
```

If you run the commands, it should successfully connect to the ZCU106 PCIE

**[PCIE Init script]**  
-> Ask for guide on Notion to automate PCIE connection
-> NEW: run ```host/pcie_install/pcie_install.sh``` to automatically run XDMA drivers for each reboot

#### 3. program ZCU106 boards using Vitis Application Project

1. Power both ZCU106 boards on. One should have the CIS and DVS sensors connected, and another should run the NPU. If you're running only CIS-DVS and no NPU, you must remove the NPU board from the PCIE, or program it too. 
2. Connect the NPU board's jtag and uart ports to this computer.
3. Open GTKterm and connect to /dev/ttyUSB0 or /dev/ttyUSB1.
4. Open Vitis on this computer(not Vitis HLS)
5. Select Workspace : /home/nrvfpga01/xsrc/NRV_demo
6. From the explorer, select NRV_npu_system(blue one)
7. Right click, and select Run as > Launch Hardware.
8. wait till the Programming bar shows up, and the GTKTerm shows successfuly ran Hello World Application
9. If it doesn't, open up USB port again on new GTKterm window, and reprogram.
10. Unplug jtag and uart ports, and transfer cables to CIS DVS sensor ZCU106 board.
11. From the Vitis explorer, select demo_v1_system(blue one)
12. Right click, and select Run as > Launch Hardware.
13. Wait till the Programming bar shows up, and the GTKTerm shows 
Is the Camera Sensor Connected?(Y/N) or something similar.
14. Reboot this pc.
15. Open up GTKTerm, and connect to /dev/ttyUSB0 or /dev/ttyUSB1.
16. Blindly press Y, until some window outputs text from the CIS-DVS board.
17. On the terminal, run ls /dev to make sure xdma_dvs0 and xdma_zcu1060 are alive.
18. If 17 isn't working, Run lsmod | grep xdma to make sure xdma_dvs and xdma_zcu106 drivers are installed.
19. If 18 isn't working, then run lspci and show if xilinx devices show up.
20. If 19 isn't working, then try to completely turn off this PC.
21. No powered off boards should be inserted to PCIE.
22. while this PC is turned completely off, connect NPU board with an external computer. 
23. Try to run steps 2~13 with that external computer. Afterwards, turn on this PC, then press y on the terminal connected to the CIS DVS board. 
24. Yep! if you have no problems, continue onwards!

#### 4. Run the Host program

Run the following commands to execture the `PROGRAM` (`PROGRAM`: intended program ex. `./mem_polling`)

```
cd tools
make clean
make all
./PROGRAM
```

### 4. APPLICATION OPERATION

#### 1. Stream

Application working on ***MIPI2PCIE_AXIS*** system project with ***MIPI2PCIE_AXIS*** firmware code

```
cd app/Stream
make all
./main
```

#### 2. CIS_DVS

. open the terminal.

2. sudo -i

3. enter password

4. code --no-sandbox --user-data-dir /home/nrvfpga01/src/vscode-root/ (change directory to your custom vscode-root settings directory)

5. vscode root window will pop up, open terminal using Ctrl + ` and select git bash

6. cd to CIS_DVS directory

7. make

8. run ./main with the following options : 

-c : cis streaming mode
-d : dvs streaming mode
-x : dvs frame skip error checker. CURRENTLY ERROR PRONE, DON'T RUN!!
I don't know why this bugs up...
-s : cis-dvs concurrent streaming mode
-w : write dvs raw data to ./bin_files, the directory CIS_DVS/bin_files must exist
-r : dvs roi mode, displays bounding box on top of dvs streaming mode
-b : CIS bbox mode, displays bounding box on DVS and CIS streaming windows. YOU SHOULD CALIBRATE USING ./main -o BEFOREHAND!!!!!
-o : CIS DVS calibration mode. shows red grid in order to calibrate between CIS and DVS viewpoint. You should modify config.hpp according to console messages.
-f : prints DVS fps to the console, no error checking functionality.
-p : displays CIS, DVS video streams, displays their FPS. Also runs error checks. Since DVS is downsampled, the screen might be laggy or shaded less.

9. to modify parameters, open src/config.hpp


- ROI_INFLATION : increase ROI size to center and zoom out actual ROI area
- DVS_ROI_MIN_SIZE : minimum ROI size of DVS roi mode
- CIS_ROI_MIN_SIZE : minimum ROI size of CIS bbox mode

You might want to set these by hand, using ./main -o to find out the corners of DVS view frame on the CIS opencv video...

- CIS_DVS_OFFSET_X : The x position where the DVS view frame starts relative to the CIS frame width
- CIS_DVS_OFFSET_Y : The y position where the DVS view frame starts relative to the CIS frame height
- CIS_DVS_SCALE_X : The width of the DVS view frame relative to the CIS frame width
- CIS_DVS_SCALE_Y : The height of the DVS view frame relative to the DVS frame width

Also, make sure to run ls /dev to check the name of the current XDMA driver.
They should match the value of the following :

- H2C_DEVICE_DVS
- C2H_DEVICE_DVS
- H2C_DEVICE_CIS
- C2H_DEVICE_CIS

10. after you finish src/config.hpp, cd to CIS_DVS directory

11. make clean

12. make

13. run ./main again 

### 6. REVISION HISTORY

#### 1.x

- **10APR2024**: Initial NRV template

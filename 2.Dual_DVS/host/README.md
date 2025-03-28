##Steps to run host code for DUAL_DVS##

1. open the terminal.

2. sudo -i

3. enter password

4. code --no-sandbox --user-data-dir /home/nrvfpga01/src/vscode-root/ (change directory to your custom vscode-root settings directory)

5. vscode root window will pop up, open terminal using Ctrl + ` and select git bash

6. cd to host/app/CIS_DVS_store_t directory

7. make

8. run ./main with the following options : 

-c : cis streaming mode
-d : dvs streaming mode
-x : dvs frame skip error checker 
-s : cis-dvs concurrent streaming mode
-w : write dvs raw data to ./bin_files, the directory must exist
-r : dvs roi mode, displays bounding box on top of dvs streaming mode
-b : CIS bbox mode, displays bounding box on CIS streaming window inferred from DVS.

9. to modify parameters, open src/config.hpp

- ROI_THRESH : set high to reduce ROI detection noise
- ROI_INFLATION : increase ROI size to center and zoom out actual ROI area
- DVS_ROI_MIN_SIZE : minimum ROI size of DVS roi mode
- CIS_ROI_MIN_SIZE : minimum ROI size of CIS bbox mode

You might want to set these by hand, using ./main -s to find out the corners of DVS view frame on the CIS opencv video...

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

10. after you finish src/config.hpp, cd to CIS_DVS_store_t directory

11. make clean

12. make

13. run ./main again 
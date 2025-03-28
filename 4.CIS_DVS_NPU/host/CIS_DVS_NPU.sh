cd /home/cappfpga01/src/2.mccha/mipi_controller/host/app/CIS_DVS
./main &

cd /home/cappfpga01/src/2.mccha/npu/host/camera_demo_thread
bash tiny-yolov3-voc.sh &

wait

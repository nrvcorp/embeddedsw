savedcmd_xdma_dvs.ko := ld -r  -m elf_x86_64 -z noexecstack   --build-id=sha1  -T /usr/src/linux/linux-6.16.1/scripts/module.lds -o xdma_dvs.ko xdma_dvs.o xdma_dvs.mod.o .module-common.o

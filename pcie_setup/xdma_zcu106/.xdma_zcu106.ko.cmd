savedcmd_xdma_zcu106.ko := ld -r  -m elf_x86_64 -z noexecstack   --build-id=sha1  -T /usr/src/linux/linux-6.16.1/scripts/module.lds -o xdma_zcu106.ko xdma_zcu106.o xdma_zcu106.mod.o .module-common.o

savedcmd_xdma_dvs.o := ld  -m elf_x86_64 -z noexecstack    -r -o xdma_dvs.o @xdma_dvs.mod  ; /usr/src/linux/linux-6.16.1/tools/objtool/objtool --hacks=jump_label --hacks=noinstr --hacks=skylake --ibt --retpoline --rethunk --stackval --static-call --uaccess --prefix=16  --link  --module xdma_dvs.o

xdma_dvs.o: $(wildcard /usr/src/linux/linux-6.16.1/tools/objtool/objtool)

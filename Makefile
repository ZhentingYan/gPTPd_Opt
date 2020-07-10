CC=$(CROSS_COMPILE)arm-linux-gnueabihf-gcc

x86: sync.c gptpcmn.c sync.c bmc.c log.c delaymsr.c gptp.c
	$(CC) -g -o gptpdx86 -DGPTPD_BUILD_X_86 log.c gptpcmn.c bmc.c sync.c delaymsr.c gptp.c

bbb: gptpcmn.c bmc.c sync.c log.c delaymsr.c gptp.c
	$(CC) -g -o gptpd log.c gptpcmn.c bmc.c sync.c delaymsr.c gptp.c

.PHONY: clean

clean:
	rm -f *.o gptpdx86 gptpd
	

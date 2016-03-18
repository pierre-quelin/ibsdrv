KVERSION := $(shell uname -r)

obj-m = ibs.o
ibs-objs:=phoenix_ibs_pci.o

all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
install:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules_install
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
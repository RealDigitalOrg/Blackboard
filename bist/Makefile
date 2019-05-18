
all:
	vivado -mode batch -source tcl/create_project.tcl
	xsdk -batch -source tcl/xsdk_sw.tcl

SDK_FILES := $(shell ls -A bist_proj/bist.sdk/ -I system_wrapper.hdf) 

test:
	@echo $(SDK_FILES)
	@cd bist_proj/bist.sdk && rm -rfv $(SDK_FILES)

sdk:
	xsdk -batch -source tcl/xsdk_sw.tcl

sdk_clean:
	@cd bist_proj/bist.sdk && rm -rfv !("system_wrapper.hdf")

clean:
	rm -rf .Xil
	rm -f *.log
	rm -f *.jou
	rm -rf bist_proj

all:
	vivado -mode batch -source tcl/create_project.tcl

test:
	@echo $(SDK_FILES)
	@cd bist_proj/bist.sdk && rm -rfv $(SDK_FILES)

sdk_clean:
	@cd bist_proj/bist.sdk && rm -rfv !("system_wrapper.hdf")

clean:
	rm -rf .Xil
	rm -f *.log
	rm -f *.jou
	rm -rf bist_proj

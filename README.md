# Blackboard

Blackboard is a highly configurable digital design platform featuring Xilinx
Zynq-7000 APSoC, designed specifically for teaching and learning hardware and
software system development.

## Compile BIST Design

BIST reference design is flashed into the SPI flash during manufacturing and
shipped with the Blackboard as a foundation of software development and an 
example to hardware development.

To compile the BIST, browse to a directory where you want to create the BIST
project.

```bash
$ vivado -source bist/tcl/create_project.tcl --project_name bist --project_dir ./bist_proj
```

The software part can be generated using the following script.

```bash
$ xsdk -source bist/tcl/xsdk_sw.tcl --project_name bist --project_dir ./bist_proj
```

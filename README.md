# RTL Simulation

### File description:
- input_gen: convert .pcap file to .pkt, which is the input for RTL simulation.
- src: RTL source code.
- src/common: common building block generated by Quartus IP, like FIFOs, BRAMs, Mux.
- expected_res.txt: expected RTL simulation output for Dummy test.
- modelsim.ini: specify the path to Quartus compiled sim_lib. 
- run_vsim_afs.sh: run RTL simulation. 

### Steps:
1. Download the sim_lib, unzip it in AFS domain. Change the path in modelsim.ini to link the downloaded sim_lib. 
2. Make sure the Modelsim can be invoked from AFS. 
3. Go to input_gen, generate the input for RTL simulation. And then run the simulation. The printout should match the expected_res.txt if you don't change anything
```
cd input_gen
./run.sh m10_100.pcap
mv output.pkt m10_100.pkt
cd ..
./run_vsim_afs.sh
```
### SRC description:
- my_struct_s.sv is the header file that specify most of the parameters and structs. 
- tb.sv is the testbench. You should specify new .pkt file in this file and number of line of new .pkt.
```
localparam hi = X; //X is number of lines in new .pkt file
...
$readmemh("./input_gen/m10_100.pkt", arr, lo, hi); //change to new .pkt file
```
  - top.sv is the top level module of core logic. Various counters are used to monitor the correctness of the datapath.
    - input_comp.sv: Store input pkt to Global Pkt Buffer, update emptylist, extract header to Parser
    - parser.sv: Take the header flit, fill in the metadata fields, pass the metadata to Flow Director
    - flow_director_wrapper.sv: Wrapper file for flow_director. Add FIFOs and Register I/O layer for Design Partition.
      - flow_director.sv: Dummy flow director that directly passes the signals. **This is the file you need to modify first**. 
    - basic_data_mover.sv: Take the metadata from Flow Director and then fetch pkt from Global Pkt Buffer, then (1) forward the pkt to Ethernet output, or (2) drop pkt, or (3) send to PCIe, depending on the pkt_flag field.
    - pdu_gen.sv: Take the metadata from basic_data_mover and pkts data to form a block for PCIe transmission. 
  - esram_wrapper.sv: Global Pkt Buffer. In simulation, it is a BRAM to speed up simulation. During Synthesis, it is mapped to eSRAM.   
  - pcie_top.sv : PCIe top-level. Dummy module when `SIM` is defined. 
    - fpga2cpu_pcie.sv : Handle the FPGA->CPU transfer. FPGA side pushes data to ring buffer. The other side of the ring buffer will fetch data in batch and starts DMA to CPU.
      - ring_buffer.sv: The ring_buffer at FPGA side. 
  - my_stats.sv: Performance monitor to monitor the throughput of RX and TX of Ethernet core. Also able to track the latency spent of our design.  

### Note:
1. \*.ini, \*.pcap, \*.pkt are ingored in .gitignore. You should change your local .ini file and you can add new pcap/pkt in your local copy. But please do not upload to this repo.
2. In input_gen, you can change the run.sh to capture different number of pkts from the pcap. Usually start with small number.
3. run_vsim_afs.sh allow you choose GUI mode, CLI mode, and optimized CLI mode. GUI mode is great for debugging using waveform. CLI mode is good for getting the results quickly. Optimized CLI applied internal optimizations which may affect the results.
4. The `define SIM` and `define NO_PCIE` should be commented during Synthesis. In simulation, you can toggle `define NO_PCIE` to disable/enable pcie. In Synthesis, you will need to write a tcl command in JTAG system console.

# Hardware test
### Resotre the Quartus project
1. Scp the exmaple project from scotchbuild00 /home/zzhao1/front_door_consumer.qar to your compile machine. 
2. Open Quartus prime pro 19.3. Under "Project" Tab, select Restore Archived Project. Restore it to desired path. Assuming the top-level folder after resotre is called `front_door_consumer`.

### Quartus Project file description (Just introduce most related files)
- front_door_consumer (top-level folder)
  - ex_100G: folders that contains the 100Gbps Ethernet Core logic.
  - ex_100G.ip: the ip file for Ethernet Core
  - hardware_test_design (folder that we care most)
    - src: the src RTL code from RTL_sim. Note again the `define SIM` and `define NO_PCIE` should be commented in my_struct_s.sv. 
    - output_files: the bitstream file `alt_ehipc2_hw.sof` and some reports which can be viewed in Quartus.  

### Synthesize Quartus Project (you can skip this the first time)
1. Open the quartus project under `your_path/front_door_consumer/hardware_test_design/alt_ehipc2_hw.qpf`
2. Add new flow_director.sv (keep the interface the same.) You can add sub modules for flow_director.sv as well.
3. Copy the `alt_ehipc2_hw.sv` (system top level systemverilog code), `alt_ehipc2_hw.sdc`(timing constraints) and `alt_ehipc2_hw.qsf`(quartus project setting, like including which files, compile strategies) to the `your_path/front_door_consumer/hardware_test_design/`. 
4. Open the Compilation Dashboard. Click Compile Design. It may take 20 minutes if you don't change anything. This involves multiple stages. In the end, the Assembler will generate bitstream. 

### Load bitstream and setup System console
1. Download `hardware_test` folder to scotchbuild00 `your_scotchbuild_path`
2. Copy the sof file from you compile machine (`your_path/front_door_consumer/hardware_test_design/output_files/alt_ehipc2_hw.sof`) to scotchbuild00 `your_scotchbuild_path/hardware_test/`. 
3. Change `your_path` in load.cdf and path.tcl to match `your_soctchbuild_path`.
3. Run `./load_bitstream.sh` to load the bitstream. Note that the JTAG system console should be closed when loading the bitstream.

### Test Steps with PCIe
1. Reboot the machine after loading the bitstream if you need PCIe. 
2. Run JTAG system console by running `./run_console`, it may return some error the first time. Exit it using Ctrl-C. Then redo it. It would work the second time.
3. Load PCIe user application file. Look for "Commit for Front Door" in Snort3-pigasus Repo. That is the commit for front door. Eventually, it makes sense to remove all the Snort part for Front Door project.
4. Install the driver.
5. Start the user application. 
6. In the tcl console, you will need to type some tcl commands.
```
##find the main.tcl under your_path/hardware_test/hwtest
source path.tcl 
##return the counters under top.sv. Most of them should be 0, except the in_emptylist_cnt is 2688, the number of pkts we can buffer.
get_top_stats 
##return the PCIe stats
read_pcie
```
7. Then connect to pkt-gen machine (192.168.1.3). Send pkt
```
cp example.pcap /dev/shm/test.pcap
cd dpdk/pktgen-dpdk/
./run_pktgen.sh
```
Remember to set the number of pkt of the pcap. Otherwise the pktgen would send the pcap repeatedly.
```
set 0 count X
str
```
8. Recheck the counters on FPGA
```
## recheck the top counters, you should expect to see X in many of the counters; eth related counters should be 0
get_top_stats 
```
9. Exit PCIe user application. You should expect to see the rx_pkt is X. 

### Test Steps without PCIe
1. Without PCIe, the pkts will be forwarded to Ethernet output. After loading the bitstream. No need to reboot the machine.
2. Run JTAG system console by running `./run_console`, it may return some error the first time. Exit it using Ctrl-C. Then redo it. It would work the second time.
3. In the tcl console, you will need to type some tcl commands.
```
##find the main.tcl under your_path/hardware_test/hwtest
source path.tcl 
##return the counters under top.sv. Most of them should be 0, except the in_emptylist_cnt is 2688, the number of pkts we can buffer.
get_top_stats 
##disable_pcie
disable_pcie
```
4. Then connect to pkt-gen machine (192.168.1.3). Send pkt
```
cp example.pcap /dev/shm/test.pcap
cd dpdk/pktgen-dpdk/
./run_pktgen.sh
```
Remember to set the number of pkt of the pcap. Otherwise the pktgen would send the pcap repeatedly.
```
set 0 count X
str
```
5. Recheck the counters on FPGA
```
## recheck the top counters, you should expect to see X in many of the counters; PCIe related counters should be 0
get_top_stats 
```

### Quartus Project Version Control
Assuming the top-level folder after resotre is called `front_door_consumer`. To test a new quartus project, I do the version control manually. 
1. Copy `front_door_consumer` and then rename it, assuming `front_door_consumer_new`. 
2. Then replace the front_door_consumer_new/hardware_test_design/src with the new src code. 
3. Open quartus project in `front_door_consumer_new`. Redo the synthesis. 
4. If you are satisfied with `front_door_consumer_new`, zip it and then upload to Google Drive or other places you choose to back up. 
5. I have a separate README to keep track of each zipped quartus project is doing. The benefit of doing this instead of keeping editing one quartus project is that, you can easily go back and check the report and load the bitstream of a specific checkpoint. 

# Extend dummy design
All the related code are put under `useful_code_from_pigasus`. 
### Use Flow table:
Check out `flow_table_wrapper` and `para_Q`. Ignore the status and reassembly part. To start with (assuming no eviction), the input for para_Q should be connected to CPU. 
### Use CPU-FPGA path:
Check out the `pcie_top` and `cpu2fpga_pcie`. You can directly forward the `c2f_write` and `c2f_writedata` in `cpu2fpga_pcie` to flow director through `pcie_top`. Basically, replace the `pdumeta_cpu_data`. Then you should define what does this 512 bit mean and follow the contract in both hardware and software side. 
Note that all PCIe part will be ignored during RTL simulation. 
The PCIe user side application code needs uncomment the `cpu2fpga path`.


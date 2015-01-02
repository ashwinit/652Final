# README #

### Location Based Query & Data Aggregation Model ###
Routing is based on the location of the nodes not on the ETX. 
The model is implemented over ContikiOS RPL with hierarchy based 
location address

### Source Codes ###
* collect-common.h - support file to sink and sender
* collect-common.c - support file to sink and sender
* udp-sink.c - sink code 
* udp-sender.c - sender code
* rpl-dag.c - best and preferred parent selection is modified
* uip6.c - forwarding packet is taken care
* patch.sh - run the patch in the contiki folder to run the model

To run and execute the above model, do the following:
1. Run the patch in the contiki folder
2. change the #define SCENARIO in the collect-common.h to 1 or 0 based on 
	your application requirement
3. If you intend to use more number of nodes, increase the #define NUM_NODES, default is 12
4. Also the nodes are assumend to be hierarchy and continuous in addressing
	thus make sure you change the #define FIRST_NODEID 
5. Run cooja and follow the report (IEEE paper submission) or presentation.

#### Authors ###

* Ashwini Telang - ashwinit@usc.edu
* Subhashini Sundaresan - subhashi@usc.edu
* Yash Goyal - ygoyal@usc.edu

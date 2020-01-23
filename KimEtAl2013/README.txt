This is the readme for the model associated with the paper:

Kim D, Pare D, Nair SS (2013) Mechanisms contributing to the induction
and storage of pavlovian fear memories in the lateral amygdala,
Learning & Memory

This is the code that the authors used. It is written in NEURON which
is freely available at http://www.neuron.yale.edu

DATA INPUT FILES

1) Cell_list.txt: List of cells whose weight changes and Ca+
concentration will be saved.
2) Cell_type.txt: This file indicates the type of cell for the
principal cells. The types are defined in the main file. We have this
predetermined to maintain the proper distributions of cell-types (Type
A, TypeB etc).
3) NM.txt: This file indicates whether the cell has any neuromodulator
receptors present. It can be of three types, DA, NE and DANE.
4) tone2LAdd.txt: List of randomly chosen principal cell numbers in
LAdd for thalamic tone input.
5) tone2LAdd2.txt: List of randomly chosen principal cell numbers in
LAdd for cortical tone input.
6) tone2LAdv.txt: List of randomly chosen principal cell numbers in
LAdv for cortical tone input.
7) tone2LAdv2.txt: List of randomly chosen principal cell numbers in
LAdv for thalamic tone input.
8) shock2LAdd.txt: List of randomly chosen principal cell numbers in
LAdd for shock input.
9) shock2LAdv.txt: List of randomly chosen principal cell numbers in
LAdv for shock input.
10) tone2Idd.txt, tone2Idd2.txt, tone2Idv.txt, tone2Idv2.txt, etc.:
Same as above, but for interneurons
11) Syn_Matrix.txt: Synapse matrix for connections.
________________________________________

NEURON FILES

1. BgGen.hoc : File for generating background inputs.
2. function_ConnectInternal.hoc: Loads internal connectivity from
Syn_Matrix.txt and makes appropriate connections.
3. function_ConnectTwoCells.hoc: Connects two cells.
4. function_NetStimOR.hoc: Netstim template for tone and shock inputs.
5. function_TimeMonitor: indicates how much of the simulation has
completed
6. function_ToneGen.hoc: tone protocol
7. function_ToneSignalGen_Th.hoc: protocol for thalamic tone to LA
8. function_ToneSignalGen_Ctx.hoc: protocol for cortical tone to LA
9. interneuron_template.hoc: Template file for interneuron
10. LA_model_main_file: The main model file
11. LAcell_template.hoc: Template file for different LA principal cell
types.
12. shockcondi.hoc: File for generating shock protocol

NOTE: These .hoc files call several .mod files (current channels,
synapses and other functions) that are provided in the folder.

___________________________________________


OUTPUT FILES:

1) data: saves spiking time history of all cells
2) Matrix_NEW: saves internal connections with cell ids and synapse
ids for checking.
3) XXX_ca: Calcium conc. of particular synapse (with pre- and post-
cell IDs in XXX)
4) XXX_wt: weight change of particular synapse (with pre- and post-
cell IDs in XXX)

___________________________

BEFORE YOU RUN:

In the file (LA_model_main_file.hoc) set the path to save the data as
/home/yourdirectory/output for saving data correctly.

Also make sure to make an empty folder named 'output' in the directory
that has all the files. NEURON will save the output files into this
directory.

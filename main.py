"""
Created on Sun Jan 19 01:11:30 2020
"""

from netpyne import specs, sim


arq = 'blamy/BLcells_template_LFP_segconsider_all_Iinject_recordingimembrane.hoc'

# Network parameters
netParams = specs.NetParams() # object of class NetParams to store the network parameters
## Population parameter
netParams.popParams['A_pop'] = {'cellType': 'A', 'numCells': 5, 'cellModel': 'HH'}
netParams.popParams['ADA_pop'] = {'cellType': 'ADA', 'numCells': 5, 'cellModel': 'HH'}


### Import A 
netParams.importCellParams(label='A_rule', conds={'cellType': 'A', 'cellModel': 'HH'},	fileName=arq, cellName='Cell_A', importSynMechs=False)
### Import ADA 
netParams.importCellParams(label='ADA_rule', conds={'cellType': 'ADA', 'cellModel': 'HH'},	fileName=arq, cellName='Cell_ADA', importSynMechs=False)


## Synaptic mechanism 
netParams.synMechParams['AMPA'] = {'mod': 'Exp2Syn', 'tau1': 1.0, 'tau2': 5.0, 'e': 0} # soma NMDA synapse 
# Stimulation 
netParams.stimSourceParams['bkg'] = {'type': 'NetStim', 'rate': 50, 'noise': 0.5}
netParams.stimTargetParams['bkg->A'] = {'source': 'bkg', 'conds': {'cellType': 'A'}, 'weight': 0.05, 'delay': 5, 'sec': 'soma'}
## Connectivity params		
netParams.connParams['A->ADA'] = {	'preConds': {'cellType': 'A'}, 'postConds': {'cellType': 'ADA'}, # A -> ADA random	
                    'probability': 0.5, 			# max number of incoming conns to cell	'
                    'weight': 0.02, 			# synaptic weight 	
                    'delay': 5,					# transmission delay (ms) 	'
                    'sec': 'soma', 'plasticity': {'mech': 'STDP', 'params': {'hebbwt': 0.01, 'antiwt':-0.01, 'wmax': 50, 'RLon': 1, 'tauhebb': 10}}} 
# section to connect to# Simulation options
simConfig = specs.SimConfig()					# object of class SimConfig to store simulation configuration
simConfig.duration = 1*1e3 # Duration of the simulation, in ms
simConfig.dt = 0.025  # Internal integration timestep to use
simConfig.verbose = 0			
# Show detailed messages 
simConfig.recordTraces = {'V_soma':{'sec':'soma','loc':0.5,'var':'v'}} 
# Dict with traces to record
simConfig.recordStep = 1 			
# Step size in ms to save data (eg. V traces, LFP, etc)
simConfig.filename = 'model_output' 
# Set file output names
simConfig.saveJson = True 		
# Save params, network and sim output to pickle file
simConfig.analysis['plotRaster'] = {'orderInverse': True, 'saveFig': 'tut_import_raster.png'}			
# Plot a raster
simConfig.analysis['plotTraces'] = {'include': [0,7]} 			
# Plot recorded traces for this list of cells
# Create network and run simulation
sim.createSimulateAnalyze(netParams = netParams, simConfig = simConfig) 


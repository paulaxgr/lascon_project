"""
Created on Sun Jan 19 01:11:30 2020
"""

from netpyne import specs, sim
# from neuron import h

tstop = 1000.0

# arq = 'BLcells_template_LFP_segconsider_all_Iinject_recordingimembrane.hoc'
arq = './KimEtAl2013/LAcells_template.hoc'
arq2 = './KimEtAl2013/interneuron_template.hoc'

# Network parameters
netParams = specs.NetParams()
# object of class NetParams to store the network parameters
## Population parameter
netParams.popParams['A_pop'] = {'cellType': 'A', 'numCells': 1, 'cellModel': 'HH'}
netParams.popParams['B_pop'] = {'cellType': 'B', 'numCells': 1, 'cellModel': 'HH'}
netParams.popParams['C_pop'] = {'cellType': 'C', 'numCells': 1, 'cellModel': 'HH'}
netParams.popParams['IN_pop'] = {'cellType': 'Interneuron', 'numCells': 1, 'cellModel': 'HH'}

### Import A
netParams.importCellParams(label='A_rule', conds={'cellType': 'A', 'cellModel': 'HH'},	fileName=arq, cellName='Cell_A', importSynMechs=True)
### Import B
netParams.importCellParams(label='B_rule', conds={'cellType': 'B', 'cellModel': 'HH'},	fileName=arq, cellName='Cell_B', importSynMechs=True)
## Import C
netParams.importCellParams(label='C_rule', conds={'cellType': 'C', 'cellModel': 'HH'},	fileName=arq, cellName='Cell_C', importSynMechs=True)
## Import interneurons
netParams.importCellParams(label='IN_rule', conds={'cellType': 'Interneuron', 'cellModel': 'HH'}, fileName=arq2, cellName='InterneuronCell', importSynMechs=True)

## Synaptic mechanism
netParams.synMechParams['AMPA'] = {'mod': 'Exp2Syn', 'tau1': 1.0, 'tau2': 5.0, 'e': 0} # soma NMDA synapse
# Stimulation
#netParams.stimSourceParams['bkg'] = {'type': 'NetStim', 'rate': 20, 'noise': 0.5}
netParams.stimSourceParams['bkg'] = {'type': 'IClamp', 'amp': 0.4, 'dur': 600 }
netParams.stimTargetParams['bkg->A'] = {'source': 'bkg', 'conds': {'cellType': 'A'}, 'sec': 'soma', 'loc':0.5} #'weight': 0.5,'delay': 5,
netParams.stimTargetParams['bkg->B'] = {'source': 'bkg', 'conds': {'cellType': 'B'}, 'sec': 'soma', 'loc': 0.5}
netParams.stimTargetParams['bkg->C'] = {'source': 'bkg', 'conds': {'cellType': 'C'}, 'sec': 'soma', 'loc':0.5}
netParams.stimTargetParams['bkg->Interneuron'] = {'source': 'bkg', 'conds': {'cellType': 'Interneuron'}, 'sec': 'soma', 'loc'  :0.5}

'''## Connectivity params
netParams.connParams['A->ADA'] = {	'preConds': {'cellType': 'Cell_A'}, 'postConds': {'cellType': 'Cell_ADA'}, # A -> ADA random
                    'probability': 0.5, 			# max number of incoming conns to cell	'
                    'weight': 0.02, 			# synaptic weight
                    'delay': 5,					# transmission delay (ms) 	'
                    'sec': 'soma', 'plasticity': {'mech': 'STDP', 'params': {'hebbwt': 0.01, 'antiwt':-0.01, 'wmax': 50, 'RLon': 1, 'tauhebb': 10}}}'''
# section to connect to# Simulation options
simConfig = specs.SimConfig()					# object of class SimConfig to store simulation configuration
simConfig.duration = tstop#1*1e3 # Duration of the simulation, in ms
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
simConfig.analysis['plotTraces'] = {'include': [0,1,2,3]}
# Plot recorded traces for this list of cells
# Create network and run simulation
sim.createSimulateAnalyze(netParams = netParams, simConfig = simConfig)


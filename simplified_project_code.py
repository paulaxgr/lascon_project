from netpyne import specs, sim
import matplotlib.pyplot as plt
import numpy as np

tstop = 200.0

arq = './KimEtAl2013/LAcells_template.hoc'
arq2 = './KimEtAl2013/interneuron_template.hoc'

# Network parameters
netParams = specs.NetParams() # object of class NetParams to store the network parameters
# netParams.sizeX = 200 # x-dimension (horizontal length) size in um
# netParams.sizeY = 1000 # y-dimension (vertical height or cortical depth) size in um
# netParams.sizeZ = 200 # z-dimension (horizontal length) size in um
# netParams.propVelocity = 100.0 # propagation velocity (um/ms)
# netParams.shape = 'cylindrer'
#netParams.probLengthConst = 150.0 # length constant for conn probability (um)

## Population parameter

n_neurons = 100 #total number of neurons
netParams.popParams['LAddA'] = {'cellType': 'A', 'numCells': int((n_neurons*0.8)*0.5/2.0), 'cellModel': 'HH'}
netParams.popParams['LAddB'] = {'cellType': 'B', 'numCells': int((n_neurons*0.8)*0.3/2.0), 'cellModel': 'HH'}
netParams.popParams['LAddC'] = {'cellType': 'C', 'numCells': int((n_neurons*0.8)*0.2/2), 'cellModel': 'HH'}

netParams.popParams['LAdvA'] = {'cellType': 'A', 'numCells': int((n_neurons*0.8)*0.5/2.0), 'cellModel': 'HH'}
netParams.popParams['LAdvB'] = {'cellType': 'B', 'numCells': int((n_neurons*0.8)*0.3/2.0), 'cellModel': 'HH'}
netParams.popParams['LAdvC'] = {'cellType': 'C', 'numCells': int((n_neurons*0.8)*0.2/2.0), 'cellModel': 'HH'}

netParams.popParams['IN'] = {'cellType': 'Interneuron', 'numCells': int(n_neurons*0.2), 'cellModel': 'HH'}

### Import A
netParams.importCellParams(label='LAddA_rule', conds={'cellType': 'A', 'cellModel': 'HH'},
                           fileName=arq, cellName='Cell_A', importSynMechs=True)
### Import B
netParams.importCellParams(label='LAddB_rule', conds={'cellType': 'B', 'cellModel': 'HH'},
                           fileName=arq, cellName='Cell_B', importSynMechs=True)
## Import C
netParams.importCellParams(label='LAddC_rule', conds={'cellType': 'C', 'cellModel': 'HH'},
                           fileName=arq, cellName='Cell_C', importSynMechs=True)

### Import A
netParams.importCellParams(label='LAdvA_rule', conds={'cellType': 'A', 'cellModel': 'HH'},
                           fileName=arq, cellName='Cell_A', importSynMechs=True)
### Import B
netParams.importCellParams(label='LAdvB_rule', conds={'cellType': 'B', 'cellModel': 'HH'},
                           fileName=arq, cellName='Cell_B', importSynMechs=True)
## Import C
netParams.importCellParams(label='LAdvC_rule', conds={'cellType': 'C', 'cellModel': 'HH'},
                           fileName=arq, cellName='Cell_C', importSynMechs=True)


## Import interneurons
netParams.importCellParams(label='IN_rule', conds={'cellType': 'Interneuron', 'cellModel': 'HH'},
                           fileName=arq2, cellName='InterneuronCell', importSynMechs=True)

## Synaptic mechanism
netParams.synMechParams['AMPA'] = {'mod': 'Exp2Syn', 'tau1': 0.25, 'tau2': 7.0, 'e': 0}  # AMPA synaptic mechanism
netParams.synMechParams['NMDA'] = {'mod': 'Exp2Syn', 'tau1': 3.65, 'tau2': 125.0, 'e': 0}  # AMPA synaptic mechanism
netParams.synMechParams['GABA'] = {'mod': 'Exp2Syn', 'tau1': 0.13, 'tau2': 3.75, 'e': -75}  # GABA synaptic mechanism

## Synaptic weights
w_e = 0.2e-2       #excitatory synaptic strength (KimEtAl2013: gAMPA=1nS; gNMDA=0.5nS; gGABA=0.6nS)
w_i = -w_e*0.6    #inhibitory synaptic strength

w_bkg = w_e     #excitatory synaptic weight from bkg->populations

w_pp = w_e
w_ip = w_i
w_pi = w_e
w_ii = w_i

## Stimulation
netParams.stimSourceParams['bkg'] = {'type': 'NetStim', 'rate': 70, 'noise': 1}

netParams.stimTargetParams['bkg->LAddA'] = {'source': 'bkg', 'conds': {'pop': 'LAddA'}, 'weight': w_bkg, 'sec': 'soma'}
netParams.stimTargetParams['bkg->LAddB'] = {'source': 'bkg', 'conds': {'pop': 'LAddB'},'weight': w_bkg, 'sec': 'soma'}
netParams.stimTargetParams['bkg->LAddC'] = {'source': 'bkg', 'conds': {'pop': 'LAddC'}, 'weight': w_bkg, 'sec': 'soma'}
netParams.stimTargetParams['bkg->LAdvA'] = {'source': 'bkg', 'conds': {'pop': 'LAdvA'}, 'weight': w_bkg, 'sec': 'soma'}
netParams.stimTargetParams['bkg->LAdvB'] = {'source': 'bkg', 'conds': {'pop': 'LAdvB'},'weight': w_bkg, 'sec': 'soma'}
netParams.stimTargetParams['bkg->LAdvC'] = {'source': 'bkg', 'conds': {'pop': 'LAdvC'}, 'weight': w_bkg, 'sec': 'soma'}

netParams.stimTargetParams['bkg->Interneuron'] = {'source': 'bkg', 'conds': {'pop': 'IN'},'weight': w_bkg,'sec': 'soma'}

## Pulses
tp_start = np.arange(50,tstop,200)
tp_end = tp_start + 50
p_rate = 300.0 # in Hz
pulses=[]
for i in range(len(tp_start)):
	pulses.append({'start':tp_start[i], 'end':tp_end[i], 'rate':p_rate, 'noise':1.0})

# create custom list of spike times
spkTimes = [0.0]
netParams.popParams['pulse'] = {'cellModel': 'VecStim', 'numCells': 1, 'spkTimes': spkTimes, 'pulses': pulses}  # VecStim with spike times

## Connect pulses ->
netParams.connParams['pulse->S'] = { 	#  pulse -> S label
 	'preConds': {'pop': 'pulse'}, 	# conditions of presyn cells
 	'postConds': {'pop': ['LAddA', 'LAddB', 'LAddC', 'IN']}, # conditions of postsyn cells
 	'weight': w_pp, 				# synaptic weight
 	'delay': 1.0,						# transmission delay (ms)
 	'synMech': 'AMPA'}   			# synaptic mechanism

sec = 'dend'
#plast = {'mech': 'STDP', 'params': {'hebbwt': 0.01, 'antiwt':-0.01, 'wmax': 50, 'RLon': 1, 'tauhebb': 10}}
plast = ''

p = 0.2
netParams.connParams['LAddA->all_d'] = {'preConds': {'pop': 'LAddA'}, 'postConds': {'pop': ['LAddB','LAddC']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAddB->all_d'] = {'preConds': {'pop': 'LAddB'}, 'postConds': {'pop': ['LAddA','LAddC']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAddC->all_d'] = {'preConds': {'pop': 'LAddC'}, 'postConds': {'pop': ['LAddB','LAddA']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAdvA->all_v'] = {'preConds': {'pop': 'LAdvA'}, 'postConds': {'pop': ['LAdvB','LAdvC']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAdvB->all_v'] = {'preConds': {'pop': 'LAdvB'}, 'postConds': {'pop': ['LAdvA','LAdvC']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAdvC->all_v'] = {'preConds': {'pop': 'LAdvC'}, 'postConds': {'pop': ['LAdvB','LAdvA']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['Interneuron->all'] = {'preConds': {'pop': 'IN'}, 'postConds':
                                            {'pop': ['LAddA','LAddB','LAddC','LAdvA','LAdvB','LAdvC','IN']},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight': w_ip,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'GABA',
                    'plasticity': plast}

netParams.connParams['all->Interneuron'] = {'preConds': {'pop': ['LAddA','LAddB','LAddC','LAdvA','LAdvB','LAdvC']},
                                            'postConds': {'pop': 'IN'},
                    'probability': p,  # max number of incoming conns to cell	'
                    'weight':  w_pi,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

p_r = 0.1
netParams.connParams['LAddA->all_v'] = {'preConds': {'pop': 'LAddA'}, 'postConds': {'pop': ['LAdvB','LAdvC']},
                    'probability': p_r,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAddB->all_v'] = {'preConds': {'pop': 'LAddB'}, 'postConds': {'pop': ['LAdvA','LAdvC']},
                    'probability': p_r,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAddC->all_v'] = {'preConds': {'pop': 'LAddC'}, 'postConds': {'pop': ['LAdvB','LAdvA']},
                    'probability': p_r,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}


netParams.connParams['LAdvA->all_d'] = {'preConds': {'pop': 'LAdvA'}, 'postConds': {'pop': ['LAddB','LAddC']},
                    'probability': p_r,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAdvB->all_d'] = {'preConds': {'pop': 'LAdvB'}, 'postConds': {'pop': ['LAddA','LAddC']},
                    'probability': p_r,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

netParams.connParams['LAdvC->all_d'] = {'preConds': {'pop': 'LAdvC'}, 'postConds': {'pop': ['LAddB','LAddA']},
                    'probability': p_r,  # max number of incoming conns to cell	'
                    'weight': w_pp,  # synaptic weight
                    'delay': 2, # transmission delay (ms) '
                    'sec': sec,
                    'synMech': 'AMPA',
                    'plasticity': plast}

# Simulation options
simConfig = specs.SimConfig()   # object of class SimConfig to store simulation configuration
simConfig.duration = tstop      # Duration of the simulation, in ms
simConfig.dt = 0.025            # Internal integration timestep to use
simConfig.verbose = False
simConfig.hParams = {'celsius':31}

# Show detailed messages
#simConfig.recordTraces = {'V_soma':{'sec':'soma','loc':0.5,'var':'v'}}
# Dict with traces to record
simConfig.recordStep = 0.1
# Step size in ms to save data (eg. V traces, LFP, etc)
simConfig.filename = 'model_output'
# Set file output names
simConfig.saveJson = True

# Plot a raster
# simConfig.analysis['plotRaster'] = {'orderInverse': True,'include': list(range(0, 80)), 'saveFig': 'exc_raster.png'}
# simConfig.analysis['plotRaster'] = {'orderInverse': True,'include': list(range(81, 101)), 'saveFig': 'in_raster.png'}
simConfig.analysis['plotRaster'] = {'orderInverse': True,}

#simConfig.analysis['plotTraces'] = {'include': [0,10,13,15]}
# simConfig.analysis['plot2Dnet'] = True
# simConfig.analysis['plotConn'] = True

# Plot recorded traces for this list of cells
# Create network and run simulation
sim.createSimulateAnalyze(netParams = netParams, simConfig = simConfig)

# calculate firing rate
# numSpikes = float(len(sim.simData['spkt']))
# numCells = float(len(sim.net.cells))

numSpikes_in = np.sum((np.array(sim.gatherData()['spkid'])>= 80)*(np.array(sim.gatherData()['spkt'])>100))
numCells_in = float(len(sim.net.cells[80:]))

numSpikes_exc = np.sum((np.array(sim.gatherData()['spkid'])< 80)*(np.array(sim.gatherData()['spkt'])>100))
numCells_exc = float(len(sim.net.cells[:80]))

duration = (simConfig.duration-100)/1000.0
# netFiring = numSpikes/numCells/duration
netFiring_exc = numSpikes_exc/numCells_exc/duration
netFiring_in = numSpikes_in/numCells_in/duration

print('firing rate: exc: ' + str(netFiring_exc) + ', in: ' + str(netFiring_in))

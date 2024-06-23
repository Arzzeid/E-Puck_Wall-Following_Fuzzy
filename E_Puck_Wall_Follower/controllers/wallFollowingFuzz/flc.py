import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# variabel dan membership functions
error = ctrl.Antecedent(np.arange(-100, 101, 0.01), 'error')
delta_error = ctrl.Antecedent(np.arange(-3, 20, 0.01), 'delta_error')
control = ctrl.Consequent(np.arange(-6.28, 6.29, 0.01), 'control')

# Input membership functions
error.automf(names=['SK', 'K', 'N', 'B', 'SB'])
delta_error.automf(names=['SK', 'K', 'N', 'B', 'SB'])

control['SL'] = fuzz.gaussmf(control.universe, -6.28, 1.256)
control['L'] = fuzz.gaussmf(control.universe, -3.14, 1.256)
control['N'] = fuzz.gaussmf(control.universe, 0, 1.256)
control['C'] = fuzz.gaussmf(control.universe, 3.14, 1.256)
control['SC'] = fuzz.gaussmf(control.universe, 6.28, 1.256)

# rules
rules = []
rules.append(ctrl.Rule(error['SK'] & delta_error['SK'], control['SL']))
rules.append(ctrl.Rule(error['SK'] & delta_error['K'], control['SL']))
rules.append(ctrl.Rule(error['SK'] & delta_error['N'], control['L']))
rules.append(ctrl.Rule(error['SK'] & delta_error['B'], control['L']))
rules.append(ctrl.Rule(error['SK'] & delta_error['SB'], control['N']))

rules.append(ctrl.Rule(error['K'] & delta_error['SK'], control['SL']))
rules.append(ctrl.Rule(error['K'] & delta_error['K'], control['L']))
rules.append(ctrl.Rule(error['K'] & delta_error['N'], control['N']))
rules.append(ctrl.Rule(error['K'] & delta_error['B'], control['N']))
rules.append(ctrl.Rule(error['K'] & delta_error['SB'], control['C']))

rules.append(ctrl.Rule(error['N'] & delta_error['SK'], control['L']))
rules.append(ctrl.Rule(error['N'] & delta_error['K'], control['N']))
rules.append(ctrl.Rule(error['N'] & delta_error['N'], control['N']))
rules.append(ctrl.Rule(error['N'] & delta_error['B'], control['C']))
rules.append(ctrl.Rule(error['N'] & delta_error['SB'], control['C']))

rules.append(ctrl.Rule(error['B'] & delta_error['SK'], control['L']))
rules.append(ctrl.Rule(error['B'] & delta_error['K'], control['N']))
rules.append(ctrl.Rule(error['B'] & delta_error['N'], control['C']))
rules.append(ctrl.Rule(error['B'] & delta_error['B'], control['C']))
rules.append(ctrl.Rule(error['B'] & delta_error['SB'], control['SC']))

rules.append(ctrl.Rule(error['SB'] & delta_error['SK'], control['N']))
rules.append(ctrl.Rule(error['SB'] & delta_error['K'], control['C']))
rules.append(ctrl.Rule(error['SB'] & delta_error['N'], control['C']))
rules.append(ctrl.Rule(error['SB'] & delta_error['B'], control['SC']))
rules.append(ctrl.Rule(error['SB'] & delta_error['SB'], control['SC']))

# control system
control_ctrl = ctrl.ControlSystem(rules)
control_sim = ctrl.ControlSystemSimulation(control_ctrl)

# fungsi FLC
def flc_compute(nilai_error, nilai_delta_error):
    control_sim.input['error'] = nilai_error
    control_sim.input['delta_error'] = nilai_delta_error
    control_sim.compute()
   
    return control_sim.output['control']

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np

r = 0.6

# Fuzzy inference

input = ctrl.Antecedent(np.arange(0, 0.9, 0.1), 'r')
output = ctrl.Consequent(np.arange(0.8, 2.1, 0.1), 's')

# Define membership functions
input['less'] = fuzz.trimf(input.universe, [0, 0, 0.3])
input['equal'] = fuzz.trimf(input.universe, [0.1, 0.4, 0.7])
input['more'] = fuzz.trimf(input.universe, [0.5, 0.8, 0.8])

output['less'] = fuzz.trimf(output.universe, [0.8, 0.8, 1.2])
output['equal'] = fuzz.trimf(output.universe, [1, 1.4, 1.8])
output['more'] = fuzz.trimf(output.universe, [1.6, 2.0, 2.0])

# Fuzzy rules
rule1 = ctrl.Rule(input['equal'], output['equal'])
rule2 = ctrl.Rule(input['more'], output['more'])
rule3 = ctrl.Rule(input['less'], output['less'])

fuzzy_control = ctrl.ControlSystem([rule1, rule2, rule3])
fuzzy = ctrl.ControlSystemSimulation(fuzzy_control)

fuzzy.input['r'] = r
fuzzy.compute()

input.view()
output.view()

output.view(sim = fuzzy)
input.view(sim = fuzzy)


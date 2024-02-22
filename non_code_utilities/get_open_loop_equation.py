import os
import numpy as np
from sklearn.linear_model import LinearRegression

if os.path.exists('open_loop_equations.txt'):
    os.remove('open_loop_equations.txt')

files = [element for element in os.listdir(os.getcwd()) if '.csv' in element]

coeficcient, intercept = [], []
for file in files:
    data = []
    with open(file, 'r') as csv:
        data = [line.split(',') for line in csv.readlines()]
    voltage, rpm = [], []
    if not data[0][0].isnumeric():
        data[0][0] = 0
    for pair in data:
        voltage.append(float(pair[0]))
        rpm.append(float(pair[1].rstrip('\n')))
    voltage = np.array(voltage).reshape(-1, 1)
    rpm = np.array(rpm).reshape(-1, 1)
    model = LinearRegression().fit(rpm, voltage)
    with open('open_loop_equations.txt', 'a') as out:
        coeficcient.append(model.coef_[0][0])
        intercept.append(model.intercept_[0])
        out.write(f"{file.split('_')[-1].rstrip('.csv')}: {model.coef_[0][0]}rpm{model.intercept_[0]:+}\n")

mean_coef = sum(coeficcient) / len(coeficcient)
intercept = [abs(element) for element in intercept]
mean_intercept = sum(intercept) / len(intercept)

with open('open_loop_equations.txt', 'a') as out:
    out.write(f"\nAveraged Coefficients: ({mean_coef}, {mean_intercept})\n")

print('\nOpen loop equations written to open_loop_equations.txt')
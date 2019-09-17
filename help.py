import numpy as np

B = 1.27e29
E = 1.42

T = 500

ni = (B * T**3 * np.exp(-1 * (E / ( (8.62e-5) * T))))

print("{:.2E}".format(ni))
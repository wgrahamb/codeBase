import numpy as np

def integrate(dy_new, dy, y, step):
	return y + (dy_new + dy) * step / 2
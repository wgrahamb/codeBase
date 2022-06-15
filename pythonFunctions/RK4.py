import numpy as np

def rk4(step, lowerlimit, upperlimit, function, functionprime):
	steps = np.arange(lowerlimit, upperlimit, step)
	w = function(lowerlimit)
	for h in steps:
		K1 = step * functionprime(h, w)
		K2 = step * functionprime(h + (step / 2),  w + (K1 / 2))
		K3 = step * functionprime(h + (step / 2), w + (K2 / 2))
		K4 = step * functionprime(h + step, w + K3)
		new_w = w + ((K1 + (2 * K2) + (2 * K3) + K4) / 6)
		print(round(h + step, 2), "->", "ACTUAL:", function(round(h + step, 2)), "NUMERICAL:", new_w)
		w = new_w

if __name__ == "__main__":

	def fxn(t):
		return (t ** 2) + (2 * t) + 1 - (0.5 * np.exp(t))

	def fxnprime(t, y):
		return y - (t ** 2) + 1

	lower = 0
	upper = 2
	stepsize = 0.001
	rk4(stepsize, lower, upper, fxn, fxnprime)

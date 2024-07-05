import numpy as np
from scipy.signal import butter, sosfilt_zi, sosfilt
from matplotlib import pyplot as plt


sig = np.random.random(1000)
sig[500:] += 1
hp10 = butter(2, 10, 'high', False, 'sos', 1000)
lp10 = butter(2, 30, 'low', False, 'sos', 1000)

hp10_zi = sig[0] * sosfilt_zi(hp10)
lp10_zi = sig[0] * sosfilt_zi(lp10)

sig_hp10, _ = sosfilt(hp10, sig, zi=hp10_zi)
sig_lp10, _ = sosfilt(lp10, sig, zi=lp10_zi)

f, axs = plt.subplots(1,1)
axs.plot(sig)
axs.plot(sig_hp10)
axs.plot(sig_lp10)
axs.plot(sig - sig_lp10)

axs.legend(["Signal", "HP10", "LP10", "Signal - LP10"])

f.show()
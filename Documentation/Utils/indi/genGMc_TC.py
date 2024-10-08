#!/usr/bin/env python

import numpy as np
from scipy.constants import g as GRAVITY

def inertiaFromPendulumPeriod(P, R, m):
    # assuming a phyiscal pendulum with centroid R from the rotation point and
    # mass m. If it oscillates with period P, we can readily calculate the 
    # inertia I.
    # However, if R is rather large (m*R*R > I), then accuracy of this
    # calculation decreases, especially P and R must be known very precisely 
    # (+-1%) for a reasonable estimate (+- 10%)

    # https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Book%3A_University_Physics_I_-_Mechanics_Sound_Oscillations_and_Waves_(OpenStax)/15%3A_Oscillations/15.05%3A_Pendulums
    # T = 2pi * sqrt(I/(mgR)) where R is distance axle to CoG
    # I = T^2 / (4pi^2) * mgR

    # inertia around rotation point
    Iaxle = (m*GRAVITY*R*P*P) / (4*np.pi*np.pi)

    # subtract parallel axis term, as Iaxle = I + m*R*R
    return Iaxle - m*R*R

def inertiaFromGyrationRadius(m, Rg):
    return m * Rg * Rg

def inertiaBellFromMotorNumber(num):
    D = int( num / 100 ) + 2 # assume 2mm bell thickness (including magnets)
    L  = num % 100
    G_PER_L_AND_D2 = 7.6 / 7 / (14*14)  * (D / 14) # N = 1... assume linear relation between D and thickness of material, probably wrong for bigger motors or very small motors
    m = 1e-3 * G_PER_L_AND_D2 * L * (D * D)
    Rg = 1e-3 * (D - 2.) / 2. # assume radius of gyration is at inside edge of magnets

    return inertiaFromGyrationRadius(m, Rg)

# fall back inertia examples
def inertiaFromPropData(m, Dinch):
    Rg_prop = 0.4 * (Dinch * 0.0254 / 2.) # assume gyration radius 40% of actual
    return inertiaFromGyrationRadius(m, Rg_prop)

###################################
#%% properties
###################################

#%% dimensions and configuration

# using betaflight numbering
#   ^
#   |      # don't ask...
# 4   2
#  \ / 
#  / \
# 3   1
width = 53e-3 # width in meters
length = 53e-3
# dimensions from motor axle to motor axle
diagonal = np.hypot(width, length)


m = 0.040 # allup mass in kg
#m = 0.070 # allup mass in kg, with PI
direc = [1, -1, -1, 1] # motor rotation directions (positive -> right hand along prop thrust vector), sequence FL, FR, RR, RL


#%% inertia measurements

# Option 1: from pendulum periods
# let quad oscillate as a pendulum around the motor axles in all 3 directions
# record oscillation periods in seconds using the IMU
# MAKE SURE THAT COG IS EXACTLY IN THE CENTER
# FOR SAFETY, REMOVE PROPS. WE'LL ADD THEIR CONTRIBUTION LATER
Px = None
Py = None
Pz = None
Raxle = 5 * 0.5e-3 # motor axle radius (to calculate location of rotation point)

# Option 2: direct
rg_over_diag_xx = 0.20 # guesswork...
rg_over_diag_yy = 0.27
rg_over_diag_zz = 0.28
Ixx = 1.5* inertiaFromGyrationRadius(m, rg_over_diag_xx * diagonal)
Iyy = 1.5* inertiaFromGyrationRadius(m, rg_over_diag_yy * diagonal)
Izz = 1.5* inertiaFromGyrationRadius(m, rg_over_diag_zz * diagonal)


#%% prop and motor inertia

# always needed: prop mass
mp = 0.88e-3

# Option 1: prop inertia measurements (measured for red 2.1inch prop) using pendulum period
Pp = None
Rp = None
# 1% mass error: 1% error in inertia --> estimated precision 0.02g, so 1.5% error

# Option 2: prop inertia estimation via mass and diameter
Dpinch = 40. / 25.4 # prop diameter in inch

# get motor bell inertia from motor dimensions, very crude
motorNumber = 803


#%% propeller/ESC/motor performance at 2S battery (see prop.py)

tau = 0.025 # spinup/spindown time constant
Tmax = 0.4 # https://brushlessmotorsparade.com/brushless-motor/racerstar-br0703b-racing-edition-15000kv
k = Tmax / (75000. / 60. * 2 * np.pi)**2 # constant in T = k omega^2 -- shameless guess
#k = 2.66e-7 # black 3inch pitch prop
#Tmax = 4.5 # max thrust black prop
CM = 0.01 # steady-state moment coefficient M = CM * T

# ESC+motor+prop performance at around 60% charge
k_ESC = 0.40 # who knows


#%% configuration: position, thrust axis and direction.

# body coordinates
#
#   ^ x
#   | 
# z x--> y 

# configuration
N = 4
X = .5 * np.array([
    [-length, +width, 0.],
    [+length, +width, 0.],
    [-length, -width, 0.],
    [+length, -width, 0.]
]).T
axes = [0., 0., -1.] # normlized thrust axis 
axes = np.reshape(np.repeat(axes, N), (3, N)) # repeated N times


###################################
#%% perform calculations
###################################

#%% inertias
Rx = width/2 - Raxle
Ry = length/2 - Raxle
Rz = diagonal/2 - Raxle # distance for z-axis rotation

# inertia tests are to be done without props, so we add them back in
if (Ixx and Iyy and Izz):
    pass
elif (Px and Py and Pz):
    Ixx = inertiaFromPendulumPeriod(Px, Rx, m - (4*mp)) + (4*mp) * (width/2)**2
    Iyy = inertiaFromPendulumPeriod(Py, Ry, m - (4*mp)) + (4*mp) * (length/2)**2
    Izz = inertiaFromPendulumPeriod(Pz, Rz, m - (4*mp)) + (4*mp) * (diagonal/2)**2
else:
    raise ValueError("Must have either Px, Py, Pz or Ixx, Iyy, Izz defined")

if (Pp and Rp and mp):
    Iprop = inertiaFromPendulumPeriod(Pp, Rp, mp)
elif (mp and Dpinch):
    Iprop = inertiaFromPropData(mp, Dpinch)
else:
    raise ValueError("Must have either Pp, Rp, mp or Dpinch, mp defined")

Imotorprop = Iprop + inertiaBellFromMotorNumber(motorNumber)

# 1% error in P: 7% error in inertia --> estimated precision 0.5ms, so bueno
# 1% error in R: 1.5% error in inertia --> estimated precision 5mm, so 8%...
# 1% error in m: 1% error in inertia --> estimated precision 0.1g, so bueno

# these probably highly depend on the type of quad

"""
Rg_frac_xx = 0.58
Rg_frac_yy = 0.85
Rg_frac_zz = 0.5

Ixx_bak = inertiaFromGyrationRadius(m, Rg_frac_xx * width/2)
Iyy_bak = inertiaFromGyrationRadius(m, Rg_frac_yy * length/2)
Izz_bak = inertiaFromGyrationRadius(m, Rg_frac_zz * diagonal/2)
Iprop_bak = inertiaFromPropData(mp, 3.)
Iprop_bak += inertiaBellFromMotorNumber(1407)
"""


#%% matrices

# let O = (Fx Fy Fz Mx My Mz)
# idea: DeltaO = B1 * DeltaT  +  B2 * DeltaWdot
# 
# where B1 holds information about thrust axes and motor locations
# and   B2 holds information about thrust axes and propeller inertia
# 
# using w = sqrt(T/k), first-order dynamics wdot = (w - w0)/tau and taylor 
# expansion of the square root results in:
# 
#   DeltaO = B1 DeltaT  +  B2 / (2*w0*tau*k) * (DeltaT - DeltaTprev)
#
# Introduce specific generalized forces A = (fx fy fz taux tauy tauz) with 
# units (N/kg N/kg N/kg Nm/(kgm^2) Nm/(kgm^2) Nm/(kgm^2)) and
# the normalized unitless control U = T / Tmax
#
#   DeltaA = G1 DeltaU  +  G2 / (2*w0*tau*k) * (DeltaU - DeltaUprev)
#      where  G1   == Minv * Tmax * B1  and  G2 == Minv * Tmax * B2
#      and    Minv == inv(diag(m,m,m,Ixx,Iyy,Izz))  called generalized mass matrix
# 
# this can later be inverted to compute DeltaU by solving:
#
#   DeltaA + G2n / w0 DeltaU_prev = ( G1 + G2 / w0 )  DeltaU
#      where G2n = G2 / (2*tau*k)
#
# or, assuming wdot feedback is available
#
#   DeltaA + G2 / Tmax * wdot_prev = ( G1 + G2n / w0 ) DeltaU

B1 = np.zeros((6, N))
B2 = np.zeros((6, N))
B2[3:, :] = -np.array(direc)*axes * Imotorprop

# force components of G1
B1[:3, :] = axes

# moment components of G1
for i in range(N):
    # moment due to thrust axes offset from CoG
    B1[3:, i] = np.cross(X[:, i], axes[:, i])

    # moment due to rotor drag
    B1[3:, i] += -CM*direc[i]*axes[:, i] # negative because reaction force

M = np.diag([m,m,m,Ixx,Iyy,Izz]) # generalized mass
G1 = np.linalg.solve(M, B1) * Tmax
G2 = np.linalg.solve(M, B2) * Tmax
G2n = G2 / (2*tau*k)
w0_hover = np.sqrt(m*GRAVITY/N / k)
Ginv_hover = np.linalg.pinv(G1 + G2n / w0_hover)


#%% print
print(f"G1:\n{G1}")
print(f"G2:\n{G2}")
print(f"G2_normalizer: {1/(2*tau*k)}")
print(f"G2n:\n{G2n}")
print(f"inv(G1 + G2n/w0) at hover:\n{Ginv_hover}")

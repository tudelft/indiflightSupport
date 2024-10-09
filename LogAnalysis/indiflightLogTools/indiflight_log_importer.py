# Load and cache indiflight logs into SI units, also provide simple plotting
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

import pandas as pd
from orangebox import Parser as BFLParser

import logging
import pickle
import os
import re
from matplotlib import pyplot as plt
from hashlib import md5

logger = logging.getLogger(__name__)

local_rc = plt.rcParams.copy()
local_rc.update({
#    "text.usetex": True,
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 12,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 12,
    "axes.titlesize": 16,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.loc": 'best',
    "legend.fontsize": 9,
    'figure.subplot.bottom': 0.05,
    'figure.subplot.left': 0.075,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.925,
    'figure.subplot.hspace': 0.2,
    'figure.subplot.wspace': 0.15,
    'figure.titlesize': 'large',
})

class IndiflightLog(object):
    UNIT_FLOAT_TO_UNSIGNED16VB = ((127 << 7) - 1)
    UNIT_FLOAT_TO_SIGNED16VB = ((127 << 6) - 1)
    RADIANS_TO_DEGREES = 57.2957796
    RADIANS_TO_DECADEGREES = 1e-1 * RADIANS_TO_DEGREES
    RADIANS_TO_DECIDEGREES = 10 * RADIANS_TO_DEGREES
    RADIANS_TO_HUNDRESOFRADIANS = 0.01
    METER_TO_MM = 1000.
    METER_TO_CM = 100.
    RAD_TO_MRAD = 1000.
    ONE_G = 9.80665
    PERCENT = 100.
    DSHOT_MIN = 158.
    DSHOT_MAX = 2048.
    CACHE_NAME = "indiflight_logs"

    @staticmethod
    def modeToText(bits):
        # rc_modes.h:boxId_e, but only the first 32 bits i guess, because
        # blackbox.c:1200 only memcpys 4 bytes
        def single(bit):
            match(bit):
                case 0: return "ARM"
                case 1: return "ANGLE"
                case 2: return "HORIZON"
                case 3: return "MAG"
                case 4: return "HEADFREE"
                case 5: return "PASSTHRU"
                case 6: return "FAILSAFE"
                case 7: return "GPSRESCUE"
                case 8: return "VELCTL"
                case 9: return "POSCTL"
                case 10: return "CATAPULT"
                case 11: return "LEARNER"
                case 12: return "PIDCTL"
                case 13: return "NNCTL"
                case 14: return "ANTIGRAVITY"
                case 15: return "HEADADJ"
                case 16: return "CAMSTAB"
                case 17: return "BEEPERON"
                case 18: return "LEDLOW"
                case 19: return "CALIB"
                case 20: return "OSD"
                case 21: return "TELEMETRY"
                case 22: return "SERVO1"
                case 23: return "SERVO2"
                case 24: return "SERVO3"
                case 25: return "BLACKBOX"
                case 26: return "AIRMODE"
                case 27: return "3D"
                case 28: return "FPVANGLEMIX"
                case 29: return "BLACKBOXERASE"
                case 30: return "RESETHOME"
                case 31: return "CAMERA1"
                case 32: return "CAMERA2"
                case 33: return "CAMERA3"
                case 34: return "FLIPOVERAFTERCRASH"
                case 35: return "PREARM"
                case 36: return "THROWTOARM"
                case 37: return "BEEPGPSCOUNT"
                case 38: return "VTXPITMODE"
                case 39: return "PARALYZE"
                case 40: return "USER1"
                case 41: return "USER2"
                case 42: return "USER3"
                case 43: return "USER4"
                case 44: return "PIDAUDIO"
                case 45: return "ACROTRAINER"
                case 46: return "VTXCONTROLDISABLE"
                case 47: return "LAUNCHCONTROL"
                case 48: return "MSPOVERRIDE"
                case 49: return "STICKCOMMANDDISABLE"
                case 50: return "BEEPERMUTE"
                case 51: return "READY"

        try:
            return [single(bit) for bit in bits]
        except TypeError:
            return single(bits)

    def __init__(self, filename, timeRange=None, useCache=True):
        self.filename = filename

        if useCache:
            self.raw, self.parameters = self._tryCacheLoad(filename)

        if not useCache or self.raw is None or self.parameters is None:
            # import raw data using orangebox parser
            logger.info("Parsing logfile")
            self.bfl = BFLParser.load(filename)

            if self.bfl.reader.log_count > 1:
                raise NotImplementedError("IndiflightLog not implemented for multiple\
                                           logs per BFL or BBL file. Use bbsplit\
                                           cmd line util")

            # dump data rows into pandas frame. # TODO: only import until range?
            logger.info("Importing into dataframe")
            try:
                data = [frame.data for frame in self.bfl.frames()]
            except (IndexError, TypeError):
                # work around really annoying issue in orangebox
                logger.warning("Encountered internal error, trying to append EOF to datafile")
                with open(filename, 'ab') as file:
                    EOF = bytes(1024) # loads of zeros
                    EOF += b'E' + bytes.fromhex("ff") + b'End of log' + bytes.fromhex("00")
                    file.write(EOF)

                self.bfl = BFLParser.load(filename)
                data = [frame.data for frame in self.bfl.frames()]
                logger.warning("Recovered succesfully")

            self.raw = pd.DataFrame(data, columns=self.bfl.field_names )
            self.raw.set_index('loopIteration', inplace=True)

            # get parameters
            self.parameters = self.bfl.headers

            # pickle, if requested
            if useCache:
                self._storeCache()

        self.num_learner_vars = sum([
            re.match(r'^fx_p_rls_x\[[0-9]+\]$', c) is not None 
            for c in self.raw.columns])

        # crop to time range and apply scaling
        logger.info("Apply scaling and crop to range")
        self.data = self._processData(timeRange)

        # parse rc box mode change events (use LogData.modeToText to decode)
        logger.info("Convert flight modes to events")
        self.flags = self._convertModeFlagsToEvents()
        logger.info("Done")

    @staticmethod
    def clearCache():
        from platformdirs import user_cache_dir

        cachedir = user_cache_dir(IndiflightLog.CACHE_NAME, IndiflightLog.CACHE_NAME)

        logger.info(f"Clearing cache at {cachedir}")
        try:
            for file_name in os.listdir(cachedir):
                os.remove(os.path.join(cachedir, file_name))
        except FileNotFoundError:
            pass

    def crop(self, start, stop):
        timeS = self.data['timeS']
        boolarr = (timeS >= start) & (timeS <= stop)
        return self.data[boolarr], timeS[boolarr].to_numpy()

    def _tryCacheLoad(self, filename):
        from platformdirs import user_cache_dir

        cachedir = user_cache_dir(self.CACHE_NAME, self.CACHE_NAME)
        if not os.path.exists(cachedir):
            os.makedirs(cachedir)

        BUF_SIZE = 65536 # chunksize

        # idea: filename is md5 digest of both the log and this file. That 
        #       should prevent reading any stale/corrupt cache
        hash = md5()
        with open(filename, 'rb') as f:
            while True:
                data = f.read(BUF_SIZE)
                if not data:
                    break
                hash.update(data)
        with open(__file__, 'rb') as f:
            while True:
                data = f.read(BUF_SIZE)
                if not data:
                    break
                hash.update(data)

        cacheFileStem=hash.hexdigest()

        # Join the folder path with the filename to get the complete file path
        self.cacheFilePath = os.path.join(cachedir, cacheFileStem+'.pkl')
        if os.path.exists(self.cacheFilePath):
            logger.info(f"Using cached pickle in {cachedir} instead of reading BFL log for {filename}")
            with open(self.cacheFilePath, 'rb') as f:
                raw, parameters = pickle.load(f)
            return raw, parameters
        else:
            logger.info(f"No cached pickle found in {cachedir} for {filename}")
            return None, None

    def _storeCache(self):
        # to get here, tryCacheLoad has to have been called
        logger.info("Caching raw logs to pickle")
        logger.debug(f"Cache pickle location: {self.cacheFilePath}")
        with open(self.cacheFilePath, 'wb') as f:
            pickle.dump((self.raw, self.parameters), f)

    def _processData(self, timeRange):
        # crop relevant time range out of raw, and adjust time
        t0 = self.raw['time'].iloc[0]
        if timeRange is not None:
            data = self.raw[ ( (self.raw['time'] - t0) > timeRange[0]*1e3 )
                        & ( (self.raw['time'] - t0) <= timeRange[1]*1e3) ].copy(deep=True)
        else:
            data = self.raw.copy(deep=True)

        self.N = len(data)

        # manage time in s, ms and us
        data['time'] -= t0
        data.rename(columns={'time':'timeUs'}, inplace=True)
        timeUs = data['timeUs'].to_numpy()
        data['timeMs'] = 1e-3 * timeUs
        data['timeS'] = 1e-6 * timeUs

        # adjust column units
        if "blackbox_high_resolution" in self.parameters.keys():
            highRes = 10. if self.parameters['blackbox_high_resolution'] else 1.
        else:
            highRes = 1.

        for col in data.columns:
            if col == 'loopIteration':
                data[col] = data[col].astype(int)
            elif col == 'rcCommand[3]':
                data[col] -= 1000. * highRes
                data[col] /= 1000. * highRes
            elif col.startswith('rcCommand'):
                data[col] /= 500. * highRes
            elif col.startswith('gyro'):
                data[col] /= self.RADIANS_TO_DEGREES * highRes
            elif col.startswith('accSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('acc'):
                data[col] *= self.ONE_G / self.parameters['acc_1G']
            elif re.match(r'^motor\[[0-9]+\]$', col):
                data[col] -= self.DSHOT_MIN
                data[col] /= (self.DSHOT_MAX - self.DSHOT_MIN)
            elif col.startswith('quat') or col.startswith('extQuat') or col.startswith('ekf_quat'):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('alpha'):
                data[col] /= self.RADIANS_TO_DECADEGREES
            elif col.startswith('spfSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('dv'):
                data[col] /= 10.
            elif col.startswith('u['):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('u_state'):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('omega_dot'):
                data[col] *= 100
            elif col.startswith('omega'):
                data[col] /= 1.
            elif col.startswith('pos') or col.startswith('extPos') or col.startswith('ekf_pos'):
                data[col] /= self.METER_TO_MM
            elif col.startswith('vel') or col.startswith('extVel') or col.startswith('ekf_vel'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('extAtt') or col.startswith('ekf_att'):
                data[col] /= 1000.
            elif col.startswith('ekf_acc_b'):
                data[col] /= 1000.
            elif col.startswith('ekf_gyro_b'):
                data[col] /= self.RADIANS_TO_DEGREES
            elif (match := re.match(r'^motor_[0-9]+_rls_x\[([0-9]+)\]$', col)):
                bbscaler = 1000.
                yscaler = 0.001
                if match.group(1) in ['0', '1', '2']:
                    # a, b, and w0
                    ascaler = 1.
                elif match.group(1) in ['3']:
                    # time constant
                    ascaler = 0.0001
                else:
                    raise NotImplementedError(f"Regressor {match.group(1)} not expected")

                data[col] /= bbscaler * yscaler / ascaler
            elif (match := re.match(r'^fx_([xyzpqr])_rls_x\[([0-9]+)\]$', col)):
                bbscaler = 1000.
                if match.group(1) in ['x', 'y', 'z']:
                    # forces. All 4 regressors have scale 1e-5. Output has scaler 10.
                    yscaler = 10.
                    ascaler = 1e-5
                elif match.group(1) in ['p', 'q', 'r']:
                    # rotations. First 4 regressors have scale 1e-5. Last 4 scale 1e-3 Output has scaler 1.
                    yscaler = 1.
                    num_vars = 8 if self.num_learner_vars > 8 else 4
                    w2vars = [str(i) for i in range(num_vars)]
                    wdotvars = [str(i) for i in range(num_vars, num_vars*2)]
                    if match.group(2) in w2vars:
                        ascaler = 1e-5
                    elif match.group(2) in wdotvars:
                        ascaler = 1e-3
                    else:
                        raise NotImplementedError(f"Regressor {match.group(2)} not expected")
                else:
                    raise NotImplementedError(f"Output {match.group(1)} not expected")

                data[col] /= bbscaler * yscaler / ascaler
            elif (match := re.match(r'^imu_rls_x\[([0-9]+)\]$', col)):
                if match.group(1) not in ['0', '1', '2']:
                    raise NotImplementedError(f"Output {match.group(1)} not expected")
                    # forces. All 4 regressors have scale 1e-5. Output has scaler 10.
                bbscaler = 1000.
                yscaler = 1.
                ascaler = 1e-2
                data[col] /= bbscaler * yscaler / ascaler
            elif (col.startswith("learnerGains")):
                data[col] /= 10.
            elif (col.startswith("hoverAttitude")):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif (match := re.match(r'^.*_lambda$', col)):
                data[col] /= self.UNIT_FLOAT_TO_UNSIGNED16VB
            elif (match := re.match(r'^.*_e_var$', col)):
                data[col] /= 0.1 * ((1 << 16) - 1)
            elif (col == "flightModeFlags") or (col == "stateFlags")\
                    or (col == "failsafePhase") or (col == "rxSignalReceived")\
                    or (col == "rxFlightChannelValid"):
                with pd.option_context("future.no_silent_downcasting", True):
                    data.replace({col: {"": 0}}, inplace=True)
                data[col] = data[col].astype(int)

        return data

    def _convertModeFlagsToEvents(self):
        # parse changes in flightModeFlags column into its own dataframe
        flags = []
        lastFlags = 0
        for index, row in self.data.iterrows():
            # for some reason, flightModeFlags in the logs doesnt correspond to
            # flightModeFlags in betaflight, but to rcModeActivationMask...
            currentFlags = int(row['flightModeFlags'])
            e, d = IndiflightLog._getModeChanges(currentFlags, lastFlags)
            lastFlags = currentFlags

            if (len(e) + len(d)) > 0:
                flags.append({'loopIteration': index, 
                                'timeUs': int(row['timeUs']),
                                "enable": e, 
                                "disable": d})

        df = pd.DataFrame(flags)
        df.set_index('loopIteration', inplace=True)
        return df

    @staticmethod
    def _getModeChanges(new, old=0):
        # find if bits have been turned on (enabled) or turned off (enabled) 
        # between two int32
        enabled = []
        disabled = []
        for i in range(32):
            bitSel = (1 << i)
            if (new & bitSel) and not (old & bitSel):
                enabled.append(i)
            elif not (new & bitSel) and (old & bitSel):
                disabled.append(i)

        return enabled, disabled

    def resetTime(self):
        self.data['timeS'] -= self.data['timeS'].iloc[0]
        self.data['timeMs'] -= self.data['timeMs'].iloc[0]
        self.data['timeUs'] -= self.data['timeUs'].iloc[0]

    def plot(self):
        with plt.rc_context(rc=local_rc):
            # plot some overview stuff
            f, axs = plt.subplots(4, 4, figsize=(12,9), sharex='all', sharey='row')
            axs[2, 3].axis('off')
            axs[3, 3].axis('off')

            for i in range(4):
                line1, = axs[0, i].plot(self.data['timeMs'], self.data[f'omega[{i}]'], label=f'onboard rpm omega[{i}]')

                yyax = axs[0, i].twinx()
                line2, = yyax.plot(self.data['timeMs'], self.data[f'u[{i}]'], label=f'command u[{i}]', color='orange')
                yyax.tick_params('y', colors='orange')
                yyax.set_ylim(bottom=-0.1, top=1.1)

                lines = [line1, line2]
                labels = [line.get_label() for line in lines]

                axs[0, i].legend(lines, labels)
                axs[0, i].set_ylim(bottom=-0.1, top=1.1)
                if (i==0):
                    axs[0, i].set_ylabel("Motor command/output [-], [rad/s]")

            for i in range(4):
                axs[1, i].plot(self.data['timeMs'], self.data[f'omega_dot[{i}]'], label=f'onboard drpm omega_dot[{i}]')
                axs[1, i].legend()
                if (i==0):
                    axs[1, i].set_ylabel("Motor acceleration [rad/s/s]")
                if (i==3):
                    axs[1, i].set_xlabel("Time [ms]")


            for i in range(3):
                axs[2, i].plot(self.data['timeMs'], self.data[f'alpha[{i}]'], label=f'onboard angular accel alpha[{i}]')
                axs[2, i].legend()
                if (i==0):
                    axs[2, i].set_ylabel("Angular acceleration [rad/s/s]")

            for i in range(3):
                axs[3, i].plot(self.data['timeMs'], self.data[f'accSmooth[{i}]'], label=f'onboard linear accSmooth[{i}]')
                axs[3, i].legend()
                axs[3, i].set_xlabel("Time [ms]")
                if (i==0):
                    axs[3, i].set_ylabel("Specific force [N/kg]")

            # Maximize the window on Linux
            mgr = plt.get_current_fig_manager()
            mgr.resize(1920, 1080)

            f.show()
            return f

    def compare(self, other, other_offset=0, self_name='A', other_name='B'):
        a = self_name
        b = other_name

        with plt.rc_context(rc=local_rc):
            f, axs = plt.subplots(4, 4, figsize=(12,9), sharex='all', sharey='row')
            f.suptitle(f"Log comparison -- {a} vs {b}")
            axs[2, 3].axis('off')
            axs[3, 3].axis('off')

            otherTimeMs = other.data['timeMs'] - other_offset;

            for i in range(4):
                line1, = axs[0, i].plot(self.data['timeMs'], self.data[f'omega[{i}]'], label=f'{a}: onboard rpm omega[{i}]')
                line1b, = axs[0, i].plot(otherTimeMs, other.data[f'omega[{i}]'], linestyle='--', label=f'{b}: onboard rpm omega[{i}]')

                yyax = axs[0, i].twinx()
                line2, = yyax.plot(self.data['timeMs'], self.data[f'u[{i}]'], label=f'{a}: command u[{i}]', color='green')
                line2b, = yyax.plot(otherTimeMs, other.data[f'u[{i}]'], linestyle='--', label=f'{b}: command u[{i}]', color='black')
                yyax.tick_params('y', colors='green')
                yyax.set_ylim(bottom=0)

                lines = [line1, line1b, line2, line2b]
                labels = [line.get_label() for line in lines]

                axs[0, i].legend(lines, labels)
                axs[0, i].set_ylim(bottom=0)
                if (i==0):
                    axs[0, i].set_ylabel("Motor command/output [-], [rad/s]")

            for i in range(4):
                axs[1, i].plot(self.data['timeMs'], self.data[f'omega_dot[{i}]'], label=f'{a}: onboard drpm omega_dot[{i}]')
                axs[1, i].plot(otherTimeMs, other.data[f'omega_dot[{i}]'], linestyle='--', label=f'{b}: onboard drpm omega_dot[{i}]')
                axs[1, i].legend()
                if (i==0):
                    axs[1, i].set_ylabel("Motor acceleration [rad/s/s]")
                if (i==3):
                    axs[1, i].set_xlabel("Time [ms]")

            for i in range(3):
                axs[2, i].plot(self.data['timeMs'], self.data[f'alpha[{i}]'], label=f'{a}: onboard angular accel alpha[{i}]')
                axs[2, i].plot(otherTimeMs, other.data[f'alpha[{i}]'], linestyle='--', label=f'{b}: onboard angular accel alpha[{i}]')
                axs[2, i].legend()
                if (i==0):
                    axs[2, i].set_ylabel("Angular acceleration [rad/s/s]")

            for i in range(3):
                axs[3, i].plot(self.data['timeMs'], self.data[f'accSmooth[{i}]'], label=f'{a}: onboard linear accSmooth[{i}]')
                axs[3, i].plot(otherTimeMs, other.data[f'accSmooth[{i}]'], linestyle='--', label=f'{b}: onboard linear accSmooth[{i}]')
                axs[3, i].legend()
                axs[3, i].set_xlabel("Time [ms]")
                if (i==0):
                    axs[3, i].set_ylabel("Specific force [N/kg]")

            # Maximize the window on Linux
            mgr = plt.get_current_fig_manager()
            mgr.resize(1920, 1080)

            f.show()
            return f

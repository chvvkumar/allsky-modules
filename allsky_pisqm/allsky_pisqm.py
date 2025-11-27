'''
allsky_pisqm.py

A Refactored AllSky Module for PiSQM (TSL2591)
Original Author: chvvkumar
Refactored for AllSky Module system from https://github.com/chvvkumar/PiSQM

Heavily inspired by Richard's work on the ESP platform
https://github.com/rabssm/Radiometer

This module reads sky quality data from a TSL2591 light sensor connected via I2C and writes the
calculated MPSAS value to a JSON file for AllSky Overlay.
'''
import allsky_shared as s
import time
import math
import json
import os
import sys

# -------------------------------------------------------------------------
# EMBEDDED TSL2591 DRIVER CLASS
# -------------------------------------------------------------------------
try:
    import smbus2
except ImportError:
    s.log(0, "ERROR: smbus2 module not found. Please install requirements.")

# Constants
VISIBLE = 2
INFRARED = 1
FULLSPECTRUM = 0

# Default I2C Address (Overridable)
DEFAULT_ADDRESS = 0x29

READBIT = 0x01
COMMAND_BIT = 0xA0
CLEAR_BIT = 0x40
WORD_BIT = 0x20
BLOCK_BIT = 0x10

ENABLE_POWERON = 0x01
ENABLE_POWEROFF = 0x00
ENABLE_AEN = 0x02
ENABLE_AIEN = 0x10
CONTROL_RESET = 0x80

# Lux Coefficients
LUX_DF = 408.0
LUX_COEFB = 1.64
LUX_COEFC = 0.59
LUX_COEFD = 0.86

# Register Addresses
REGISTER_ENABLE = 0x00
REGISTER_CONTROL = 0x01
REGISTER_THRESHHOLDL_LOW = 0x02
REGISTER_THRESHHOLDL_HIGH = 0x03
REGISTER_THRESHHOLDH_LOW = 0x04
REGISTER_THRESHHOLDH_HIGH = 0x05
REGISTER_INTERRUPT = 0x06
REGISTER_CRC = 0x08
REGISTER_ID = 0x0A
REGISTER_CHAN0_LOW = 0x14
REGISTER_CHAN0_HIGH = 0x15
REGISTER_CHAN1_LOW = 0x16
REGISTER_CHAN1_HIGH = 0x17

# Integration Times
INTEGRATIONTIME_100MS = 0x00
INTEGRATIONTIME_200MS = 0x01
INTEGRATIONTIME_300MS = 0x02
INTEGRATIONTIME_400MS = 0x03
INTEGRATIONTIME_500MS = 0x04
INTEGRATIONTIME_600MS = 0x05

# Gains
GAIN_LOW = 0x00   # 1x
GAIN_MED = 0x10   # 25x
GAIN_HIGH = 0x20  # 428x
GAIN_MAX = 0x30   # 9876x

class Tsl2591:
    def __init__(self, i2c_bus=1, sensor_address=DEFAULT_ADDRESS, integration=INTEGRATIONTIME_200MS, gain=GAIN_MED):
        self.bus_id = i2c_bus
        self.address = sensor_address
        self.bus = smbus2.SMBus(self.bus_id)
        self.integration_time = integration
        self.gain = gain
        
        self.set_timing(self.integration_time)
        self.set_gain(self.gain)
        self.disable() 

    def enable(self):
        self.bus.write_byte_data(
            self.address,
            COMMAND_BIT | REGISTER_ENABLE,
            ENABLE_POWERON | ENABLE_AEN | ENABLE_AIEN
        )

    def disable(self):
        self.bus.write_byte_data(
            self.address,
            COMMAND_BIT | REGISTER_ENABLE,
            ENABLE_POWEROFF
        )

    def set_timing(self, integration):
        self.integration_time = integration
        self.enable()
        self.bus.write_byte_data(
            self.address,
            COMMAND_BIT | REGISTER_CONTROL,
            self.integration_time | self.gain
        )

    def set_gain(self, gain):
        self.gain = gain
        self.enable()
        self.bus.write_byte_data(
            self.address,
            COMMAND_BIT | REGISTER_CONTROL,
            self.integration_time | self.gain
        )

    def get_int_time_ms(self):
        case_integ = {
            INTEGRATIONTIME_100MS: 100,
            INTEGRATIONTIME_200MS: 200,
            INTEGRATIONTIME_300MS: 300,
            INTEGRATIONTIME_400MS: 400,
            INTEGRATIONTIME_500MS: 500,
            INTEGRATIONTIME_600MS: 600
        }
        return case_integ.get(self.integration_time, 100)

    def calculate_light(self, full, ir):
        if (full >= 0xFFFF) or (ir >= 0xFFFF):
            return 0.0, 0.0
            
        atime = float(self.get_int_time_ms())
        case_gain = {
            GAIN_LOW: 1.,
            GAIN_MED: 24.5,
            GAIN_HIGH: 400.,
            GAIN_MAX: 9876.
        }
        again = case_gain.get(self.gain, 24.5)
        cpuW0 = (atime / 100.0) * (again / 400.0) * 264.1
        
        if cpuW0 == 0:
            return 0.0, 0.0

        fullc = full / cpuW0
        irc = ir / cpuW0
        return fullc, irc

    def read_word(self, register):
        try:
            return self.bus.read_word_data(self.address, COMMAND_BIT | register)
        except Exception as e:
            s.log(0, f"ERROR: I2C Read Error: {e}")
            return 0

    def forced_read(self):
        """
        Takes a single reading using the currently configured manual settings.
        """
        self.enable()
        # Wait for integration time + margin
        wait_time = (self.get_int_time_ms() / 1000.0) + 0.12 
        time.sleep(wait_time)
        
        full = self.read_word(REGISTER_CHAN0_LOW)
        ir = self.read_word(REGISTER_CHAN1_LOW)
        self.disable()
        return full, ir

    def advanced_read(self):
        """
        Auto-ranging read function.
        """
        self.enable()
        max_attempts = 15
        attempt = 0
        
        while attempt < max_attempts:
            attempt += 1
            wait_time = (self.get_int_time_ms() / 1000.0) + 0.12 
            time.sleep(wait_time)
            
            full = self.read_word(REGISTER_CHAN0_LOW)
            ir = self.read_word(REGISTER_CHAN1_LOW)
            
            if full > 60000: 
                changed = False
                if self.gain > GAIN_LOW:
                    if self.gain == GAIN_MAX: self.set_gain(GAIN_HIGH)
                    elif self.gain == GAIN_HIGH: self.set_gain(GAIN_MED)
                    elif self.gain == GAIN_MED: self.set_gain(GAIN_LOW)
                    changed = True
                elif self.integration_time > INTEGRATIONTIME_100MS:
                    new_time = self.integration_time - 1
                    self.set_timing(new_time)
                    changed = True
                if changed: continue 
                else: break

            if full < 200:
                changed = False
                if self.gain < GAIN_MAX:
                    if self.gain == GAIN_LOW: self.set_gain(GAIN_MED)
                    elif self.gain == GAIN_MED: self.set_gain(GAIN_HIGH)
                    elif self.gain == GAIN_HIGH: self.set_gain(GAIN_MAX)
                    changed = True
                elif self.integration_time < INTEGRATIONTIME_600MS:
                    new_time = self.integration_time + 1
                    self.set_timing(new_time)
                    changed = True
                if changed: continue 
                else: break

            break
            
        self.disable()
        return full, ir

# -------------------------------------------------------------------------
# ALLSKY MODULE DEFINITION
# -------------------------------------------------------------------------

metaData = {
    "name": "PiSQM TSL2591",
    "description": "Reads Sky Quality (MPSAS) from TSL2591 and writes to JSON for Overlay",
    "module": "allsky_pisqm",
    "version": "v1.0.3",
    "events": [
        "periodic",
        "night",
        "day"
    ],
    "experimental": "false",
    "arguments": {
        "m0": "-16.07",
        "ga": "25.55",
        "i2c_address": "0x29",
        "gain": "Auto",
        "integration": "Auto",
        "json_file_path": "/home/pi/allsky/config/overlay/extra/tsl2591.json"
    },
    "argumentdetails": {
        "m0": {
            "required": "true",
            "description": "Magnitude Zero Point (M0)",
            "help": "Calibration value. Default is -16.07",
            "type": { "fieldtype": "text" }
        },
        "ga": {
            "required": "true",
            "description": "Glass Attenuation (GA)",
            "help": "Correction for enclosure glass. Default is 25.55",
            "type": { "fieldtype": "text" }
        },
        "i2c_address": {
            "required": "true",
            "description": "I2C Address",
            "help": "The I2C address of the sensor (Default 0x29)",
            "type": { "fieldtype": "text" }
        },
        "gain": {
            "required": "false",
            "description": "Gain",
            "help": "Sensor Gain. Auto is recommended.",
            "type": {
                "fieldtype": "select",
                "values": "Auto,Low,Medium,High,Max"
            }
        },
        "integration": {
            "required": "false",
            "description": "Integration Time",
            "help": "Sensor Integration Time. Auto is recommended.",
            "type": {
                "fieldtype": "select",
                "values": "Auto,100ms,200ms,300ms,400ms,500ms,600ms"
            }
        },
        "json_file_path": {
            "required": "true",
            "description": "Output JSON Path",
            "help": "Full path to save the JSON overlay data.",
            "type": { "fieldtype": "text" }
        }
    }
}

def pisqm(params, event):
    # Parse configuration
    try:
        M0 = float(params.get("m0", -16.07))
        GA = float(params.get("ga", 25.55))
        
        # Parse I2C Address
        i2c_addr_str = params.get("i2c_address", "0x29")
        try:
            i2c_addr = int(i2c_addr_str, 16)
        except ValueError:
            s.log(0, f"ERROR: Invalid I2C Address {i2c_addr_str}. Using default 0x29.")
            i2c_addr = 0x29

        # Updated default file name to tsl2591.json
        json_path = params.get("json_file_path", "/home/pi/allsky/config/overlay/extra/tsl2591.json")
        
        # Parse Gain & Integration Overrides
        ui_gain = params.get("gain", "Auto")
        ui_int = params.get("integration", "Auto")
        
        gain_map = {
            "Low": GAIN_LOW,
            "Medium": GAIN_MED,
            "High": GAIN_HIGH,
            "Max": GAIN_MAX
        }
        
        int_map = {
            "100ms": INTEGRATIONTIME_100MS,
            "200ms": INTEGRATIONTIME_200MS,
            "300ms": INTEGRATIONTIME_300MS,
            "400ms": INTEGRATIONTIME_400MS,
            "500ms": INTEGRATIONTIME_500MS,
            "600ms": INTEGRATIONTIME_600MS
        }

        # Initialize Sensor
        try:
            # We initialize with default or user manual settings
            init_gain = gain_map.get(ui_gain, GAIN_MED)
            init_int = int_map.get(ui_int, INTEGRATIONTIME_200MS)
            
            tsl = Tsl2591(i2c_bus=1, sensor_address=i2c_addr, integration=init_int, gain=init_gain)
        except Exception as e:
            err_msg = f"Failed to initialize TSL2591 at {hex(i2c_addr)}: {e}. Check I2C connection."
            s.log(0, f"ERROR: {err_msg}")
            return err_msg

        # Measurement Logic
        if ui_gain == "Auto" and ui_int == "Auto":
            # Fully Automatic
            full, ir = tsl.advanced_read()
        else:
            # Manual Override
            # TSL is already initialized with the correct manual settings in __init__
            # If "Auto" was selected for one but not the other, we treat it as specific default (Med/200ms)
            # because mixed auto/manual logic is complex. 
            # If user wants manual, they generally override both or accept the default for the other.
            full, ir = tsl.forced_read()
            s.log(1, f"INFO: PiSQM using Manual Settings (Gain: {ui_gain}, Int: {ui_int})")

        full_C, ir_C = tsl.calculate_light(full, ir)
        
        # Calculate MPSAS
        flux_diff = full_C - ir_C
        if flux_diff > 0:
            mpsas = M0 + GA - 2.5 * math.log10(flux_diff)
        else:
            mpsas = 25.0 # Dark limit convention
        
        mpsas_str = f"{mpsas:.2f}"
        
        # Write to JSON for Overlay
        # Updated to use AS_PISQM prefix
        sqm_data = {
            "AS_PISQM_MPSAS": float(mpsas_str),
            "AS_PISQM_GAIN": tsl.gain,
            "AS_PISQM_INT": tsl.get_int_time_ms(),
            "AS_PISQM_TIMESTAMP": time.strftime('%Y-%m-%d %H:%M:%S'),
            "AS_PISQM_CONFIG_M0": M0,
            "AS_PISQM_CONFIG_GA": GA
        }
        
        try:
            s.checkAndCreatePath(json_path)
            os.makedirs(os.path.dirname(json_path), exist_ok=True)
            with open(json_path, 'w') as f:
                json.dump(sqm_data, f)
            s.log(1, f"INFO: PiSQM wrote {mpsas_str} to {json_path}")
        except Exception as e:
            s.log(0, f"ERROR: Failed to write JSON: {e}")

        return f"SQM: {mpsas_str} (Gain: {tsl.gain})"

    except Exception as e:
        s.log(0, f"ERROR: PiSQM Fatal Error: {e}")
        return "Error"

def pisqm_cleanup():
    moduleData = {
        "metaData": metaData,
        "cleanup": {
            "files": {
                "tsl2591.json"
            },
            "env": {}
        }
    }
    s.cleanupModule(moduleData)
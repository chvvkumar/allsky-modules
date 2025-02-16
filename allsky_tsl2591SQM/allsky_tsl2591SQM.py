'''
allsky_boilerplate.py

Part of allsky postprocess.py modules.
https://github.com/thomasjacquin/allsky


'''
import allsky_shared as s
import time
import math
import smbus2

metaData = {
    "name": "Sky brightness using TSL2591",
    "description": "This module uses the TSL2591 sensor to measure the sky brightness.",
    "module": "allsky_tsl2591SQM",
    "version": "v1.0.0",    
    "events": [
        "periodic"
    ],
    "experimental": "true",
    "arguments":{
        "extradatafilename": "allskytsl2591SQM.json",
        "period": 60,
        "GA": 25.55
        },
    "argumentdetails": {
        "extradatafilename" : {
            "required": "true",
            "description": "Extra Data Filename",
            "help": "The name for the extra variables file"
        },
        "period" : {
            "required": "false",
            "description": "Read Every",
            "help": "Reads data every x seconds.. Zero will disable this and run the check every time the periodic jobs run",
            "type": {
                "fieldtype": "spinner",
                "min": 0,
                "max": 600,
                "step": 1
            }
        },
        "GA" : {
            "required": "true",
            "description": "Glass Attenuation Factor",
            "help": "Use this to adjust the sky brightness value to account for the glass attenuation factor"
        }
    },
    "businfo": [
    ],
    "changelog": {
        "v1.0.0" : [
            {
                "author": "Kumar Challa",
                "authorurl": "https://github.com/chvvkumar",
                "changes": "Initial Release"
            }
        ]
    }
}

# TSL2591 Constants
VISIBLE = 2
INFRARED = 1
FULLSPECTRUM = 0

SENSOR_ADDRESS = 0x29
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

LUX_DF = 408.0
LUX_COEFB = 1.64
LUX_COEFC = 0.59
LUX_COEFD = 0.86

UP = 1
DOWN = 0

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

INTEGRATIONTIME_100MS = 0x00
INTEGRATIONTIME_200MS = 0x01
INTEGRATIONTIME_300MS = 0x02
INTEGRATIONTIME_400MS = 0x03
INTEGRATIONTIME_500MS = 0x04
INTEGRATIONTIME_600MS = 0x05

GAIN_LOW = 0x00
GAIN_MED = 0x10
GAIN_HIGH = 0x20
GAIN_MAX = 0x30

# TSL2591 Class definition
class Tsl2591:
    def __init__(self, sensor_id, integration=INTEGRATIONTIME_200MS, gain=GAIN_HIGH):
        self.sensor_id = sensor_id
        self.bus = smbus2.SMBus(1)  # Use I2C bus 1 on Raspberry Pi
        self.integration_time = integration
        self.gain = gain
        self.set_timing(self.integration_time)
        self.set_gain(self.gain)
        self.disable()

    def set_timing(self, integration):
        self.enable()
        self.integration_time = integration
        self.bus.write_byte_data(
            SENSOR_ADDRESS,
            COMMAND_BIT | REGISTER_CONTROL,
            self.integration_time | self.gain
        )
        self.disable()

    def set_gain(self, gain):
        self.enable()
        self.gain = gain
        self.bus.write_byte_data(
            SENSOR_ADDRESS,
            COMMAND_BIT | REGISTER_CONTROL,
            self.integration_time | self.gain
        )
        self.disable()

    def enable(self):
        self.bus.write_byte_data(
            SENSOR_ADDRESS,
            COMMAND_BIT | REGISTER_ENABLE,
            ENABLE_POWERON | ENABLE_AEN | ENABLE_AIEN
        )

    def disable(self):
        self.bus.write_byte_data(
            SENSOR_ADDRESS,
            COMMAND_BIT | REGISTER_ENABLE,
            ENABLE_POWEROFF
        )

    def calculate_light(self, full, ir):
        if (full == 0xFFFF) | (ir == 0xFFFF):
            return -1
            
        case_integ = {
            INTEGRATIONTIME_100MS: 100.,
            INTEGRATIONTIME_200MS: 200.,
            INTEGRATIONTIME_300MS: 300.,
            INTEGRATIONTIME_400MS: 400.,
            INTEGRATIONTIME_500MS: 500.,
            INTEGRATIONTIME_600MS: 600.
        }
        
        if self.integration_time in case_integ.keys():
            atime = case_integ[self.integration_time]
        else:
            atime = 600.

        case_gain = {
            GAIN_LOW: 1.,
            GAIN_MED: 24.5,
            GAIN_HIGH: 400.,
            GAIN_MAX: 9876.
        }

        if self.gain in case_gain.keys():
            again = case_gain[self.gain]
        else:
            again = 9876.

        # spec sheet of TSL2591 has 264.1 counts per uW/cm2 at GAIN_HIGH (400) and 100 MS for CH0
        cpuW0 = (atime/100.) * (again/400.) * 264.1
        fullc = full / cpuW0
        irc = ir / cpuW0
        return fullc, irc

    def adjTime(self, adjDirection):
        if (self.integration_time == INTEGRATIONTIME_100MS):
            if (adjDirection > DOWN):
                self.set_timing(INTEGRATIONTIME_200MS)
            else:
                self.set_timing(INTEGRATIONTIME_100MS)
        elif (self.integration_time == INTEGRATIONTIME_200MS):
            if (adjDirection > DOWN):
                self.set_timing(INTEGRATIONTIME_300MS)
            else:
                self.set_timing(INTEGRATIONTIME_100MS)
        elif (self.integration_time == INTEGRATIONTIME_300MS):
            if (adjDirection > DOWN):
                self.set_timing(INTEGRATIONTIME_400MS)
            else:
                self.set_timing(INTEGRATIONTIME_200MS)
        elif (self.integration_time == INTEGRATIONTIME_400MS):
            if (adjDirection > DOWN):
                self.set_timing(INTEGRATIONTIME_500MS)
            else:
                self.set_timing(INTEGRATIONTIME_300MS)
        elif (self.integration_time == INTEGRATIONTIME_500MS):
            if (adjDirection > DOWN):
                self.set_timing(INTEGRATIONTIME_600MS)
            else:
                self.set_timing(INTEGRATIONTIME_400MS)
        else:
            if (adjDirection > DOWN):
                self.set_timing(INTEGRATIONTIME_600MS)
            else:
                self.set_timing(INTEGRATIONTIME_500MS)

    def adjGain(self, adjDirection):
        if (self.gain == GAIN_LOW):
            if (adjDirection > DOWN):
                self.set_gain(GAIN_MED)
            else:
                self.set_gain(GAIN_LOW)
        elif (self.gain == GAIN_MED):
            if (adjDirection > DOWN):
                self.set_gain(GAIN_HIGH)
            else:
                self.set_gain(GAIN_LOW)
        elif (self.gain == GAIN_HIGH):
            if (adjDirection > DOWN):
                self.set_gain(GAIN_MAX)
            else:
                self.set_gain(GAIN_MED)
        else:
            if (adjDirection > DOWN):
                self.set_gain(GAIN_MAX)
            else:
                self.set_gain(GAIN_HIGH)

    def read_word(self, register):
        """Read a word from the I2C device"""
        return self.bus.read_word_data(SENSOR_ADDRESS, COMMAND_BIT | register)

    def advanced_read(self):
        self.enable()
        time.sleep(0.120 * self.integration_time + 1)
        
        full = self.read_word(REGISTER_CHAN0_LOW)
        ir = self.read_word(REGISTER_CHAN1_LOW)
        
        while ((full > 0xFFE0) & (self.gain > GAIN_LOW)):
            self.adjGain(DOWN)
            full = self.read_word(REGISTER_CHAN0_LOW)
            ir = self.read_word(REGISTER_CHAN1_LOW)
            
        while ((full > 0xFFE0) & (self.integration_time > INTEGRATIONTIME_100MS)):
            self.adjTime(DOWN)
            full = self.read_word(REGISTER_CHAN0_LOW)
            ir = self.read_word(REGISTER_CHAN1_LOW)
            
        while ((full < 0x0010) & (self.gain < GAIN_MAX)):
            self.adjGain(UP)
            full = self.read_word(REGISTER_CHAN0_LOW)
            ir = self.read_word(REGISTER_CHAN1_LOW)
            
        while ((full < 0x0010) & (self.integration_time < INTEGRATIONTIME_600MS)):
            self.adjTime(UP)
            full = self.read_word(REGISTER_CHAN0_LOW)
            ir = self.read_word(REGISTER_CHAN1_LOW)
            
        self.disable()
        return full, ir

    def read_low_lux(self):
        """Do repeated reads for very low light levels to reduce noise"""
        nread = 1
        full, ir = self.advanced_read()
        fullSum = full
        irSum = ir
        visSum = fullSum - irSum
        
        while ((visSum < 128) & (nread < 40)):
            nread = nread + 1
            full, ir = self.advanced_read()
            fullSum = fullSum + full
            irSum = irSum + ir
            visSum = fullSum - irSum
            
        full = fullSum/nread
        ir = irSum/nread
        return self.calculate_light(full, ir)

# Main module function
def tsl2591SQM(params, event):
    result = ""
    extra_data = {}
    extradatafilename = params['extradatafilename']
    period = int(params['period'])
    GA = float(params['GA'])
    M0 = -16.07
    mpsas = -25.

    # Check if the module should run
    should_run, diff = s.shouldRun(metaData['module'], period)

    if should_run:
        try:
            # Initialize the TSL2591 sensor
            try:
                tsl = Tsl2591(1)
                tsl.set_gain(GAIN_MED)
                tsl.set_timing(INTEGRATIONTIME_300MS)
                result = "TSL2591 sensor initialized"
                s.log(1, f'INFO: {result}')
            except Exception as e:
                result = "Failed to initialize TSL2591 sensor"
                s.log(1, f'ERROR: Failed to initialize TSL2591 sensor: {e}')

            s.log(1, f'INFO: Reading TSL2591 sensor data at {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}')

            # Read sensor data
            full, ir = tsl.advanced_read()
            full_C, ir_C = tsl.calculate_light(full, ir)
            
            # Calculate sky brightness
            if ((full_C - ir_C) != 0):
                mpsas = round(M0 + GA - 2.5 * math.log10(full_C - ir_C), 2)
                extra_data = mpsas
                result = f"MPSAS: {extra_data}"
                s.log(1, f'INFO: {result}')
            else:
                mpsas = -25.
                extra_data = mpsas
                result = f"MPSAS: {extra_data}"
                s.log(1, f'INFO: Sky brightness is out of range')

            s.log(1, f'INFO: Calculated mpsas value: {result}')
            s.saveExtraData(extradatafilename, result)
            s.setLastRun(metaData['module'])

        # Handle exceptions
        except Exception as e:
            s.log(1, f'ERROR: {e}')
            result = f'ERROR: {e}'

    # Return the result if the module should not run yet    
    else:
        result = f'Will run in {(period - diff):.2f} seconds'
        s.log(1,f'INFO: {result}')
        return result

    return result

# Cleanup function to be called when the module is disabled
def tsl2591SQM_cleanup():
    moduleData = {
        "metaData": metaData,
        "cleanup": {
            "files": [
                "allskytsl2591SQM.json"
            ],
            "env": {}
        }
    }
    s.cleanupModule(moduleData)

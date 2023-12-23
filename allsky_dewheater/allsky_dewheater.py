'''
allsky_dewheater.py

Part of allsky postprocess.py modules.
https://github.com/thomasjacquin/allsky

Changelog
v1.0.1 by Damian Grocholski (Mr-Groch)
- Added extra pin that is triggered with heater pin
- Fixed dhtxxdelay (was not implemented)
- Fixed max heater time (was not implemented)
V1.0.2 by Alex Greenland
- Updated code for pi 5
V1.0.3 by Alex Greenland
- Add AHTx0 i2c sensor
V1.0.4 by Adler6907 
- Added Solo Cloudwatcher
'''
import allsky_shared as s
import time
import sys
import json
import urllib.request
import board
import adafruit_sht31d
import adafruit_dht
import adafruit_ahtx0
from adafruit_bme280 import basic as adafruit_bme280
from adafruit_htu21d import HTU21D
from meteocalc import heat_index
from meteocalc import dew_point
from digitalio import DigitalInOut, Direction, Pull
    
metaData = {
    "name": "Sky Dew Heater Control",
    "description": "Controls a dew heater via a temperature and humidity sensor",
    "module": "allsky_dewheater",
    "version": "v1.0.4",
    "events": [
        "periodic"
    ],
    "experimental": "false",
    "arguments":{
        "type": "None",
        "inputpin": "",
        "heaterpin": "",
        "extrapin": "",
        "i2caddress": "",
        "heaterstartupstate": "OFF",
        "invertrelay": "False",
        "invertextrapin": "False",
        "frequency": "0",
        "limit": "10",
        "force": "0",
        "max": "0",
        "dhtxxretrycount": "2",
        "dhtxxdelay" : "500",
        "extradatafilename": "allskydew.json",
        "sht31heater": "False",
        "solourl": ""
    },
    "argumentdetails": {
        "type" : {
            "required": "false",
            "description": "Sensor Type",
            "help": "The type of sensor that is being used.",
            "tab": "Sensor",
            "type": {
                "fieldtype": "select",
                "values": "None,SHT31,DHT22,DHT11,AM2302,BME280-I2C,HTU21,AHTx0,SOLO-Cloudwatcher",
                "default": "None"
            }
        },
        "inputpin": {
            "required": "false",
            "description": "Input Pin",
            "help": "The input pin for DHT type sensors, not required for i2c devices",
            "tab": "Sensor",
            "type": {
                "fieldtype": "gpio"
            }
        },
        "i2caddress": {
            "required": "false",
            "description": "I2C Address",
            "help": "Override the standard i2c address for a device. NOTE: This value must be hex i.e. 0x76",
            "tab": "Sensor"
        },
        "dhtxxretrycount" : {
            "required": "false",
            "description": "Retry Count",
            "help": "The number of times to retry the sensor read",
            "tab": "DHTXX",
            "type": {
                "fieldtype": "spinner",
                "min": 0,
                "max": 5,
                "step": 1
            }
        },
        "dhtxxdelay" : {
            "required": "false",
            "description": "Delay",
            "help": "The delay between faild sensor reads in milliseconds",
            "tab": "DHTXX",
            "type": {
                "fieldtype": "spinner",
                "min": 0,
                "max": 5000,
                "step": 1
            }
        },
        "heaterpin": {
            "required": "false",
            "description": "Heater Pin",
            "help": "The pin the heater control relay is connected to",
            "tab": "Heater",
            "type": {
                "fieldtype": "gpio"
            }
        },
        "extrapin": {
            "required": "false",
            "description": "Extra Pin",
            "help": "Extra pin that will be triggered with heater pin",
            "tab": "Heater",
            "type": {
                "fieldtype": "gpio"
            }
        },
        "heaterstartupstate" : {
            "required": "false",
            "description": "heater Startup State",
            "help": "The initial state of the dew heater when allsky is started. This is only used if there is no previous status",
            "tab": "Heater",
            "type": {
                "fieldtype": "select",
                "values": "ON,OFF",
                "default": "OFF"
            }
        },
        "invertrelay" : {
            "required": "false",
            "description": "Invert Relay",
            "help": "Normally a GPIO pin will go high to enable a relay. Selecting this option if the relay is wired to activate on the GPIO pin going Low",
            "tab": "Heater",
            "type": {
                "fieldtype": "checkbox"
            }
        },
        "invertextrapin" : {
            "required": "false",
            "description": "Invert Extra Pin",
            "help": "Normally a GPIO extra pin will go high when ebabling heater. Selecting this option inverts extra pin to go low when enabling heater",
            "tab": "Heater",
            "type": {
                "fieldtype": "checkbox"
            }
        },
        "frequency" : {
            "required": "false",
            "description": "Delay",
            "help": "The delay between sensor reads in seconds. Zero will disable this and run the check every time the periodic jobs run",
            "tab": "Dew Control",
            "type": {
                "fieldtype": "spinner",
                "min": 0,
                "max": 1000,
                "step": 1
            }
        },
        "limit" : {
            "required": "false",
            "description": "Limit",
            "help": "If the temperature is within this many degrees of the dew point the heater will be enabled or disabled",
            "tab": "Dew Control",
            "type": {
                "fieldtype": "spinner",
                "min": -100,
                "max": 100,
                "step": 1
            }
        },
        "force" : {
            "required": "false",
            "description": "Forced Temperature",
            "help": "Always enable the heater when the ambient termperature is below this value, zero will disable this.",
            "tab": "Dew Control",
            "type": {
                "fieldtype": "spinner",
                "min": -100,
                "max": 100,
                "step": 1
            }
        },
        "max" : {
            "required": "false",
            "description": "Max Heater Time",
            "help": "The maximum time in seconds for the heater to be on. Zero will disable this.",
            "tab": "Dew Control",
            "type": {
                "fieldtype": "spinner",
                "min": 0,
                "max": 86400,
                "step": 1
            }
        },
        "extradatafilename": {
            "required": "true",
            "description": "Extra Data Filename",
            "tab": "Misc",
            "help": "The name of the file to create with the dew heater data for the overlay manager"
        },
        "sht31heater" : {
            "required": "false",
            "description": "Enable Heater",
            "help": "Enable the inbuilt heater on the SHT31",
            "tab": "SHT31",
            "type": {
                "fieldtype": "checkbox"
            }
        },
        "solourl": {
            "required": "false",
            "description": "URL from solo",
            "help": "Read weather data from lunaticoastro.com 'Solo Cloudwatcher'",
            "tab": "Solo"
        }        
    },
    "changelog": {
        "v1.0.0" : [
            {
                "author": "Alex Greenland",
                "authorurl": "https://github.com/allskyteam",
                "change": "Initial Release"
            }
        ],
        "v1.0.1" : [
            {
                "author": "Damian Grocholski (Mr-Groch)",
                "authorurl": "https://github.com/Mr-Groch",
                "changes": [
                    "Added extra pin that is triggered with heater pin",
                    "Fixed dhtxxdelay (was not implemented)",
                    "Fixed max heater time (was not implemented)"
                ]
            }
        ],
        "v1.0.2" : [
            {
                "author": "Alex Greenland",
                "authorurl": "https://github.com/allskyteam",
                "change": "Initial Release"
            }
        ],
        "v1.0.3" : [
            {
                "author": "Alex Greenland",
                "authorurl": "https://github.com/allskyteam",
                "change": "Add AHTx0 i2c sensor"
            }
        ],
        "v1.0.4" : [
            {
                "author": "Adler6907",
                "authorurl": "https://github.com/Adler6907",
                "change": "Added Solo Cloudwatcher"
            }
        ]                                
    }
}

def readSHT31(sht31heater):
    temperature = None
    humidity = None
    try:
        i2c = board.I2C()
        sensor = adafruit_sht31d.SHT31D(i2c)
        sensor.heater = sht31heater
        temperature = sensor.temperature
        humidity = sensor.relative_humidity
    except RuntimeError as e:
        eType, eObject, eTraceback = sys.exc_info()
        s.log(4, f"ERROR: Module readSHT31 failed on line {eTraceback.tb_lineno} - {e}")

    return temperature, humidity

def doDHTXXRead(inputpin):
    temperature = None
    humidity = None

    try:
        pin = s.getGPIOPin(inputpin)
        dhtDevice = adafruit_dht.DHT22(pin, use_pulseio=False)
        try:
            temperature = dhtDevice.temperature
            humidity = dhtDevice.humidity
        except RuntimeError as e:
            eType, eObject, eTraceback = sys.exc_info()
            s.log(4, f"ERROR: Module doDHTXXRead failed on line {eTraceback.tb_lineno} - {e}")
    except Exception as e:
        eType, eObject, eTraceback = sys.exc_info()
        s.log(4, f"ERROR: Module doDHTXXRead failed on line {eTraceback.tb_lineno} - {e}")

    return temperature, humidity

def readDHT22(inputpin, dhtxxretrycount, dhtxxdelay):
    temperature = None
    humidity = None
    count = 0
    reading = True

    while reading:
        temperature, humidity = doDHTXXRead(inputpin)

        if temperature is None and humidity is None:
            s.log(4, "INFO: Failed to read DHTXX on attempt {}".format(count+1))
            count = count + 1
            if count > dhtxxretrycount:
                reading = False
            else:
                time.sleep(dhtxxdelay/1000)
        else:
            reading = False

    return temperature, humidity

def readBme280I2C(i2caddress):
    temperature = None
    humidity = None
    pressure = None
    relHumidity = None
    altitude = None

    if i2caddress != "":
        try:
            i2caddressInt = int(i2caddress, 16)
        except Exception as e:
            eType, eObject, eTraceback = sys.exc_info()
            s.log(0, f"ERROR: Module readBme280I2C failed on line {eTraceback.tb_lineno} - {e}")

    try:
        i2c = board.I2C()
        if i2caddress != "":
            bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, i2caddressInt)
        else:
            bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

        temperature =  bme280.temperature
        humidity = bme280.relative_humidity
        relHumidity = bme280.relative_humidity
        altitude = bme280.altitude
        pressure = bme280.pressure
    except ValueError as e:
        eType, eObject, eTraceback = sys.exc_info()
        s.log(0, f"ERROR: Module readBme280I2C failed on line {eTraceback.tb_lineno} - {e}")

    return temperature, humidity, pressure, relHumidity, altitude

def readHtu21(i2caddress):
    temperature = None
    humidity = None

    if i2caddress != "":
        try:
            i2caddressInt = int(i2caddress, 16)
        except:
            result = "Address {} is not a valid i2c address".format(i2caddress)
            s.log(0,"ERROR: {}".format(result))

    try:
        i2c = board.I2C()
        if i2caddress != "":
            htu21 = HTU21D(i2c, i2caddressInt)
        else:
            htu21 = HTU21D(i2c)

        temperature =  htu21.temperature
        humidity = htu21.relative_humidity
    except ValueError as e:
        eType, eObject, eTraceback = sys.exc_info()
        s.log(4, f"ERROR: Module readHtu21 failed on line {eTraceback.tb_lineno} - {e}")
        
    return temperature, humidity

def readAHTX0(i2caddress):
    temperature = None
    humidity = None

    if i2caddress != "":
        try:
            i2caddressInt = int(i2caddress, 16)
        except:
            result = "Address {} is not a valid i2c address".format(i2caddress)
            s.log(0,"ERROR: {}".format(result))

    try:
        i2c = board.I2C()
        sensor = adafruit_ahtx0.AHTx0(i2c)
        temperature = sensor.temperature
        humidity = sensor.relative_humidity
    except ValueError as e:
        eType, eObject, eTraceback = sys.exc_info()
        s.log(4, f"ERROR: Module readAHTX0 failed on line {eTraceback.tb_lineno} - {e}")

    return temperature, humidity

def readSolo(url):
    temperature = None
    humidity = None
    pressure = None
    dewPoint = None
    
    try: 
        #Read Weaterdata from SOLO Website
        jsonData = urllib.request.urlopen(url).read()  
        response = json.dumps(json.loads(jsonData)["LastReadings"]) 
        
        #response = '{ "LastReadings": { "dataGMTTime" : "2023/12/18 18:22:07", "cwinfo" : "Serial: 2550, FW: 5.89", "clouds" : -18.130000, "cloudsSafe" : "Safe", "temp" : 7.820000, "wind" : 7, "windSafe" : "Safe", "gust" : 8, "rain" : 3100, "rainSafe" : "Safe", "lightmpsas" : 19.82, "lightSafe" : "Safe", "switch" : 1, "safe" : 1, "hum" : 40, "humSafe" : "Safe", "dewp" : -4.940000, "rawir" : -22.680000, "abspress" : 998.450000, "relpress" : 1027.527780, "pressureSafe" : "Safe" } }'
        currentWeatherdata =  json.loads(response)['LastReadings']

        # that is what you should receive
        #    { "LastReadings": {
        #    "dataGMTTime" : "2023/12/17 22:52:40",
        #    "cwinfo" : "Serial: 2550, FW: 5.89",
        #    "clouds" : -14.850000,
        #    "cloudsSafe" : "Unsafe",
        #    "temp" : 8.450000,
        #    "wind" : 12,
        #    "windSafe" : "Safe",
        #    "gust" : 13,
        #    "rain" : 3072,
        #    "rainSafe" : "Safe",
        #    "lightmpsas" : 20.31,
        #    "lightSafe" : "Safe",
        #    "switch" : 0,
        #    "safe" : 0,
        #    "hum" : 65,
        #    "humSafe" : "Safe",
        #    "dewp" : 2.250000,
        #    "rawir" : -19.150000,
        #    "abspress" : 1003.600000,
        #    "relpress" : 1032.722598,
        #    "pressureSafe" : "Safe"
        #    }
        #    }        
            
        temperature = float(currentWeatherdata['temp'])
        humidity = float(currentWeatherdata['hum'])
        pressure = float(currentWeatherdata['relpress'])
        dewPoint = float(currentWeatherdata['dewp'])
    except Exception as e:
        eType, eObject, eTraceback = sys.exc_info()
        s.log(4, f"ERROR: Module readSolo failed on line {eTraceback.tb_lineno} - {e}")
            
    return temperature, humidity, pressure, dewPoint

def turnHeaterOn(heaterpin, invertrelay, extra=False):
    if extra:
        type = 'Extra'
    else:
        type = 'Heater'
        
    result = f"Turning {type} on using pin {heaterpin}"
    pin = DigitalInOut(heaterpin)
    pin.switch_to_output()
    
    if invertrelay:
        pin.value = 0
    else:
        pin.value = 1

    if not s.dbHasKey("dewheaterontime"):
        now = int(time.time())
        s.dbAdd("dewheaterontime", now)
    s.log(1,f"INFO: {result}")

def turnHeaterOff(heaterpin, invertrelay, extra=False):
    if extra:
        type = 'Extra'
    else:
        type = 'Heater'
                    
    result = f"Turning {type} off using pin {heaterpin}"
    pin = DigitalInOut(heaterpin)
    pin.direction = Direction.OUTPUT
    
    if invertrelay:
        pin.value = 1
    else:
        pin.value = 0
        
    if s.dbHasKey("dewheaterontime"):
        s.dbDeleteKey("dewheaterontime")
    s.log(1,f"INFO: {result}")

def getSensorReading(sensorType, inputpin, i2caddress, dhtxxretrycount, dhtxxdelay, sht31heater, soloURL):
    temperature = None
    humidity = None
    dewPoint = None
    heatIndex = None
    pressure = None
    relHumidity = None
    altitude = None

    if sensorType == "SHT31":
        temperature, humidity = readSHT31(sht31heater)
    elif sensorType == "DHT22" or sensorType == "DHT11" or sensorType == "AM2302":
        temperature, humidity = readDHT22(inputpin, dhtxxretrycount, dhtxxdelay)
    elif sensorType == "BME280-I2C":
        temperature, humidity, pressure, relHumidity, altitude = readBme280I2C(i2caddress)
    elif sensorType == "HTU21":
        temperature, humidity = readHtu21(i2caddress)
    elif sensorType == "AHTx0":
        temperature, humidity = readAHTX0(i2caddress)
    elif sensorType == "SOLO-Cloudwatcher":
        temperature, humidity, pressure, dewPoint = readSolo(soloURL)         
    else:
        s.log(0,"ERROR: No sensor type defined")

    if temperature is not None and humidity is not None:
        dewPoint = dew_point(temperature, humidity).c
        heatIndex = heat_index(temperature, humidity).c

        tempUnits = s.getSetting("temptype")
        if tempUnits == 'F':
            temperature = (temperature * (9/5)) + 32
            dewPoint = (dewPoint * (9/5)) + 32
            heatIndex = (heatIndex * (9/5)) + 32
            s.log(4,"INFO: Converted temperature to F")

        temperature = round(temperature, 2)
        humidity = round(humidity, 2)
        dewPoint = round(dewPoint, 2)
        heatIndex = round(heatIndex, 2)

    return temperature, humidity, dewPoint, heatIndex, pressure, relHumidity, altitude

def getLastRunTime():
    lastRun = None

    if s.dbHasKey("dewheaterlastrun"):
        lastRun = s.dbGet("dewheaterlastrun")

    return lastRun

def debugOutput(sensorType, temperature, humidity, dewPoint, heatIndex, pressure, relHumidity, altitude):
    s.log(1,f"INFO: Sensor {sensorType} read. Temperature {temperature} Humidity {humidity} Relative Humidity {relHumidity} Dew Point {dewPoint} Heat Index {heatIndex} Pressure {pressure} Altitude {altitude}")
    
def dewheater(params, event):    
    result = ""
    sensorType = params["type"]
    heaterstartupstate = params["heaterstartupstate"]
    try:
        heaterpin = int(params["heaterpin"])
    except ValueError:
        heaterpin = 0
    try:
        extrapin = int(params["extrapin"])
    except ValueError:
        extrapin = 0
    force = int(params["force"])
    limit = int(params["limit"])
    invertrelay = params["invertrelay"]
    invertextrapin = params["invertextrapin"]
    try:
        inputpin = int(params["inputpin"])
    except ValueError:
        inputpin = 0
    frequency = int(params["frequency"])
    maxontime = int(params["max"])
    i2caddress = params["i2caddress"]
    dhtxxretrycount = int(params["dhtxxretrycount"])
    dhtxxdelay = int(params["dhtxxdelay"])
    extradatafilename = params['extradatafilename']
    sht31heater = params["sht31heater"]

    try:
        soloURL = params["solourl"]
    except ValueError:
        soloURL = ''
                
    temperature = 0
    humidity = 0
    dewPoint = 0
    heatIndex = 0
    heater = 'Off'

    shouldRun, diff = s.shouldRun('allskydew', frequency)

    if shouldRun:                        
        try:
            heaterpin = int(heaterpin)
        except ValueError:
            heaterpin = 0

        if heaterpin != 0:
            heaterpin = s.getGPIOPin(heaterpin)
            if extrapin !=0:
                extrapin = s.getGPIOPin(extrapin)
            lastRunTime = getLastRunTime()
            if lastRunTime is not None:
                now = int(time.time())
                lastRunSecs = now - lastRunTime
                if lastRunSecs >= frequency:
                    s.dbUpdate("dewheaterlastrun", now)
                    temperature, humidity, dewPoint, heatIndex, pressure, relHumidity, altitude = getSensorReading(sensorType, inputpin, i2caddress, dhtxxretrycount, dhtxxdelay, sht31heater, soloURL)
                    if temperature is not None:
                        lastOnSecs = 0
                        if s.dbHasKey("dewheaterontime"):
                            lastOnTime = s.dbGet("dewheaterontime")
                            lastOnSecs = now - lastOnTime
                        if maxontime != 0 and lastOnSecs >= maxontime:
                            result = "Heater was on longer than maximum allowed time {}".format(maxontime)
                            s.log(1,"INFO: {}".format(result))
                            turnHeaterOff(heaterpin, invertrelay)
                            if extrapin != 0:
                                turnHeaterOff(extrapin, invertextrapin, True)
                            heater = 'Off'
                        elif force != 0 and temperature <= force:
                            result = "Temperature below forced level {}".format(force)
                            s.log(1,"INFO: {}".format(result))
                            turnHeaterOn(heaterpin, invertrelay)
                            if extrapin != 0:
                                turnHeaterOn(extrapin, invertextrapin, True)
                            heater = 'On'
                        else:
                            if ((temperature-limit) <= dewPoint):
                                turnHeaterOn(heaterpin, invertrelay)
                                if extrapin != 0:
                                    turnHeaterOn(extrapin, invertextrapin)
                                heater = 'On'
                                result = "Temperature within limit temperature {}, limit {}, dewPoint {}".format(temperature, limit, dewPoint)
                                s.log(1,"INFO: {}".format(result))
                            else:
                                result = "Temperature outside limit temperature {}, limit {}, dewPoint {}".format(temperature, limit, dewPoint)
                                s.log(1,"INFO: {}".format(result))
                                turnHeaterOff(heaterpin, invertrelay)
                                if extrapin != 0:
                                    turnHeaterOff(extrapin, invertextrapin, True)
                                heater = 'Off'
                            
                        extraData = {}
                        extraData["AS_DEWCONTROLAMBIENT"] = str(temperature)
                        extraData["AS_DEWCONTROLDEW"] = str(dewPoint)
                        extraData["AS_DEWCONTROLHUMIDITY"] = str(humidity)
                        extraData["AS_DEWCONTROLHEATER"] = heater
                        if pressure is not None:
                            extraData["AS_DEWCONTROLPRESSURE"] = pressure
                        if relHumidity is not None:
                            extraData["AS_DEWCONTROLRELHUMIDITY"] = relHumidity
                        if altitude is not None:
                            extraData["AS_DEWCONTROLALTITUDE"] = altitude

                        s.saveExtraData(extradatafilename,extraData)

                        debugOutput(sensorType, temperature, humidity, dewPoint, heatIndex, pressure, relHumidity, altitude)

                    else:
                        result = "Failed to read sensor"
                        s.log(0, "ERROR: {}".format(result))
                        s.deleteExtraData(extradatafilename)
                else:
                    result = "Not run. Only running every {}s. Last ran {}s ago".format(frequency, lastRunSecs)
                    s.log(1,"INFO: {}".format(result))
            else:
                now = int(time.time())
                s.dbAdd("dewheaterlastrun", now)
                s.log(1,"INFO: No last run info so assuming startup")
                if heaterstartupstate == "ON":
                    turnHeaterOn(heaterpin, invertrelay)
                    if extrapin != 0:
                        turnHeaterOn(extrapin, invertextrapin)
                    heater = 'On'
                else:
                    turnHeaterOff(heaterpin, invertrelay)
                    if extrapin != 0:
                        turnHeaterOff(extrapin, invertextrapin)
                    heater = 'Off'
        else:
            s.deleteExtraData(extradatafilename)
            result = "heater pin not defined or invalid"
            s.log(0,"ERROR: {}".format(result))

        s.setLastRun('allskydew')

    else:
        result = 'Will run in {:.2f} seconds'.format(frequency - diff)
        s.log(1,"INFO: {}".format(result))

    return result

def dewheater_cleanup():
    moduleData = {
        "metaData": metaData,
        "cleanup": {
            "files": {
                "allskydew.json"
            },
            "env": {}
        }
    }
    s.cleanupModule(moduleData)

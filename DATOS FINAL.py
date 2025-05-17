from machine import Pin, ADC, SPI, I2C
import time
import math
import ubluetooth
import json
from micropython import const

# BMP280 Constants and Configuration
BMP280_I2C_ADDR = const(0x76)
BMP280_OSAMPLE_1 = const(1)
BMP280_OSAMPLE_2 = const(2)
BMP280_OSAMPLE_4 = const(3)
BMP280_OSAMPLE_8 = const(4)
BMP280_OSAMPLE_16 = const(5)

BMP280_REGISTER_DIG_T1 = const(0x88)
BMP280_REGISTER_DIG_P1 = const(0x8E)
BMP280_REGISTER_CHIPID = const(0xD0)
BMP280_REGISTER_VERSION = const(0xD1)
BMP280_REGISTER_SOFTRESET = const(0xE0)
BMP280_REGISTER_CONTROL = const(0xF4)
BMP280_REGISTER_CONFIG = const(0xF5)
BMP280_REGISTER_PRESSUREDATA = const(0xF7)
BMP280_REGISTER_TEMPDATA = const(0xFA)

# BMP280 Class
class BMP280:
    def __init__(self, i2c, addr=BMP280_I2C_ADDR):
        self._i2c = i2c
        self._addr = addr
        self._load_calibration()
        self._i2c.writeto_mem(self._addr, BMP280_REGISTER_CONTROL, b'\x3F')

    def _load_calibration(self):
        calib = self._i2c.readfrom_mem(self._addr, BMP280_REGISTER_DIG_T1, 24)
        self.dig_T1 = int.from_bytes(calib[0:2], 'little')
        self.dig_T2 = int.from_bytes(calib[2:4], 'little', signed=True)
        self.dig_T3 = int.from_bytes(calib[4:6], 'little', signed=True)
        self.dig_P1 = int.from_bytes(calib[6:8], 'little')
        self.dig_P2 = int.from_bytes(calib[8:10], 'little', signed=True)
        self.dig_P3 = int.from_bytes(calib[10:12], 'little', signed=True)
        self.dig_P4 = int.from_bytes(calib[12:14], 'little', signed=True)
        self.dig_P5 = int.from_bytes(calib[14:16], 'little', signed=True)
        self.dig_P6 = int.from_bytes(calib[16:18], 'little', signed=True)
        self.dig_P7 = int.from_bytes(calib[18:20], 'little', signed=True)
        self.dig_P8 = int.from_bytes(calib[20:22], 'little', signed=True)
        self.dig_P9 = int.from_bytes(calib[22:24], 'little', signed=True)

    def _read_raw_temp(self):
        data = self._i2c.readfrom_mem(self._addr, BMP280_REGISTER_TEMPDATA, 3)
        return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

    def _read_raw_pressure(self):
        data = self._i2c.readfrom_mem(self._addr, BMP280_REGISTER_PRESSUREDATA, 3)
        return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

    def get_temperature(self):
        adc = self._read_raw_temp()
        var1 = (((adc >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
        var2 = (((((adc >> 4) - self.dig_T1) * ((adc >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        return ((self.t_fine * 5 + 128) >> 8) / 100

    def get_pressure(self):
        self.get_temperature()  # Must be done first to get t_fine
        adc = self._read_raw_pressure()
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            return 0
        p = 1048576 - adc
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (self.dig_P8 * p) >> 19
        return ((p + var1 + var2) >> 8) / 25600  # Return in hPa

    def get_altitude(self, sea_level_pressure=1013.25):
        pressure = self.get_pressure()  # in hPa
        return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903))

# BLE Class
class BLE:
    def __init__(self, name):
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.connected = False
        self.connection_handle = None
        
        # Registrar callback para eventos BLE
        self.ble.irq(self.ble_irq)
        
        self.register()
        self.advertiser()
        
        print(f"Dispositivo BLE '{name}' iniciado y en espera de conexión...")
        
    def ble_irq(self, event, data):
        if event == 1:  # _IRQ_CENTRAL_CONNECT
            self.connected = True
            self.connection_handle = data[0]
            print(f"¡Dispositivo conectado! Handle: {self.connection_handle}")
            # Detener anuncios después de conectarse
            self.ble.gap_advertise(None)
            
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            self.connected = False
            self.connection_handle = None
            print("Dispositivo desconectado")
            # Reiniciar anuncios después de desconexión
            self.advertiser()
            print("Esperando nueva conexión...")
        
    def register(self):
        # Registrar servicios BLE y características
        SERVICE_UUID = ubluetooth.UUID('6E400001-B5A3-F393-E0A9-E50E24DCCA9E')
        CHAR_UUID_TX = ubluetooth.UUID('6E400003-B5A3-F393-E0A9-E50E24DCCA9E')
        
        BLE_SENSOR = (SERVICE_UUID, (
            (CHAR_UUID_TX, ubluetooth.FLAG_NOTIFY),
        ))
        
        SERVICES = (BLE_SENSOR,)
        ((self.tx,),) = self.ble.gatts_register_services(SERVICES)
        
    def advertiser(self):
        # Configurar anuncios
        name = bytes(self.name, 'UTF-8')
        adv_data = bytearray([0x02, 0x01, 0x02]) + bytearray([len(name) + 1, 0x09]) + name
        self.ble.gap_advertise(100, adv_data)
        print("Modo visible activado, esperando conexión...")
        
    def send_data(self, data):
        if not self.connected:
            return False
            
        # Formato: datos en JSON
        try:
            data_bytes = json.dumps(data).encode()
            self.ble.gatts_notify(self.connection_handle, self.tx, data_bytes)
            return True
        except Exception as e:
            print(f"Error al enviar datos: {e}")
            return False
    
    def is_connected(self):
        return self.connected

# Función para estimar lux por tramos de voltaje
def estimar_lux(v):
    if v < 0.2:
        return 1
    elif v < 0.4:
        return 5
    elif v < 0.6:
        return 10
    elif v < 1.2:
        return 50
    elif v < 1.8:
        return 100
    elif v < 2.3:
        return 200
    elif v < 2.8:
        return 500
    elif v < 3.2:
        return 800
    else:
        return 1000

def promedio_movil(valores):
    if not valores:
        return 0
    return sum(valores) / len(valores)

# Inicializar BLE
print("Iniciando BLE - Nombre del dispositivo: ESP32_Estacion_Meteo")
ble = BLE("ESP32_Estacion_Meteo")

# LED para indicar conexión (opcional, usar GPIO2 que normalmente tiene LED integrado)
led_conexion = Pin(2, Pin.OUT)
led_conexion.value(0)  # Apagado al inicio

print("ESPERANDO CONEXIÓN BLUETOOTH...")
print("No se iniciarán los sensores hasta que haya una conexión.")

# Esperar hasta que haya conexión
while not ble.is_connected():
    # Parpadeo del LED para indicar que está esperando conexión
    led_conexion.value(1)
    time.sleep_ms(100)
    led_conexion.value(0)
    time.sleep_ms(900)  # Parpadeo lento = esperando conexión

# Una vez conectado, inicializar los sensores
print("¡CONEXIÓN ESTABLECIDA! Inicializando sensores...")
led_conexion.value(1)  # LED encendido fijo = conectado

# Inicializar I2C para BMP280
print("Inicializando sensor BMP280...")
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
try:
    bmp = BMP280(i2c)
    print("BMP280 inicializado correctamente")
    bmp_ok = True
except Exception as e:
    print(f"Error al inicializar BMP280: {e}")
    bmp_ok = False

# Configuración del sensor de viento (PIN 4)
sensor_viento = Pin(4, Pin.IN)
estado_anterior = 0
tiempo_anterior = time.ticks_ms()
diametro = 0.04  # 4 cm
perimetro = math.pi * diametro
K = 1.0  # Factor de calibración

# Lista para almacenar últimas N velocidades
N = 5
lecturas_viento = []
velocidad_promedio = 0

# Configuración del sensor de luz (PIN 34)
ldr_pin = ADC(Pin(34))
ldr_pin.atten(ADC.ATTN_11DB)     # Rango 0–3.3V
ldr_pin.width(ADC.WIDTH_12BIT)   # Resolución 0–4095
luz_lux = 0

# Variables para controlar cuándo ejecutar cada medición
ultimo_tiempo_luz = time.ticks_ms()
ultimo_tiempo_bmp = time.ticks_ms()
ultimo_tiempo_ble = time.ticks_ms()
intervalo_luz = 500      # Medir la luz cada 500ms
intervalo_bmp = 2000     # Medir BMP280 cada 2000ms
intervalo_ble = 3000     # Enviar datos BLE cada 3000ms

# Variables para almacenar lecturas del BMP280
temperatura_bmp = 0
presion = 0
altitud = 0

print("Sensores inicializados. Comenzando mediciones...")

# Bucle principal - solo corre mientras hay conexión
while True:
    # Verificar si sigue conectado
    if not ble.is_connected():
        print("Conexión perdida. Volviendo a modo de espera...")
        led_conexion.value(0)
        
        # Esperar hasta que se vuelva a conectar
        while not ble.is_connected():
            # Parpadeo del LED mientras espera reconexión
            led_conexion.value(1)
            time.sleep_ms(100)
            led_conexion.value(0)
            time.sleep_ms(900)
            
        # Cuando se reconecta
        print("¡Reconectado! Reanudando mediciones...")
        led_conexion.value(1)
        tiempo_anterior = time.ticks_ms()  # Reiniciar tiempo para el sensor de viento
        ultimo_tiempo_luz = time.ticks_ms()
        ultimo_tiempo_bmp = time.ticks_ms()
        ultimo_tiempo_ble = time.ticks_ms()
    
    # Lectura del sensor de viento
    estado_actual = sensor_viento.value()
    if estado_actual == 1 and estado_anterior == 0:
        tiempo_actual = time.ticks_ms()
        delta_t = time.ticks_diff(tiempo_actual, tiempo_anterior) / 1000
        if delta_t > 0:
            rps = 1 / delta_t
            velocidad = K * rps * perimetro
            # Agregar nueva lectura y mantener solo N
            lecturas_viento.append(velocidad)
            if len(lecturas_viento) > N:
                lecturas_viento.pop(0)
            velocidad_promedio = promedio_movil(lecturas_viento)
            print("Velocidad del viento (promedio): {:.2f} m/s".format(velocidad_promedio))
        tiempo_anterior = tiempo_actual
    estado_anterior = estado_actual
    
    # Lectura del sensor de luz cada intervalo_luz ms
    tiempo_actual = time.ticks_ms()
    if time.ticks_diff(tiempo_actual, ultimo_tiempo_luz) >= intervalo_luz:
        raw = ldr_pin.read()
        voltage = raw * 3.33 / 4095
        porcentaje_luz = (voltage / 3.3) * 100
        luz_lux = estimar_lux(voltage)
        print("Voltaje:", round(voltage, 2), "V | Luz:", luz_lux, "lux | % Luz:", round(porcentaje_luz, 1), "%")
        ultimo_tiempo_luz = tiempo_actual
    
    # Lectura del sensor BMP280 cada intervalo_bmp ms
    if bmp_ok and time.ticks_diff(tiempo_actual, ultimo_tiempo_bmp) >= intervalo_bmp:
        try:
            temperatura_bmp = bmp.get_temperature()
            presion = bmp.get_pressure()
            altitud = bmp.get_altitude()
            print(f"T_bar(C): {temperatura_bmp:.2f} | Presion(hPA): {presion:.2f} | Altitud(m): {altitud:.2f}")
        except Exception as e:
            print(f"Error al leer BMP280: {e}")
        ultimo_tiempo_bmp = tiempo_actual
    
    # Enviar datos por BLE cada intervalo_ble ms
    if time.ticks_diff(tiempo_actual, ultimo_tiempo_ble) >= intervalo_ble:
        # Crear un diccionario con los datos a enviar
        data = {
            "viento": round(velocidad_promedio, 2),
            "luz": luz_lux,
            "timestamp": time.ticks_ms()
        }
        
        # Añadir datos del BMP280 si está funcionando
        if bmp_ok:
            data["temperatura"] = round(temperatura_bmp, 2)
            data["presion"] = round(presion, 2)
            data["altitud"] = round(altitud, 2)
        
        if ble.send_data(data):
            print("Datos enviados por BLE:", data)
        
        ultimo_tiempo_ble = tiempo_actual
        
    time.sleep_ms(5)  # Pequeña pausa para no saturar el CPU
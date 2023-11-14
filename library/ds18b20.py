import ds18x20, onewire
import machine

class DS18B20:
    
    def __init__(self, pin_ds18b20, pin_cpu):
        try:
            self.ds_pin = machine.Pin(pin_ds18b20)
            self.ds_sensor = ds18x20.DS18X20(onewire.OneWire(self.ds_pin))
            self.roms = self.ds_sensor.scan()
            self.ds_sensor.convert_temp()
            # Температура CPU
            self.t_CPU = ADC(pin_cpu)
        except Exception as e:
            raise e
        
    # Получение t c DS18B20 (с 1 датчика)
    def do_18B20():
        ds_sensor.convert_temp()
        try:
            for rom in self.roms:
                t1 = self.ds_sensor.read_temp(rom)
            return t1
        except Exception as e:
            raise e
    
    # Получение температуры CPU
    def do_t_CPU():
        v_tcpu = t_CPU.read_u16() * 3.3 / (65535)
        t1 = 27 - (v_tcpu - 0.709) / 0.001721
        return t1

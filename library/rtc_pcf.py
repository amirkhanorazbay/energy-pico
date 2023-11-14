# Часы на PCF85063
import binascii
import machine


class RTC_PCF:
    
    mon = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"]
    
    
    def __init__(self, bus, address=0x51, register=0x04):
        self.bus = bus
        self.rtc_adr = address  
        self.rtc_reg = register  
        
    # Method for setting the Time
    def SetTime(self, NowTime=b"\x00\x23\x12\x28\x14\x07\x21"):
        # sec min hour day week month year
        self.bus.writeto_mem(int(self.rtc_adr), self.rtc_reg, NowTime)

    # Data in bcd format. This has to be converted to a binary format.
    def bcd2bin(self, value):
        return (value or 0) - 6 * ((value or 0) >> 4)

    # Add a 0 in front of numbers smaller than 10
    def pre_zero(self, value):
        fl = True  
        if fl:
            if value < 10: value = f"0{value}"  
        return value

    # Read RealTime 
    def ReadTime(self, mode=0):
        try:
            buffer = self.bus.readfrom_mem(int(self.rtc_adr), int(self.rtc_reg), 7)
            year = self.bcd2bin(buffer[6])
            month = self.pre_zero(self.bcd2bin(buffer[5]))  
            day = self.pre_zero(self.bcd2bin(buffer[3])) 
            hour = self.pre_zero(self.bcd2bin(buffer[2]))  
            minute = self.pre_zero(self.bcd2bin(buffer[1]))
            bb = buffer[0] & 0b01111111
            second = self.pre_zero(self.bcd2bin(bb)) 
            if mode == 0:  
                return second, minute, hour, day, month, year
            if mode == 1:  
                time_string = f"{day}.{month}.{year}{hour}:{minute}:{second}"
                return time_string
            if mode == 2:  
                time_string = f"{year}.{month}.{day} {hour}:{minute}:{second}"
                return time_string
        except Exception as e:
            print('Error: RTC not connected or other problem',e)
            return False
             
    # Установка будильника регистр=1 30 сек (или 1 мин)
    def SetBud(bb=b'\xA0'):
        self.bus.writeto_mem(int(self.rtc_adr), 1, bb)

    # Установка времени будильника регистр=\x0B 
    def SetBudDT(bb=b'\x10\x05\x12\x00'):	#s,m,h,d bit7=0, \x80 нет уставки
        self.bus.writeto_mem(int(self.rtc_adr), b'\x0B', bb)

    def set_time(self, str_time):
        machine.RTC().datetime((int('20' + str_time[8:10]),int(str_time[11:13]), int(str_time[14:16]), 0, int(str_time[17:19]), int(str_time[20:22]), int(str_time[23:25]), 0))
        sp0 = str_time[23:25]
        sp0 = sp0 + str_time[20:22]
        sp0 = sp0 + str_time[17:19]
        sp0 = sp0 + str_time[14:16]
        sp0 = sp0 + '01' #День недели
        sp0 = sp0 + str_time[11:13]
        sp0 = sp0 + str_time[8:10] #Год/
        bp0 = binascii.unhexlify(sp0)
        self.SetTime(NowTime=bp0)

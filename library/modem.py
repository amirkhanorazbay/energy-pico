import time
import machine
import re

from machine import Pin, UART
from config import OBJECT_ID, SERVER_IP, PORT, EMERGENCY_PHONE
from library.logger import Logger
from library.constants import *
# * * * * * * * * * * * * MODEM * * * * * * * * * * * * * *
class Modem:
    # MODEM_STATUS переменная для обазначения статуса модема SIM800
    # 0 - модем выключен
    # 1 - модем включен
    # 2 - модем настроен
    # 3 - модем настроил TCPIP среду
    # 4 - модем подключен к серверу
    # 5 - модем отправил данные на сервер
    # 6 - модем отправил данные по смс
    ERROR = -1
    OFF = 0
    ON = 1
    CONFIGURED = 2
    TCPIP_STATE = 3
    CONNECTED_TO_SERVER = 4
    SENDED_TO_SERVER = 5
    SMS_STATE = 5
    
    def __init__(self, pin, uart_pin, server_ip, port, phone_number, ):
        try:
            self.on_mod = Pin(pin, Pin.OUT)  # Включение Модема
            self.on_mod.value(0)  # 0 - Выкл.
            self.MODEM_STATUS = self.OFF
            self.uart_modem = UART(uart_pin[0], tx=machine.Pin(uart_pin[1]), rx=machine.Pin(uart_pin[2]))
            self.uart_modem.init(baudrate=38400, bits=8, parity=None, stop=1, timeout=20)
            self.log = Logger(f'object_{OBJECT_ID}_modem.log')
            self.last_time_cmd_sends = time.time()
            
            self.AT_INIT = [("AT+CPIN?", "\+CPIN: READYOK"), ("AT+CSQ", "\+CSQ: .+,.+OK"), ("AT+CREG=0", "OK"), ("AT+CREG?", "\+CREG: 0,1OK"),
                ("AT+CGATT?", "\+CGATT: 1OK"), ("AT+CIPMODE=0", "OK"), ("AT+CIPMUX=0", "OK")]
            
            self.AT_TCPIP = [("AT+CSTT=internet.beeline.kz", "OK"), ("AT+CIICR", "OK"), ("AT+CIFSR", ".*\..*\..*\..*",),
                             (f"AT+CIPSTART=TCP,{server_ip},{port}", "OKCONNECT OK"), ("AT+CIPSEND", ">"), ("AT+CIPCLOSE", "CLOSE OK"), ("AT+CIPSHUT", "SHUT OK")]
            
            self.AT_SMS = [("AT+CMGF=1", "OK"), ("AT+CSCS=\"GSM\"", "OK"),
                           ("", ">"), (f"AT+CMGS=\"{phone_number}\"", ".*\+CMGS:.+OK")]
            self.SEND_CHAR = (chr(26), "SEND OK<.*>")
            self.AT_CCLK = ("AT+CCLK?", '\+CCLK: "../../..,..:..:..\+.."OK')
            # ответы ошибки от модема для каждой команды
            # в данный момент в разработке
            self.CONNECT_FAIL = "CONNECT FAIL"
            self.SIM_NOT_INSERTED_ERROR = "\+CME ERROR: SIM not inserted"
            
            self.AT_TIME = ("AT+CCLK?", "\+CCLK: \"../../..,..:..:..\+..\"OK")
            
            self.AT_CHECK_IP_STATUS = ("AT+CIPSTATUS", "OKSTATE:.*")
            self.IP_STATUSES = ["IP START", "IP GPRSACT", "IP STATUS", "CONNECT OK", "CONNECT OK", "TCP CLOSED", "IP INITIAL"]
            
            self.at_curr_timeout_init = [i for i in range(len(self.AT_INIT))]
            
            self.at_timeout_in_sec = 10
            
            self.ALLOW_NEXT_CMD = True
            self.CHECK_IP = False
            
            self.current_at_cmd_index = 0
            self.response = ""
            self.sended = False
        except Exception as e:
            raise e
    
    def clear_log(self):
        self.log.clear_log()
    
    def on(self):
        self.off()
        self.on_mod.value(self.ON)
        self.MODEM_STATUS = self.ON
        last_time_sec = time.time()
        while True:
            if self.uart_modem.any():
                s = self.uart_modem.readline()
                self.response += "".join([chr(b) for b in s])
                self.response = self.response.strip()
                print(self.response)
            if time.time() - last_time_sec >= 15:
                break
        return self.MODEM_STATUS

    def off(self):
        self.on_mod.value(self.OFF)
        self.MODEM_STATUS = self.OFF
        last_time_sec = time.time()
        while True:
            if time.time() - last_time_sec >= 10:
                break
        return self.MODEM_STATUS

    def get_status(self):
        return self.MODEM_STATUS
    
    # Передать строку модему
    def send_str(self, cmd):
        self.log.log(self.log.INFO,f"CMD - {cmd}")
        cmd = cmd + "\r\n"
        self.uart_modem.write(cmd.encode())
        self.last_time_cmd_sends = time.time()
        return True

    def send_number(self, s):
        self.uart_modem.write(s)
        return True
    
    def init(self):
        answer = [None] * len(self.AT_INIT)
        for i, (at_cmd, at_resp) in enumerate(self.AT_INIT):
            self.send_str(at_cmd)
            
            while True:
                if self.uart_modem.any():
                    s = self.uart_modem.readline()
                    self.response += "".join([chr(b) for b in s])
                    self.response = self.response.strip()
                    self.log.log(self.log.INFO,f"RESP - {self.response}")
                    if re.search(at_resp, self.response):
                        answer[i] = self.response[:]
                        self.response = ""
                        self.log.log(self.log.INFO,f"ANSWER - {answer}")
                        break
                    elif re.search(self.SIM_NOT_INSERTED_ERROR, self.response):
                        self.log.log(self.log.ERROR, self.response)
                        self.response = ""
                        self.MODEM_STATUS = self.ERROR
                        answer[i] = self.SIM_NOT_INSERTED_ERROR
                        break
                    elif re.search("\+CME ERROR:.*", self.response):
                        self.log.log(self.log.ERROR, 'CME ERROR')
                        self.off()
                        machine.reset()
                elif self.sended and abs(time.time() - self.last_time_cmd_sends) > self.at_timeout_in_sec:
                    self.log.log(self.log.ERROR, "Timeout with cmd")
                    self.sended = False
                    self.response = ""
                    return -1
        return self.MODEM_STATUS
    
    def get_time(self):
        self.send_str(self.AT_CCLK[0])
            
        while True:
            data = self.process(self.AT_CCLK[1])
            if data:
                return data
            elif abs(time.time() - self.last_time_cmd_sends) > self.at_timeout_in_sec:                  
                return "TIMEOUT"

    def send_to_server(self, data):
        answer = [None] * len(self.AT_TCPIP)
        for i, (at_cmd, at_resp) in enumerate(self.AT_TCPIP):
            self.send_str(at_cmd)
            
            while True:
                if self.uart_modem.any():
                    s = self.uart_modem.readline()
                    self.response += "".join([chr(b) for b in s])
                    self.response = self.response.strip()
                    if re.search(self.AT_TCPIP[self.current_at_cmd_index][1], self.response):
                        answer[i] = self.response
                        self.log.log(self.log.INFO,f"Answer - {answer}")
                        self.response = ""
                        break
                    elif re.search(self.SEND_CHAR[1], self.response):
                        answer[i] = self.response
                        self.log.log(self.log.INFO,f"Answer - {answer}")
                        self.response = ""
                        self.send_str(data)
                        self.send_str(self.SEND_CHAR[0])
                        break
                    elif re.search(self.CONNECT_FAIL, self.response):
                        answer[i] = self.response
                        self.log.log(self.log.ERROR,f"Answer - {answer}")
                        self.response = ""
                        break
                    elif re.search(self.SIM_NOT_INSERTED_ERROR, self.response):
                        self.log.log(self.log.ERROR, self.response)
                        self.response = ""
                        self.MODEM_STATUS = self.ERROR
                        break
                    elif re.search("\+CME ERROR:.*", self.response):
                        self.log.log(self.log.ERROR, 'CME ERROR')
                        self.off()
                        machine.reset()
                        break
                elif self.sended and abs(time.time() - self.last_time_cmd_sends) > self.at_timeout_in_sec:
                    cmd = self.AT_TCPIP[self.current_at_cmd_index][0]
                    self.log.log(self.log.ERROR, "Timeout with cmd " + cmd)
                    self.response = ""
                    self.current_at_cmd_index = -1                    
                    break
        return None

    def connect_to_server(self):
        if self.sended:
            return
        if self.ALLOW_NEXT_CMD:
            self.ALLOW_NEXT_CMD = False
            cmd = self.AT_TCPIP[self.current_at_cmd_index][0]
            self.send_str(cmd)
        elif self.CHECK_IP:
            self.send_str(self.AT_CHECK_IP_STATUS[0])
        self.sended = True     
    
    def send_data(self):
        cmd = self.SEND_CHAR[0]
        self.send_str(cmd)
        self.sended = True   
        return self.MODEM_STATUS
    
    def send_at_cmd(self):
        self.current_at_cmd_index = 4
        cmd = self.AT_TCPIP[self.current_at_cmd_index][0]
        self.send_str(cmd)
        self.sended = True   
        return self.MODEM_STATUS
    
    def process_tcpip(self):
        if self.uart_modem.any():
            s = self.uart_modem.readline()
            self.response += "".join([chr(b) for b in s])
            self.response = self.response.strip()
            if re.search(self.AT_CHECK_IP_STATUS[1], self.response) and self.IP_STATUSES[self.current_at_cmd_index] == self.response[self.response.index(': ') + 2:]:
                self.current_at_cmd_index += 1
                self.current_at_cmd_index %= len(self.AT_TCPIP)
                self.ALLOW_NEXT_CMD = True
                self.log.log(self.log.INFO,f"Answer - {self.response}")
                self.CHECK_IP = False
                self.sended = False
                data = self.response
                self.response = ""
                return data
            elif not self.CHECK_IP and re.search(self.AT_TCPIP[self.current_at_cmd_index][1], self.response):
                self.sended = False
                data = self.response
                self.log.log(self.log.INFO,f"Answer - {self.response}")
                self.response = ""
                self.CHECK_IP = True
                return data
            elif re.search(self.SEND_CHAR[1], self.response):
                self.log.log(self.log.INFO,f"Answer - {self.response}")
                data = self.response
                self.response = ""
                self.sended = False
                return data
            elif re.search(self.CONNECT_FAIL, self.response):
                self.log.log(self.log.ERROR,f"Answer - {self.response}")
                self.sended = False
                self.response = ""
                return self.CONNECT_FAIL
            elif re.search(self.SIM_NOT_INSERTED_ERROR, self.response):
                self.log.log(self.log.ERROR, self.response)
                self.response = ""
                self.MODEM_STATUS = self.ERROR
                return self.SIM_NOT_INSERTED_ERROR
            elif re.search("\+CME ERROR:.*", self.response):
                self.log.log(self.log.ERROR, 'CME ERROR')
                self.off()
                machine.reset()
                return "error is related to ME functionality"
        elif self.sended and abs(time.time() - self.last_time_cmd_sends) > self.at_timeout_in_sec:
            cmd = self.AT_TCPIP[self.current_at_cmd_index][0]
            self.log.log(self.log.ERROR, "Timeout with cmd " + cmd)
            self.sended = False
            self.CHECK_IP = False
            self.ALLOW_NEXT_CMD = True
            self.response = ""
            self.current_at_cmd_index = -1                    
            return "TIMEOUT"
        return None
    
    def process_sms(self):
        if self.uart_modem.any():
            s = self.uart_modem.readline()
            self.response += "".join([chr(b) for b in s])
            self.response = self.response.strip()
            if re.search(self.AT_SMS[self.current_at_cmd_index][1], self.response):
                self.sended = False
                data = self.response
                self.log.log(self.log.INFO,f"Answer - {self.response}")
                self.response = ""
                return data
            elif re.search(self.SIM_NOT_INSERTED_ERROR, self.response):
                self.log.log(self.log.ERROR, self.response)
                self.response = ""
                self.MODEM_STATUS = self.ERROR
                return self.SIM_NOT_INSERTED_ERROR
            elif re.search("\+CME ERROR:.*", self.response):
                self.log.log(self.log.ERROR, 'CME ERROR')
                self.off()
                machine.reset()
                return "error is related to ME functionality"
        elif self.sended and abs(time.time() - self.last_time_cmd_sends) > self.at_timeout_in_sec:
            self.log.log(self.log.ERROR, "Timeout with cmd " + cmd)
            self.sended = False
            self.response = ""
            return "TIMEOUT"
        return None
        
    def send_sms_mode_on(self):
        if self.MODEM_STATUS in range(2,6) and  not self.sended:
            cmd = self.at_cmd[2][self.current_at_cmd_index]
            self.current_at_cmd_type = 2
            self.send_str(cmd)
            self.sended = True
        return self.MODEM_STATUS
    
    def send_sms(self, msg):
        while True:
            self.send_sms_mode_on()
            data = self.process_sms()
            if data and '>' in data:
                self.send_number(f'{msg}'.encode())
                self.send_data()
                break

# modem_var = Modem(3, (1,4,5), SERVER_IP, PORT, EMERGENCY_PHONE)
# modem_var.on()
# modem_var.init()
# while True:
#     modem_var.send_str(input("Enter AT cmd: "))
#     while True:
#         if modem_var.uart_modem.any():
#             a = modem_var.uart_modem.readline()
#             print(a)
#         if input('COnt?') == 'break':
#             break
# print(modem_var.init())
# print(modem_var.send_to_server('<211609459231{16|0030:32768;0031:32768;0032:32768;0033:32768;0034:32768;0035:32768;0036:32768;0037:32768;003b:32768;,17|0030:32768;0031:32768;0032:32768;0033:32768;0034:32768;0035:32768;0036:32768;0037:32768;003b:32768;,18|0030:32768;0031:32768;0032:32768;0033:32768;0034:32768;0035:32768;0036:32768;0037:32768;003b:32768;,}{reset:0,t_cpu:17.08398,VP:12.33201,t_air:19.1875,stat:0,}>'))
# print('AMIRKHS')
# e = 2
# while True:
#     if e == 2:
#         modem_var.connect_to_server()
#     data = modem_var.process_tcpip()
#     if data and 'CLOSE' in data:
#         print("CLOSE CAME TO ME AND I DIE")
#         e = 2
#     elif data and "SHUT" in data:
#         break
#     elif data and '>' in data:
#         e = 1
#         modem_var.send_number(b'<\x01\x00\x02\x00\x00\x00\x00{\x05\x00\xFF\xFF\xFF\xFF,}>')
#         modem_var.send_data()
#     

# print('ORAZBAY')



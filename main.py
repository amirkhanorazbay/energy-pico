"""
Main py file
ver 1.0.15
"""

import time

import machine
import uos
import os
import re
import onewire

from config import (
    OBJECT_ID,
    DI_32_COUNT,
    DI_32_ADDRESSES,
    DI_32_CELL_NUMBER,
    CELL_COUNT,
    CELL_TYPE,
    TRANSFORMATTOR,
    TRANSFORMATTOR_SHFIT,
    TRANSFORMATTOR_MASK,
    MICOM_CELL_NUMBER,
    MICOM_RELAY_COUNT,
    MICOM_EMERGENCY_REGISTER,
    MICOM_REGULAR_REGISTER,
    MICOM_RELAY_ADDRESSES,
    DRY_CONTACT_PER_CELL_COUNT,
    EMERGENCY_PHONE,
    RS_PORT,
    SERVER_IP,
    PORT,
    START_TIME,
    REGULAR_PACKET_PERIOD_SECONDS
)
from machine import Timer
from machine import ADC
import obj_name_dev as dev 

from machine import WDT
import library.ads1x15 as ads1x15
from library.flash import Flash
from library.oled import Oled
from library.modem import Modem
from library.logger import Logger
from library.file import File
from library.sc16is7xx import SC16IS7XX
from library.rtc_pcf import RTC_PCF
from library.crc import crc16, check_crc
from library.ds18b20 import DS18B20
from library.constants import (
    MODBUS_WAIT_PERIOD_MILLIS,
    PACKET_START_CHARACTER,
    PACKET_END_CHARACTER,
    DATA_START_CHARACTER,
    DATA_END_CHARACTER,
    CELL_NUM_VALUE_DELIMITER,
    REGISTER_NUM_VALUE_DELIMITER,
    REGISTER_DELIMITER,
    DATA_DELIMITER,
    REGULAR_PACKET_TYPE,
    EMERGENCY_PACKET_TYPE,
    SERVER_SEND_NOT_STATE,
    SERVER_SEND_REGULAR_STATE,
    SERVER_SEND_EMERGENCY_STATE,
    SERVER_SEND_UNSENDED_STATE,
    REGULAR_PACKET_STATE,
    EMERGENCY_PACKET_STATE,
)
from machine import Pin, UART, SPI, I2C
from time import localtime, mktime, ticks_ms

# ------------------------------- FUNCTION ---------------------------------


def init_modules():
    statuses = 255

    i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100000)
    devices = i2c0.scan()
    log.log(log.INFO, f"I2C0 devices found: {len(devices)}")
    for device in devices:
        log.log(log.INFO, f"Decimal address: {device} | Hex address: {hex(device)}")

    # I2C1 Init
    # RTC, ADS1113, SSD1306, PCA,
    i2c1 = I2C(1, scl=Pin(11), sda=Pin(10), freq=100000)
    devices = i2c1.scan()
    log.log(log.INFO, f"I2C1 devices found: {len(devices)}")
    for device in devices:
        log.log(log.INFO, f" Decimal address: {device} | Hex address: {hex(device)}")
    log.log(
        log.INFO, f"I2C1 Address: {hex(i2c1.scan()[0]).upper()}"
    )  # Display device address
    log.log(log.INFO, f"I2C1 Configuration: {str(i2c1)}")

    # I2C0 Init
    # RS-482-2 (SC16IS74),

    # SPI
    spi = SPI(0, 10_000_000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

    # GD25Q64 Flash 64 Kb
    cs_GD64 = Pin(26, Pin.OUT)
    cs_GD64.value(1)
    flash_gd = None
    try:
        flash_gd = Flash(spi, cs_GD64)
        statuses = statuses & ~(1 << 0)
        log.log(log.INFO, "W25qxx is on")
    except Exception as e:
        statuses = statuses | (1 << 0)
        log.log(log.ERROR, f"W25qxx is {e}")

    # DS18B20 Температура с One Wire ++
    fl_DS18B20 = False
    ds_sensor = None
    try:
        ds_sensor = DS18B20(15, 4)
        statuses = statuses & ~(1 << 1)
        log.log(log.INFO, f"Found DS18B20")
    except Exception as e:
        fl_DS18B20 = False
        statuses = statuses | (1 << 1)
        log.log(log.ERROR, f"DS18B20: NOT {e}")

    # OLED
    WIDTH = 128
    HEIGHT = 32
    oled = None
    try:
        oled = Oled(WIDTH, HEIGHT, i2c1)
        statuses = statuses & ~(1 << 2)
        oled.displayText("Init Modules...", (10, 10))
        log.log(log.INFO, "OLED is on")
    except Exception as e:
        statuses = statuses | (1 << 2)
        log.log(log.ERROR, f"OLED: NOT {e}")

    # RTC_PCF
    rtc_p = None
    try:
        rtc_p = RTC_PCF(i2c1)
        rtc_p.ReadTime()
        statuses = statuses & ~(1 << 3)
        log.log(log.INFO, "RTC_PCF is on")
    except Exception as e:
        statuses = statuses | (1 << 3)
        log.log(log.ERROR, f"RTC_PCF: NOT {e}")

    # ADS1113
    ads = None
    try:
        ads = ads1x15.ADS1115(i2c1)
        statuses = statuses & ~(1 << 4)
        log.log(log.INFO, "ADS1115 is on")
    except Exception as e:
        statuses = statuses | (1 << 4)
        log.log(log.ERROR, f"ADS1115: NOT {e}")
    addr_PCA = 0x41
    i2c1.writeto(addr_PCA, b"\x03\x00")  # pin -> out

    # Port RS-485-2, через I2C. sc16is740 speed: 19200
    rs_2 = None
    try:
        i2c1.writeto(addr_PCA, b"\x01\xFF")
        rs_2 = SC16IS7XX(i2c0, debug=False)
        statuses = statuses & ~(1 << 5)
        log.log(log.INFO, "SC16IS7XX is on")
    except Exception as e:
        statuses = statuses | (1 << 5)
        log.log(log.ERROR, f"SC16IS7XX: NOT {e}")

    modem_var = Modem(3, (1, 4, 5), SERVER_IP, PORT, EMERGENCY_PHONE)

    oled.Oled(["Turning on", "modem ...", ""])
    if modem_var.on() == 1:
        log.log(log.INFO, "Modem is on")
        oled.displayText("Modem is on", (0, 0))
        statuses = statuses & ~(1 << 6)
    else:
        statuses = statuses | (1 << 6)
        log.log(log.ERROR, "cmd Mod On: Bad!")
    
    return (
        flash_gd,
        ds_sensor,
        oled,
        rtc_p,
        ads,
        rs_2,
        modem_var,
        statuses,
    )

def display_general_info():
    display_list = []

    # Вывод времени
    datetime = rtc_p.ReadTime(1)
    display_list.append(datetime)
    
    # DS18B20
    temperature = ds_sensor.do_18B20()
    dict_val["t_air"] = temperature
    dict_val["t_cpu"] = ds_sensor.do_t_CPU()    
    display_list.append("t_DS:{0:0.1f}".format(temperature))
    
    V = ads.read_ADS(1, 10)
    display_list[-1] = display_list[-1] + "Vs:{0:0.1f}".format(V)
    dict_val["VP"] = V
    display_list.append(f"{modules_statuses:08b}  {error_code:02X}")
    dict_val["stat"] = modules_statuses
    if not (modules_statuses & 2 << 1) >> 2:
        oled.Oled(display_list)
    else:
        log.log(log.ERROR, "OLED: Bad")


def send_modbus_req(s, Pin_RS0):
    global ms_RS, is_wait_modbus_res
    Pin_RS0.on()
    ms_RS = time.ticks_add(time.ticks_ms(), len(s) + 2)
    uart.write(s)
    is_wait_modbus_res = True
    time.sleep_ms(5)
    Pin_RS0.off()


def next_cell():
    global server_send_state, packet_form_state, curr_micom_reg_register_id, curr_micom_add_id, curr_cell_id, curr_micom_emerg_add_id, curr_DI_32_add_id, EMERGENCY_FLAG
    
    if packet_form_state == REGULAR_PACKET_STATE:
        curr_micom_reg_register_id += 1
        if curr_micom_reg_register_id == len(MICOM_REGULAR_REGISTER):
            curr_micom_add_id += 1
    elif packet_form_state == EMERGENCY_PACKET_STATE:
        if (CELL_TYPE & 1 << curr_cell_id) >> curr_cell_id:
            curr_micom_emerg_add_id += 1
        else:
            curr_DI_32_add_id += 1
    
    if curr_micom_add_id == MICOM_RELAY_COUNT and curr_micom_reg_register_id == len(MICOM_REGULAR_REGISTER):
        packet_buffer.append(form_packet(REGULAR_PACKET_STATE))
        packet_form_state = EMERGENCY_PACKET_STATE
        DO_1.off()
    
    curr_cell_id += 1
    if curr_cell_id == CELL_COUNT:
        DO_0.off()
        
    if curr_cell_id == CELL_COUNT and EMERGENCY_FLAG:
        packet_buffer.append(form_packet(EMERGENCY_PACKET_STATE))
        EMERGENCY_FLAG = False
    
    curr_cell_id %= CELL_COUNT
    
    curr_micom_add_id %= MICOM_RELAY_COUNT
    curr_micom_reg_register_id %= len(MICOM_REGULAR_REGISTER)
    curr_micom_emerg_add_id %= MICOM_RELAY_COUNT
    curr_DI_32_add_id %= DI_32_COUNT

def timeout_cell():
    global packet_form_state, curr_micom_reg_register_id, curr_micom_add_id, cell_reg_val, curr_cell_id, curr_micom_emerg_add_id, cell_emerg_map_view
    if packet_form_state == REGULAR_PACKET_STATE:
        reg_in_hex = convert_register_to_hex(
            MICOM_REGULAR_REGISTER[curr_micom_reg_register_id]
        )
        add = MICOM_REQUEST_LIST[curr_micom_add_id][curr_micom_reg_register_id][0]
        cell_reg_val[MICOM_CELL_NUMBER[add]][reg_in_hex] = 0x8000
    elif packet_form_state == EMERGENCY_PACKET_STATE:
        if (CELL_TYPE & 1 << curr_cell_id) >> curr_cell_id:
            add = MICOM_EMERG_REQUEST_LIST[curr_micom_emerg_add_id][0]
            cell_emerg_map_view[MICOM_CELL_NUMBER[add]] = 0x80
        else:
            for i in cell_emerg_map_view.keys():
                cell_emerg_map_view[i] = 0x80

def reset_timeout_cell():
    pass

def parse_emerg_val():
    global cell_emerg_map_view
    for i in range(DI_32_COUNT):
        response_32_bit_value = int.from_bytes(DI_32_val[DI_32_ADDRESSES[i]], "big")
        for j in range(len(DI_32_CELL_NUMBER[i])):
            mask = ((1 << DRY_CONTACT_PER_CELL_COUNT) - 1)
            number_of_shfits =  j * DRY_CONTACT_PER_CELL_COUNT
            cell_num = DI_32_CELL_NUMBER[i][j]
            cell_emerg_map_view[cell_num] = (response_32_bit_value & (mask << number_of_shfits)) >> number_of_shfits
        cell_emerg_map_view[TRANSFORMATTOR[i]] = (response_32_bit_value & (TRANSFORMATTOR_MASK << TRANSFORMATTOR_SHFIT)) >> TRANSFORMATTOR_SHFIT

def write_fail_packet(packet_in):
    File("unsended_packets.txt").write(f"{packet_in.decode()}\n", mode="a")

def form_packet(packet_type):
    packet = PACKET_START_CHARACTER
    
    packet += str(OBJECT_ID).encode()
    
    if packet_type == EMERGENCY_PACKET_STATE:
        packet += EMERGENCY_PACKET_TYPE
    elif packet_type == REGULAR_PACKET_STATE:
        packet += REGULAR_PACKET_TYPE
    else:
        packet += packet_type
        
    packet += str(time.time()).encode()
    packet += DATA_START_CHARACTER
    
    if packet_type == EMERGENCY_PACKET_STATE:
        for cell_number in sorted(cell_emerg_map_view.keys()):
            packet += str(cell_number).encode()
            packet += REGISTER_NUM_VALUE_DELIMITER
            packet += str(cell_emerg_map_view[cell_number]).encode()
            packet += DATA_DELIMITER
    if packet_type == REGULAR_PACKET_STATE:
        for cell_number in sorted(cell_reg_val.keys()):
            packet += str(cell_number).encode()
            packet += CELL_NUM_VALUE_DELIMITER
            for reg_num in sorted(cell_reg_val[cell_number].keys()):
                packet += reg_num.encode()
                packet += REGISTER_NUM_VALUE_DELIMITER
                packet += str(cell_reg_val[cell_number][reg_num]).encode()
                packet += REGISTER_DELIMITER
            packet += DATA_DELIMITER
        packet += DATA_END_CHARACTER
        packet += DATA_START_CHARACTER
        for name in dict_val.keys():
            packet += name.encode()
            packet += REGISTER_NUM_VALUE_DELIMITER
            packet += str(dict_val[name]).encode()
            packet += DATA_DELIMITER
    packet += DATA_END_CHARACTER
    packet += PACKET_END_CHARACTER
    return packet


# ------------------------------- END FUNCTION ------------------------------------

# ---------------------------------START-------------------------------------------

log = Logger(f"object_{OBJECT_ID}_main.log")
ms_RS = 0  # Задержка для передачи пакета RS-485

fl_ADS = False  # ADS в процессе

packet_form_state = EMERGENCY_PACKET_STATE  # идет ли формиравиние регулярного пакета
# EMERGENCY_PACKET_STATE - формиравиние регулярного пакета не идет
# REGULAR_PACKET_STATE - идет опрос регистров по ModBus
server_send_state = SERVER_SEND_NOT_STATE
#
is_wait_modbus_res = False
is_file_down = False
is_file_down_succ = False
is_clock_set = False
is_fail_msg_sended = False
is_sms_send = False
sended_rs_work = False
sended_rs_d_work = False
EMERGENCY_FLAG = False
#
dict_val = {}

# Modbus var
DI_32_REQUEST_TEMPLATE = bytearray([0x01, 0x03, 0x00, 0x63, 0x00, 0x02])

DI_32_REQUEST_LIST = []

for i in range(DI_32_COUNT):
    request = DI_32_REQUEST_TEMPLATE[:]
    request[0] = DI_32_ADDRESSES[i]
    crc = crc16(request)
    request += crc

    DI_32_REQUEST_LIST.append(request)

MICOM_REQUEST_TEMPLATE = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])

MICOM_REQUEST_LIST = []

for i in range(MICOM_RELAY_COUNT):
    MICOM_REQUEST_REG_LIST = []
    request = MICOM_REQUEST_TEMPLATE[:]
    request[0] = MICOM_RELAY_ADDRESSES[i]
    for reg_i in range(len(MICOM_REGULAR_REGISTER)):
        request[2] = MICOM_REGULAR_REGISTER[reg_i][0]
        request[3] = MICOM_REGULAR_REGISTER[reg_i][1]

        crc = crc16(request)
        MICOM_REQUEST_REG_LIST.append(request + crc)
    MICOM_REQUEST_LIST.append(MICOM_REQUEST_REG_LIST)

MICOM_EMERG_REQUEST_LIST = []

for i in range(MICOM_RELAY_COUNT):
    request = MICOM_REQUEST_TEMPLATE[:]
    request[0] = MICOM_RELAY_ADDRESSES[i]
    request[2] = MICOM_EMERGENCY_REGISTER[0]
    request[3] = MICOM_EMERGENCY_REGISTER[1]
    crc = crc16(request)
    MICOM_EMERG_REQUEST_LIST.append(request + crc)

convert_register_to_hex = lambda reg: "".join(f"{x:02x}" for x in reg)

error_code = 0x00

response = b""
# время передачи модбас запроса
last_time_modbus_req = time.ticks_ms()
# время передачи регулярного пакета
last_time_regular_packet = time.time()
# время для секундного цикла
last_time_sec = time.time()
# мигание
last_time_DO_0 = time.ticks_ms()
last_time_DO_1 = time.ticks_ms()

curr_cell_id = 0
curr_DI_32_add_id = 0
curr_micom_add_id = 0
curr_micom_reg_register_id = 0
curr_micom_emerg_add_id = 0

# Значения регистров и входов ModBus устройств
cell_reg_val = {}
cell_emerg_map_view = {}
for i in MICOM_CELL_NUMBER.values():
    cell_reg_val[i] = {}
    for j in range(len(MICOM_REGULAR_REGISTER)):
        reg_in_hex = convert_register_to_hex(MICOM_REGULAR_REGISTER[j])
        cell_reg_val[i][reg_in_hex] = 0xFFFF
for i in DI_32_CELL_NUMBER:
    for j in i:
        cell_emerg_map_view[j] = 0xFF
DI_32_val = {}
for i in DI_32_ADDRESSES:
    DI_32_val[i] = b'\xFF\xFF\xFF\xFF'

micom_emerg_val = {}
for i in MICOM_RELAY_ADDRESSES:
    micom_emerg_val[i] = b'\xFF\xFF'
cell_reg_res_fail_count = [0 for i in range(MICOM_RELAY_COUNT)]
cell_emerg_res_fail_count = [0 for i in range(DI_32_COUNT)]

packet_buffer = []

#
last_rs_state = 0
to_count = 0
# Pins
DO_0 = Pin(7, Pin.OUT)  # Pin Output
DO_1 = Pin(8, Pin.OUT)
DO_2 = Pin(9, Pin.OUT)
Pin(29, machine.Pin.IN)  # Напряжение с CPU
Pin_sys = ADC(3)  #
Pin_int_RS2 = Pin(28, Pin.IN, Pin.PULL_UP)  # Pin I2C --> Rs-485_2
Pin_RTC_INT = Pin(27, Pin.IN, Pin.PULL_UP)  # RTC DS3231

# LED
led = machine.Pin(25, machine.Pin.OUT)
fl_led = False
led.value(0)

log.log(log.INFO, uos.uname())  # Общая инфо об плате

# UART
Pin_RS0 = Pin(2, Pin.OUT)  # RS-485 RE/DE
Pin_RS0.value(0)

# Модули
(
    flash_gd,
    ds_sensor,
    oled,
    rtc_p,
    ads,
    rs_2,
    modem_var,
    modules_statuses,
) = init_modules()

# Выбираем RS порт
uart = None
if RS_PORT == 1:
    modules_statuses = modules_statuses & ~(1 << 7)
    uart = UART(0, tx=machine.Pin(0), rx=machine.Pin(1))
    uart.init(baudrate=19200, bits=8, parity=None, stop=1, timeout=30)

# Коммутатор каналов
addr_PCA = 0x41

display_general_info()

# Функции прерываний
# interrupt от таймера - мигаем Led
timer0 = Timer()
timer0.init(
    period=1000, mode=Timer.PERIODIC, callback=lambda t: led.value(not led.value())
)
# Базовые настройки модема
if modem_var.get_status():
    modem_var.init()
    log.log(log.INFO, "Modem are configured")

# Счетчик перезагрузок
attempt_count_file = File("attempt.txt", init_content=0)
attempts_count = int(attempt_count_file.read())
dict_val["reset"] = attempts_count
# Проверяем состояние модулей
if modules_statuses == 0:
    is_fail_msg_sended = True
    attempts_count = 0
elif attempts_count < 3:
    attempts_count += 1
    attempt_count_file.write(attempts_count, mode='w')
    log.log(log.ERROR, "Some modules are fail to start. Initiate restart process")
    machine.reset()

packet_form_state = REGULAR_PACKET_STATE

wdt = WDT(timeout=8000)  # enable it with a timeout of 8s

while True:
    if attempts_count >= 3 and not is_fail_msg_sended:
        packet_buffer.append(form_packet(REGULAR_PACKET_TYPE))

    if modules_statuses == 0 and is_sms_send:
        modem_var.send_sms("MESSAGE")
        log.log(
            log.ERROR,
            "Some modules are fail to start. 3 times attempted to restart system. Doesn't help.",
        )
        is_sms_send = False
    
    # RS-485 ключение режима передачи
    if Pin_RS0.value():
        if time.ticks_diff(time.ticks_ms(), ms_RS) <= 0:
            Pin_RS0.off()

    # Timeout для ответа по Modbus протоколу
    if (
        is_wait_modbus_res
        and time.ticks_diff(time.ticks_ms(), last_time_modbus_req)
        > MODBUS_WAIT_PERIOD_MILLIS
    ):
        timeout_cell()
        is_wait_modbus_res = False
        next_cell()

    # Рабочий цикл 1 сек
    if abs(time.time() - last_time_sec) >= 1:
        s = os.statvfs('/')
        if s[0]*s[3]/1024 < 500:
            with open("unsended_packets.txt",'w') as file:
                pass
            modem.clear_log()
            log.clear_log()
        display_general_info()
        last_time_sec = time.time()
    
    # Свободная ли линия ModBus
    if not is_wait_modbus_res:
        # Отправка запроса на Micom
        if packet_form_state == REGULAR_PACKET_STATE:
            DO_1.on()
            DO_0.off()
            
            send_modbus_req(MICOM_REQUEST_LIST[curr_micom_add_id][curr_micom_reg_register_id],Pin_RS0)
        elif packet_form_state == EMERGENCY_PACKET_STATE:
            DO_0.on()
            DO_1.off()
            
            if (CELL_TYPE & 1 << curr_cell_id) >> curr_cell_id:
                send_modbus_req(MICOM_EMERG_REQUEST_LIST[curr_micom_emerg_add_id], Pin_RS0)
            else:
                send_modbus_req(DI_32_REQUEST_LIST[curr_DI_32_add_id], Pin_RS0)
        last_time_modbus_req = time.ticks_ms()

    # Переход в режим отпроса регулярных регистров
    if abs(time.time() - last_time_regular_packet) >= REGULAR_PACKET_PERIOD_SECONDS:
        packet_form_state = REGULAR_PACKET_STATE
        last_time_regular_packet = time.time()

    # Опрос регулярных регистров закончен можно отпралять на сервер
    if  modem_var.get_status() and len(packet_buffer) > 0:
        modem_var.connect_to_server()

    # * * * * * * * * * * * Прием с UART портов * * * * * * * * * * *
    wdt.feed()
    # Получние ответа от модема
    data = modem_var.process_tcpip()

    # Успешное закрытие соединения с сервером
    if data and "IP INITIAL" in data:
        # Переход в опрос аварийных сигналов
        server_send_state = SERVER_SEND_NOT_STATE
        packet_form_state = EMERGENCY_PACKET_STATE
        packet_buffer = packet_buffer[1:]
        log.log(log.INFO, f"Parsed view of cells : {cell_emerg_map_view}")
        last_time_regular_packet = time.time()
    elif data and re.search(".*<.*>.*", data):
        error_code = error_code & ~(1 << 0)
        display_general_info()
        cmd_from_server = data[data.index("<") + 1 : data.rindex(">")]
        if "RESTART" in cmd_from_server:
            log.log(log.INFO, f"From server reciveid cmd : restart")
            attempts_count = int(attempt_count_file.read())
            attempts_count += 1
            attempt_count_file.write(attempts_count, mode='w')
            machine.reset()
        elif "SETTIME" in cmd_from_server:
            str_time = cmd_from_server[cmd_from_server.index(":") + 1 :]
            log.log(log.INFO, f"From server reciveid time: {str_time}")
            rtc_p.set_time(str_time)
            is_clock_set = True
        elif "F=" in cmd_from_server:
            file_name = cmd_from_server[cmd_from_server.index('=')+1:cmd_from_server.rindex('=')]
            file_content = cmd_from_server[cmd_from_server.index('=')+1:]
            log.log(log.INFO, f"From server reciveid new conf: name:{file_name}, content:{file_content}")
            new_conf_file = File(file_name + '.py')
            new_conf_file.write(file_content, mode='w')
            attempts_count = int(attempt_count_file.read())
            attempts_count += 1
            attempt_count_file.write(attempts_count, mode='w')
            machine.reset()
    elif data and "CONNECT FAIL" in data:
        error_code = error_code | (1 << 0)
        write_fail_packet(form_packet(packet_form_state))
        packet_form_state = EMERGENCY_PACKET_STATE
        server_send_state = SERVER_SEND_NOT_STATE
        log.log(log.ERROR, "Can't connect to server - CONNECT FAIL")
    elif data and to_count < 3 and "TIMEOUT" in data:
        log.log(log.INFO, "Can't connect to server - TIMEOUT")
        to_count += 1
        modem_var.connect_to_server()
    elif to_count >= 3:
        log.log(log.ERROR, "Can't connect to server - TIMEOUT more 3 times")
        to_count = 0
        error_code = error_code | (1 << 0)
        modem_var.current_at_cmd_index = 0
        display_general_info()
        write_fail_packet(form_packet(packet_form_state))
        packet_form_state = EMERGENCY_PACKET_STATE
        server_send_state = SERVER_SEND_NOT_STATE
        if not is_fail_msg_sended:
            is_sms_send = True
        last_time_regular_packet = time.time()
    # Модем ожидает данных для отправки на сервер
    elif data and ">" in str(data):
        if not is_fail_msg_sended:
            log.log(log.ERROR, "Some modules are fail to start. Send message to server")
            is_fail_msg_sended = True
        modem_var.send_number(packet_buffer[0])
        modem_var.send_data()
        log.log(log.INFO, f'Send packet - {packet_buffer[0]}')

    # Проверка на наличие ответа от подключенных устройств по Modbus протоколу
    if uart.any():
        start_time = time.ticks_ms()

        # Чтение всех данных если не все были прочитанны с первой попытки
        while uart.any() and time.ticks_diff(time.ticks_ms(), time.ticks_ms()) < 5:
            # Чтение ответа от подключенных устройств по Modbus протоколу
            response += uart.readline()

        # Проверка на валдность адрес в ответе, валидность фун. кода, правильно ли расчитан crc
        if not (
            check_crc(response)
            and response[1] < 129
            and (response[0] in DI_32_ADDRESSES or response[0] in MICOM_RELAY_ADDRESSES)
        ):
            log.log(log.INFO, f"Error from modbus: {response}")
            response = b""
            continue
        
        if packet_form_state == REGULAR_PACKET_STATE and response[0] in MICOM_RELAY_ADDRESSES:
            reg_in_hex = convert_register_to_hex(
                MICOM_REGULAR_REGISTER[curr_micom_reg_register_id]
            )
            cell_reg_val[MICOM_CELL_NUMBER[response[0]]][reg_in_hex] = int.from_bytes(
                response[3 : 3 + response[2]], "big"
            )
            log.log(log.INFO, f"Response from modbus: {response}")
        elif packet_form_state == EMERGENCY_PACKET_STATE:
            if response[0] in MICOM_RELAY_ADDRESSES and (CELL_TYPE & 1 << curr_cell_id) >> curr_cell_id and  micom_emerg_val[response[0]] != response[3 : 3 + response[2]]:
                micom_emerg_val[response[0]] = response[3 : 3 + response[2]]
                response_16_bit_value = int.from_bytes(micom_emerg_val[response[0]], 'big')
                right_first_bit = (response_16_bit_value & (1 << 15 * 1)) >> 15 * 1
                left_last_four_bit = (response_16_bit_value & (15 << 0 * 4)) >> 0 * 4
                cell_emerg_map_view[MICOM_CELL_NUMBER[response[0]]] = (right_first_bit << 4) + left_last_four_bit
                EMERGENCY_FLAG = True
                log.log(log.INFO, f"Parsed view of cells : {cell_emerg_map_view}")
                log.log(log.INFO, f"Response from modbus: {response}")
            elif response[0] in DI_32_ADDRESSES and not ((CELL_TYPE & 1 << curr_cell_id) >> curr_cell_id) and DI_32_val[response[0]] != response[3 : 3 + response[2]]:
                DI_32_val[response[0]] = response[3 : 3 + response[2]]
                parse_emerg_val()
                EMERGENCY_FLAG = True
                log.log(log.INFO, f"Parsed view of cells : {cell_emerg_map_view}")
                log.log(log.INFO, f"Response from modbus: {response}")
        # При успешном ответе переходим на следующие устройство
        if response is not None:
            next_cell()
            reset_timeout_cell()
            response = b""
            is_wait_modbus_res = False
        else:
            log.log(log.ERROR, "No response received.")
    wdt.feed()
# ================================= END ================================

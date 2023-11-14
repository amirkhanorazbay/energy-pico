from library.constants import *
from obj_name_dev import obj_name_dev
from obj_name_conf import obj_name_conf

OBJECT_ID = obj_name_conf['OBJECT_ID']
DI_32_COUNT = len(obj_name_dev['block'])
DI_32_ADDRESSES = [i for i,j in obj_name_dev['block']]
DI_32_CELL_NUMBER = [j[:-1] for i,j in obj_name_dev['block']]

MICOM_RELAY_COUNT = len([i for i,j in obj_name_dev['cell'] if i != 0])
MICOM_RELAY_ADDRESSES = [i for i,j in obj_name_dev['cell'] if i != 0]

MICOM_CELL_NUMBER = {key:value for (key,value) in obj_name_dev['cell'] if key != 0}

TRANSFORMATTOR = [j[-1] for i,j in obj_name_dev['block']]
TRANSFORMATTOR_SHFIT = 30
TRANSFORMATTOR_MASK = (1 << 0x2) - 1

CELL_COUNT = obj_name_conf['CELLS']
CELL_TYPE = obj_name_conf['CELL_TYPE']
MAX_CELL_COUNT = 24

MICOM_EMERGENCY_REGISTER = [0x00, 0x0C]

MICOM_REGULAR_REGISTER = [
    bytearray([0x00, 0x30]),
    bytearray([0x00, 0x31]),
    bytearray([0x00, 0x32]),
    bytearray([0x00, 0x33]),
    bytearray([0x00, 0x34]),
    bytearray([0x00, 0x35]),
    bytearray([0x00, 0x36]),
    bytearray([0x00, 0x37]),
    bytearray([0x00, 0x3B])
]

DRY_CONTACT_PER_CELL_COUNT = 5

EMERGENCY_PHONE = obj_name_conf['TLF']

RS_PORT = 1

SERVER_IP = obj_name_conf['IP']
PORT = obj_name_conf['PORT']

START_TIME = "<SETTIME:+CCLK:  23/08/38,17:48:00>"

REGULAR_PACKET_PERIOD_SECONDS = obj_name_conf['Cicle']
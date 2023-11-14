import library.w25qxx as w25qxx
from library.logger import Logger
from config import OBJECT_ID

# + + + + + + + + + + + + Flash + + + + + + + + + + + + + +
class Flash:
    
    def __init__(self, spi, cs_GD64):
        try:
            self.flash_gd = w25qxx.W25QXX_BlockDev(SPI=spi, CS_PIN=cs_GD64)
            self.log = Logger(f'object_{OBJECT_ID}_flash.log')
        except Exception as e:
            raise e
    
    # Очистка сектора
    def clear_sector(num):
        if self.flash_gd.__eraseSector(num):
            return True

    # Очистка всей Flash
    def clear_flash():
        return self.flash_gd.eraseChip()

    # Запись данных во flash в сектор с чисткой, <= 256 bytes
    def write_flash(block_num, buff):
        if clear_sector(block_num):
            print("Sector: Clear!")
        self.flash_gd.writeblocks(block_num, buff)
        return True

    # Чтение данных из flash, < 256 bytes
    def read_flash(block_num, buff):
        self.flash_gd.readblocks(block_num, buff)
        return buff

    # Чтение из Flash
    def Read_Flash(block_num, buff):
        buf = bytearray(len(buff))
        read_flash(block_num, buf)
        return buf

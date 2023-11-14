# Base driver for sc16is7xx
# Copyright (ivp) 2022 

from time import ticks_ms

class SC16IS7XX:
    DEFAULT_CRYSTAL_FREQ = 14_745_600
    DEFAULT_I2C_ADDRESS = 0x4a

    # Register, use 3 left shift ( << 3) to get actually value for i2c write
    # -- General Registers
    REG_RHR = 0x00  # Receive Holding Register (R)
    REG_THR = 0x00  # Transmit Holding Register (W)
    REG_IER = 0x01  # Interrupt Enable Register (R/W)
    REG_FCR = 0x02  # FIFO Control Register (W)
    REG_IIR = 0x02  # Interrupt Identification Register (R)
    REG_LCR = 0x03  # Line Control Register (R/W)
    REG_MCR = 0x04  # Modem Control Register (R/W)
    REG_LSR = 0x05  # Line Status Register (R)
    REG_MSR = 0x06  # Modem Status Register (R)
    REG_SPR = 0x07  # Scratchpad Register (R/W)
    REG_TCR = 0x06  # Transmission Control Register (R/W)
    REG_TLR = 0x07  # Trigger Level Register (R/W)
    REG_TXLVL = 0x08  # Transmit FIFO Level Register (R)
    REG_RXLVL = 0x09  # Receive FIFO Level Register (R)
    REG_IODIR = 0x0A  # I/O pin Direction Register (R/W)
    REG_IOSTATE = 0x0B  # I/O pin States Register (R)
    REG_IOINTENA = 0x0C  # I/O Interrupt Enable Register (R/W)
    REG_IOCONTROL = 0x0E  # I/O pins Control Register (R/W)
    REG_EFCR = 0x0F  # Extra Features Register (R/W)

    # -- Special Register Set (Requires LCR[7] = 1 & LCR != 0xBF to use)
    REG_LCR7_DLL = 0x00  # Divisor Latch LSB (R/W)
    REG_LCR7_DLH = 0x01  # Divisor Latch MSB (R/W)

    # -- Enhanced Register Set (Requires LCR = 0xBF to use)
    REG_LCR_0XBF_EFR = 0x02  # Enhanced Feature Register (R/W)
    REG_LCR_0XBF_XON1 = 0x04  # XOn Nr.1 Word (R/W)
    REG_LCR_0XBF_XON2 = 0x05  # XOff Nr.1 Word (R/W)
    REG_LCR_0XBF_XOFF1 = 0x06  # XOn Nr.2 Word (R/W)
    REG_LCR_0XBF_XOFF2 = 0x07  # XOff Nr.2 Word (R/W)

    # Parity
    PARITY_NONE = None
    PARITY_ODD = 0
    PARITY_EVEN = 1
    PARITY_FORCE_ONE = 2
    PARITY_FORCE_ZERO = 3
    
    _hRegLCR = 0x00
    _bLineSet = False
    _timeout = 10

    def __init__(self, bus, address_or_ss=None, baudrate=19200, debug = True, crystalfreq = DEFAULT_CRYSTAL_FREQ):
        self._bus = bus
        self._crystalfreq = crystalfreq
        self._debug = debug

        if type(self._bus).__name__ == "I2C":
            self._addr = self.DEFAULT_I2C_ADDRESS
            self._read_reg = self._read_reg_i2c
            self._write_reg = self._write_reg_i2c
        else:
            raise ValueError("Unsupported bus {}".format(type(self._bus).__name__))

        self._reset()
        self._fifoenable(True)
        self.init(baudrate)
        self.write_reg_IER()
        self.write_reg_RS485()
        

    def _debugmsg(self, msg):
        if not self._debug: return
        print("{}: {}".format(type(self).__name__, msg))

    def _read_reg_i2c(self, reg, count = 1):
        val = self._bus.readfrom_mem(self._addr, reg << 3, count)
        self._debugmsg("Read i2c reg {}: value {}".format(hex(reg), hex(val[0])))
        return bytearray(val)

    def _write_reg_i2c(self, reg, val):
        self._debugmsg("Write i2c reg {}: value {}".format(hex(reg), hex(val[0])))
        self._bus.writeto_mem(self._addr, reg << 3, val)
    
    def _reset(self):
        reg = self._read_reg(self.REG_IOCONTROL)
            
    def _fifoenable(self, enable):
        reg = self._read_reg(self.REG_FCR)
        if enable: reg[0] |= 0x01
        else: reg[0] &= ~0x01
        self._write_reg(self.REG_FCR, reg)

    def _setbaudrate(self, baudrate=115200):
        prescaler = None
        tmp = self._read_reg(self.REG_MCR)
        prescaler = 4 if tmp[0] & 0x80 else 1
        divisor = (self._crystalfreq / prescaler) / (baudrate * 16)
        tmplcr = self._read_reg(self.REG_LCR)
        tmplcr[0] |= 0x80  # Divisor Latch enable (bit 7) - Allow access to DLL and DHL registers
        self._write_reg(self.REG_LCR, tmplcr)
        # Write new baudrate
        msg = bytearray()
        msg.append(int(divisor) & 0xFF)
        self._write_reg(self.REG_LCR7_DLL, msg)
        msg1 = bytearray()
        msg1.append((int(divisor)>>8) & 0xFF)
        self._write_reg(self.REG_LCR7_DLH, msg1)

        tmplcr[0] &= ~0x80  # Divisor Latch disable (bit 7)
        self._write_reg(self.REG_LCR, tmplcr)
        actual_baudrate = (self._crystalfreq/prescaler)/(16*divisor)
        error = (actual_baudrate-baudrate)*1000.0/baudrate

        self._debugmsg("Desired baudrate: {}, Calculated divisor: {}, Actual baudrate: {}, Baudrate error: {}"
                .format(baudrate, divisor, actual_baudrate, error))
        return error

# Уставки COM порта
    def _setline(self, bits = 8, parity = PARITY_NONE, stopbits = 1):
        reg = self._read_reg(self.REG_LCR)
        reg[0] &= 0xC0  # Clear actual settings
        if bits == 5:
            reg[0] |= 0x00
        elif bits == 6:
            reg[0] |= 0x01
        elif bits == 7:
            reg[0] |= 0x02
        elif bits == 8:
            reg[0] |= 0x03
        else:
            raise ValueError("Error data length {} is not supported. Supported 5, 6, 7, 8".format(bits))

        if stopbits not in [1, 1.5, 2]:
            raise ValueError("Stop bits {} is not supported. Supported 1, 1.5, 2".format(stopbits))
        if stopbits > 1:
            reg[0] |= 0x04
            
        if parity == self.PARITY_NONE:
            reg[0] |= 0x00
        elif parity == self.PARITY_ODD:
            reg[0] |= 0x08
        elif parity == self.PARITY_EVEN:
            reg[0] |= 0x18
        elif parity == self.PARITY_FORCE_ONE:
            reg[0] |= 0x28
        elif parity == self.PARITY_FORCE_ZERO:
            reg[0] |= 0x38
        else:
            ValueError("Invalid parity {} Use {}.PARITY_*".format(parity, type(self).__name__))

        self._write_reg(self.REG_LCR, reg)

# Инициализация COM порта
    def init(self, baudrate=9600, bits=8, parity=None, stop=1, *, timeout=30):
        self._timeout = timeout
        self._setline(bits, parity, stop)
        self._setbaudrate(baudrate)
        
# Установка режима RS-485
    def write_reg_RS485(self):
        reg = self._read_reg(self.REG_EFCR)
        #print('reg RS-485:', reg)
        reg[0] = 0x31
        self._write_reg(self.REG_EFCR, reg)

# Установка режима прерываний reg: IER
    def write_reg_IER(self):
        reg = self._read_reg(self.REG_IER)
        #print('reg IER:', reg)
        reg[0] = 0x02
        #print('reg IER:', reg)
        
        self._write_reg(self.REG_IER, reg)
        reg = self._read_reg(self.REG_IIR)
        #print('reg IIR:',reg)
        return reg
    
# Чтение reg: IIR прерываний
    def read_reg_IIR(self):
        reg = self._read_reg(self.REG_IIR)
        return reg
  
# Чтение - сколько байт в RX FIFO
    def any(self):				
        reg = self._read_reg(self.REG_RXLVL)
        return int(reg[0])
    
# Чтение буфера RX
    def read(self, nbytes=None):
        bufferlen = 0
        start = ticks_ms()
        while bufferlen == 0:
            bufferlen = self.any()
            if ticks_ms() > start + self._timeout: return None
        if nbytes is not None:
            bufferlen = nbytes if nbytes < bufferlen else bufferlen
        return bytes(self._read_reg(self.REG_RHR, bufferlen))

# Запись Byte и TX
    def write(self, data):
        self._write_reg(self.REG_THR, data)
  

    def readinto(self, buf, nbytes=None):
        raise NotImplementedError()

    def readline(self):
        raise NotImplementedError()


    def sendbreak(self):
        raise NotImplementedError()


    def irq(self, prio, handler):
        raise NotImplementedError()

    def GetLineStatus(self):
        reg = self._read_reg(self.REG_LSR)
        return int(reg[0])
    
# Размер TX FIFO
    def col_tx_FIFO(self):
        reg = self._read_reg(self.REG_TXLVL)
        return int(reg[0])
    
# Установка доступа к Enhanced Register
    def _ExposeEnhancedRegisterSet(self, bExposeRegisterSet):
        # -- Expose the Register Set
        if ((bExposeRegisterSet == True) and (self._hRegLCR == 0x00)):
            # -- Read in the current LCR register
            _hRegLCR = self._read_reg(self.REG_LCR)
            # -- Save the current LCR register state
            self._hRegLCR = _hRegLCR

            # -- Enable Enhanced Feature Register with LCR = 0xBF
            self._write_reg(self.REG_LCR, 0xbf)

        elif ( (bExposeRegisterSet == False) and (self._hRegLCR != 0x00) ):
            # -- Retrieve the prior LCR register state
            _hRegLCR = self._hRegLCR
            self._hRegLCR = 0x00

            # -- Restore the LCR Register with to the previous state
            self._write_reg(self.REG_LCR, _hRegLCR)

        else:
            # -- Something was in the wrong state...
            return False

        # -- If everything worked, return True
        return True
    
    
    def _EnableEnhancedFunctionSet(self, bEnableAdvancedSet):
        # -- Enable Enhanced Register access
        if (self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ): return False
        # -- Read the EFR register
        _hRegValue = self._read_reg(self.REG_LCR_0XBF_EFR)
        # -- Enable/Disable Enhanced Function Set with EFR[4]
        if (bEnableAdvancedSet == True):
            _hRegValue |= 0x10
        else:
            _hRegValue &= 0xef
        # -- Write out the modified EFR register
        self._write_reg(self.REG_LCR_0XBF_EFR, _hRegValue)
        # -- Disable Enhanced Register access
        if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ): return False
        return True

# Проверка Sc16 Ping  
    def Ping(self):	
        msg = bytearray()
        msg.append(0xAA)
        self._write_reg(self.REG_SPR, msg)
        reg = self._read_reg(self.REG_SPR)
        if reg[0] == 0xAA: return True
        else: return False
        
# Чтение reg IIR
    def read_IIR(self):	
        reg = self._read_reg(self.REG_IIR)
        print('reg IIR:', reg)
        return int(reg[0])
    
# Установка FIFO    
    def SetFifo(self, bFifoEnable = True, iRxFifo = 0, iTxFifo = 0):
        # iRxFifo 0,1,2,3, iTxFifo 0,1,2,3
        flEnhanced = False
        _hRegFCR = self._read_reg(self.REG_FCR)
        if (bFifoEnable == False):
            self._write_reg(REG_FCR, 0x00)
            return True
        _hRegFCR |= 0x01
        _hRegFCR |= (iRxFifo << 6)
        if flEnhanced: _hRegFCR |= (iTxFifo << 4)
        self._write_reg(REG_FCR, _hRegFCR)
        return True
# * * * * * * * * * * Режимы сна * * * * * * * * * *  
    def GetSleepState(self):
        _hRegIER = self._read_reg(self.REG_IER)
        if ((int(_hRegIER) & (1 << 4)) > 0):		# sleep mode bit is set on IER[4]
            return True
        else: return False

    def SetSleepState(self, bDiscardRxBuffer = False):
        '''
        # -- Check that there is no data is in the RX buffer
        if (self.RxFifoBufferUsed() > 0 ):
            print("SetSleepState: Data present in RX buffer; Cannot sleep now.")
            return False
        # -- Check that there is no data is in the TX buffers
        if ( GetLineStatus()['thr-tsr-empty'] == False ):
            print("SetSleepState: Data present in TX hold or send buffers; Cannot sleep now.")
            return False
        # -- Enable enhanced function mode on EFR[4]
        #if (self._EnableEnhancedFunctionSet(bEnableAdvancedSet = True) == False):	return False
        '''
        _hRegIER = self._read_reg(self._REG_IER)
        _hRegIER |= 0x10
        self._write_reg(self.REG_IER, _hRegIER)
        return True
    
    def SetWakeState(self):
        if (self.GetSleepState() == False):
            self._debugmsg("SetWakeState: UART not sleeping and already awake.")
            return True
        _hRegIER = self._read_reg(self._REG_IER)
        _hRegIER &= 0xef
        self._write_reg(self.REG_IER, _hRegIER)
        return True
# = = = = = = = = = = = = = = = = = = = = = = = = =

# Отказ от контроля потоков
    def SetNoFlowcontrol(self):
        # -- Enable Enhanced Register access
        if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ):	return False
        # -- Read the EFR register
        _hRegEFR = self._read_reg(self.REG_LCR_0XBF_EFR)
        # -- Disable all bits in EFR except EFR[4:5]
        _hRegEFR &= 0x10
        # -- Write out the modified EFR register
        self._write_reg(self.REG_LCR_0XBF_EFR,_hRegEFR)
        
        self._write_reg(self.REG_LCR_0XBF_XON1, 0x00)
        self._write_reg(self.REG_LCR_0XBF_XON2, 0x00)
        self._write_reg(self.REG_LCR_0XBF_XOFF1, 0x00)
        self._write_reg(self.REG_LCR_0XBF_XOFF2, 0x00)

        # -- Disable Enhanced Register access
        if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ):	return False
        return True

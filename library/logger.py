import time

from library.file import File

class Logger:
    
    LOG_LEVELS = [
        'FATAL',
        'ERROR',
        'WARN',
        'INFO'
    ]
    
    FATAL = 0
    ERROR = 1
    WARN = 2
    INFO = 3  

    def __init__(self, log_file_name):
        year, month, mday, hour, minute, second, weekday, yearday = time.localtime()
        self.log_file = File(f'{mday:02}.{month:02}.{year:02}' + log_file_name)
    
    def log(self, log_level, text, is_print=True, is_file=False):
        year, month, mday, hour, minute, second, weekday, yearday = time.localtime()
        msg = f'[{mday:02}.{month:02}.{year:02} {hour:02}:{minute:02}:{second:02}][{self.LOG_LEVELS[log_level]}] {text}'
        if self.log_file.file_name[0:10] != f'{mday:02}.{month:02}.{year:02}':
            self.log_file.file_name = f'{mday:02}.{month:02}.{year:02}' + self.log_file.file_name[10:]
        
        if is_print:
            print(msg)
        
        if is_file or log_level in range(0,3):
            self.log_file.writeln(msg)
    
    def clear_log(self):
        with open(self.log_file.file_name,'w') as file:
            pass

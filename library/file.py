import time

class File:
    
    def __init__(self, file_name, init_content=""):
        self.file_name = file_name
        # TODO
        # check if file exeist
        # create file if not
        try:
            f = open(self.file_name, "r")
            f.close()
        except Exception:
            self.write(str(init_content))
        
    def write(self, data, mode='a'):
        with open(self.file_name, mode) as f:
            f.write(str(data))
            f.close()
    
    def writeln(self, data, mode='a'):
        with open(self.file_name, mode) as f:
            f.write(str(data) + "\n")
            f.close()
            
    # Чтение файла в RP
    def read(self, mode='r'):
        with open(self.file_name, mode) as f:
            s = f.read()
            f.close()
        return s

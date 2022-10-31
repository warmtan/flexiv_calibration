
__all__=[
    "Base_Logging"
]


class Base_Logging:
    """outout info:
       error:
       debug:
       info:
       warning
       """
    def __init__(self):
        self.logger_name="no logger name"
        self.LOG_LEVEL="debug"

    def init_lib_logger(self,
                        logger_name="no logger name",
                        LOG_LEVEL="info"):
        self.logger_name=logger_name
        self.LOG_LEVEL=LOG_LEVEL
        return self

    def init_app_logger(self,LOG_LEVEL="info"):
        self.LOG_LEVEL=LOG_LEVEL
        self.logger_name="app_log"
        return self

    def info(self,info):
        if self.LOG_LEVEL=="info":
            print(self.logger_name+info)
        else:
            pass

    def error(self,error):
        if self.LOG_LEVEL=="error":
            print(self.logger_name+error)
        else:
            pass

    def debug(self,debug):
        if self.LOG_LEVEL=="error":
            print(self.logger_name+debug)
        else:
            pass

    def warning(self,warning):
        if self.LOG_LEVEL=="warning":
            print(self.logger_name+warning)
        else:
            pass

Logging=Base_Logging()

    

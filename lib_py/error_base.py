__all__=[
    "ErrorBase",
]

class ErrorBase:
    def __init__(self,lib="ethernet_robot"):
        self.lib=lib
        self.ok=0
    
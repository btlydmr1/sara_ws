import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/betul/Downloads/SARA/sara_ws/install/sara_control'

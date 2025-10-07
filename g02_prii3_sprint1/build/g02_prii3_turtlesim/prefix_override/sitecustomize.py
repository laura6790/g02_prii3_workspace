import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/laura/Descargas/g02_prii3_workspace-master(1)/install/g02_prii3_turtlesim'

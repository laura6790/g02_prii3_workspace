import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/laura/Imágenes/g02_prii3_workspace-master/install/g02_prii3_turtlesim'

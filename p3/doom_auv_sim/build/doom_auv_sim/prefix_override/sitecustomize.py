import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sergio/Escritorio/aereos y sub/p3/doom_auv_sim/install/doom_auv_sim'

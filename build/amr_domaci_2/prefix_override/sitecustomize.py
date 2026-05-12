import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cevilko/amr_dz2_ws/amr_domaci_2/install/amr_domaci_2'

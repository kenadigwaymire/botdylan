import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/sf_ME_133a/packages/botdylan/install/botdylan'

# ----------------------------------------------
#
#           SCURM_Vision_Module_Debugs
#               Coding By Pikachu
#                  调试信息输出
#
#            LAST_UPDATE:OCT30/2020
#
# ----------------------------------------------
import time
from . import GlobalConfig as config

# ----------------------------------------------
debug_dat = ['INFOS', 'WARNS', 'ERROR', 'PANIC']


# ----------------------------------------------
def debug_log(in_heads, in_texts, in_level=0, in_lines="\n"):
    if config.global_debug_flag:
        debug_log_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print("[" + debug_log_time + "][" + debug_dat[in_level] + "][" + in_heads.ljust(6, '-') + "]", in_texts,
              end=in_lines)
        return True
    return False

#!/usr/bin/env python
# -*- coding: UTF-8 -*

import linuxcnc
from linuxcnc_control import  LinuxcncControl
from machinekit import hal

class LongTimeTest:
    
    def __init__(self):
        self.e = LinuxcncControl(1) # 設定 timeout 為 1 秒， for linuxcnc.command.wait_complete(timeout)
        self.e.g_raise_except = False
        self.init_hal()

    def machine_on_and_home_all(self):
        self.e.set_mode(linuxcnc.MODE_MANUAL)
        self.e.set_state(linuxcnc.STATE_ESTOP_RESET)# 異常復歸
        self.e.set_state(linuxcnc.STATE_ON)         # Servo ON

        if (self.e.is_all_homed() == False):        # 未作 homing ，就執行 home all 
            self.e.do_home(-1)
            while(self.e.is_all_homed()==False):    # 等待 homing 完成
                pass
        
        if (self.e.ok_for_mdi() == False):          # 不能下達 G code 就離開
            print "ERROR: not ready for MDI, quit ..."
            quit()
        else:
            self.e.prepare_for_mdi()

    def init_hal(self):
        # 訊號連接此處增加
        h = hal.Component("python-ui")
        h.ready() # mark the component as 'ready'

    def do_open_ngc(self):
        self.e.s.poll()                             # 更新
        self.e.open_program("./nc_files/60hr.ngc")  # 開啟 ngc file
        self.e.run_full_program()                   # 執行 G code
        self.e.wait_running()                       # 等待執行完畢

def main():
    long_time = LongTimeTest()          # 初始設定
    long_time.machine_on_and_home_all() # 先 Servo ON 再做 home all
    long_time.do_open_ngc()             # 讀取檔案，執行 G code

if __name__ == "__main__": main()
            

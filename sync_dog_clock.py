#!/usr/bin/env python3
"""把狗的系统时钟同步到开发机（狗无 RTC/NTP 时 TF 时间戳会对不上）。

用法：python3 sync_dog_clock.py
前置：狗可达（192.168.123.18 或改 HOST）。
"""
import sys
import time

import paramiko

HOST = '192.168.123.18'
USER = 'unitree'
PWD = '123'


def main():
    dev_epoch = int(time.time())
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=5)
    _, out, _ = c.exec_command('date +%s')
    dog_epoch = int(out.read().decode().strip())
    diff = dev_epoch - dog_epoch
    print('开发机: %d  狗: %d  差: %d s' % (dev_epoch, dog_epoch, diff))
    if abs(diff) < 2:
        print('时钟已同步，无需处理')
        c.close()
        return
    _, out, err = c.exec_command(
        'echo %s | sudo -S date -s @%d' % (PWD, dev_epoch))
    print(out.read().decode().strip() or err.read().decode().strip())
    _, out, _ = c.exec_command('date')
    print('狗现在:', out.read().decode().strip())
    c.close()


if __name__ == '__main__':
    sys.exit(main())

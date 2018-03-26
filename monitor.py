#!/usr/bin/python

import Adafruit_CharLCD as LCD
from subprocess import *
from time import sleep, strftime
from datetime import datetime
import os 


# Raspberry Pi pin setup
lcd_rs = 25
lcd_en = 24
lcd_d4 = 23
lcd_d5 = 17
lcd_d6 = 21
lcd_d7 = 22
lcd_backlight = 2

# Define LCD column and row size for 16x2 LCD.
lcd_columns = 16
lcd_rows = 2

lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows, lcd_backlight)

cmd = "ip addr show wlan0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1"

# Return CPU temperature as a character string                                      
def getCPUtemperature():
    res = os.popen('cat /sys/class/thermal/thermal_zone0/temp').readline()
    return(res.replace("temp=","").replace("'C\n",""))

# Return RAM information (unit=kb) in a list                                        
# Index 0: total RAM                                                                
# Index 1: used RAM                                                                 
# Index 2: free RAM                                                                 
def getRAMinfo():
    p = os.popen('free')
    i = 0
    while 1:
        i = i + 1
        line = p.readline()
        if i==2:
            return(line.split()[1:4])


# Return % of CPU used by user as a character string                                
def getCPUuse():
    return(str(os.popen("top -n1 | awk '/Cpu\(s\):/ {print $2}'").readline().strip('\n')))

# Return information about disk space as a list (unit included)                     
# Index 0: total disk space                                                         
# Index 1: used disk space                                                          
# Index 2: remaining disk space                                                     
# Index 3: percentage of disk used                                                  
def getDiskSpace():
    p = os.popen("df -h /")
    i = 0
    while 1:
        i = i +1
        line = p.readline()
        if i==2:
            return(line.split()[1:5])

#lcd.begin(16,1)

# Return GPU temperature as a character string                                      
def getGPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=","").replace("'C\n",""))


def run_cmd(cmd):
        p = Popen(cmd, shell=True, stdout=PIPE)
        output = p.communicate()[0]
        return output

while 1:
        lcd.clear()
        ipaddr = run_cmd(cmd)
		
#        lcd.message(datetime.now().strftime('%b %d  %H:%M:%S\n'))
#        lcd.message('IP %s' % ( ipaddr ) )
        lcd.message(datetime.now().strftime('%b %d  %H:%M:%S\n'))
        lcd.message('IP %s' % ( ipaddr ) )
        sleep(2)

        strc = getCPUtemperature()
        T = [int(s) for s in strc.split() if s.isdigit()][0]
        T = T/1000
        CPU = T
		
        lcd.clear()
        lcd.message('CPU: %sC\n' % ( CPU ) )
        lcd.message('Use: %s' % ( getCPUuse() ) )
        sleep(2)

        a = [int(s) for s in getRAMinfo() if s.isdigit()][1]
        b = [int(s) for s in getRAMinfo() if s.isdigit()][0]
        c = float(a)/float(b)*100
        RAM = c
        lcd.clear()
        lcd.message('GPU %s C\n' % ( getGPUtemperature() ) )
        lcd.message('RAM Use  %.4s%%' % ( RAM ))
        sleep(2)

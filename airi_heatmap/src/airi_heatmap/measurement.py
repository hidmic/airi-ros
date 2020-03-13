#!/usr/bin/env python

import subprocess
from airi_heatmap.utils import log

#sudo nmcli -f SSID,BSSID,CHAN,FREQ,RATE,SIGNAL,SECURITY dev wifi rescan
#sudo nmcli -f SSID,BSSID,CHAN,FREQ,RATE,SIGNAL,SECURITY dev wifi

def get_measure(model):
    if model is 'MacOS':
        measure = subprocess.check_output(['airport', '-s'])
    elif model is 'Raspi':
        measure = subprocess.check_output(['airport', '-s'])
    elif model is 'Linux':
        subprocess.check_output(['sudo', 'nmcli', '-f', 'SSID,BSSID,CHAN,FREQ,RATE,SIGNAL,SECURITY', 'dev', 'wifi', 'rescan'])
        measure = subprocess.check_output(['sudo', 'nmcli', '-f', 'SSID,BSSID,CHAN,FREQ,RATE,SIGNAL,SECURITY', 'dev', 'wifi'])
    else:
        log('heatmap','ERROR','System not recognized.')
    return bash_to_list(measure.decode('utf-8'))

def bash_to_list(measure):
    measure_list = []
    for wifi_measure in measure.splitlines():
        wifi_measure_element = wifi_measure.split()
        for text in wifi_measure_element:
            if ':' in text and wifi_measure_element.index(text) != 1:
                for i in reversed(range(1, wifi_measure_element.index(text))):
                    wifi_measure_element[i-1] = wifi_measure_element[i-1] + ' ' + wifi_measure_element.pop(i)
                break
        # find returns -1 when there's no match
            if (wifi_measure_element[-2].find('WPA') != -1):
                wifi_measure_element[-1] = '-'.join((wifi_measure_element.pop(-2), wifi_measure_element[-1]))
            if wifi_measure_element[-5].find('MHz') != -1:
                wifi_measure_element.pop(-5)
            if wifi_measure_element[-3].find('Mbit/s') != -1:
                wifi_measure_element.pop(-3)
        measure_list.append(wifi_measure_element)
    # Delete headers
    del measure_list[0]
    return measure_list

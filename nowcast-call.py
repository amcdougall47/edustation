import requests
import time
import os, sys
from os import path

t = time.localtime()
current_time = time.strftime("%Y,%m,%d,%H:%M:%S", t)

response = requests.get("http://api.erg.kcl.ac.uk/airquality/Data/Nowcast/lat=51.494572/lon=-0.086570/Json")

data = response.text

sys.stdout = open("site1.txt","a+")
print(current_time,',',data)
# print('\n')
sys.stdout.close()

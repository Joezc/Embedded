'''
    download the map file from the server, decode to json and write to file.
'''
from urllib.request import urlopen
import json

buildingname = input('input the building name:')
levelno = input('input the level:')

url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building="+buildingname+"&Level="+levelno

with urlopen(url) as response:
    html = response.read().decode("utf-8")

map1 = json.loads(html)

print('the node number is : %d' % len(map1['map']))

filename = input('input the file name:')

try:
    f = open(filename, 'w')
    f.write(html)
    f.close()
except:
    print('write to file fails')
    pass

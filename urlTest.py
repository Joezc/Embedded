from urllib.request import urlopen
import json

buildingname = input('input the building name:')
levelno = input('input the level:')

url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building="+buildingname+"&Level="+levelno

with urlopen(url) as response:
    html = response.read().decode("utf-8")

#print(html)

map1 = json.loads(html)

print('the node number is :')
print(len(map1['map']))
filename = input('input the file name:')

f = open(filename, 'w')
f.write(html)
f.close()

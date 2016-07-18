'''
    download the map file from the server, decode to json and write to file.
'''
from urllib.request import urlopen
import json
import IOwithSpeak
import os

def loadMap():
    buildingname = IOwithSpeak.inputWithVoice('Please input the building ID:')
    levelno = IOwithSpeak.inputWithVoice('Please input the level:')

    filename = buildingname+'_'+levelno
    path = r'/home/lzc/Workspace/Git/embedded/map/'+filename
    if os.path.isfile(path):
        pass
    else:
        try:
            url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building="+buildingname+"&Level="+levelno

            try:
                with urlopen(url) as response:
                    html = response.read().decode("utf-8")
            except:
                IOwithSpeak.outputWithVoice('Something wrong with nerwork')
                return(-1)

            map1 = json.loads(html)

            IOwithSpeak.outputWithVoice('the node number is : %d' % len(map1['map']))
            if len(map1['map'])==0:
                IOwithSpeak.outputWithVoice('there is no such map')
                return(-1)

            f = open('./map/'+filename, 'w')
            f.write(html)
            f.close()
        except:
            IOwithSpeak.outputWithVoice('Sorry, download map fails')
            return(-1)

    IOwithSpeak.outputWithVoice('load map file successful, now start navigation')
    return(0)

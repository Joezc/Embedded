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

    ans = 'fail'

    if os.path.isfile(path):
        f = open(path, 'r')
        ans = f.readlines()[0]
    else:
        try:
            url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building="+buildingname+"&Level="+levelno

            try:
                with urlopen(url) as response:
                    html = response.read().decode("utf-8")
            except:
                IOwithSpeak.outputWithVoice('Something wrong with nerwork')
                return(ans)

            map1 = json.loads(html)

            IOwithSpeak.outputWithVoice('the node number is : %d' % len(map1['map']))
            if len(map1['map'])==0:
                IOwithSpeak.outputWithVoice('there is no such map')
                return(ans)

            f = open('./map/'+filename, 'w')
            f.write(html)
            ans = html
            f.close()
        except:
            IOwithSpeak.outputWithVoice('Sorry, download map fails')
            return(ans)

    IOwithSpeak.outputWithVoice('load map file successful, now start navigation')
    return(ans)

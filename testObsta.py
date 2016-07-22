
def obsta(sonarF, sonarL, sonatR, dir, dis):
    if (sonarF <= MiniDis):
        canFront = False;
    if (sonarL <= MiniDis):
        canLeft = False;
    if (sonarR <= MiniDis):
        canRight = False;
    if should == 'left':
        if canLeft:
            output = 'left'
        elif canFront:
            output = 'front'
        elif canRight:
            output = 'right'
        else output = 'back'
    elif (should == 'right'):
        

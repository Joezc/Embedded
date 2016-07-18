from espeak import espeak

def inputWithVoice(inStr):
    temp = input(inStr)
    espeak.synth(inStr)
    return temp

def outputWithVoice(outStr):
    print(outStr)
    espeak.synth(outStr)

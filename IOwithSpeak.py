from espeak import espeak

def inputWithVoice(inStr):
    espeak.synth(inStr)
    temp = input(inStr)
    return temp

def outputWithVoice(outStr):
    espeak.synth(outStr)
    print(outStr)

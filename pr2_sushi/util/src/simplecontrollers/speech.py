from os import system


def speak(text):
    cmd = "echo \"" + text + "\" | festival --tts"
    system(cmd)
# robot_tts.py
import pyttsx3

engine = pyttsx3.init()

# Configure la voix française si disponible
for voice in engine.getProperty('voices'):
    if "fr" in voice.languages or "French" in voice.name:
        engine.setProperty('voice', voice.id)
        break

engine.setProperty('rate', 170)  # vitesse de parole
engine.setProperty('volume', 1.0)

def speak(text: str):
    engine.say(text)
    engine.runAndWait()



import queue
import sounddevice as sd
import vosk
import json
import sys

COMMANDES = {
    "debout": "stand",
    "stand": "stand",
    "stop": "stand",
    "assis": "idle",
    "idle": "idle",
    "marche": "walk",
    "walk": "walk",
    "avance": "forward",
    "avant": "forward",
    "recule": "backward",
    "arrière": "backward",
    "gauche": "rotate_left",
    "droite": "rotate_right",
    "forward": "forward",
    "backward": "backward",
    "left": "rotate_left",
    "right": "rotate_right"
}

try:
    model = vosk.Model(r"C:\Users\admin\PycharmProjects\SPOTMICRO__PFE__2025__SORBONNE\spot_micro_voc\model\vosk-model-small-fr-0.22\vosk-model-small-fr-0.22")  # modèle français
except:
    print("ERREUR : modèle Vosk introuvable.")
    print("dl here : https://alphacephei.com/vosk/models")
    sys.exit(1)

audio_queue = queue.Queue()

def audio_callback(indata, frames, time, status):
    audio_queue.put(bytes(indata))

stream = sd.RawInputStream(
    samplerate=16000,
    blocksize=8000,
    dtype='int16',
    channels=1,
    callback=audio_callback
)

recognizer = vosk.KaldiRecognizer(model, 16000)

def detecter_commande(texte):
    texte = texte.lower()

    for mot, commande in COMMANDES.items():
        if mot in texte:
            return commande

    return None

def ecouter_commandes():
    print("🎤 En écoute... dites par exemple :")
    print("   'avance', 'tourne à gauche', 'reste immobile', 'marche', etc.\n")

    # Reset recognizer
    global recognizer
    recognizer = vosk.KaldiRecognizer(model, 16000)

    # Clear old audio
    while not audio_queue.empty():
        audio_queue.get()

    # Create a fresh stream
    with sd.RawInputStream(
        samplerate=16000,
        blocksize=8000,
        dtype='int16',
        channels=1,
        callback=audio_callback
    ):
        while True:
            data = audio_queue.get()
            if recognizer.AcceptWaveform(data):
                resultat = json.loads(recognizer.Result())
                texte = resultat.get("text", "")

                if texte.strip():
                    print(f"🗣 J'ai entendu : {texte}")

                    commande = detecter_commande(texte)
                    if commande:
                        print(f"➡ Commande détectée : {commande}\n")
                        return commande

if __name__ == "__main__":
    """
    while True:
        commande = ecouter_commandes()
        # Envoie cette commande à ton robot comme avant
    """
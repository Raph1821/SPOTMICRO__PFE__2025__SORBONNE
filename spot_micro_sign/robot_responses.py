

def robot_reply(command):
    responses = {
        "stand":        "woofwoof", #"Je me tiens prêt.",
        "idle":         "hhamhhamm...",# "Je passe en mode repos.",
        "walk":         "Yipyipyippy!", #"Mode marche activé.",
        "forward":      "Quackwoof", # "J'avance.",
        "backward":     "aaouh",
        "rotate_left":  "heehee",
        "rotate_right": "ouhouh"
    }
    return responses.get(command, "Commande reçue, mais je ne comprends pas encore cette action.")

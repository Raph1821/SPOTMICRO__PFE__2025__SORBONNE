

def robot_reply(command):
    responses = {
        "stand":        "woofwoof", #"Je me tiens prêt.",
        "idle":         "hhamhhamm...",# "Je passe en mode repos.",
        "walk":         "Yipyipyippy!", #"Mode marche activé.",
        "forward":      "Quackwoof", # "J'avance.",
        "backward":     "Je recule.",
        "rotate_left":  "Je tourne à gauche.",
        "rotate_right": "Je tourne à droite."
    }
    return responses.get(command, "Commande reçue, mais je ne comprends pas encore cette action.")

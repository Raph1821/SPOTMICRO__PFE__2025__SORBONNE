

def robot_reply(command):
    responses = {
        "stand":        "Je me tiens prêt.",
        "idle":         "Je passe en mode repos.",
        "walk":         "Mode marche activé.",
        "forward":      "J'avance.",
        "backward":     "Je recule.",
        "rotate_left":  "Je tourne à gauche.",
        "rotate_right": "Je tourne à droite."
    }
    return responses.get(command, "Commande reçue, mais je ne comprends pas encore cette action.")

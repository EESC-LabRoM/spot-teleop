"""Operator-facing alert naming what the tracker cannot see. Shown in the UI only; motion never depends on it.

The body-pose wrist no longer drives translation (it sits ~4 cm from the hand model's wrist, and switching between
the two stepped the scaled command by 0.1-0.4 m). When the hand is lost the tracker publishes nothing, the teleop
pauses on its watchdog, and this alert tells the operator which part of the body to bring back into view.
"""

ALERTS = {
    'body': 'Operador fora da imagem: sente-se de frente para a câmera',
    'hand': 'Mão direita fora da imagem: traga a mão para o campo da câmera',
    'fingers': 'Mão direita não reconhecida: mostre a palma e os dedos, sem objetos na frente',
    'wrist': 'Punho fora da imagem: afaste a mão da borda',
    'depth': 'Mão sem profundidade: aproxime ou afaste a mão da câmera',
    'arm': 'Braço direito não visível: mostre ombro, cotovelo e punho direitos',
    'operator': 'Mão não associada ao operador: mostre só a mão direita, perto do corpo',
    'palm': 'Palma parcialmente visível: giro mantido, mostre a palma e os dedos',
}
IDENTITY_REASONS = ('Mão direita não corresponde ao braço calibrado', 'Duas mãos próximas', 'Separe as mãos')


def visibility_alert(diag):
    """dict(missing=<key of ALERTS or None>, alert=<message or None>) for one tracker diagnostic."""
    reason = str(diag.get('reason', ''))
    missing = None
    if diag.get('valid'):
        # Translation from the current hand wrist while the palm frame is rejected: still moving, rotation held.
        if not diag.get('orientation_valid'): missing = 'palm'
    elif reason.startswith(('Calibrando braço', 'Mostre o braço direito')):
        missing = 'arm'
    elif reason.startswith('Mão não detectada'):
        if 'pose_wrist_visibility' not in diag: missing = 'body'      # body pose found no person in this frame
        elif diag['pose_wrist_visibility'] < .5 or diag.get('pose_wrist_near_edge'): missing = 'hand'
        else: missing = 'fingers'                                       # wrist in view, hand model did not lock
    elif reason == 'Punho fora do quadro':
        missing = 'wrist'
    elif reason.startswith('Sem profundidade'):
        missing = 'depth'
    elif reason.startswith(IDENTITY_REASONS):
        missing = 'operator'
    # Anything else (late frames, camera format, detector errors) is a system problem, not the operator's: no alert.
    return dict(missing=missing, alert=ALERTS.get(missing))

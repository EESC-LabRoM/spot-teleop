import unittest
from tracking_alerts import visibility_alert, ALERTS


class VisibilityAlertTests(unittest.TestCase):
    def missing(self, **diag): return visibility_alert(diag)['missing']

    def test_full_6d_has_no_alert(self):
        self.assertEqual(visibility_alert(dict(valid=True, orientation_valid=True)), dict(missing=None, alert=None))

    def test_position_only_warns_about_the_palm(self):
        out = visibility_alert(dict(valid=True, orientation_valid=False, reason='Punho atual · translação ativa, giro mantido'))
        self.assertEqual(out, dict(missing='palm', alert=ALERTS['palm']))

    def test_lost_hand_is_split_by_what_the_body_pose_sees(self):
        lost = 'Mão não detectada: mantenha dedos e punho visíveis acima da mesa'
        self.assertEqual(self.missing(valid=False, reason=lost), 'body')
        self.assertEqual(self.missing(valid=False, reason=lost, pose_wrist_visibility=.1), 'hand')
        self.assertEqual(self.missing(valid=False, reason=lost, pose_wrist_visibility=.9), 'fingers')
        # a confident pose wrist on the image border: the hand is cut off, not unrecognised
        self.assertEqual(self.missing(valid=False, reason=lost, pose_wrist_visibility=.9, pose_wrist_near_edge=True), 'hand')

    def test_each_visibility_reason_names_a_part(self):
        cases = {'Punho fora do quadro': 'wrist', 'Sem profundidade consistente na palma/punho': 'depth',
                 'Mostre o braço direito para confirmar o operador após a perda': 'arm',
                 'Calibrando braço 3/20: mostre ombro, cotovelo e punho direitos': 'arm',
                 'Mão direita não corresponde ao braço calibrado': 'operator',
                 'Duas mãos próximas; mostre apenas a mão do operador': 'operator'}
        for reason, part in cases.items():
            self.assertEqual(self.missing(valid=False, reason=reason), part, reason)

    def test_system_problems_are_not_blamed_on_the_operator(self):
        for reason in ('Quadro atrasado ou repetido', 'Processamento atrasado; pose descartada', 'Aguardando calibração da câmera'):
            self.assertEqual(visibility_alert(dict(valid=False, reason=reason)), dict(missing=None, alert=None))


if __name__ == '__main__': unittest.main()

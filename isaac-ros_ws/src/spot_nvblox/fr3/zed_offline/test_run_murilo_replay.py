"""Timeline and inventory assertions with positive receipt/acquisition checks."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from run_murilo_replay import enumerate_takes
from murilo_sim_adapter import rebase_packet


def test_all13_including_repeated05(tmp_path):
    names = ['00_neutro', '01_giro_plano_imagem', '02_inclinar_dedos', '03_virar_palma',
             '04_frente_tras', '05_direita_esquerda', '05_direita_esquerda_2',
             '06_cima_baixo', '07_abrir_fechar', '08_cruz', '09_gesto_em_movimento',
             '10_perda_retorno', '11_distancias']
    for name in names:
        directory = tmp_path/name/'d8'
        directory.mkdir(parents=True)
        (directory/'metadata.yaml').touch()
    assert enumerate_takes(tmp_path) == names
    assert len(set(enumerate_takes(tmp_path))) == 13


def test_original_timing_pair_and_acquisition_lag():
    start, epoch = 1000000000, 9000000000
    captured = [start+100000000, start+170000000]
    receipt = [start+160000000, start+250000000]
    rebased = [rebase_packet(dict(stamp_ns=c, orientation_stamp_ns=c,
                                roll_stamp_ns=c, p=[1, 2, 3]), start, epoch) for c in captured]
    assert rebased[1]['stamp_ns']-rebased[0]['stamp_ns'] == 70000000
    for c, r, packet in zip(captured, receipt, rebased):
        delivered = epoch+(r-start)
        assert delivered-packet['stamp_ns'] == r-c
        assert packet['stamp_ns'] == packet['orientation_stamp_ns'] == packet['roll_stamp_ns']
        assert packet['p'] == [1, 2, 3]
    assert receipt[1]-receipt[0] == 90000000

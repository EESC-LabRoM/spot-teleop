"""Build the final offline correction report and before/after measured plots."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[4];OUT=ROOT/'clips_out/zed_protocol_analysis'
sys.path.insert(0,str(HERE))
from compare import build
build();comparison=json.loads((OUT/'comparison.json').read_text());review=json.loads((OUT/'simulation_review.json').read_text());rows=comparison['rows']
selected=[r for r in rows if r['stage'] in ('01_giro_plano_imagem','02_inclinar_dedos','03_virar_palma','04_frente_tras','05_direita_esquerda_2','06_cima_baixo','07_abrir_fechar') and 'simulated_span_cm' in r]
fig,axs=plt.subplots(1,2,figsize=(14,5),layout='constrained');x=np.arange(len(selected));labels={'01_giro_plano_imagem':'Giro plano','02_inclinar_dedos':'Inclinar','03_virar_palma':'Virar palma','04_frente_tras':'Frente/trás','05_direita_esquerda_2':'Lateral','06_cima_baixo':'Cima/baixo','07_abrir_fechar':'Garra'}
for ax,key,title in zip(axs,['original_real_span_cm','simulated_span_cm'],['Movimento real gravado (perfil legado)','Replay corrigido: movimento medido em simulação']):
 for j,(axis,color) in enumerate(zip(['Alcance','Horizontal','Altura'],['#6088cc','#36a67f','#e39c53'])):ax.bar(x+(j-1)*.25,[r[key][j] for r in selected],.25,label=axis,color=color)
 ax.set_xticks(x,[labels[r['stage']] for r in selected],rotation=30,ha='right');ax.set_ylabel('Amplitude pico a pico (cm)');ax.set_ylim(0,43);ax.set_title(title);ax.legend();ax.grid(axis='y',alpha=.2)
fig.savefig(OUT/'before_after_simulation.png',dpi=150);plt.close(fig)
text='''# Replay corrigido das bags ZED em simulação

Os 13 arquivos originais foram preservados. Foram recuperados **6.684 pares RGB/profundidade com timestamp idêntico**, de 6.753 imagens RGB gravadas. O rastreamento foi executado para cada par, sem descartar quadros para acompanhar o relógio de parede. A pose foi válida em **6.580 pares (98,44%)** e a orientação em **6.139 (91,85%)**. As porcentagens incluem todo o material, também antes do engate e depois do Stop; não são diretamente a mesma população dos pacotes esparsos emitidos ao vivo.

## Por que o robô real se moveu daquele jeito

- O processo real aplicava `target_position = anchor + zed_scale * (hand - hand_anchor)` nos eixos legados. O perfil aprovado, `[handZ, handY, -handX]`, não estava selecionado. Assim frente/trás e cima/baixo comandavam eixos diferentes da intenção registrada na revisão.
- O modo real era 6D livre: ao girar a mão, qualquer deslocamento do punho também deslocava o TCP. No giro no plano, a trajetória real teve cerca de 18,3 cm de alcance, 16,7 cm de horizontal e 9,5 cm de altura. O perfil do exercício precisa manter o TCP e girar só a garra.
- A rotação real aplicava a diferença espacial da mão à esquerda da orientação do robô (`dq * anchor_q`). O exercício de girar no plano pede giro no eixo local da ferramenta, implementado na revisão pelo último eixo do punho. São comandos geométricos diferentes.
- A escala corporal já estava no pacote (aproximadamente 1,76 no início, depois cerca de 1,72). `speed_scale=0.5` limita velocidade; não transforma a amplitude da mão em metade da amplitude do robô. `rot_scale` também limita velocidade angular. Confundir limite de velocidade com ganho de amplitude muda a expectativa do movimento.
- A câmera registrou 15–19 Hz, mas a teleop recebeu apenas 2–5 pontos/s, já atrasados. Pontos expiravam; pausas e recuperações recapturavam a referência, mudando a relação de deslocamento ao longo do mesmo exercício. Não basta reproduzir só esses pacotes esparsos.
- No clipe 09, houve adicionalmente proteção por estado antigo do robô (`robot state stale`). A cadeia TF dinâmica teve lacunas superiores a 2 s apesar de `current_pose` rápida. Esse defeito pertence ao caminho do estado ao vivo; o replay usa o estado medido do simulador.

## O que mudou neste teste

1. Novo reprocessador ZED usa o decoder lossless de `compressedDepth`, intrínsecos gravados e sincronização exata dos timestamps. Cada exercício começa com filtros novos e restaura o comprimento do braço registrado na própria etapa. O relógio do rastreador é o da captura. Os resultados e o vídeo são derivados, separados dos originais.
2. O replay usa poses novas em todos os quadros processáveis. Os timestamps são convertidos para o relógio atual apenas na reprodução, preservando intervalos e validade; não se apresentam pacotes antigos como percepção ao vivo.
3. Os exercícios usam explicitamente o mapeamento aprovado: lateral → horizontal; frente/trás → altura; cima/baixo → alcance; giros → posição fixa; abrir/fechar → braço parado. **O isolamento é escolhido pelo exercício, não inferido automaticamente.** Isso valida cada movimento esperado, mas não prova ainda um controle 6D livre em qualquer gesto.
4. Corrigida na simulação a aceitação do gesto: punho válido e gesto confiável podem confirmar abrir/fechar mesmo quando a orientação da palma é mantida. Permanecem as verificações de pacote recente, controle ativo e confirmação temporal. A alteração não foi implantada no controlador físico.
5. A revisão ZED tem dataset separado, sem apagar comentários, poses ou vídeos da RealSense. Foram acrescentados ao trace gesto, estado da garra, abertura medida e desempenho da física.
6. Isaac PhysX e IK cuRobo executam na **GPU física 1**, domínio DDS **188**, tópicos `/fr3_sim/*`. Nesta revisão cartesiana, o seguidor usa IK GPU; giro simples usa o último eixo do punho e poses paradas usam retenção das juntas. O MPC é carregado para os outros movimentos do painel, mas os números desta revisão não validam MPC com ESDF/nvblox.

## Medidas do replay

Amplitude pico a pico após a ativação e um segundo inicial. Eixos físicos do robô: alcance X, horizontal Y, altura Z. As amplitudes são de pose medida, não só de alvo calculado. Pequenas diferenças fora do eixo vêm da dinâmica/seguimento do simulador.

| Exercício | Alcance cm | Horizontal cm | Altura cm | Maior variação fora dos eixos habilitados cm |
|---|---:|---:|---:|---:|
'''
for r in rows:
 if r.get('simulated_span_cm'):
  vals=r['simulated_span_cm'];off=f"{r['off_axis_max_cm']:.2f}" if r['off_axis_max_cm'] is not None else '—';text+=f"| {r['stage']} | {vals[0]:.2f} | {vals[1]:.2f} | {vals[2]:.2f} | {off} |\n"
grip=next((r for r in rows if r['stage']=='07_abrir_fechar'),{})
if grip.get('gripper_measured_width_range_mm'):
 text+='\nAbertura medida da garra em 07: '+str(grip['gripper_measured_width_range_mm'])+' mm. Transições confirmadas no estado do simulador: '+str([x['command'] for x in grip.get('gripper_transitions',[])])+'.\n'
text+='''
![Antes e depois: amplitudes medidas](before_after_simulation.png)

## Limites da validação

09 não contém o exercício físico completo, 10 não contém um ciclo completo de perda e retorno, e 11 não engatou no robô real. As imagens permitem testar percepção e execução em simulação, mas não inventam movimentos que não foram feitos. Os intervalos dos replays terminam no primeiro Stop sempre que há material suficiente, preservando o período anterior para capturar a referência. O clipe curto 09 precisa ser identificado como diagnóstico parcial.

A física está configurada em 60 Hz na GPU, mas a taxa efetiva varia com a carga compartilhada da máquina. Os traces novos registram a taxa observada. O vídeo mostra os movimentos medidos sob essa carga, não garante desempenho em tempo real do conjunto ao vivo.

A métrica antiga `intended_sign_match` em `run_clips` foi criada para rótulos direcionais individuais e não deve ser usada para certificar ciclos completos com rótulo genérico; este relatório usa a trajetória completa e os eixos habilitados. O campo histórico `mpc_feasible_ratio` nos jobs desta revisão representa a viabilidade reportada pelo seguidor IK, não uma validação de MPC real.

Scripts em `isaac-ros_ws/src/spot_nvblox/fr3/zed_offline/`: `reprocess.py`, `simulate.py`, `compare.py`, `report.py`. Dados: `reprocessed_v1/`, `simulation_review.json`, `comparison.json`; comparativos e traces nos jobs referenciados em `clips_out/manual_clips/`.

Testes: 59 verificações passaram para mapeamento, aceitação de gesto, separação de datasets e persistência dos comentários. As correções deste teste não enviaram comandos ao robô físico.
'''
(OUT/'SIMULATION_RESULTS.md').write_text(text)
print('Saved SIMULATION_RESULTS.md and before_after_simulation.png')

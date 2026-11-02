# Bags da ZED controlando o Franka em simulação

Este fluxo usa DDS **188**, tópicos `/fr3_sim/*`, o Isaac e o MPC da cena
`remote_workcell.usda`. Não inicia drivers físicos nem o relay UDP da bancada.
O painel do laboratório na 8788 pertence ao fluxo físico, separado deste replay.

## O que a mão está controlando

1. A ZED3 filma o operador e fornece RGB, profundidade alinhada e intrínsecos.
2. `operator_hand_tracker.py` associa a mão ao operador. A profundidade mede a
   posição do punho; os landmarks da palma fornecem a orientação. O comprimento
   ombro–cotovelo–punho determina a escala `0.855 / comprimento_do_braço` quando
   a calibração por pessoa está habilitada. Há filtros e rejeição de saltos.
3. A saída `teleop_sample.p` **já inclui transformação de eixos, escala e filtro**.
   Não aplique novamente a escala do braço no replay `recorded`.
4. Start ancora a mão e a ferramenta atuais. Só as diferenças posteriores
   comandam o robô; coordenadas negativas da mão não são alvos absolutos do TCP.
5. A simulação mapeia o deslocamento relativo da mão (`dX, dY, dZ = p_mão - p_mão_inicial`)
   em coordenadas cilíndricas da base `fr3_link0`:
   - **Altura (`z`)**: aproximar a mão da câmera (`dX > 0`) abaixa o TCP em direção à mesa
     (`z_alvo = z_inicial - ganho_z * dX`); afastar sobe.
   - **Giro da base (`θ`)**: mover a mão lateralmente (`dY`) gira o TCP em arco em torno
     do eixo `z` da base mantendo o raio (`dθ = ganho_y * dY / max(r_0, 0.2)`). Mão para a
     direita do próprio operador (`dY < 0`, esquerda da imagem) gira o braço para a direita
     do robô (`dθ < 0`, sentido horário visto de cima).
   - **Alcance radial (`r`, premissa assumida)**: subir/descer a mão (`dZ`) estende/retrai
     radialmente o alcance (`r = clip(r_0 + ganho_x * dZ, 0.15, 0.80)`). Esta associação de
     `dZ` ao alcance radial é uma premissa assumida para o grau de liberdade restante.
   - **Orientação**: a rotação da palma em torno da normal da palma capturada no Start
     (`n_0 = -R(q_0)[:,0]`, orientada para longe da câmera, `-X`) é extraída por
     decomposição swing-twist de `dq = q * q_0⁻¹` e aplicada como giro em torno do eixo `z`
     da própria ferramenta (`q_alvo = q_TCP_inicial * Rz(ganho_giro * ângulo)`). A inclinação
     espacial (swing) é ignorada por padrão (`tilt_gain = 0`, mantendo a ferramenta como na
     âncora) e pode ser escalada opcionalmente por `tilt_gain` em `[0, 3]`.
6. Limites de velocidade, workspace e afastamento da pose medida limitam esse
   alvo. O cuRobo encontra juntas; a física produz a pose observada. Por isso
   comparar só a mão com o TCP não separa escala de saturação/erro do MPC.

Na transformação do pacote do rastreador (`CAMERA_TO_ROBOT`), aproximar a mão da
câmera aumenta `X` (abaixa o TCP), mover para a direita da imagem / esquerda do
operador aumenta `Y` (gira a base no sentido anti-horário visto de cima) e subir
a mão aumenta `Z` (estende o alcance radial, por premissa assumida). Girar a palma
no sentido horário visto pela câmera gira a garra no sentido horário vista de cima.

Palma aberta/punho fechado também comandam a garra após confirmação temporal.
O controlador aceita orientação atual apenas quando está pareada com a posição.
Depois de uma primeira orientação válida, pacotes v3 de posição apenas continuam
a translação mantendo a última orientação medida; não acionam gestos da garra.
Eles não são apresentados como novas medições 6D. Sem posição válida, o movimento pausa.
A retomada e o fallback do backend físico não são idênticos aos da simulação.

## Bags encontradas e verificadas

| Sessão de 23/10/2026 | RGB | Profundidade | Amostras de mão | Uso |
| --- | ---: | --- | ---: | --- |
| `20261023_195220_zed1` | 2573 | 2573 compressedDepth | 855 | rastreador/mapeamento; d77 sem comandos |
| `20261023_200217_zed_curobo1` | 1454 | 1454 compressedDepth | 511 | replay e comparação futura com o robô |
| `20261023_200513_mao1` | 2108 | 2103 crua 32FC1 | 608 | aproximadamente 74,7 s, foco no operador |

O d77 da segunda sessão contém 583 `/teleop/command_tool`, 583
`/fr3_curobo/equilibrium_pose` e 190 trajetórias de juntas. Isso confirma comandos
gravados; não constitui, por si só, avaliação de fidelidade do movimento real.
O player atual usa o d8. Não reproduz estados/juntas físicos como estado simulado.

`compressedDepth` 32FC1 armazena uma representação inversa quantizada em PNG;
o PNG é sem perdas, mas a conversão de float para essa representação quantiza.
16UC1 armazena milímetros. O decoder respeita o cabeçalho e marca pixels ausentes.
As imagens com landmarks precisam ser recalculadas: não foram gravadas nesses bags.

## Executar

Na máquina `hipolito-tailscale`:

```bash
cd /home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim
bash run_isaac.sh > /tmp/fr3-bag-isaac.log 2>&1 &
bash run_bag_control.sh /home/hipolito/spot-teleop/bags/20261023_200513_mao1 recorded
```

Abra [replay da câmera](http://100.88.134.127:8792/) e
[Franka simulado](http://100.88.134.127:8791/). Espere o MPC e o estado do
simulador ficarem prontos. No painel 8791, clique **Iniciar bag + robô**: o mesmo comando arma a
teleop e toca a bag. **Parar** também pausa a bag. Ao terminar, o painel mostra
**Bag finalizada** e o botão **Repetir bag** reinicia o modo recorded. O robô simulado espera uma referência estável;
se o trecho inicial já tem movimento, a calibração pode esperar uma pausa da mão.
O replay começa pausado. Pausar deixa as amostras expirarem e interrompe seguir
a mão; continuar passa pelos critérios de recuperação da sessão simulada.

**Para repetir no modo recorded**, clique Parar no painel do robô e **Voltar ao
início** no painel do replay. Ajuste ganhos no painel 8791, dê Start e toque a bag.
Os ganhos escolhidos continuam no processo. A volta ao início recusa uma sessão
de controle ativa. O robô permanece na pose atual: para comparar trajetórias
absolutas, retorne à mesma pose pelo Home da simulação antes de cada Start.

**Para trocar o modo, a bag ou zerar também o rastreador**, clique Parar no painel
do robô e substitua somente o container de replay criado por este fluxo:

```bash
docker stop fr3-bag-control
docker rm fr3-bag-control
FR3_SIM_POSITION_GAINS=1,1,1 FR3_SIM_ROTATION_GAIN=1 \
  bash run_bag_control.sh /home/hipolito/spot-teleop/bags/20261023_200513_mao1 recorded
```

O Isaac pode continuar aberto. Não execute `run_control.sh` junto com
`run_bag_control.sh`: os dois usam a mesma sessão simulada. O launcher recusa
um `fr3-remote-control` ativo. Para encerrar também o Isaac deste fluxo:
`docker stop fr3-remote-isaac` (o launcher usa `--rm`).

## Ajustar direção, escala e rotação

Mude uma coisa por vez, repetindo a mesma bag e usando a mesma região de Start.

| Intenção | Configuração antes de `bash run_bag_control.sh … recorded` |
| --- | --- |
| Preservar o mapeamento gravado | `FR3_SIM_POSITION_GAINS=1,1,1 FR3_SIM_ROTATION_GAIN=1` |
| Metade do deslocamento em todos os eixos | `FR3_SIM_POSITION_GAINS=0.5,0.5,0.5` |
| Inverter frente/trás | `FR3_SIM_POSITION_GAINS=-1,1,1` |
| Reduzir só movimento vertical | `FR3_SIM_POSITION_GAINS=1,1,0.5` |
| Metade da rotação relativa | `FR3_SIM_ROTATION_GAIN=0.5` |
| Fixar a orientação capturada no Start | `FR3_SIM_ROTATION_GAIN=0` |

Os ganhos aceitam XYZ entre -5 e 5, rotação entre 0 e 3. Zero em um eixo
trava só a contribuição da mão nesse eixo. Eles ficam registrados em `/state`
do painel 8791. O slider de velocidade limita a resposta; não é escala da mão.
O controlador mantém seus limites de velocidade/workspace/erro durante os ajustes.

Também é possível ajustar diretamente no painel 8791: **Parar → Ajustar direção,
escala e rotação da mão → Aplicar → Start**. Campos só são aplicados com o controle
parado; a nova referência evita reutilizar a âncora de outro ganho. Ajustes feitos
no painel valem para o processo atual; use as variáveis acima para persistir ao
recriar o container. No modo retrack, recrie o container para repetir com filtros
e calibração novamente zerados.

Comece por sinais com ganho 1; depois escala XYZ; por último rotação. Olhe as
diferenças da mão mostradas no painel e a pose medida. Se o alvo estiver limitado,
reduza o ganho ou escolha um trecho menor antes de concluir que o eixo está errado.

## Recalcular o rastreador com as imagens gravadas

Substitua o container de replay como acima, escolhendo:

```bash
bash run_bag_control.sh /home/hipolito/spot-teleop/bags/20261023_200513_mao1 retrack
```

Este modo publica RGB descomprimido, depth e intrínsecos para o mesmo
`operator_hand_tracker.py` atual, com calibração do braço habilitada e processamento
do quadro mais recente. A calibração parte do zero; precisa de braço visível e
amostras suficientes. O resultado original fica em
`/fr3_sim/operator/recorded_sample`; só o resultado recalculado controla a simulação.
O painel 8791 mostra o overlay recalculado; 8792 mostra o RGB original.
No modo recorded, diagnósticos como tempo de processamento e escala são os
registrados na bag; não medem uma nova inferência.
Este modo avalia o código atual, não restaura automaticamente o código de 23/10.

Um terceiro argumento opcional define velocidade da bag, por exemplo `retrack 0.5`.
O player adapta os intervalos dos timestamps à velocidade e conserva os atrasos
relativos nesse relógio. Playback lento altera os filtros, calibração e critérios
temporais; use **1.0 para comparar fidelidade temporal**. Não há `/clock` de bag:
o Isaac já publica seu próprio relógio; percepção/controle usam tempo de parede.
Não basta executar `ros2 bag play` com datas antigas: o rastreador e o controle
rejeitam esses dados por idade. O player faz a adaptação sem remover essas verificações.

## Diagnóstico e limites

```bash
docker exec fr3-bag-control tail -30 /tmp/bag_replay.log
docker exec fr3-bag-control tail -30 /tmp/bag_session.log
docker exec fr3-bag-control tail -30 /tmp/bag_tracker.log  # somente retrack
curl -s http://127.0.0.1:8792/state
curl -s http://127.0.0.1:8791/state
```

A cena usa obstáculos estáticos simplificados, não o mapa nvblox registrado da
bancada. Não é uma reprodução dinâmica exata do robô real. A taxa de física,
renderização e inferência depende da carga compartilhada das GPUs. Ganhos aqui
não alteram configurações do robô físico. Nenhum ajuste é promovido automaticamente.

## Validação local em 27/10/2026

- Leitura/decodificação completa das três bags, sem erro: 6135 RGB e 6130 depths.
  Inventário detalhado em `run/bag_inventory.json`.
- `mao1` completa em tempo de parede, modo recorded: 2108 RGB, 608 amostras,
  465 pares 6D e 143 amostras de posição apenas; Isaac/cuRobo receberam os pares,
  ativaram o acompanhamento e moveram o TCP, com pausas/retomadas observadas.
- `mao1` completa em retrack: 2108 RGB e 2103 depths publicados sem erro;
  calibração nova de aproximadamente 0,485 m, escala 1,764, saídas 6D e de posição
  apenas observadas. A calibração original registrada era aproximadamente
  0,477 m/escala 1,794: recalcular não reproduz exatamente os resultados anteriores.
- 37 testes de replay/mapeamento, tracking, calibração e distâncias passaram.
  Após finalizar repetição do player/painel, 22 testes de replay e tracking
  passaram novamente; Python e JavaScript passaram na verificação de sintaxe.
- HTTP: tocar, pausar, voltar ao início e aplicar/restaurar ganhos funcionaram.
  Mudar ganhos ou voltar ao início durante calibração foi recusado até Parar.
- Na carga compartilhada observada, física/render ficaram abaixo de tempo real
  (aproximadamente 0,1–0,4× em vários intervalos). O replay funcional não demonstra
  fidelidade temporal nesta máquina sob essa carga, nem valida o robô físico.


## Correção após teste de uso no painel

O Start do painel 8791 passou a controlar também o player, evitando uma sessão
armada sem bag tocando. O fim da bag encerra o acompanhamento e informa o motivo;
Repetir bag volta ao início no modo recorded. Posição válida sem orientação
continua translacionando, com a orientação anterior mantida e sem gesto de garra.

Na ponte do Isaac, passos manuais PhysX não atualizam Fabric por padrão. Antes
de cada imagem, os transforms são sincronizados explicitamente com Fabric.
Atualizações USD do marcador/trilha foram limitadas aos frames renderizados;
o relatório de desempenho separa tempo gasto em física e em renderização.

O loop de avanço da física também limita cada bloco a cerca de 20 ms de tempo
de parede. Isso impede que um atraso acumulado prenda a atualização da imagem
por vários segundos. A taxa de física e de imagem aparecem separadas no painel.
A verificação com `mao1` confirmou mudança visível da pose e deslocamento medido
do TCP de aproximadamente 16 cm nos primeiros 24 s após Start com um clique.

Após reiniciar com o limite do bloco de física, a imagem ficou em aproximadamente
2,5–3 quadros/s (antes, aproximadamente 0,3–0,5). Start único, fim da bag e
repetição automática pelo botão foram verificados via HTTP. A física continua
abaixo de tempo real na carga atual; a posição medida e a imagem mudam durante
o acompanhamento. Os 24 testes de replay/mapeamento/tracking passaram.

## Simulador MuJoCo (CPU)

Para liberar a GPU usada pelo Isaac Sim, o nó físico/visual pode ser executado em
CPU via MuJoCo (mantendo a mesma interface ROS 2 em `/fr3_sim/*`, `run/view.jpg` e
`run/performance.json`). O braço inicia na pose **HOME** da bancada (`q = [0, -π/4, 0, -3π/4, 0, π/2, π/4]`, garra aberta em `0.04` m, ferramenta apontando para baixo) e publicar `std_msgs/msg/Empty` em `/fr3_sim/reset` restaura imediatamente o braço em HOME e todos os objetos da cena às poses iniciais com velocidade zero. O `run_isaac.sh` permanece intacto como fallback. Antes de
iniciar o MuJoCo no domínio 188, encerre o container `fr3-remote-isaac` caso esteja ativo:

```bash
cd /home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim
bash run_mujoco.sh > /tmp/fr3-bag-mujoco.log 2>&1 &
bash run_bag_control.sh /home/hipolito/spot-teleop/bags/20261023_200513_mao1 recorded
```

## Recortes para revisão

Para extrair movimentos curtos da bag gravada (`20261023_200513_mao1/d8`), reproduzir cada trecho partindo de **HOME** pela pipeline real (`bag_replay` → `sim_session` → cuRobo MPC → MuJoCo) e gerar vídeos lado a lado (câmera do operador à esquerda e MuJoCo à direita em 15 fps):

```bash
# 1. Identificar e rotular até 10 recortes em R/run/clips.json
docker exec fr3-bag-control bash -c '
  source /opt/ros/humble/setup.bash
  python3 /home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim/make_clips.py \
    /home/spot-teleop/bags/20261023_200513_mao1
'

# 2. Executar todos os recortes sequencialmente e gerar os vídeos + relatório
docker exec fr3-bag-control bash -c '
  source /opt/ros/humble/setup.bash
  export PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator:${PYTHONPATH:-}
  python3 /home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim/run_clips.py --run-id mao1_review
'

# 3. (No host) Converter para H.264 yuv420p compatível com navegador via ffmpeg
python3 /home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim/run_clips.py \
  --transcode-dir /home/hipolito/spot-teleop/clips_out/mao1_review
```

Os arquivos gerados ficam em `/home/hipolito/spot-teleop/clips_out/<run_id>/` (no container: `/home/spot-teleop/clips_out/<run_id>/`):
- `clip_NN.mp4` e `clip_NN/trace.json`: vídeo lado a lado em H.264 (`yuv420p`) e série temporal completa de cada recorte.
- `index.html` e `summary.json`: página de revisão com todos os vídeos e resumo numérico por eixo (`hand_delta`, `robot_tcp_delta`, razões e verificação de sinal).



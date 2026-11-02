# Cortador interativo da teleop

Abra http://100.88.134.127:8793/ ou clique em **Cortar vídeos e escolher movimentos**
no painel 8791. A ferramenta oferece as três câmeras completas das bags e os dez
comparativos de `clips_out/mao1_review`.

1. Escolha a gravação e arraste as duas alças. Também pode marcar **I** e **O**
   durante a reprodução, ou digitar os tempos. Setas avançam/voltam um quadro da fonte;
   Shift + seta desloca 1 s.
2. Use **Ver só o corte** para conferir em loop; dê um nome e **Salve o corte**.
3. **Exportar MP4** gera um arquivo novo H.264 com os limites escolhidos, sem
   modificar a fonte. O download aparece à direita e inclui um JSON com os tempos.
4. Nas câmeras originais, **Simular da home** executa o trecho com o runner
   existente, retornando à home antes, e gera outra comparação lado a lado.
   A bag correspondente precisa estar carregada e o painel 8791 parado. Deixe
   aproximadamente 1 s de mão estável no começo para capturar a referência.

O editor salva seleções em `spot-teleop/clips_out/manual_clips/selections.json`.
Cada exportação tem uma pasta própria com MP4 e metadados. Os vídeos das câmeras
usam o relógio de recebimento da bag, preservando os intervalos entre imagens.
Eles são amostrados a 25 fps; a precisão visual do corte é de aproximadamente
um quadro (0,04 s). Se o primeiro RGB chega depois do início da bag, a primeira
imagem fica parada até aquele instante. Profundidade e rastreamento continuam
nos bags originais; os MP4 exportados não contêm esses canais.

Vídeos comparativos já renderizados incluem calibração e cauda; não há uma
correspondência exata entre seu relógio e o da bag. Por isso o botão de simular
está disponível somente nas fontes originais da ZED. Simular não resolve erros
de rastreamento/mapeamento do trecho; métricas e avisos do runner são mantidos.

Para iniciar no host, sem trocar containers nem ligar o Isaac:

```bash
cd /home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim
python3 prepare_clip_media.py  # primeira preparação; usa fr3-bag-control para ler ROS
python3 clip_editor.py
```

O host precisa de Python 3 e ffmpeg/libx264. A preparação lê as bags dentro do
container ROS, mas não inicia publishers. O editor/exports usam CPU. **Simular
da home** usa o MuJoCo e cuRobo já existentes, em domínio de simulação 188.

O servidor implementa HTTP Range e Content-Length para que vídeos abram,
avancem e voltem diretamente pelo navegador, sem depender de arquivos relativos
em um visualizador de anexos. Há validação de intervalos e de origem nas escritas.

Validação: testes de limites HTTP/cortes; navegador Chrome em desktop e 390 px,
incluindo reprodução, arraste, prévia em loop, persistência e MP4 exportado.
Um corte manual de `mao1`, 7,516–12,000 s, completou o runner desde a home e
gerou comparação H.264 sem erro reportado pelo runner.

## Mostrar o movimento movendo o robô

Clique em **Mover o robô para mostrar o movimento**. A câmera da pessoa e o
MuJoCo ficam lado a lado. Pause a pessoa, use alcance/base/altura/giro da garra
(e inclinação/X/Y nos ajustes extras), e clique **Salvar esta pose no instante
do vídeo**. Avance a pessoa e salve a próxima pose. Home reinicia a simulação
na configuração HOME; o modo manual pausa o acompanhamento automático da bag.

As demonstrações ficam em `clips_out/manual_clips/demonstrations.json`, com
tempo do vídeo/bag, pose medida do TCP, sete juntas, garra e alvo solicitado.
A precisão temporal é a do vídeo pausado e da captura do estado via HTTP, não
um sincronismo de sensores. As poses servem como referência para ajustar o
mapeamento posteriormente; salvar não treina nem altera o mapeamento sozinho.

O movimento manual usa IK local do URDF e limites de juntas, sem passar pelo
MPC da teleop automática. A simulação física permanece no MuJoCo. Passos até
5 cm/15° são limitados e o comando de juntas avança em incrementos limitados;
poses sem solução são recusadas. FK da HOME confere com MuJoCo a 1 micrômetro
no teste numérico; cinco passos de posição/rotação passam no teste de IK.
No teste do navegador, subir 1 cm resultou em 9,96 mm medidos; giro da base,
captura em dois instantes, persistência e interface móvel também passaram.

## RealSense reprocessada na simulação

Os 12 movimentos estão no grupo “RealSense · movimentos reprocessados”.
`Carregar no simulador` troca a bag e pausa o controle. `Rodar este corte no robô`
salva o intervalo escolhido e executa a simulação a partir da home. A pessoa e
o robô ficam lado a lado; o vídeo acompanha o relógio da bag. É possível trocar
entre ZED e RealSense sem reiniciar cuRobo.

`Mover o robô para mostrar o movimento` abre o modo manual. Pause a pessoa,
use alcance/base/altura/giro/inclinação e `Salvar esta pose no instante do vídeo`.
Cada pose medida fica vinculada ao source_id e bag_time_s. Essas demonstrações
registram a intenção do operador; ainda não alteram automaticamente os ganhos.
O parâmetro de URL `?source=ID&manual=1` abre diretamente esse fluxo.

`realsense/prepare_simulation.py` cria bags próprias de replay em
`bags/*_reprocessed/replay`. Os pacotes de pose são copiados sem alteração;
a imagem é uma referência visual do vídeo anotado. Essas bags não têm
profundidade e o modo retrack deve recusá-las. RGB-D original permanece no
notebook. `bag_replay.py` aceita os tópicos ZED e RealSense e permite `load`
somente com a sessão simulada parada e caminhos dentro da pasta bags.

Validação: os 12 sources trocaram corretamente por HTTP; 14 testes de replay e
cortes passaram; navegador carregou vídeo/robô sem erro JavaScript; o botão
manual de +1 cm produziu 9,95 mm medidos em Z e voltou à home. Um trecho lateral
de 25 s rodou em DDS 188 a ~15 Hz, gerando vídeo e métricas, com sinal de
movimento esperado. Cortes e demonstrações anteriores foram preservados.

## Backend ativo: física na GPU 1

Em 29/10/2026 o backend ativo foi trocado de MuJoCo CPU para Isaac/PhysX GPU.
`run_isaac.sh` expõe a GPU física 1; dentro do container ela aparece como cuda:0.
O startup verifica dinâmica GPU, broadphase GPU e tensores CUDA, recusando fallback
CPU. cuRobo permanece na mesma GPU física 1. O container fr3-remote-mujoco foi
parado. A interface e o trabalho de envio/recepção de mensagens permanecem no host.

Foram mantidos /fr3_sim/reset e /fr3_sim/record, JPEG, CSV e estado usados pelo
editor. O retorno à home aguarda a pose medida antes de reativar o manual, pois
o backend GPU aplica reset no passo seguinte. O cabeçalho do editor identifica
Isaac / GPU 1 a partir do estado real. As 11 poses de esquerda/direita foram
preservadas byte a byte e há backup em clips_out/manual_clips/backups/.

Verificação: movimento manual de +1 cm medido em 9,51 mm, gravação de oito frames
JPEG/CSV, navegador sem erros e processos Isaac/cuRobo confirmados na GPU 1 pelo
nvidia-smi. Na carga atual, a física roda em cerca de 34 passos/s (RTF ~0,17),
mais lenta que o antigo MuJoCo CPU a 200 passos/s; GPU não implica melhoria de
velocidade neste cenário. Dados: clips_out/realsense_analysis/gpu1_backend_validation.json.
Os 12 vídeos gravados anteriormente continuam sendo resultados do backend MuJoCo;
não foram substituídos por resultados Isaac.

## Revisão sem anotações de pose

`http://100.88.134.127:8793/review` reúne os novos vídeos e dois botões por
movimento: Está certo / Está errado. Comentários como “inverte”, “mais” ou
“menos giro” são salvos em `clips_out/manual_clips/review_feedback.json` com o
job_id do vídeo avaliado. Avaliações de versões antigas não são aplicadas a
vídeos atualizados. As demonstrações de poses continuam preservadas, mas não
são necessárias para este fluxo.

Eixos usados pelo usuário: X horizontal, Y altura, Z frente/alcance. Internamente
isso corresponde a Y, Z, X do robô. O novo alvo cartesiano não usa arco de base:
mão lateral → horizontal, profundidade → altura (sinal invertido), mão vertical →
alcance. O modo anterior permanece acessível como legacy para comparação.

`cartesian_review.py` define perfis explícitos para os exercícios registrados:
translações isoladas mantêm orientação; giros isolados mantêm posição; abrir/fechar
mantém toda a pose; cruz usa horizontal+alcance; exercícios compostos combinam
translação e garra. Esses perfis não são um classificador automático de intenção:
são ensaios separados para o usuário aprovar primeiro os eixos e sinais.

A geração está em `realsense/run_cartesian_review.py`, com histórico em
`clips_out/realsense_analysis/cartesian_review*.json`. A amplitude inicial usa ganho
0,3, zona morta de 8 mm na saída e limite de 20 cm por eixo. O giro tem zona morta
de 3° e limite de 75°. O modo livre usa translações cartesianas + giro no plano.

Para seguir o alvo, o controlador usa IK cuRobo na GPU 1 com checagens de colisão,
passos cartesianos curtos e saturação sincronizada das juntas. O ensaio de giro
no plano aciona somente a junta 7; neutro e garra mantêm as juntas do braço.
Física e renderização continuam no Isaac/PhysX GPU 1. Não há comandos físicos.

### Ajustes após comentários (revisão 3)

`run_cartesian_review.py --resume --feedback` regenera os exercícios reprovados,
preserva os demais vídeos e armazena a execução anterior em `attempts`. Há um
backup dos comentários, seleções, poses, relatório e configuração em
`clips_out/manual_clips/backups/before_review3_20261029_210800/`.

Ganho de translação passa de 0,3 para 1,0; giro no plano 1,6; demais giros 2,0.
Velocidade do controlador 100%, passo de IK 25 mm / 0,08 rad e saturação
sincronizada de 0,12 rad por ciclo. Isaac usa passo de física de 1/60 s, mantendo
PhysX e tensores em CUDA na GPU física 1. O desempenho efetivo continua inferior
a tempo real sob carga; o ajuste reduz o atraso em relação ao passo de 1/200 s,
sem afirmar execução em tempo real. O enquadramento aproxima o braço.

A página distingue avaliações antigas de avaliações do vídeo atual e mostra o
comentário anterior sem reutilizar sua aprovação/reprovação na nova versão.

### Revisão dos dois giros restantes

Os vídeos de inclinar os dedos e virar a palma exibem ângulos assinados da mão
e da garra medida, calculados em `review_rotation.py`. O sinal da mão usa a
rotação espacial; o da garra usa a rotação local relativa à orientação ancorada.
As duas métricas compartilham os eixos de `cartesian_review.target`; não usam
ângulos do alvo como se fossem medições. A haste laranja na cena acompanha
o TCP e a orientação extraídos do estado da articulação PhysX.

O passo de orientação de IK desses dois exercícios fica em 0,08 rad por ciclo, com saturação sincronizada de 0,06 rad nas juntas.
O ganho de rotação passa a 1,2 para evitar saturar ambos em 75 graus. A escala
de damping das juntas do simulador passa a 0,25 (`FR3_SIM_ARM_DAMPING_SCALE`);
nenhum parâmetro do controlador físico é modificado. O ajuste de propriedades
de drive usa os metadados host exigidos pela API PhysX; dinâmica e estados
continuam em CUDA na GPU 1. O valor original é recuperável com escala 1.

Geração seletiva: `run_cartesian_review.py --resume --feedback --profiles=pitch,yaw`.
O início espera backend CUDA recente e feedback recente do simulador; os demais
vídeos e avaliações continuam associados aos mesmos jobs. Tentativas que falham
ficam no histórico; apenas novas execuções concluídas são apresentadas para avaliar.

A seleção final usa ajuste específico por eixo: pitch com passo de orientação
0,08 rad e juntas 0,06 rad; yaw com 0,12 rad e juntas 0,12 rad. O giro lateral
da tentativa `94e90a836c3e` foi mantido por mostrar os dois sinais; a tentativa
posterior mais lenta não conseguiu acompanhar o trecho negativo. A escolha e
todas as tentativas ficam registradas no relatório, sem aprovar pelo usuário.

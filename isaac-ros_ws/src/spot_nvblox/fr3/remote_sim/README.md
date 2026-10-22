# ZED no notebook → FR3 + cuRobo na A2000

## Sessão local em 22/10/2026

- Notebook `mhc-lenovo` / `100.118.129.74`, usuário `mhc`, ZED 2i serial 34046618, USB3. Arquivos em `/home/mhc/fr3-zed-remote`.
- A2000 `hipolito-tailscale` / `100.88.134.127`: Isaac Sim 6.0 na GPU física 0, cuRobo na GPU física 1. Dentro de um container com uma única GPU visível, o índice CUDA é **0**.
- Painel de simulação: **http://100.88.134.127:8791/**. O painel existente do laboratório na 8788 foi preservado. A 8790 já pertence a outro servidor.
- Containers próprios: `fr3-zed-sender` no notebook; `fr3-remote-isaac` e `fr3-remote-control` na A2000. O Isaac usa IPC privado (2 GiB) e DDS UDP para evitar colisões `carb-RStringInternals-11` entre instâncias Kit em containers. DDS 188, comandos exclusivamente `/fr3_sim/*`. Nenhum driver físico é iniciado.

O notebook captura HD720/15 fps e calcula profundidade no modo **PERFORMANCE** do SDK 5.0.7. Envia RGB JPEG 640×360 e profundidade alinhada 320×180, quantizada em milímetros e comprimida com zlib. A2000 amplia a profundidade com vizinho mais próximo para 640×360, roda `operator_hand_tracker.py` (MediaPipe GestureRecognizer em modo VIDEO, quadro completo) e o MPC cuRobo. O rastreador desta sessão usa a mão direita diretamente, sem depender da detecção dos ombros ou do cotovelo. O detector físico anterior não foi substituído.

Para esta sessão relativa, a escala do braço é fixa em 1,315 (855/650 mm); `online_scale_estimation:=false` evita bloquear o primeiro teste nos 90 quadros da calibração de comprimento. O Start ainda calibra posição e orientação relativas. A configuração física do laboratório não foi alterada.

O transporte TCP 30020 envia somente o quadro mais recente a cada solicitação; mantém o timestamp da captura. Não retransmite uma fila de quadros antigos. Nas primeiras medições úteis, a idade na recepção ficou em aproximadamente 150–200 ms, com variações. A câmera é acessível pela interface Tailscale; o protocolo pressupõe dispositivos confiáveis na mesma tailnet.

O stream UDP nativo do SDK mostrou perdas de metadados mesmo com chunks 1024. Os arquivos `zed_sender.cpp` e `zed_stream_receiver.py` permanecem para diagnóstico e **não são o caminho ativo**. A compilação do modelo NEURAL_LIGHT esgotou RAM/swap do notebook; foi encerrada, sem encerrar os aplicativos do usuário. PERFORMANCE evita essa preparação e consome menos memória, com menor qualidade de profundidade.

## Operação

1. Abra o painel na 8791. Confira imagem da simulação e imagem do operador.
2. Mantenha a câmera na posição normal, sem rotação, e enquadre a mão direita, com a palma visível. Confira a idade da captura e a indicação de rastreamento no painel.
3. Clique **Start teleop ZED**, mantendo a mão parada durante a calibração (3 amostras novas e pelo menos 0,4 s). O prazo de calibração é 60 s.
4. Translacionar/girar a mão controla a pose relativa à calibração. Palma aberta abre a garra, punho fechado fecha; gestos precisam ser confirmados em 3 quadros/0,35 s. Use os botões da garra para verificar a mecânica independentemente do reconhecimento.
5. **Parar** segura a configuração medida; **Home** volta à pose TCP observada na inicialização da sessão de controle. Os botões de deslocamento/giro permitem conferir o MPC sem mão rastreada.

Velocidade inicial 50% dos limites do joystick: 0,3 m/s no plano, 0,2 m/s vertical, 45°/s; slider até 0,6 / 0,4 / 90°. O objetivo tem limite de afastamento de 6 cm/30° da pose medida. O workspace fica restrito à bancada. A velocidade observada também depende da física, da latência de captura e da precisão do MPC.

Na **cena equivalente**, avançar o braço significa +X do robô, em direção aos objetos. O backend físico mantém sua configuração separada `zed_forward_sign=-1`, como solicitado para a bancada real. Não copie esse sinal entre cenas sem conferir a orientação da base.

A captura remota deve chegar ao detector em até 0,55 s (parâmetro somente desta sessão; o padrão físico continua 0,30 s). Amostras 6D pareadas devem ter menos de 0,65 s para controle. Perda pausa o movimento; recuperação curta exige estabilidade e recalibra a referência relativa; a recuperação automática exige 3 amostras novas durante pelo menos 0,25 s e ancora novamente na pose medida do robô, evitando saltos. Após 30 s de pausa a sessão desarma e exige Start. Parar sempre desarma, mesmo que a mão reapareça. Timestamps repetidos ou orientação sem o mesmo timestamp da posição são rejeitados.

## Cena e colisões

`build_scene.py` gera `remote_workcell.usda` com o modelo FR3 oficial público NVIDIA, mesa, furadeira feita de primitivas e três obstáculos. `isaac_remote_arm.py` adapta a ponte ROS/física do `franka_ros_arm.py` do Murilo, encontrado em `/home/murilo/nvblox_sim_l40s/franka_mpc/`.

**A cena não é a cópia exata da imagem do Nexus.** Os assets privados não estão na cópia local e o login SSH tentado não foi aceito. O mundo de colisões cuRobo usa cuboides fixos da mesa e dos três obstáculos, mais autocolisão; a furadeira é tratada como objeto alvo. **Não há nvblox/ESDF ou SAM2 ao vivo nesta sessão remota.** Obstáculos empurrados na física não atualizam esses cuboides. Esta bancada permite depurar teleoperação, orientação e garra; não valida a pipeline completa de percepção de obstáculos.

## Reiniciar

Pré-requisitos já disponíveis nesta máquina: imagens `spot-isaac-sim:6.0.0` e `fr3-curobo-remote-sim:checkpoint`; repo montado; dependências Python locais `tmp/cv_site_hipolito` e `tmp/mp_site_hipolito`; `/opt/curobo` compilado e mensagens nvblox no snapshot. A imagem checkpoint foi criada a partir do container cuRobo existente, para evitar modificar/recriar o container compartilhado com acesso GPU inválido.

Na A2000, a partir desta pasta:

```bash
bash run_isaac.sh > /tmp/fr3-remote-isaac.log 2>&1 &
bash run_control.sh
```

Os scripts da A2000 recusam nomes de containers já existentes. O controlador usa NVIDIA CDI (`--device=nvidia.com/gpu=1`), disponível neste host, para evitar a perda de acesso à GPU observada com o runtime legado. Para reiniciar somente esta sessão, pare/remova **apenas** `fr3-remote-isaac` e `fr3-remote-control`, e execute novamente. `cache/` e `kit_cache/` guardam caches entre reinícios. A primeira carga de assets/shaders do Isaac pode levar vários minutos. O modelo FR3 depende do asset público NVIDIA por HTTPS.

No notebook:

```bash
cd /home/mhc/fr3-zed-remote
bash run_sender.sh
```

O notebook já tem imagem `stereolabs/zed:5.0-runtime-cuda12.8-ubuntu22.04`, NVIDIA Container Toolkit com CDI em `/etc/cdi/nvidia.yaml` e caches SDK persistentes `settings/` / `resources/`. Não é necessário instalar ROS no notebook. O emissor usa somente os dispositivos ZED encontrados em `/dev/v4l/by-id/*ZED*` e USB SDK, com serial 34046618 fixado no código. O launcher substitui somente sua instância anterior parada e conserva as últimas linhas do log; limita as tentativas de reinício. O TCP usa TCP_NODELAY. Start no painel controla a sessão de teleoperação; o emissor do notebook deve estar iniciado separadamente.

Para recompilar na A2000: `bash build_sender.sh` (usa SDK/CUDA/g++/libjpeg/zlib do `zed-build`). Copie `bin/zed_rgbd_sender` e `run_notebook_sender.sh` para `bin/zed_rgbd_sender` e `run_sender.sh` no notebook com o login SSH habitual. Nenhuma senha é salva nos scripts.

Logs:

```bash
docker logs --tail 10 fr3-zed-sender  # no notebook
docker exec fr3-remote-control tail -20 /tmp/fr3_remote_rgbd.log
docker exec fr3-remote-control tail -20 /tmp/fr3_remote_perception.log
docker exec fr3-remote-control tail -20 /tmp/fr3_remote_session.log
```

Para encerrar: pare os dois containers próprios da A2000 e o emissor do notebook. Não pare os containers antigos do laboratório.

Validação: os 74 testes existentes de teleop/percepção passaram após a mudança no transporte de `Image.data`. Benchmark local da atribuição de 640×360×4 bytes: aproximadamente 119 ms com `bytes`, 0,03 ms com `array('B')`. `python3 smoke_test.py` verifica deslocamento, giro, abertura/fechamento medidos e Home no simulador; ele recusa um servidor sem identificação explícita de simulação/DDS 188.

A ponte executa explicitamente a atualização ROS/controle antes de cada passo de física: `SimulationManager.step` direto não dispara o evento `PHYSICS_PRE_STEP` da timeline. O avanço automático da timeline é desativado para manter uma única contagem de tempo. A captura contínua usa `app.update` e o anotador RGB, sem aguardar `rep.orchestrator.step` após passos manuais.

Resultado do teste de física/MPC desta sessão: deslocamento solicitado de −30 mm em Y terminou em −22,26 mm (tolerância de término 8 mm); giro solicitado de 10° mediu 8,31°; fechamento chegou a 0,44 mm entre dedos e abertura a 79,6 mm; Home completou. Estado da simulação foi recebido com idade de cerca de 20–30 ms e MPC em aproximadamente 9–14 ms. O teste ao vivo com a câmera foi adiado a pedido do usuário; a ZED estava desconectada do USB no encerramento. O binário e o launcher atualizados já foram copiados para o notebook.


## Rastreamento, orientação e recuperação

6D significa três coordenadas da posição do **punho** e três graus de orientação obtidos da **palma** (landmarks do punho e metacarpos). Não é uma medição dos ângulos anatômicos da articulação do punho. Mover o braço com a palma rígida muda a posição; inclinar/girar a palma muda a orientação relativa do TCP. O painel apresenta essas duas partes separadamente.

A transformação câmera → robô é uma rotação própria: aproximar a mão da câmera aumenta X em direção à mesa, subir na imagem aumenta Z e ir à direita aumenta Y. A calibração alinha a orientação neutra da palma com a pose atual da ferramenta; não força a garra para baixo na primeira amostra. Confira os sinais na cena antes de reutilizar outra montagem de câmera.

A posição usa filtro One Euro adaptativo e rejeição de saltos. A profundidade do punho é estimada com amostras consistentes do punho e dos quatro metacarpos, todas do mesmo quadro de profundidade; não reutiliza profundidade antiga durante uma oclusão. Posição e orientação publicadas têm o mesmo timestamp. O reconhecimento de abertura também usa a extensão geométrica dos dedos; o bloqueio de orientação durante a transição da garra dura apenas 0,4 s ao fechar / 0,35 s ao abrir.

Antes de liberar Start, o backend prepara o CUDA graph do MPC com três passos sem publicar comandos. Nesta máquina o primeiro passo levou cerca de 787 ms; fazê-lo durante o primeiro movimento acionava indevidamente a pausa e absorvia o deslocamento na nova referência. O painel agora mostra preparação, progresso da calibração, pausa, último comando HTTP e erros persistentes. Cliques repetidos em Start não reiniciam uma sessão já ativa. O MPC usa custo de pose cartesiana, sem atração à configuração inicial das juntas, e segura a configuração medida se o resultado for inviável.

A física avança em passos de 200 Hz independentemente da captura RGB 640×360. O relógio considera o tempo gasto na renderização, sem executar atualizações gráficas extras nos passos de física. Foram medidos aproximadamente 198–200 Hz e fator de tempo real 0,99–1,00; o render é mais lento e varia com a carga. Sob carga posterior compartilhada, a taxa caiu para aproximadamente 49–74 Hz e fator 0,25–0,37, com render próximo de 0,3 fps. Portanto 200 Hz é a meta e uma medição obtida, não uma garantia sustentada neste host. As métricas atuais aparecem no painel. Isso não promete a mesma taxa de quadros da câmera ou do rastreador.

Referências consultadas: [MediaPipe GestureRecognizer](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer/python) para rastreamento VIDEO e redetecção; [hand-control-robot](https://github.com/advaithdasu/hand-control-robot) e [handrobot](https://github.com/guptabhishekumar/handrobot) para comparar filtragem One Euro, referência relativa e retenção na perda; [NVIDIA CDI](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/cdi-support.html) para acesso à GPU. A adaptação foi implementada localmente; não há benchmark comparativo de precisão entre projetos.

Validação adicional: `python3 -m unittest discover -s . -p 'test_*.py'` passou 16 testes de filtragem, eixos, fusão de profundidade, timestamps, calibração e recuperação. `check_start_recovery.py`, executado dentro do container ROS com a câmera parada, exercitou amostras sintéticas via ROS e comandos reais HTTP: Start, clique repetido, movimento medido, pausa maior que 1,5 s, rejeição de dados antigos, recuperação em outra posição sem salto e Stop persistente passaram. O deslocamento de mão solicitado de 15 mm produziu cerca de 7,45 mm no robô em 2,5 s: o MPC mantém erro residual milimétrico, e este teste não valida precisão de captura da mão real. Ambos os testes de integração comandam somente a simulação e encerram parados.

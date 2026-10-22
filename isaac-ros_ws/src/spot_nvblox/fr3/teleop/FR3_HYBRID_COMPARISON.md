# Spot, G1 e Franka: comparação e escolhas

Revisão de código em 22/10/2026. Não é um benchmark comparativo de precisão nem uma validação de movimento do robô real.

## Versões efetivamente consultadas

- Spot oficial `master`, `9e22d7b17b77852cb81c720a07a7152499660e7e`: `arm_pose_estimator.py` e `hand_orientation_estimator.py`. É a versão antiga com referência visual e orientação simplificada.
- Spot oficial `dev`, `eb03146a00f130533cbeb02492ae75aae3b5c934`: `wrist_detector.py`, `hand_pose_estimator.py` e `curobo_mpc_node.py`. É a base relevante do nosso workspace; não confundir com `master`.
- G1 **`main`**, `181e73694fc78a4bfea9b5eb18189c6064fa36a7`: pipeline `dexhand/src/dex_hand`, incluindo modelo da mão, frames, movimento, grip, filtros, transporte e IK. Não confundir com o antigo `master` ROS.
- Franka local: `remote_sim/operator_hand_tracker.py`, `arm_calibration.py`, `hand_tracking_math.py`, backend `teleop/fr3_input_teleop.py` e variante real `fr3_input_teleop_nobox.py`.

## Comparação por função

| Função | Spot oficial | G1 main | Escolha para a Franka |
|---|---|---|---|
| Escala da pessoa | `dev`: mediana/IQR do comprimento ombro–cotovelo–punho; 90 amostras. | Calibra proporções da mão; escala métrica pela profundidade, régua opcional ou escala média MediaPipe. Referência pelo ombro está em estudo. | **Ativo:** escala do braço por pessoa, sem fallback fixo no caminho real. Pelo menos 20 medidas distintas e 2,5 s; consenso dominante de comprimento. Escala permanece fixa durante a sessão para evitar saltos. |
| Dependência do corpo | `dev`: posição e eixos dependem da Pose e referências do corpo. | Mão é a entrada principal; Pose roda para gravação de estudo do corpo. | **Ativo:** corpo calibra/monitora escala e ajuda a associar a mão à pessoa; após calibrar, ausência breve do ombro não invalida por si só a mão. Landmarks do corpo continuam no painel quando medidos. |
| Profundidade | `master`: mediana local do punho e até cinco quadros de predição; `dev`: RGB-D sincronizado e filtros. | Ajuste cinemático robusto das observações 2D/3D, rejeição por erro de reprojeção e saltos de profundidade. | **Ativo:** fusão de punho e quatro metacarpos do mesmo quadro de profundidade. Sem profundidade atual válida, não prever movimento do robô. Modelo cinemático completo do G1 fica para avaliação separada. |
| 6D | `master`: orientação simplificada; `dev`: referencial do corpo e entrada de rotação separada. | Referencial completo da palma; padrão da VM é orientação absoluta a cada retomada. Também suporta relativo e clutch. | **Ativo:** posição do punho + orientação 3D da palma, pareadas no mesmo timestamp. Referência relativa ao TCP medido no Start, sem forçar uma orientação inicial. Eixos/sinal de avanço próprios da bancada FR3. |
| Filtragem | EMA e rejeição de saltos. | One Euro por landmark, modelo da mão e suavização de pose com covariância. | **Ativo:** One Euro de posição, filtro de quaternion e rejeição geométrica. Não acrescentar todos os filtros em cascata sem medir o atraso. Filtro/modelo por landmark do G1 ainda não foi portado. |
| Perda da mão | Filtros e predições locais não equivalem a um watchdog de comandos completo. | Mantém âncora por perda curta; libera após 15 quadros; ao retomar, reancora. | **Ativo:** amostra velha interrompe seguimento imediatamente. Lacuna inferior a 0,8 s e retorno próximo preserva a referência; retorno mais distante exige estabilidade e reancora no TCP medido. Após 5 s desarma. Stop impede retomada automática. Usar segundos, não copiar 15 quadros para uma câmera com outra taxa. |
| Abrir/fechar | Classificador de gestos em nó separado. | Distância punho–pontas normalizada pelos ossos; mediana de médio/anelar/mínimo e pinch; retargeting de mão dexterosa. | **Ativo:** extensão geométrica ajuda OpenPalm/ClosedFist, confirmada com debounce. Conflito forte entre geometria e classificador segura o estado anterior. Pinch é apenas diagnóstico; articulações da mão G1 não se aplicam à garra paralela. |
| Mudança de gesto | Entradas de pose e gesto separadas. | Clutch por variação das distâncias dos dedos, segurando pose durante a preensão. | **Ativo:** posição/orientação/gesto têm um timestamp; clutch de orientação nas transições. Clutch completo de posição por velocidade dos dedos é candidato para teste, ainda não foi adotado. |
| Transporte/atraso | Nós ROS e overlay passam por diferentes filas. | Filas latest-only; eventos separados do fluxo descartável. | **Ativo:** worker RGB-D com apenas a última amostra pendente; preview JPEG independente, uma requisição por vez no navegador; diagnósticos não renovam watchdog de movimento. |
| Controle e colisões | `dev`: MPC cuRobo com estado real, autocolisão e ESDF/nvblox. | cuRobo V2 com warm start, fallback de sementes, limites de passo, autocolisão/mesa no MuJoCo. | **Real atual:** controlador cartesiano usado pelo joystick, mesmos limites de velocidade e limites de erro. **Não está ligado ao cuRobo/PF de obstáculos.** Simulação FR3 tem MPC e obstáculos fixos; não é prova de desvio no robô real. |

## Correções instaladas nesta rodada

1. Start comum preserva a calibração da pessoa e as amostras acumuladas; reinicia apenas a referência da mão. Cliques repetidos não apagam o progresso.
2. Consenso de comprimento: grupo de até 6 cm contendo pelo menos 20 medidas e 65% da janela, com IQR menor que 3,5 cm. Medidas anatômicas/profundidades inválidas não contam. O painel mostra a causa e a contagem; não arma só por decurso de tempo.
3. Pequenas lacunas de frescor não zeram imediatamente a estabilidade de partida. Comandos continuam exigindo amostra atual.
4. Recuperação curta preserva a referência somente se o retorno estiver próximo: menos de 4 cm e 12 graus. Não reancorar em cada pequeno dropout.
5. Medida geométrica de abertura dos dedos, inspirada no G1, com testes de invariância a escala/rotação e rejeição de um dedo discrepante.
6. Botão **Nova pessoa / recalibrar**: Stop confirmado, reset explícito da escala, novo Start. Start normal não recalibra o comprimento já aceito. Percepção recebe reset por UDP loopback 47815.
7. Landmarks originais do MediaPipe para corpo e dedos, disco verde no punho e eixos. TF medido `fr3_operator_right_wrist` no frame óptico real; nenhuma extrínseca câmera→base foi inventada.
8. Preservadas as alterações concorrentes do controlador: variante `nobox`, argumentos de processo e exigência de joystick neutro após reengate. Nenhum ganho do controlador físico foi alterado.

## O que testar antes de escolher novas adaptações

O README do G1 declara que o modelo cinemático da mão foi avaliado com dados sintéticos e ainda não testado numa mão real. Portanto não há evidência para prometer que ele será mais rápido ou preciso nesta ZED. Comparar em gravação idêntica: disponibilidade de pose válida, erro/ruído de posição e rotação, latência de captura até saída, falsos comandos da garra e recuperação após oclusão. Somente então adotar HandFit, covariância ou clutch completo.

A transposição do MPC exige modelo FR3, TCP correto, TF/extrínsecas medidos e obstáculos atuais; não copiar limites articulares, mesa, torques ou rotinas de inicialização do G1/Spot para o braço real.

## Operação atual e validação

Painel **http://100.88.134.127:8788/**, recarregar após atualização. Na calibração inicial, enquadrar ombro, cotovelo e punho direitos com profundidade válida; a mesa atualmente esconde ombro/cotovelo de uma pessoa sentada. Depois da calibração o seguimento da mão é independente da presença contínua dos ombros. O comprimento é uma estimativa visual, não uma medição antropométrica validada.

26 testes de matemática/calibração e 51 testes isolados de backend/dashboard passaram (48 anteriores mais três novos). Os novos testes verificam guarda HTTP, ordem Stop→reset→Start e recusa de Start se Stop falha. Nenhum teste aciona serviços de movimento reais. O painel real permanece desengatado; o operador inicia o teste.

Snapshot HTTP 200 observado após atualização, aproximadamente 183 ms de idade de captura numa amostra; inferência variou aproximadamente 3–8 Hz sob carga. Resposta HTTP de poucos milissegundos não significa atraso de câmera de poucos milissegundos. Os dois containers próprios da simulação anterior foram parados para liberar recursos; os containers compartilhados da percepção e controle real foram preservados. Não há benchmark controlado provando aumento de FPS.

## Fontes primárias

- [Spot master: arm_pose_estimator.py](https://github.com/EESC-LabRoM/spot-teleop/blob/9e22d7b17b77852cb81c720a07a7152499660e7e/spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/arm_pose_estimator.py)
- [Spot dev: wrist_detector.py](https://github.com/EESC-LabRoM/spot-teleop/blob/eb03146a00f130533cbeb02492ae75aae3b5c934/spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/wrist_detector.py)
- [Spot dev: curobo_mpc_node.py](https://github.com/EESC-LabRoM/spot-teleop/blob/eb03146a00f130533cbeb02492ae75aae3b5c934/spot-ros2_ws/src/spot_operation_ros2/spot_operation_ros2/curobo_mpc_node.py)
- [G1 main: README e limites da validação](https://github.com/EESC-LabRoM/G1_teleop/blob/181e73694fc78a4bfea9b5eb18189c6064fa36a7/dexhand/README.md)
- [G1 main: pipeline](https://github.com/EESC-LabRoM/G1_teleop/blob/181e73694fc78a4bfea9b5eb18189c6064fa36a7/dexhand/src/dex_hand/pipeline.py)
- [G1 main: movimento](https://github.com/EESC-LabRoM/G1_teleop/blob/181e73694fc78a4bfea9b5eb18189c6064fa36a7/dexhand/src/dex_hand/motion.py)
- [G1 main: geometria da preensão](https://github.com/EESC-LabRoM/G1_teleop/blob/181e73694fc78a4bfea9b5eb18189c6064fa36a7/dexhand/src/dex_hand/grip.py)
- [G1 main: modelo da mão](https://github.com/EESC-LabRoM/G1_teleop/blob/181e73694fc78a4bfea9b5eb18189c6064fa36a7/dexhand/src/dex_hand/hand_model.py)
- [G1 main: controle IK](https://github.com/EESC-LabRoM/G1_teleop/blob/181e73694fc78a4bfea9b5eb18189c6064fa36a7/dexhand/src/dex_hand/arm_ik.py)

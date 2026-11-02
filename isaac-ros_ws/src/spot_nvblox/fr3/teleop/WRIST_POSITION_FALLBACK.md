# Punho independente da orientação — 2026-10-23

O caminho anterior descartava a translação medida quando o ajuste da palma era rejeitado. Agora os pacotes v3 declaram separadamente `position_valid` e `orientation_valid`. O backend aceita translação atual sem orientação, mantém o quaternion comandado e reancora somente a rotação na primeira orientação recuperada. A referência de translação permanece. Calibração de início usa estabilidade da posição; não exige estabilidade angular no protocolo v3.

Prioridade: punho do Hands com profundidade atual; se não houver mão identificável, RIGHT_WRIST do Pose com profundidade do MESMO quadro. Na ausência de dedos, Pose processa o quadro atual, sem reutilizar uma observação antiga como nova. Após calibrar a pessoa, o punho não depende de ombro/cotovelo continuamente. A associação usa proximidade ao último punho; após ausência prolongada, ombro atual compatível com a calibração e alcance plausível permitem recuperar a identidade. Nenhum movimento é extrapolado; sem punho e profundidade atuais, as guardas de perda continuam valendo.

A escala inicial por pessoa e os limites de velocidade, workspace, erro e avanço permanecem. Não há gesto nem novo TF de orientação nos pacotes de posição apenas. O retorno do punho pode manter o controle ativo, mas não inventa uma abertura/fechamento da garra. Nenhum Start, Home ou comando de garra foi disparado na validação real.

Referências verificadas: [Spot master, arm_pose_estimator.py](https://github.com/EESC-LabRoM/spot-teleop/blob/master/spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/arm_pose_estimator.py) usa o punho de Pose para posição com profundidade local. [G1 main, motion.py](https://github.com/EESC-LabRoM/G1_teleop/blob/main/dexhand/src/dex_hand/motion.py) mantém referências relativas, limita velocidade angular e reancora após perdas. O protocolo de validade independente e a recuperação apenas angular são adaptações para esta Franka; o G1 não oferece diretamente esse fallback para Pose. Não copiamos a predição de posição do Spot para o braço real.

Validação: 47 testes de backend, incluindo quatro novos de posição sem palma, congelamento/retorno da orientação e watchdog de posição; sete testes de percepção com RGB-D sintético, incluindo palma rejeitada, landmarks de orientação ausentes, dedos ausentes, profundidade ausente, Pose antigo, calibração ausente e recuperação de identidade. Domínios isolados 191/193, publicador de movimento substituído por mock. Os testes fixam explicitamente o sinal de eixo histórico -1; o runtime do laboratório preserva +1 configurado pela outra sessão, Home articular e guarda de recentralização do joystick.

## Referência do ombro e recuperação visual — segunda atualização

A inspeção na ZED encontrou um punho de Pose com confiança 0,649 e profundidade atual válida, rejeitado pelo corte 0,65. Agora uma confiança intermediária pode ser apoiada por ombro/cotovelo atuais ou pontos da mão com profundidade coerente, alcance plausível e associação temporal. Um ponto de confiança muito baixa continua insuficiente para comandar o braço.

A translação usa `punho - ombro_filtrado + ombro_de_referência`. Se o ombro faltar, a referência é mantida e o punho atual continua sendo usado. A TF de percepção conserva a posição física medida na câmera; não recebe a posição compensada para controle. A associação pode usar o braço completo atual com segmentos e comprimento compatíveis com a calibração, permitindo que o mesmo operador se levante após calibrar sentado.

Se Hands falhar no quadro inteiro, uma segunda detecção procura num recorte ao redor do punho de Pose. Esse recorte pode ser sugerido por um ponto de baixa confiança, mas ele apenas define onde procurar: não autoriza movimento. Os landmarks detectados são remapeados para o RGB-D completo, e a mão precisa passar pelas verificações normais de profundidade e associação. Na comparação somente de leitura com a câmera atual, o detector encontrou zero mãos no quadro inteiro e uma no recorte do punho.

Uma referência recente da escala medida pode ser restaurada após reiniciar a visão (até 120 s, mesma matriz da câmera), sem fabricar uma amostra de punho nem engatar o robô. Nova pessoa elimina esse estado. Nesta atualização, a escala aceita de 0,47657 m foi preservada e a referência do ombro foi medida novamente com o controle parado.

24 testes isolados passaram: incluem punho com confiança intermediária e apoio corporal, dedo com profundidade coerente, compensação do torso, ausência/retorno do ombro, restauração/expiração da escala, coordenadas do recorte e recuperação de associação após levantar da cadeira. O controlador físico não foi modificado.

Validação somente de leitura com o operador presente, janela de aproximadamente 10 s: 51 diagnósticos, 50 amostras de posição válidas (47 `hand_wrist`, 3 `pose_wrist`); 44 orientações válidas e seis amostras de posição sem orientação. Um quadro rejeitado por duas mãos próximas. Faixa de posição normalizada para controle [0,439; 0,601; 0,456] m nos três eixos, demonstrando atualização da posição durante o movimento do operador. Isso valida a continuidade observada da percepção nessa janela, não o movimento físico nem ausência garantida de perdas futuras.

## Retomada com a mão em movimento

Após ativação autorizada pelo operador, foi observado deslocamento físico do TCP. A inspeção seguinte encontrou intervalos de captura de até 633 ms, maiores que o watchdog de 350 ms. O controle parava corretamente, mas a retomada exigia uma janela estacionária de 650 ms e podia expirar em 5 s mesmo com a percepção já válida e o operador continuando a mover.

Para os pacotes v3, a retomada aceita duas posições atuais distintas, separadas por até 500 ms e coerentes com o limite de salto da posição. Reancora ao TCP medido e à mão atual, descartando movimento durante a ausência. O watchdog e os limites de comando permanecem. O Start inicial continua exigindo estabilidade; os protocolos antigos mantêm sua retomada anterior. 50 testes de backend passaram, incluindo retorno com mão em movimento, recusa de replay e recusa de posições incompatíveis.

Teste físico autorizado após a atualização: houve três transições pausa→ativo sem novo Start, mantendo o engate. Na janela observada, a distância comandado–medido do TCP foi em média 10 mm e no máximo 33 mm. O TCP saiu de [0,2621; -0,2109; 0,3493] para [0,2364; -0,1921; 0,4085] m. As pausas por falta de dados atuais ainda ocorreram; a alteração corrige a exigência de imobilidade na retomada, não a frequência irregular dos quadros.

## Limite de alcance e frequência RGB-D

A retomada também podia desarmar ao detectar o TCP transitoriamente fora do workspace enquanto assentava junto à borda X=0,10 m. Agora permanece pausada e aguarda o TCP voltar para dentro, sem publicar movimento fora do limite; continua sujeita ao timeout de recuperação. O Start inicial mantém sua recusa quando o TCP está fora.

Ao saturar o workspace, a referência de translação é deslocada pelo excesso inalcançável, seguindo a estratégia de `limit_position` do G1. Assim, inverter o movimento da mão altera imediatamente o alvo alcançável, sem precisar percorrer de volta toda a distância comandada além do limite. Não muda os limites de velocidade, avanço, erro ou workspace. 52 testes de backend passaram, incluindo saturação/reversão e espera de assentamento sem publicação.

Medição independente de tópicos durante cerca de 9 s: RGB 14,9 Hz, profundidade 8,3 Hz, processamento 7,2 Hz. Maior intervalo entre capturas: RGB 534 ms, profundidade 433 ms, processamento 500 ms. Idade média no recebimento: RGB 44 ms, profundidade 47 ms, diagnósticos 110 ms. Há perda/irregularidade antes da inferência: o recebimento da profundidade é um gargalo restante; esta medição não identifica sozinha a causa no SDK, publicação ou transporte.

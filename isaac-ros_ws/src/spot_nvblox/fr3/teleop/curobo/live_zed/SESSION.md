# ZED → Murilo cuRobo → Franka, sessão de 30/10/2026

O MPC agora carrega diretamente `/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py`, montado como `/node/curobo_mpc_node.py`, pelo adaptador `murilo_current.py`. Hash e arquivo realmente carregados em `logs/murilo_source.json`. O adaptador acrescenta metadados de frescor/cobertura no limite de publicação privado, preservando o algoritmo e as trajetórias nativas do Murilo. A fonte dele não foi editada. A publicação nativa vai apenas para `/fr3_joystick_mpc/joint_preview` (JointTrajectory). A ponte física valida frescor, viabilidade, cobertura, limites e propriedade do controlador.

Configuração atual: GPU física 0; DDS142; modelo FR3 vcap1.2_acc10; `command_type=joint_trajectory`, `command_period=0.02`, `mpc_on_infeasible=keep`, ESDF 4 Hz. Essas opções são as do novo código do Murilo; os limites da ponte continuam aplicados. A simulação própria foi parada, e os 13 vídeos da revisão continuam acessíveis.

A ZED da mão (zed3, serial34046618) foi recuperada reiniciando apenas seu driver após reconexão USB. ZED1 de mapeamento também envia dados. ZED2 de mapeamento (39498278) informa CAMERA NOT DETECTED. Não se afirmou cobertura observada por duas câmeras.

A conexão Franka foi recuperada com reinício passivo do fr3-ros2 após erro de comunicação. Broadcasters de estado ativos; controlador de trajetória do Murilo carregado INATIVO, com seu YAML atual. Input ZED roteado para `/fr3_curobo`, mapeamento `review`, exercício `full_6d`; o controlador cartesiano direto não recebe os alvos desta sessão.

O mapa dedicado de planejamento continha resíduo sobre `fr3_link1` (duas esferas do ombro, sobreposição estimada de 3,4 e 1,8 cm). Diagnóstico separou limites=0, auto-colisão=0, mundo=105,58. Foi salvo o mapa completo antes de reconstruir SOMENTE nvBlox_curobo. O mapa principal nvBlox_human permaneceu intacto. Após reconstrução por profundidade segmentada atual: world collision cost=0 e comandos viáveis. Não foram desativadas verificações de obstáculos. Backup `.nvblx` em `backup/`; comando do mapa em `logs/nvblox_refresh_command.json`.

O Start do dashboard 8088 (também encaminhado pelo painel 8788) verifica comando válido e fontes recentes, propriedade exclusiva da saída e controladores. Somente após o clique do operador ele ativa o controlador de trajetória, arma a ponte e inicia calibração ZED. Uma falha ao armar desfaz a troca. Parar cancela preparação pendente, desengata a mão e desarma a ponte; há testes das corridas de cancelamento. Os modos de exercício só mudam parado.

Estado na verificação final: desarmado, engaged=False, controlador de trajetória inativo, zero comandos físicos enviados. Vinte amostras consecutivas anteriores deram command_ready=True. O teste foi de preview, sem invocar Start físico.

Validações: 3 testes do adaptador sobre a fonte atual; 16 da política de Start/Stop e validação de pacotes; 9 do dashboard; 3 de propriedade do controlador no ambiente ROS, sem escrita de hardware. `logs/murilo_integration_result.json` contém a evidência final.

Rollback: container anterior preservado (nome em `logs/murilo_launch.json`); mapa anterior e fontes anteriores em `backup/`. `.curobo_enabled` seleciona a entrada revisada no próximo real-teleop. Um reinício completo da infraestrutura ainda requer recompor o preload inativo, relays e preview; o sistema não auto-arma o robô ao reiniciar.

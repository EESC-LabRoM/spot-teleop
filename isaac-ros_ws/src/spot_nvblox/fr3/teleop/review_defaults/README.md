# Configuração instalada para Start da ZED real

Padrão persistido em `/home/murilo/franka_ws/lab/fr3_joy_teleop.yaml`: mapeamento `review`, exercício `full_6d`, ganho de translação 1, garra por gesto habilitada. Eixos relativos: `[handZ, handY, -handX]`. O pacote já tem a escala corporal. Ganhos angulares: roll 1,6; pitch/yaw 1,2; faixa máxima 75 graus. O limite de deslocamento do perfil é 20 cm por eixo a partir da referência.

O processo ativo usa `fr3_input_teleop_nobox.py`, mantendo o controlador cartesiano direto existente e seus limites. O próximo Start não habilita cuRobo. O modo 6D combina eixos; o seletor “Movimento ZED” no painel 8088 escolhe os exercícios isolados da revisão. Troca recusada durante controle ativo, calibração, pausa ou Home. No reinício volta a 6D livre revisado. A mudança de modo pelo painel vale para a sessão atual.

Instalados runtime, módulos de mapeamento, backend do painel e `fr3_dashboard.html`. `lab_dashboard.html` neste diretório é uma versão preparada; não foi instalada no painel agregado de Murilo, cujo proxy não encaminha a rota nova. Use o painel direto 8088 para selecionar exercício.

Verificados 59 testes na fonte canônica e 56 no runtime exato, em DDS isolado. Verificada troca roll → full_6d pelo painel sem Start, Home ou ação da garra. Após reinício: engaged=False e published=0. Rastreamento sem dados recentes e controlador não confirmado ativo; as proteções continuam bloqueando engate nessas condições.

`installed.json` identifica os parâmetros, runtime, comandos de reinício e backup. O backup inclui runtime anterior, módulos, dashboard e YAML. A configuração fica pronta para a próxima ativação; não comprova validação física de 6D livre nem corrige sozinha latência da percepção/TF.

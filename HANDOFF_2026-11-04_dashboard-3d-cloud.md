# Handoff — Visualização 3D do dashboard: nuvem + follow ao vivo (2026-11-04)

Lane: `Visualização 3D do dashboard` (projeto RAL). Session `01a1a6bd-241d-7000-bbde-925a73cc0879`.
Colabora com: `Grasp no robô real` (session `58e05c74ecf7448a86a47e84c72e04cf`) e `Recuperação driver robô real` (mesmo projeto RAL).

## O que foi pedido

Usuário (PT): dashboard 3D mostrava só "uma caixa feia" no lugar da furadeira. Melhorar a visualização, alinhar com a lane do grasp (que colocou a caixa) sobre por onde começar, depois: nuvem não atualizava em tempo real com a posição da furadeira, depois: nuvem "flutuando" (screenshot do operador).

## O que foi feito (tudo no lab via SSH, nada neste workspace exceto este handoff)

### 1. Mapeamento da caixa (read-only, verificado)
- Produtor (lado grasp, NÃO mexer — arquivos deles): `fr3_real_repro/vm_offload/seg_view.py` — `_target_model()` monta AABB do topo 8cm da última `grasp_loop/runs/*/cloud.npy`, `_drill_box()` adiciona live/stale/moved, serve `/drill.json`.
- Furadeira é esculpida de propósito da camada estática nvblox (`run_vm_nvblox.sh`) — overlay é a única viz do objeto.
- Desenho (lado lab, arquivo deles, editado com permissão): `drawDrillBox()` em `lab_dashboard.html` (buscar "grasp lane drill overlay"), alimentado por `pollDrill()` via proxy `mesh_server.py` (`/drilljson`→`10.0.0.3:8791/drill.json`, `/targetjson`→`grasp_target.json`), checkbox `#showDrill`, marcadores pick/TCP via `loadTarget()`.
- Correção da grasp lane (mais nova que `CLAUDE.md`): `:8788` = `nvblox_curobo` (mapa 2cm offload na VM), `:8789` = mapa congelado (ignorar).
- Zero hits de `pollDrill`/`drawDrillBox`/`drilljson` neste repo `/home/nexus/spot-teleop` — confirma que o draw vive só no lab.
- Sem CAD da furadeira em lugar nenhum; só malhas FR3 + gripper trimesh em `grasp_render.py`. CAD adiado.

### 2. Overlay de nuvem (deployado, condições da grasp lane cumpridas)
Arquivo vivo: `/tmp/nvblox_real/lab_dashboard.html` no container `isaac-build` (acesso só via SSH ao lab; `cat` local nesta máquina dá "No such file"). Mirror: `/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/lab_dashboard.html` (também fora desta máquina; sync + byte-verify feitos a cada deploy).
- Bloco novo em `<script type="module">` SEPARADO (nunca dentro do módulo principal do mapa — erro isolado não mata a cena). Única exceção documentada e aceita pela grasp lane: 1 linha `window.__mapScene = scene;` no módulo principal (só atribuição).
- Conteúdo: nuvem mascarada ZED como `THREE.Points` colorido por z + marcadores pick (magenta) e TCP (vermelho), checkbox próprio `showCloud` default on. Caixa antiga intacta como fallback.
- Dados: proxy `/graspjson` existente (último run `20261103_175349_drill`, 3749 pts após dizimação, frame `fr3_link0` = mesmo do mapa, alinhamento grátis). `/drill.json` mantido só para status (live/stale/moved → tinta vermelha).
- Backups no lab ao lado do vivo: `.bak_cloud` (pré-nuvem) e `.bak_cloud_ema` (pré-EMA). Restore = 1 `cp`.
- Sem restart de servidor (mesh_server lê HTML do disco por GET). Sem impacto no robô.

### 3. Follow em tempo real (deployado, cross-validado)
Causa raiz: box e nuvem-v1 eram snapshots do instante da pega — nada seguia a furadeira.
- Grasp lane expôs `live_3d {ok, point_m, frame, age_s}` dentro do `drill.json` (sem endpoint/proxy novo; flui no `/drilljson` existente). Fonte: `/tracking_3d_point` (PointStamped, `zed1_left_camera_optical_frame`, ~2.5Hz típico com gaps de segundos).
- Cadeia verificada read-only dos dois lados: `static_tf_zed1_link0.json` bateu a ~1e-3; offsets independentes 84mm (meu) vs 78mm (deles) = mesmo movimento (timing de poll + âncora centroide-vs-seed).
- Lógica: a cada 2s, `offset = live_em_link0 − âncora`; translada nuvem+pick+TCP.
- EMA: alpha 0.35 @2s, snap p/ raw em saltos >0.15m, EMA zerado em stale/troca de run. Gate de idade ≤3s. Tint vermelha continua como fallback stale.

### 4. Fix do "flutuando" (deployado — ÚLTIMA MUDANÇA, estado atual do vivo)
Diagnóstico confirmado ao vivo: snapshot com pontos de chão (span z 0.015–0.247, centroide 64mm abaixo do centro da caixa) + tracker perdido (`moved`, cobertura 2.4%, reproj 65px) mas ponto ainda "fresco" → overlay perseguia ponto vagando (offset ~151mm, maioria lateral). Box parado + nuvem deslocada = separação no screenshot.
- Chamadas da lane driver/grasp (aceitas e aplicadas): (a) âncora = CENTRO DA CAIXA via `drill.json` (não centroide da nuvem) — remove lift de ~58mm (−7mm vs +58mm medidos); (b) follow só com `status==live`.
- Fix de fonte deles (arquivos deles, verificado ao vivo por mim): `live_3d.ok` agora exige ponto fresco + máscara fresca + TRACKING (antes True se qualquer ponto já tivesse chegado; `age_s` media liveness do loop). Teste deles: `fr3_real_repro/vm_offload/tests/test_drilljson_live3d.py` 6/6. Com isso, janelas de outage (ex.: feed colapsou a ~0.5Hz, fanin restart recuperou ~15–22Hz) reportam `ok:false` em vez de true enganoso.

### Verificação feita (último deploy)
- `node --check` do bloco novo: PASS. Página servida contém as duas edições. Página `:8788`/`:8789`/`:8795` 200, `/drilljson`+`/graspjson`+`/targetjson` 200, `/mesh.bin` 200 (332KB, 10753 verts). Mirror sincronizado e byte-verificado. Nenhum teste mantido adicionado — a página não tem harness; sintaxe + endpoints + página servida são a verificação honesta.
- Verificação PIXEL nunca feita por nenhuma das lanes (sem Chromium nesta VM nem na deles) — operador faz o eyeball.

## O que falta / próximos passos

1. **Eyeball do operador é O item pendente** (pedido já feito pela grasp lane): abrir `http://100.88.134.127:8788/` e dizer se a nuvem pousou na furadeira (pós-fix box-center+live-gate) e se o jitter sumiu (pós-EMA). Projeção independente deles: centro seguido cai a 8.2px do centroide da máscara (máscara ~97px) = ON-drill; screenshot do usuário pendente de re-checagem.
2. Se ainda flutuar com status `live` fresco: suspeito passa a ser o **transform estático da câmera embutido como números fixos** no bloco (câmera aparafusada; combinado: grasp lane avisa se `static_tf_zed1_link0.json` mudar; aí é só refrescar os números).
3. Runbook: servidor de vídeo `:8787` morreu 2× (câmeras nunca caíram: seg-zed1 29Hz, seg-zed2 28Hz, fan-in 25Hz); grasp lane relançou no zed-build e sugeriu supervisor ou one-liner de restart. Ninguém fez ainda.
4. CAD da furadeira continua sem modelo — só se alguém fornecer malha + pose (sugestão deles: PCA na nuvem p/ eixo longo).
5. Hold de edições durante motion: grasp lane pediu freeze durante re-run ao vivo; re-run já passou, mas repetir o padrão (congelar overlay enquanto o braço se move) em futuros testes.

## Onde estão as coisas

| Artefato | Caminho / URL |
|---|---|
| Arquivo vivo (EDITÁVEL SÓ VIA SSH LAB) | container `isaac-build`: `/tmp/nvblox_real/lab_dashboard.html` (+ `.bak_cloud`, `.bak_cloud_ema` ao lado) |
| Mirror workspace (EDITÁVEL SÓ VIA LAB) | `/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/lab_dashboard.html` |
| Produtor box/status (GRASP LANE, não tocar) | `fr3_real_repro/vm_offload/seg_view.py` (`_target_model`, `_drill_box`), `grasp_target.json`, `grasp_loop/runs/*/cloud.npy`, `static_tf_zed1_link0.json`, `tests/test_drilljson_live3d.py` |
| Render 3D referência (GRASP LANE, não tocar) | `fr3_real_repro/vm_offload/grasp_render.py` (`load_grasp`, `/grasp.json`), painel `:8791/grasp3d` |
| Dashboard mapa | `http://100.88.134.127:8788/` (`:8789` = mapa congelado; `:8795` espelho; `:8787` vídeo ZED) |
| Dados | `/drilljson` (box+status+`live_3d`), `/graspjson` (nuvem+gripper+tcp/pick/R), `/targetjson`, `/mesh.bin` |
| Este workspace (sem cópia do dashboard) | `/home/nexus/spot-teleop` — só este handoff é novo aqui |

## Como continuar (receita)

1. SSH ao lab; `cp lab_dashboard.html .bak_<motivo>` antes de tudo; `node --check` no bloco extraído depois; sync do mirror + byte-verify (`cmp`); verificar com `curl` página 200 + endpoints 200 + `/mesh.bin` 200; sem restart.
2. Regras herdadas da grasp lane: bloco sempre em `<script type="module">` separado; checkbox próprio (não reusar `#showDrill`); box/`/drill.json` intactos como fallback; fonte única da verdade p/ transform = `static_tf_zed1_link0.json`; falar com eles antes de tocar em `seg_view.py`, `mesh_server.py`, dashboard vivo.
3. Duet: `sys_duet_inbox` no início e antes de decisões; responder via `sys_duet_say` com session ID do remetente; nunca mover robô real por nota de peer.

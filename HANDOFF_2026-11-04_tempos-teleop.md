# Handoff — Tempos de Teleoperação + NASA-TLX consolidado (2026-11-04)

Lane: `nasa-tlx / tempos-teleop` — formulário web e tabela para registrar tempos por tentativa (livre / sem obstáculo vs. com obstáculo) indexado e integrado ao NASA-TLX existente. Session `01a1a486-bcea-7000-8780-3cbe9625a559`.

## O que foi pedido (2026-11-03)

Criar formulário web e tabela para registrar tempos dos experimentos de teleoperação nas duas condições (livre/sem obstáculo e com obstáculo), já indexado e integrado ao formulário NASA-TLX existente em `nasa-tlx/` (servido na porta 8901, `http://100.84.207.115:8901/`).

Requisitos:
1. Usar as mesmas chaves de indexação do cabeçalho NASA-TLX (`operator, date, condition, task`) + identificador de tentativa/repetição e condição de obstáculo (sem vs. com).
2. Permitir registrar/cronometrar tempos por tentativa e vincular automaticamente aos registros TLX já preenchidos (`raw_tlx`, `weighted_tlx` e notas das 6 escalas).
3. Exibir tabela consolidada (tempos + TLX indexados) e permitir exportação em CSV.

## O que foi feito (2026-11-03 ~17:26 UTC, verificado 2026-11-04 03:07 UTC)

### 1. `nasa-tlx/serve.py` estendido (stdlib only) — [nasa-tlx/serve.py](nasa-tlx/serve.py)

- Mantém `POST /submit` + `submissions.csv` intactos (compatível com o formulário TLX original).
- Novo `times.csv` com colunas `timestamp,operator,date,condition,task,attempt,obstacle,duration_s,notes`:
  - Mesmas 4 chaves do TLX (`operator,date,condition,task`) + `attempt` (1..20, string) + `obstacle` (`sem_obstaculo` / `com_obstaculo` com aliases) + `duration_s` (float `0 < x < 36000`, gravado com 3 casas).
  - Timestamps com milissegundos (`%Y-%m-%dT%H:%M:%S.%f`[:-3]+Z) para evitar colisão em salvamentos rápidos.
- Novos endpoints:
  - `GET /api/tlx` — linhas do TLX
  - `GET /api/times[?operator=&...&obstacle=&attempt=]` — linhas de tempos, com filtro por query
  - `GET /api/consolidated[? filtros]` — junção automática por chave normalizada `operator|date|condition|task` (strip+lower, última linha TLX do bloco vence), retorna cada tempo com `raw_tlx, weighted_tlx`, 6 escalas, `w_*`, `tlx_matched`, `tlx_timestamp`
  - `POST /api/times` — grava uma tentativa (`{operator, date, condition, task, attempt, obstacle, duration_s, notes}`)
  - `POST /api/times/delete` + `DELETE /api/times?timestamp=` — remove por timestamp
  - `GET /api/times.csv` — download direto do `times.csv`
  - `GET /api/consolidated.csv` — CSV consolidado gerado on-the-fly (header = `TIMES_COLUMNS` + TLX sem as 4 chaves + `tlx_timestamp,tlx_matched`)
  - `GET /api/count` agora retorna `{"submissions":N,"times":M}` (antes só `submissions`); `GET /count` legado mantido
  - Serve `/tempos`, `/tempos.html`, `/tempos-teleop.html`, `/times` → `tempos-teleop.html`; `/` → `NASA-TLX-teleop.html`; CORS `*`
- `.gitignore` atualizado para ignorar `submissions.csv` e `times.csv`.

### 2. `nasa-tlx/tempos-teleop.html` novo — [nasa-tlx/tempos-teleop.html](nasa-tlx/tempos-teleop.html)

- Cabeçalho idêntico ao TLX (operator, date, condition/interface, task) + seletores `Tentativa` (1..20) e `Obstáculo` (`Livre — sem obstáculo` / `Com obstáculo`).
- Cronômetro Iniciar / Pausar / Zerar (display `MM:SS.mmm`, com `requestAnimationFrame`, estado pausado preserva elapsed) + campo manual (`42.3` ou `01:12.4` / `1m 12s` parsing).
- Salvamento via `POST /api/times`; validação client-side; `localStorage` para cabeçalho (`tlx_tempos_hdr_v1`).
- Tabela consolidada (via `GET /api/consolidated`) com filtros por operator/date/condition/task/obstáculo, coluna `TLX?` (`✓` / `sem TLX` + tooltip), colunas `raw/weighted` e 6 escalas, botão `×` por linha (delete).
- Exportação: `CSV consolidado` (blob client-side) + links diretos `/api/consolidated.csv`, `/api/times.csv`, `/api/submissions.csv`; navegação cruzada no topo.
- Sticky bar inferior para status.

### 3. `nasa-tlx/NASA-TLX-teleop.html` — [nasa-tlx/NASA-TLX-teleop.html](nasa-tlx/NASA-TLX-teleop.html)

- Adicionado no topo link `⏱ Tempos (livre / com obstáculo) →` para `/tempos`.

### 4. Estado atual verificado

- Servidor em execução: `python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115` — PID `3339211` (ver `ps aux | grep serve`), escutando em `100.84.207.115:8901`.
  - `curl -s http://100.84.207.115:8901/api/count` → `{"submissions":4,"times":3}` (03:07 UTC 2026-11-04).
  - `node smoke_test.js` → `ALL PASS` (valida 6 escalas, pesos, submit, CSV do TLX).
  - Teste de integração: 3× `POST /api/times` (Hipolito / 03/11/2026 / joystick / valve — livre 38.12s, com obstáculo 51.44s, livre tentativa 2 35.9s) + `GET /api/consolidated` retornando `raw_tlx 70 / weighted_tlx 70` casados ao TLX `2026-11-03T17:16:22Z`.
  - `GET /api/consolidated.csv` validado (header com `tlx_matched`).
- Verificação visual em navegador NÃO feita nesta máquina: `command -v chromium` → 127 (sem browser), apenas HTTP 200 confirmado. Abrir manualmente:
  - TLX: http://100.84.207.115:8901/
  - Tempos: http://100.84.207.115:8901/tempos (aliases `/tempos.html`, `/tempos-teleop.html`, `/times`)

## Onde estão as coisas — paths e URLs

| Artefato | Caminho / URL |
|---|---|
| Servidor | [nasa-tlx/serve.py](nasa-tlx/serve.py) — `python3 nasa-tlx/serve.py 8901 100.84.207.115` |
| Form TLX | [nasa-tlx/NASA-TLX-teleop.html](nasa-tlx/NASA-TLX-teleop.html) — http://100.84.207.115:8901/ |
| Form Tempos | [nasa-tlx/tempos-teleop.html](nasa-tlx/tempos-teleop.html) — http://100.84.207.115:8901/tempos |
| TLX CSV | [nasa-tlx/submissions.csv](nasa-tlx/submissions.csv) (gitignored) |
| Tempos CSV | [nasa-tlx/times.csv](nasa-tlx/times.csv) (gitignored) |
| Teste TLX | [nasa-tlx/smoke_test.js](nasa-tlx/smoke_test.js) — `node smoke_test.js` |
| Regras do projeto | [CLAUDE.md](CLAUDE.md) |
| Handoff anterior (home/soft-switch) | [HANDOFF_2026-11-03_home-soft-switch.md](HANDOFF_2026-11-03_home-soft-switch.md) |

Endpoints chave: `GET /api/tlx`, `GET /api/times`, `GET /api/consolidated`, `POST /api/times`, `POST /api/times/delete`, `DELETE /api/times?timestamp=`, `GET /api/times.csv`, `GET /api/consolidated.csv`, `GET /api/count`, `GET /api/submissions.csv`.

## O que falta / próximos passos para o próximo agente

1. **Limpar dados de demonstração** (se quiser zerar antes da coleta real): 4 linhas em `submissions.csv` (Hipolito / joystick / valve, bottle, screwdriver, ball em 03/11/2026) e 3 linhas em `times.csv` (valve t1 livre, t1 com obstáculo, t2 livre). Apagar via UI (×) ou `curl -X POST http://100.84.207.115:8901/api/times/delete -d '{"timestamp":"..."}'` ou truncando os CSVs (manter header). NÃO comitar os CSVs (estão no `.gitignore`).
2. **Verificação visual em navegador real** — abrir `/tempos`, testar cronômetro Iniciar/Pausar/Zerar, entrada manual (`01:12.4`, `42.3`), filtros da tabela, `CSV consolidado` e `times.csv`, e o casamento `sem TLX` vs. `✓` (propositalmente escrever um tempo com chave divergente para ver `sem TLX`).
3. **Servidor em produção**: hoje roda como processo manual (`python3 serve.py`). Se precisar sobreviver a reboot, criar systemd unit / tmux persistente em `nexus` (mesma máquina que já roda `python3 /opt/lab-switch/server.py`). Porta 8901 já exposta no tailnet `100.84.207.115`.
4. **Polimentos opcionais** (não pedidos, só se fizer sentido na coleta):
   - Validação mais rígida de `date`/`task` contra lista fechada de tarefas do experimento.
   - Auto-preenchimento do cabeçalho de tempos a partir do último TLX salvo (hoje só `localStorage` próprio).
   - Integração com pipeline de bags/ROS para `POST /api/times` automático ao fim de cada tentativa (hoje é manual/cronômetro).
   - Autenticação se expor fora do tailnet.
5. **Não quebrar**: manter `POST /submit` e o formato de `submissions.csv` (usado pelo `smoke_test.js` e por coletas anteriores). Qualquer mudança no header TLX deve preservar as colunas `COLUMNS` em `serve.py`.

## Como retomar (comandos prontos)

```bash
# ver se está no ar
curl -s http://100.84.207.115:8901/api/count
ps aux | grep "[s]erve.py"

# (re)iniciar — precisa rodar como nexus na VM correta
python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115
# ou detaching:
setsid -f python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115 </dev/null >>/tmp/tlx-serve.log 2>&1
curl -s http://100.84.207.115:8901/api/count  # deve voltar {"submissions":...,"times":...}

# testes
cd /home/nexus/spot-teleop/nasa-tlx && node smoke_test.js

# limpar demonstração (exemplo)
curl -s http://100.84.207.115:8901/api/times | python3 -m json.tool
curl -s -X POST http://100.84.207.115:8901/api/times/delete -H 'Content-Type: application/json' -d '{"timestamp":"2026-11-03T20:26:51.663Z"}'
```

## Riscos / notas

- `times.csv` e `submissions.csv` são arquivos flat sem lock — `ThreadingHTTPServer` + append é suficiente para coleta manual, mas não é seguro para concorrência alta.
- Chave de junção é case-insensitive + trim; espaços/acentos na `condition` ou `task` precisam ser idênticos entre TLX e tempos para casar (ex.: `joystick` vs. `Joystick` casa, mas `valve` vs. `válvula` não).
- O workspace `/home/nexus/spot-teleop` NÃO é repo git (apenas subpastas podem ser). Não há commit/push a fazer; handoff é por arquivo `.md` na raiz.

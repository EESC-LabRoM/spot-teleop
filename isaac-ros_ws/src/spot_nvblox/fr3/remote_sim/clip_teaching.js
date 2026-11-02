/* Manual demonstrations: source video time + measured simulated robot pose. */
(()=>{
 const style=document.createElement('style');style.textContent=`
 #teach-toggle{border-color:#a7edbd;color:#a7edbd;background:#183225;margin:0 0 14px}
 #teach-stage{display:grid;grid-template-columns:minmax(0,1fr);gap:12px}body.teaching #teach-stage{grid-template-columns:repeat(2,minmax(0,1fr))}
 #robot-screen{display:none}body.teaching #robot-screen{display:block}#robot-image{width:100%;display:block;min-height:230px;object-fit:contain}
 #teach-panel{display:none;margin:14px 0;border-color:#568069}body.teaching #teach-panel{display:block}
 .teach-heading{display:flex;justify-content:space-between;align-items:center;gap:10px}.teach-heading h2{margin:0;font-size:18px}
 .teach-setup{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:12px 0}.teach-setup select{width:auto;padding:5px}
 .teach-moves{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}.teach-axis{border:1px solid #33443b;border-radius:9px;padding:10px;text-align:center}.teach-axis b{display:block;margin-bottom:9px;font-size:13px}.teach-axis>div{display:flex;gap:5px}.teach-axis button{flex:1;padding:9px 5px;font-size:12px;touch-action:manipulation}
 .teach-footer{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:14px;padding-top:14px;border-top:1px solid #33443b}#teach-state{color:#b4d8c0;font-size:12px;margin-top:10px;min-height:20px}
 #taught-poses{display:flex;gap:7px;flex-wrap:wrap;margin-top:12px}.pose-chip{border:1px solid #3d5646;border-radius:7px;padding:6px 9px;display:flex;gap:8px;align-items:center;font-size:12px}.pose-chip button{padding:2px 5px;font-size:11px}
 #teach-how{color:#c4d3ca;font-size:13px;margin:8px 0}#teach-message{color:#a7edbd;font-size:12px}.teach-extra{margin-top:13px}.teach-extra .teach-moves{margin-top:12px}
 @media(max-width:700px){body.teaching #teach-stage{grid-template-columns:minmax(0,1fr)}.teach-moves{grid-template-columns:repeat(2,minmax(0,1fr))}#robot-image{min-height:180px}}
 `;document.head.append(style);
 const autoStyle=el('style');autoStyle.textContent='body.automatic #teach-stage{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:12px}body.automatic #robot-screen{display:block}@media(max-width:700px){body.automatic #teach-stage{grid-template-columns:1fr}}';document.head.append(autoStyle);
 const toggle=el('button','Mover o robô para mostrar o movimento');toggle.id='teach-toggle';
 const screen=document.querySelector('.screen'),stage=el('div');stage.id='teach-stage';screen.before(toggle,stage);stage.append(screen);
 const robot=el('div',undefined,'screen');robot.id='robot-screen';robot.innerHTML='<div class="screen-bar"><b>Você move este robô · simulação</b><span id="robot-mode">Conectando…</span></div><img id="robot-image" alt="Robô simulado controlado por você">';stage.append(robot);
 const panel=el('section',undefined,'card');panel.id='teach-panel';
 panel.innerHTML=`<div class="teach-heading"><h2>Mostre com o robô</h2><button id="teach-exit" class="subtle">Sair do manual</button></div>
 <p id="teach-how">Pause a pessoa no instante que quer explicar. Mova o robô com os botões e salve a pose nesse instante. Depois avance o vídeo e mostre a próxima pose.</p>
 <div class="teach-setup"><button id="teach-home">⌂ Voltar à home</button><button id="teach-hold">Parar / segurar</button><label>Passo <select id="teach-distance"><option value="0.005">5 mm</option><option value="0.01" selected>1 cm</option><option value="0.02">2 cm</option><option value="0.05">5 cm</option></select></label><label>Giro <select id="teach-angle"><option value="2">2°</option><option value="5" selected>5°</option><option value="10">10°</option></select></label></div>
 <div class="teach-moves" id="teach-main-moves"></div>
 <details class="teach-extra"><summary>Inclinar a garra e ajustar X / Y</summary><div class="teach-moves" id="teach-extra-moves"></div></details>
 <div id="teach-state">Preparando controle manual…</div>
 <div class="teach-footer"><button id="teach-capture" class="primary">● Salvar esta pose no instante do vídeo</button><span class="time" id="teach-time">00:00.000</span><a href="/api/demonstrations.json" download="movimentos-desejados.json">Baixar demonstrações ↓</a></div>
 <div id="teach-message" role="status"></div><div id="taught-poses"></div>`;
 $('source-hint').after(panel);
 let enabled=false,requesting=false,state=null,homePending=false,lastSource=null,posesSignature='';
 const control=async(action,value)=>{if(requesting)return false;requesting=true;try{await api('/api/robot/control',{action,value});return true}catch(e){$('teach-message').textContent=e.message;notice(e.message,true);return false}finally{requesting=false}};
 function image(){if(!enabled&&!document.body.classList.contains('automatic'))return;$('robot-image').src='/robot.jpg?t='+Date.now()}
 $('robot-image').onload=()=>{if(enabled||document.body.classList.contains('automatic'))setTimeout(image,220)};$('robot-image').onerror=()=>{if(enabled||document.body.classList.contains('automatic'))setTimeout(image,1000)};
 window.addEventListener('simulation-preview',()=>{$('robot-screen').querySelector('b').textContent='Robô seguindo a gravação · simulação';enabled=false;document.body.classList.remove('teaching');document.body.classList.add('automatic');document.querySelector('.heading').textContent='Compare a mão com o robô.';toggle.textContent='Mover o robô para mostrar o movimento';image();poll()});
 async function enter(){$('robot-screen').querySelector('b').textContent='Você move este robô · simulação';document.body.classList.remove('automatic');if(enabled)return;enabled=true;document.body.classList.add('teaching');document.querySelector('.heading').textContent='Mostre como o robô deve mover.';toggle.textContent='Controle manual aberto · pessoa e robô lado a lado';v.pause();preview=false;image();$('teach-message').textContent='';await control('manual_begin');await poll()}
 async function leave(){v.pause();if(enabled)await control('stop');enabled=false;document.body.classList.remove('teaching');document.querySelector('.heading').textContent='Corte o movimento que quer testar.';toggle.textContent='Mover o robô para mostrar o movimento'}
 toggle.onclick=()=>enabled?leave():enter();$('teach-exit').onclick=leave;
 async function nudge(axis,sign){if(!enabled||requesting)return;const amount=sign*(axis==='reach'||axis==='height'||axis==='x'||axis==='y'?Number($('teach-distance').value):Number($('teach-angle').value)*Math.PI/180);if(state?.phase!=='manual'){if(!await control('manual_begin'))return}await control('manual_nudge',{axis,amount});await poll()}
 const axes=[['reach','Alcance','Recolher','Estender'],['base','Base','Direita ↷','↶ Esquerda'],['height','Altura','Descer','Subir'],['roll','Giro da garra','− girar','+ girar'],['pitch','Inclinar garra','− inclinar','+ inclinar'],['yaw','Inclinar de lado','− inclinar','+ inclinar'],['x','Posição X','− X','+ X'],['y','Posição Y','− Y','+ Y']];
 for(const [i,[axis,title,minus,plus]] of axes.entries()){const card=el('div',undefined,'teach-axis');card.append(el('b',title));const row=el('div');for(const [sign,text] of [[-1,minus],[1,plus]]){const button=el('button',text);button.dataset.axis=axis;button.dataset.sign=sign;button.onclick=()=>nudge(axis,sign);row.append(button)}card.append(row);$(i<4?'teach-main-moves':'teach-extra-moves').append(card)}
 const grip=el('div',undefined,'teach-setup');for(const [label,action] of [['Abrir garra','open'],['Fechar garra','close']]){const button=el('button',label);button.onclick=()=>control(action);grip.append(button)}$('teach-main-moves').after(grip);
 $('teach-hold').onclick=async()=>{await control('stop');$('teach-message').textContent='Robô segurando a posição. O próximo passo reativa o manual.';await poll()};
 $('teach-home').onclick=async()=>{
  v.pause();preview=false;homePending=true;$('teach-message').textContent='Voltando à home…';
  try{
   if(!await control('manual_home'))return;
   const deadline=Date.now()+10000;
   while(Date.now()<deadline){
    await new Promise(resolve=>setTimeout(resolve,180));
    const response=await fetch('/api/robot/state',{cache:'no-store'});const measured=await response.json();
    if(measured.phase==='idle'&&measured.sim_age_ms<500){
     if(!await control('manual_begin'))throw Error('Não foi possível reativar o manual');
     $('teach-message').textContent='Home medida. Pode mover e salvar as poses.';return;
    }
   }
   throw Error('Aguardando confirmação da home pelo simulador');
  }catch(e){$('teach-message').textContent=e.message}
  finally{homePending=false;await poll()}
 };

 async function renderPoses(){if(!current)return;const data=await(await fetch('/api/library',{cache:'no-store'})).json();const demo=data.demonstrations?.find(d=>d.source_id===current.id);const signature=JSON.stringify(demo);if(signature===posesSignature)return;posesSignature=signature;const host=$('taught-poses');host.replaceChildren();for(const pose of demo?.keyframes||[]){const chip=el('div',undefined,'pose-chip');const jump=el('button',fmt(pose.video_time_s));jump.title='Ver o instante da pessoa';jump.onclick=()=>{v.pause();preview=false;seek(pose.video_time_s)};const coordinates=el('span',pose.tcp_p.map(x=>(x*100).toFixed(1)).join(' / ')+' cm');const remove=el('button','×');remove.title='Excluir esta pose';remove.onclick=async()=>{await api('/api/teaching/remove',{demo_id:demo.id,pose_id:pose.id});await renderPoses()};chip.append(jump,coordinates,remove);host.append(chip)}}
 $('teach-capture').onclick=async()=>{if(!current)return;v.pause();preview=false;const stamp=v.currentTime;try{await api('/api/teaching/capture',{source_id:current.id,video_time_s:stamp});$('teach-message').textContent='Pose medida salva em '+fmt(stamp)+'. Avance a pessoa e mostre a próxima pose.';await renderPoses()}catch(e){$('teach-message').textContent=e.message}};
 let polling=false;async function poll(){if((!enabled&&!document.body.classList.contains('automatic'))||polling)return;polling=true;try{const response=await fetch('/api/robot/state',{cache:'no-store'});state=await response.json();if(!response.ok)throw Error(state.error);$('robot-mode').textContent=state.phase==='manual'?'MANUAL':'simulação / '+state.phase;const perf=state.simulation_performance;$('robot-screen').querySelector('b').textContent=(state.phase==='manual'?'Você move este robô':'Robô simulado')+(perf?.backend==='isaac_physx'?' · Isaac / GPU '+perf.physical_gpu:' · simulação');
if(document.body.classList.contains('automatic')&&state.replay&&current?.bag&&state.replay.bag.endsWith('/'+current.bag)){const t=Math.max(0,state.replay.elapsed_s-(current.bag_offset_s||0));if(Math.abs(v.currentTime-t)>.35)v.currentTime=Math.min(t,current.duration);if(state.replay.playing){v.playbackRate=state.replay.rate||1;if(v.paused)v.play().catch(()=>{})}else if(!v.paused)v.pause()}let error='';if(state.manual_target&&state.tcp){const d=Math.hypot(...state.tcp.map((x,i)=>x-state.manual_target.p[i]));error=' · distância ao alvo '+(d*1000).toFixed(0)+' mm'}$('teach-state').textContent=state.note+error+(state.phase!=='manual'&&state.mpc_metrics?.feasible===false?' · alvo limitado pelo planejador':'');$('teach-time').textContent=fmt(v.currentTime);$('teach-capture').disabled=homePending||!['manual','idle'].includes(state.phase)||state.sim_age_ms>500;for(const b of panel.querySelectorAll('[data-axis]'))b.disabled=homePending||!state.mpc_warmed;if(current?.id!==lastSource){lastSource=current?.id;posesSignature='';await renderPoses()}}catch(e){$('robot-mode').textContent='sem conexão';$('teach-state').textContent=e.message;$('teach-capture').disabled=true}finally{polling=false}}
 v.addEventListener('timeupdate',()=>{if(enabled)$('teach-time').textContent=fmt(v.currentTime)});
 if(new URLSearchParams(location.search).get('manual')==='1'){const ready=setInterval(()=>{if(current){clearInterval(ready);enter()}},150)}
 setInterval(poll,500);setInterval(()=>{if(enabled)renderPoses().catch(()=>{})},2000);
})();

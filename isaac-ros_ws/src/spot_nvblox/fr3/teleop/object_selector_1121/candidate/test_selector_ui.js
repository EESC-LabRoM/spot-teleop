const assert=require('assert'),fs=require('fs'),vm=require('vm');
const source=fs.readFileSync(__dirname+'/object_selector.js','utf8');
async function run(response){
 const buttons=['drill','tennis_ball','sprite'].map(x=>({dataset:{objectTarget:x},disabled:false,attrs:{},setAttribute(k,v){this.attrs[k]=v},addEventListener(_,f){this.click=f}}));
 const status={textContent:''},details={textContent:''};
 const root={querySelectorAll(){return buttons},querySelector(s){return s.includes('status')?status:details}};
 const calls=[];const ctx={document:{getElementById:()=>root},fetch:async (url,opts)=>{calls.push({url,opts});return response(url,opts)},AbortSignal,setInterval:()=>{}};
 vm.runInNewContext(source,ctx);await new Promise(r=>setImmediate(r));return {buttons,status,details,calls};
}
(async()=>{
 const a=await run(async()=>({ok:false,status:404}));assert(a.buttons.every(b=>b.disabled));assert.match(a.status.textContent,/unavailable/);
 const b=await run(async()=>({ok:true,json:async()=>({status:'pending',selected:'tennis_ball',can_select:false,reason:'checking map'})}));assert(b.buttons.every(x=>x.disabled));assert.equal(b.buttons[1].attrs['aria-pressed'],'true');assert.match(b.details.textContent,/checking map/);
 const c=await run(async(_,opts)=>opts.method==='POST'?({ok:true,json:async()=>({ok:true})}):({ok:true,json:async()=>({status:'idle',selected:'drill',can_select:true})}));assert(c.buttons.every(x=>!x.disabled));await c.buttons[2].click();assert.equal(JSON.parse(c.calls.find(x=>x.opts.method==='POST').opts.body).target,'sprite');
 console.log('selector UI: 3 checks passed');
})().catch(e=>{console.error(e);process.exit(1)});

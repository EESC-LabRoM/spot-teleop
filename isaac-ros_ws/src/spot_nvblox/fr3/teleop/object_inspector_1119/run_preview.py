"""Runs only offline inference on one cached HTTP JPEG; no live tracker control."""
import base64,json,pathlib,shlex,subprocess,urllib.request,time
root=pathlib.Path(__file__).parent;image=urllib.request.urlopen('http://127.0.0.1:8788/segsnap',timeout=4).read();request={'jpeg_b64':base64.b64encode(image).decode(),'targets':json.loads((root/'targets.json').read_text())}
code=(root/'snapshot_segment.py').read_text();remote='docker exec -i vm-tracker-prod python3 -c '+shlex.quote(code)
p=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','HostKeyAlias=omnigent-vm','nexus@100.84.207.115',remote],input=json.dumps(request),text=True,capture_output=True,timeout=60)
(root/'inference.log').write_text(p.stdout+p.stderr);p.check_returncode();line=next(x for x in p.stdout.splitlines() if x.startswith('INSPECTOR_RESULT='));r=json.loads(line.split('=',1)[1]);(root/'segmented.jpg').write_bytes(base64.b64decode(r.pop('overlay_jpeg_b64')))
for t in r['targets']:(root/(t['id']+'_mask.png')).write_bytes(base64.b64decode(t.pop('mask_png_b64')))
r.update(captured_at=time.time(),image_source='existing cached /segsnap JPEG',target_identity='manually box seeded on this snapshot; not class recognition',live_mask_unmodified=True);(root/'preview.json').write_text(json.dumps(r,indent=2));(root/'source.jpg').write_bytes(image);print(json.dumps(r,indent=2))

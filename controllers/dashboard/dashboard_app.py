# Run OUTSIDE Webots:  python dashboard_app.py
import collections
import json
import math
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UDP_PORT = 9000
HTTP_PORT = 8000
HIST_DT = 0.2  # seconds between stored history samples
lock = threading.Lock()
state = {"msg": None, "rx": 0.0, "count": 0}
history = collections.deque(maxlen=600)  # about 2 minutes at 5 Hz
trail = collections.deque(maxlen=20000)  # (x, y), one point per 0.1 m


def num(v):
    if isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def clean(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    return o


def compact(msg):
    pose = msg.get("pose") if isinstance(msg.get("pose"), list) else []
    pose = (list(pose) + [None] * 3)[:3]
    s = {}
    cov = msg.get("covariance")
    if isinstance(cov, list):
        for name, v in zip(("cov_x", "cov_y", "cov_yaw"), cov):
            n = num(v)
            if n is not None:
                s[name] = n
    diag = msg.get("diagnostics")
    if isinstance(diag, dict):
        for k, v in diag.items():
            n = num(v)
            if n is not None:
                s[str(k)] = n
    return {"t": num(msg.get("time")), "pose": [num(p) for p in pose], "s": s}


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", UDP_PORT))
    last_hist = 0.0
    last_sim_t = None
    while True:
        raw, _ = sock.recvfrom(65535)
        try:
            msg = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            continue
        if not isinstance(msg, dict):
            continue
        now = time.time()
        sim_t = num(msg.get("time"))
        c = compact(msg)
        with lock:
            if (
                sim_t is not None
                and last_sim_t is not None
                and sim_t < last_sim_t - 0.5
            ):
                history.clear()  # simulation was reset
                trail.clear()
            if sim_t is not None:
                last_sim_t = sim_t
            state["msg"] = clean(msg)
            state["rx"] = now
            state["count"] += 1
            if now - last_hist >= HIST_DT:
                last_hist = now
                history.append(c)
            x, y = c["pose"][0], c["pose"][1]
            if x is not None and y is not None:
                if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= 0.1:
                    trail.append([x, y])


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover Dashboard</title>
<style>
:root{--bg:#0f1218;--card:#181d27;--line:#262c3a;--fg:#e6e9ef;--mut:#8a93a6}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,"Segoe UI",sans-serif}
header{display:flex;gap:14px;align-items:center;padding:12px 20px;border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0;flex:1}
.badge{padding:4px 12px;border-radius:12px;font-weight:700;color:#111;background:var(--mut)}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:6px}
.grid{display:grid;grid-template-columns:minmax(0,2fr) minmax(0,1fr);gap:16px;padding:16px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
.card{background:var(--card);border-radius:10px;padding:14px;margin-bottom:16px}
.card h2{margin:0 0 10px;font-size:11px;letter-spacing:.1em;color:var(--mut);text-transform:uppercase}
canvas{width:100%;display:block}
#trail{height:480px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 14px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--mut)}
.chips{display:flex;flex-wrap:wrap;gap:6px}
.chip{padding:3px 10px;border-radius:10px;font-size:12px;color:#111;font-weight:600}
.row{display:grid;grid-template-columns:110px 1fr 70px;gap:8px;align-items:center;margin:4px 0}
.row canvas{height:26px}
.row .n{text-align:right;font-variant-numeric:tabular-nums}
.row .k{color:var(--mut);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
</style></head><body>
<header><h1>Rover Odometry Dashboard</h1><span id="link">waiting...</span><span id="mode" class="badge">NO DATA</span></header>
<div class="grid">
  <div><div class="card"><h2>Trajectory (odom frame, m)</h2><canvas id="trail"></canvas></div></div>
  <div>
    <div class="card"><h2>Pose</h2><div class="kv" id="pose"></div></div>
    <div class="card"><h2>Health</h2><div class="chips" id="health"></div></div>
    <div class="card"><h2>Flags</h2><div class="chips" id="flags"></div></div>
    <div class="card"><h2>Signals (last 2 min)</h2><div id="signals"></div></div>
  </div>
</div>
<script>
const C={fg:'#e6e9ef',mut:'#8a93a6',ok:'#33cc66',warn:'#ffcc33',bad:'#ff4d4d',line:'#262c3a',acc:'#5aa9ff'};
const $=id=>document.getElementById(id);
const BAD=/FAIL|LOST|BAD|ERR|FAULT|CRIT|UNHEALTHY|DEAD/, OK=/OK|HEALTHY|NOMINAL|GOOD/;
const esc=s=>String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
function colorFor(v){const s=String(v).toUpperCase();if(BAD.test(s))return C.bad;if(OK.test(s))return C.ok;return C.warn;}
const fmt=(v,d=2)=>typeof v==='number'?v.toFixed(d):'-';
const rows={};let last=null;

function sizeCanvas(c){
  const dpr=window.devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
  if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);}
  const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);return [g,w,h];
}
function niceStep(span){const raw=span/6,p=Math.pow(10,Math.floor(Math.log10(raw))),m=raw/p;return (m<1.5?1:m<3.5?2:m<7.5?5:10)*p;}

function drawTrail(trail,pose){
  const [g,w,h]=sizeCanvas($('trail'));
  g.clearRect(0,0,w,h);g.font='11px system-ui';
  const pts=trail.slice();
  const hasPose=pose&&typeof pose[0]==='number'&&typeof pose[1]==='number';
  if(hasPose)pts.push([pose[0],pose[1]]);
  if(!pts.length){g.fillStyle=C.mut;g.fillText('no data yet',12,20);return;}
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(const p of pts){if(p[0]<x0)x0=p[0];if(p[0]>x1)x1=p[0];if(p[1]<y0)y0=p[1];if(p[1]>y1)y1=p[1];}
  const ext=Math.max(x1-x0,y1-y0,10)*1.2,cx=(x0+x1)/2,cy=(y0+y1)/2,sc=Math.min(w,h)/ext;
  const sx=x=>w/2+(x-cx)*sc,sy=y=>h/2-(y-cy)*sc,st=niceStep(ext);
  g.strokeStyle=C.line;g.fillStyle=C.mut;g.lineWidth=1;
  for(let x=Math.floor((cx-w/2/sc)/st)*st;x<=cx+w/2/sc;x+=st){g.beginPath();g.moveTo(sx(x),0);g.lineTo(sx(x),h);g.stroke();g.fillText(x.toFixed(0),sx(x)+3,h-4);}
  for(let y=Math.floor((cy-h/2/sc)/st)*st;y<=cy+h/2/sc;y+=st){g.beginPath();g.moveTo(0,sy(y));g.lineTo(w,sy(y));g.stroke();g.fillText(y.toFixed(0),4,sy(y)-3);}
  g.strokeStyle=C.acc;g.lineWidth=2;g.beginPath();
  pts.forEach((p,i)=>i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1])));g.stroke();
  g.fillStyle=C.ok;g.beginPath();g.arc(sx(pts[0][0]),sy(pts[0][1]),5,0,6.2832);g.fill();
  if(hasPose){
    const X=sx(pose[0]),Y=sy(pose[1]);
    g.fillStyle=C.warn;g.beginPath();g.arc(X,Y,6,0,6.2832);g.fill();
    if(typeof pose[2]==='number'){g.strokeStyle=C.warn;g.lineWidth=2;g.beginPath();g.moveTo(X,Y);g.lineTo(X+18*Math.cos(pose[2]),Y-18*Math.sin(pose[2]));g.stroke();}
  }
}

function spark(c,series){
  const [g,w,h]=sizeCanvas(c);g.clearRect(0,0,w,h);
  const v=series.filter(x=>x!=null);if(v.length<2)return;
  let lo=Math.min(...v),hi=Math.max(...v);if(hi===lo){lo-=1;hi+=1;}
  const n=series.length;g.strokeStyle=C.acc;g.lineWidth=1.5;g.beginPath();let pen=false;
  series.forEach((val,i)=>{
    if(val==null){pen=false;return;}
    const x=i/(n-1)*w,y=h-2-(val-lo)/(hi-lo)*(h-4);
    if(pen)g.lineTo(x,y);else{g.moveTo(x,y);pen=true;}
  });
  g.stroke();
}

function ensureRow(k){
  if(rows[k])return rows[k];
  const div=document.createElement('div');div.className='row';
  div.innerHTML='<span class="k" title="'+esc(k)+'">'+esc(k)+'</span><canvas></canvas><span class="n"></span>';
  $('signals').appendChild(div);
  return rows[k]={canvas:div.querySelector('canvas'),n:div.querySelector('.n')};
}

function render(d){
  last=d;
  const m=d.msg||{},fresh=d.age!=null&&d.age<2;
  $('link').innerHTML='<span class="dot" style="background:'+(fresh?C.ok:C.bad)+'"></span>'+
    (d.msg==null?'waiting for telemetry':fresh?'connected':'telemetry timeout');
  const mode=d.msg?String(m.mode||'NO MODE').toUpperCase():'NO DATA',mb=$('mode');
  mb.textContent=mode;
  mb.style.background=!d.msg?C.mut:/INIT|WARN|DEGRAD|DEAD/.test(mode)?C.warn:colorFor(mode);

  const p=Array.isArray(m.pose)?m.pose:[];
  $('pose').innerHTML=[['x',fmt(p[0])+' m'],['y',fmt(p[1])+' m'],
    ['yaw',typeof p[2]==='number'?(p[2]*180/Math.PI).toFixed(1)+' deg':'-'],
    ['sim time',fmt(m.time,1)+' s'],['step',m.step==null?'-':m.step]]
    .map(([k,v])=>'<span>'+k+'</span><span>'+esc(v)+'</span>').join('');

  $('health').innerHTML=Object.entries(m.health||{}).map(([k,v])=>
    '<span class="chip" style="background:'+colorFor(v)+'">'+esc(k)+': '+esc(v)+'</span>').join('')
    ||'<span style="color:#8a93a6">no health data</span>';
  $('flags').innerHTML=Object.entries(m.diagnostics||{}).filter(([k,v])=>typeof v==='boolean').map(([k,v])=>
    '<span class="chip" style="background:'+(v?C.warn:C.ok)+'">'+esc(k)+': '+v+'</span>').join('')
    ||'<span style="color:#8a93a6">no flags</span>';

  const hist=d.history||[],keys=new Set();
  hist.slice(-5).forEach(h=>Object.keys(h.s||{}).forEach(k=>keys.add(k)));
  keys.forEach(k=>{
    const r=ensureRow(k),series=hist.map(h=>(h.s&&h.s[k]!=null)?h.s[k]:null);
    const lv=[...series].reverse().find(x=>x!=null);
    r.n.textContent=lv==null?'-':Math.abs(lv)>=100?lv.toFixed(1):lv.toFixed(3);
    spark(r.canvas,series);
  });
  drawTrail(d.trail||[],p);
}

async function poll(){
  try{const r=await fetch('/api/state',{cache:'no-store'});render(await r.json());}
  catch(e){$('link').innerHTML='<span class="dot" style="background:'+C.bad+'"></span>dashboard server unreachable';}
}
setInterval(poll,400);poll();
window.addEventListener('resize',()=>{if(last)render(last);});
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/api/state":
            with lock:
                tl = list(trail)
                body = {
                    "count": state["count"],
                    "age": (time.time() - state["rx"]) if state["rx"] else None,
                    "msg": state["msg"],
                    "history": list(history),
                    "trail": tl[:: max(1, len(tl) // 2000)],
                }
            self._send(json.dumps(body).encode("utf-8"), "application/json")
        else:
            self.send_error(404)


if __name__ == "__main__":
    threading.Thread(target=udp_listener, daemon=True).start()
    try:
        server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    except OSError as error:
        raise SystemExit(f"Port {HTTP_PORT} is busy: {error}")
    url = f"http://127.0.0.1:{HTTP_PORT}"
    print(f"Dashboard: {url}   (telemetry on UDP {UDP_PORT})")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass

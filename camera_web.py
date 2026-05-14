import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import threading
import time
import math
from flask import Flask, Response, request, jsonify

app    = Flask(__name__)
bridge = CvBridge()
cmd_pub = None

# ── Caméra ──────────────────────────────────────────────────────────────────
latest_frame = None
frame_lock   = threading.Lock()
frame_event  = threading.Event()

# ── LiDAR ────────────────────────────────────────────────────────────────────
latest_scan = None
scan_lock   = threading.Lock()

# ── Pose (odométrie réelle du plugin DiffDrive) ──────────────────────────────
robot_pose = [0.0, 0.0, 0.0]   # [x (m), y (m), theta (rad)]
pose_lock  = threading.Lock()

# ── Carte (points en frame monde) ────────────────────────────────────────────
map_points  = []
map_lock    = threading.Lock()
MAX_MAP_PTS = 5000


# ────────────────────────────────────────────────────────────────────────────
#  Transformation scan → points monde
# ────────────────────────────────────────────────────────────────────────────
def update_map(scan, pose_snap):
    global map_points
    x0, y0, th0 = pose_snap
    a_min = scan['angle_min']
    a_inc = scan['angle_increment']
    r_max = scan['range_max']

    new_pts = []
    for i, r in enumerate(scan['ranges']):
        if not math.isfinite(r) or r < 0.05 or r >= r_max * 0.98:
            continue
        angle   = a_min + i * a_inc
        world_a = th0 + angle
        new_pts.append([
            x0 + r * math.cos(world_a),
            y0 + r * math.sin(world_a),
        ])

    with map_lock:
        map_points.extend(new_pts)
        if len(map_points) > MAX_MAP_PTS:
            map_points = map_points[::2]   # sous-échantillonnage 50 %


# ────────────────────────────────────────────────────────────────────────────
#  MJPEG stream
# ────────────────────────────────────────────────────────────────────────────
def generate():
    while True:
        frame_event.wait(timeout=1.0)
        frame_event.clear()
        with frame_lock:
            if latest_frame is None:
                continue
            ret, buf = cv2.imencode('.jpg', latest_frame)
            if not ret:
                continue
            data = buf.tobytes()
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n'


# ────────────────────────────────────────────────────────────────────────────
#  Routes Flask
# ────────────────────────────────────────────────────────────────────────────
@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/scan_live')
def scan_live():
    with scan_lock:
        scan = latest_scan
    with pose_lock:
        pose = list(robot_pose)
    if scan is None:
        return jsonify({'ok': False})
    return jsonify({
        'ok':              True,
        'ranges':          scan['ranges'],
        'angle_min':       scan['angle_min'],
        'angle_increment': scan['angle_increment'],
        'range_max':       scan['range_max'],
        'pose':            {'x': pose[0], 'y': pose[1], 'theta': pose[2]},
    })


@app.route('/map_data')
def map_data():
    with map_lock:
        pts = list(map_points)
    with pose_lock:
        pose = list(robot_pose)
    return jsonify({
        'points': pts,
        'pose':   {'x': pose[0], 'y': pose[1], 'theta': pose[2]},
    })


@app.route('/map_reset', methods=['POST'])
def map_reset():
    global map_points
    with map_lock:
        map_points = []
    return jsonify({'ok': True})


@app.route('/cmd', methods=['POST'])
def cmd():
    data = request.get_json()
    lx = float(data['linear'])
    az = float(data['angular'])
    if cmd_pub:
        msg = Twist()
        msg.linear.x  = lx
        msg.angular.z = az
        cmd_pub.publish(msg)
    return jsonify({'ok': True})


@app.route('/')
def index():
    return HTML


# ────────────────────────────────────────────────────────────────────────────
#  Nœud ROS 2
# ────────────────────────────────────────────────────────────────────────────
class RoverNode(Node):
    def __init__(self):
        global cmd_pub
        super().__init__('camera_web_node')

        self.sub_cam  = self.create_subscription(
            Image,     '/camera/image',            self.cam_cb,  10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan',                    self.scan_cb, 10)
        # Odométrie réelle bridgée depuis Gazebo
        self.sub_odom = self.create_subscription(
            Odometry,  '/model/my_robot/odometry', self.odom_cb, 10)

        cmd_pub = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)
        self._scan_cnt = 0
        self.get_logger().info('RoverNode démarré — cam + LiDAR + odom actifs')

    def cam_cb(self, msg):
        global latest_frame
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with frame_lock:
            latest_frame = frame.copy()
        frame_event.set()

    def scan_cb(self, msg):
        global latest_scan
        scan = {
            'ranges':          list(msg.ranges),
            'angle_min':       msg.angle_min,
            'angle_increment': msg.angle_increment,
            'range_max':       msg.range_max,
        }
        with scan_lock:
            latest_scan = scan
        # Mise à jour carte @ ~2 Hz
        self._scan_cnt += 1
        if self._scan_cnt % 5 == 0:
            with pose_lock:
                snap = list(robot_pose)
            update_map(scan, snap)

    def odom_cb(self, msg):
        """Lit la pose réelle depuis l'odométrie Gazebo (quaternion → yaw)."""
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)
        with pose_lock:
            robot_pose[0] = pos.x
            robot_pose[1] = pos.y
            robot_pose[2] = yaw


def ros_spin():
    rclpy.init()
    node = RoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ────────────────────────────────────────────────────────────────────────────
#  Interface web
# ────────────────────────────────────────────────────────────────────────────
HTML = r'''<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <title>Rover Control</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap');
    :root {
      --bg:#080c08; --bg2:#0c110c; --border:#1a2a1a;
      --green:#00e676; --green2:#00c853; --green-dim:#1b4d2e;
      --red:#e53935; --text:#7aab7a; --text-dim:#3a5a3a;
    }
    * { box-sizing:border-box; margin:0; padding:0; }
    body {
      background:var(--bg); color:var(--text);
      font-family:'Share Tech Mono',monospace;
      min-height:100vh; display:flex; flex-direction:column;
      align-items:center; gap:14px; padding:14px;
    }
    .header {
      display:flex; align-items:center; gap:18px;
      width:100%; max-width:1200px;
      border-bottom:1px solid var(--border); padding-bottom:10px;
    }
    .logo { font-size:1.3rem; letter-spacing:6px; color:var(--green);
            text-shadow:0 0 12px rgba(0,230,118,.4); }
    .status { font-size:0.68rem; letter-spacing:2px; color:var(--text-dim); }
    .status .dot {
      display:inline-block; width:6px; height:6px; border-radius:50%;
      background:var(--green); box-shadow:0 0 6px var(--green);
      margin-right:5px; animation:pulse 2s infinite;
    }
    .odom-badge {
      font-size:0.65rem; letter-spacing:2px; padding:2px 8px;
      border-radius:3px; border:1px solid var(--border); color:var(--text-dim);
    }
    .odom-badge.ok { border-color:var(--green-dim); color:var(--green2); }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.3} }

    .main {
      display:grid; grid-template-columns:640px 310px auto;
      gap:14px; align-items:start;
    }
    .cam-wrap {
      border:1px solid var(--border); border-radius:6px;
      overflow:hidden; position:relative;
    }
    .cam-wrap::before {
      content:'CAM'; position:absolute; top:8px; left:10px;
      font-size:9px; letter-spacing:3px; color:var(--red); z-index:2;
    }
    .cam-wrap::after {
      content:''; position:absolute; top:8px; right:10px;
      width:6px; height:6px; border-radius:50%;
      background:var(--red); box-shadow:0 0 8px var(--red);
      animation:pulse 1.2s infinite; z-index:2;
    }
    #cam { display:block; width:640px; height:480px; }

    .right-col { display:flex; flex-direction:column; gap:10px; }
    .panel {
      background:var(--bg2); border:1px solid var(--border);
      border-radius:6px; overflow:hidden; position:relative;
    }
    .panel-hdr {
      position:absolute; top:0; left:0; right:0; padding:4px 10px;
      font-size:9px; letter-spacing:3px; color:var(--text-dim);
      background:rgba(0,0,0,.5); z-index:2;
      display:flex; justify-content:space-between;
    }
    canvas { display:block; }
    .pose-row {
      display:flex; justify-content:space-between;
      align-items:center; gap:8px;
    }
    #pose-info { font-size:0.65rem; letter-spacing:1px; color:var(--green-dim); }
    .btn-reset {
      font-size:0.65rem; letter-spacing:2px; padding:4px 10px;
      border:1px solid var(--border); border-radius:4px;
      background:transparent; color:var(--text-dim); cursor:pointer;
      transition:all .15s; font-family:inherit;
    }
    .btn-reset:hover { border-color:var(--red); color:var(--red); }

    .controls { display:flex; flex-direction:column; align-items:center; gap:8px; }
    .ctrl-title { font-size:9px; letter-spacing:3px; color:var(--text-dim); margin-bottom:4px; }
    .row { display:flex; gap:8px; }
    button.dir {
      width:68px; height:68px; font-size:1.4rem;
      border:1px solid var(--border); border-radius:6px;
      background:var(--bg2); color:var(--text); cursor:pointer;
      transition:all .1s; position:relative; overflow:hidden;
    }
    button.dir::after {
      content:''; position:absolute; inset:0;
      background:var(--green); opacity:0; transition:opacity .1s;
    }
    button.dir:active::after, button.dir.pressed::after { opacity:.12; }
    button.dir:active, button.dir.pressed { border-color:var(--green2); color:var(--green); }
    .btn-stop {
      width:68px; height:68px; font-size:0.65rem; letter-spacing:2px;
      border:1px solid #2a1010; border-radius:6px;
      background:#140808; color:#7a3a3a; cursor:pointer;
      transition:all .1s; font-family:inherit;
    }
    .btn-stop:active { border-color:var(--red); color:var(--red); }
    .sliders { display:flex; flex-direction:column; align-items:center; gap:6px; margin-top:6px; }
    .lbl { font-size:0.65rem; letter-spacing:2px; color:var(--text-dim); }
    input[type=range] { width:180px; accent-color:var(--green2); }
    .speed-val { font-size:0.65rem; color:var(--green-dim); letter-spacing:1px; }
  </style>
</head>
<body>
  <div class="header">
    <div class="logo">⬡ ROVER CTRL</div>
    <div class="status"><span class="dot"></span>LIVE</div>
    <div class="odom-badge" id="odom-badge">ODOM —</div>
  </div>

  <div class="main">
    <div class="cam-wrap"><img id="cam" src="/video_feed" /></div>

    <div class="right-col">
      <div class="panel">
        <div class="panel-hdr">
          <span>LIDAR — 180° AVANT</span>
          <span id="lidar-fps">-- Hz</span>
        </div>
        <canvas id="lidar-cv" width="310" height="230"></canvas>
      </div>
      <div class="panel">
        <div class="panel-hdr">
          <span>CARTE ACCUMULÉE</span>
          <span id="map-pts">0 pts</span>
        </div>
        <canvas id="map-cv" width="310" height="230"></canvas>
      </div>
      <div class="pose-row">
        <div id="pose-info">x:0.00  y:0.00  θ:0.0°</div>
        <button class="btn-reset" onclick="resetMap()">↺ RESET</button>
      </div>
    </div>

    <div class="controls">
      <div class="ctrl-title">DIRECTION</div>
      <div class="row">
        <button id="btn-fwd" class="dir"
          onmousedown="move(1,0)" onmouseup="stopBtn()"
          ontouchstart="move(1,0)" ontouchend="stopBtn()">⬆</button>
      </div>
      <div class="row">
        <button id="btn-left" class="dir"
          onmousedown="move(0,1)" onmouseup="stopBtn()"
          ontouchstart="move(0,1)" ontouchend="stopBtn()">⬅</button>
        <button class="btn-stop" onclick="stopBtn()">STOP</button>
        <button id="btn-right" class="dir"
          onmousedown="move(0,-1)" onmouseup="stopBtn()"
          ontouchstart="move(0,-1)" ontouchend="stopBtn()">➡</button>
      </div>
      <div class="row">
        <button id="btn-back" class="dir"
          onmousedown="move(-1,0)" onmouseup="stopBtn()"
          ontouchstart="move(-1,0)" ontouchend="stopBtn()">⬇</button>
      </div>
      <div class="sliders">
        <div class="lbl">VITESSE LINÉAIRE</div>
        <input type="range" id="speed" min="0.1" max="1.0" step="0.1" value="0.4"
               oninput="document.getElementById('sv').textContent=this.value+' m/s'"/>
        <div class="speed-val" id="sv">0.4 m/s</div>
        <div class="lbl">VITESSE ROTATION</div>
        <input type="range" id="turn" min="0.1" max="1.5" step="0.1" value="0.8"
               oninput="document.getElementById('tv').textContent=this.value+' r/s'"/>
        <div class="speed-val" id="tv">0.8 r/s</div>
      </div>
    </div>
  </div>

<script>
function move(linear, angular) {
  const s = parseFloat(document.getElementById('speed').value);
  const t = parseFloat(document.getElementById('turn').value);
  fetch('/cmd', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({linear:linear*s, angular:angular*t}) });
}
function stopBtn() {
  fetch('/cmd', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({linear:0, angular:0}) });
}
async function resetMap() { await fetch('/map_reset', {method:'POST'}); }

const pressed = {};
const keyMap = { ArrowUp:'fwd',z:'fwd',Z:'fwd', ArrowDown:'back',s:'back',S:'back',
                 ArrowLeft:'left',q:'left',Q:'left', ArrowRight:'right',d:'right',D:'right' };
const keyAction = { fwd:()=>move(1,0), back:()=>move(-1,0), left:()=>move(0,1), right:()=>move(0,-1) };
document.addEventListener('keydown', e => {
  const a = keyMap[e.key]; if (!a || pressed[e.key]) return;
  pressed[e.key] = true; keyAction[a]();
  const id={fwd:'btn-fwd',back:'btn-back',left:'btn-left',right:'btn-right'}[a];
  if (id) document.getElementById(id).classList.add('pressed');
});
document.addEventListener('keyup', e => {
  if (!keyMap[e.key]) return; delete pressed[e.key]; stopBtn();
  ['btn-fwd','btn-back','btn-left','btn-right']
    .forEach(id => document.getElementById(id).classList.remove('pressed'));
});

// ── LiDAR canvas ─────────────────────────────────────────────
const lcv  = document.getElementById('lidar-cv');
const lctx = lcv.getContext('2d');
const LW=lcv.width, LH=lcv.height, LCX=LW/2, LCY=LH-14, L_SCALE=22, HP=Math.PI/2;
let lidarTs=Date.now(), lidarN=0;

function drawLidar(data) {
  const {ranges, angle_min, angle_increment, range_max} = data;
  lctx.fillStyle='#060c06'; lctx.fillRect(0,0,LW,LH);

  lctx.strokeStyle='#0a1f0a'; lctx.lineWidth=1;
  lctx.fillStyle='#112211'; lctx.font='8px monospace';
  for (let r=1; r*L_SCALE<LH+10; r++) {
    lctx.beginPath(); lctx.arc(LCX,LCY,r*L_SCALE,Math.PI,2*Math.PI); lctx.stroke();
    if (r*L_SCALE<LH-4) lctx.fillText(r+'m', LCX+r*L_SCALE+2, LCY-2);
  }
  lctx.strokeStyle='#091409'; lctx.fillStyle='#1b3a1b'; lctx.font='8px monospace';
  for (let deg=-90; deg<=90; deg+=30) {
    const a=deg*Math.PI/180;
    lctx.beginPath(); lctx.moveTo(LCX,LCY);
    lctx.lineTo(LCX-Math.sin(a)*260, LCY-Math.cos(a)*260); lctx.stroke();
    if (deg!==0) lctx.fillText(deg+'°',
      LCX-Math.sin(a)*(LH-18)+(deg<0?2:-18), LCY-Math.cos(a)*(LH-18));
  }
  for (let i=0; i<ranges.length; i++) {
    const angle = angle_min + i*angle_increment;
    if (angle < -HP || angle > HP) continue;
    const r = ranges[i];
    if (!isFinite(r) || r<0.05 || r>=range_max*0.98) continue;
    const px=r*L_SCALE;
    const sx=LCX-Math.sin(angle)*px, sy=LCY-Math.cos(angle)*px;
    if (sy<0) continue;
    lctx.fillStyle = r/range_max<0.4 ? '#00e676' : '#00695c';
    lctx.fillRect(sx-1.5, sy-1.5, 3, 3);
  }
  lctx.fillStyle='#2ecc71';
  lctx.beginPath(); lctx.arc(LCX,LCY,5,0,2*Math.PI); lctx.fill();
  lctx.strokeStyle='#27ae60'; lctx.lineWidth=2;
  lctx.beginPath(); lctx.moveTo(LCX,LCY); lctx.lineTo(LCX,LCY-14); lctx.stroke();

  lidarN++;
  const now=Date.now();
  if (now-lidarTs>=1000) {
    document.getElementById('lidar-fps').textContent=lidarN+' Hz';
    lidarN=0; lidarTs=now;
  }
}

// ── Carte canvas ──────────────────────────────────────────────
// Conventions :
//   +x ROS (avant)  → droite écran  :  sx = ox + (px-rx)*scale
//   +y ROS (gauche) → haut écran    :  sy = oy - (py-ry)*scale
//   cap flèche      : dx=cos(θ), dy=-sin(θ)
const mcv  = document.getElementById('map-cv');
const mctx = mcv.getContext('2d');
const MW=mcv.width, MH=mcv.height, M_SCALE=28;

function drawMap(pts, pose) {
  mctx.fillStyle='#060806'; mctx.fillRect(0,0,MW,MH);
  const rx=pose.x, ry=pose.y, rt=pose.theta;
  const ox=MW/2, oy=MH/2;

  const gx=((ox-rx*M_SCALE)%M_SCALE+M_SCALE)%M_SCALE;
  const gy=((oy+ry*M_SCALE)%M_SCALE+M_SCALE)%M_SCALE;
  mctx.strokeStyle='#0d160d'; mctx.lineWidth=1;
  for (let x=gx-M_SCALE; x<MW+M_SCALE; x+=M_SCALE) {
    mctx.beginPath(); mctx.moveTo(x,0); mctx.lineTo(x,MH); mctx.stroke(); }
  for (let y=gy-M_SCALE; y<MH+M_SCALE; y+=M_SCALE) {
    mctx.beginPath(); mctx.moveTo(0,y); mctx.lineTo(MW,y); mctx.stroke(); }

  const orX=ox-rx*M_SCALE, orY=oy+ry*M_SCALE;
  if (orX>-10&&orX<MW+10&&orY>-10&&orY<MH+10) {
    mctx.strokeStyle='#1a4a1a'; mctx.lineWidth=1;
    mctx.beginPath(); mctx.moveTo(orX-6,orY); mctx.lineTo(orX+6,orY); mctx.stroke();
    mctx.beginPath(); mctx.moveTo(orX,orY-6); mctx.lineTo(orX,orY+6); mctx.stroke();
  }

  mctx.fillStyle='rgba(0,200,100,0.75)';
  for (const [px,py] of pts) {
    const sx=ox+(px-rx)*M_SCALE, sy=oy-(py-ry)*M_SCALE;
    if (sx<-2||sx>MW+2||sy<-2||sy>MH+2) continue;
    mctx.fillRect(sx-1,sy-1,2,2);
  }

  const fwdX=Math.cos(rt), fwdY=-Math.sin(rt);  // y écran inversé
  const L=13, pX=-fwdY, pY=fwdX;
  mctx.fillStyle='#27ae60';
  mctx.beginPath(); mctx.arc(ox,oy,6,0,2*Math.PI); mctx.fill();
  mctx.strokeStyle='#e53935'; mctx.lineWidth=2;
  mctx.beginPath(); mctx.moveTo(ox,oy); mctx.lineTo(ox+fwdX*L,oy+fwdY*L); mctx.stroke();
  mctx.fillStyle='#e53935';
  mctx.beginPath();
  mctx.moveTo(ox+fwdX*L, oy+fwdY*L);
  mctx.lineTo(ox+fwdX*(L-5)+pX*3, oy+fwdY*(L-5)+pY*3);
  mctx.lineTo(ox+fwdX*(L-5)-pX*3, oy+fwdY*(L-5)-pY*3);
  mctx.closePath(); mctx.fill();

  document.getElementById('map-pts').textContent=pts.length+' pts';
}

// ── Polling ───────────────────────────────────────────────────
let odomOk=false;

async function pollLidar() {
  try {
    const d = await (await fetch('/scan_live')).json();
    if (d.ok) {
      drawLidar(d);
      const p=d.pose;
      if (!odomOk && (Math.abs(p.x)>0.001||Math.abs(p.y)>0.001||Math.abs(p.theta)>0.001)) {
        odomOk=true;
        const b=document.getElementById('odom-badge');
        b.textContent='ODOM OK'; b.classList.add('ok');
      }
      document.getElementById('pose-info').textContent =
        `x:${p.x.toFixed(2)}  y:${p.y.toFixed(2)}  θ:${(p.theta*180/Math.PI).toFixed(1)}°`;
    }
  } catch(e){}
  setTimeout(pollLidar, 100);
}

async function pollMap() {
  try {
    const d = await (await fetch('/map_data')).json();
    drawMap(d.points, d.pose);
  } catch(e){}
  setTimeout(pollMap, 500);
}

pollLidar();
pollMap();
</script>
</body>
</html>
'''

# ────────────────────────────────────────────────────────────────────────────
#  Point d'entrée
# ────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    threading.Thread(target=ros_spin, daemon=True).start()
    time.sleep(2)
    print('Serveur sur http://0.0.0.0:5000')
    app.run(host='0.0.0.0', port=5000, threaded=True)
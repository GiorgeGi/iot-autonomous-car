// -------------------- MQTT SETUP --------------------

const client = mqtt.connect('ws://10.42.0.44:8083');
const topic = 'esp32/sensor/data';

client.on('connect', () => {
  console.log("Connected to MQTT");
  client.subscribe(topic);
});

client.on('error', (err) => {
  console.error("MQTT Error:", err);
});

client.on('close', () => {
  console.warn("MQTT disconnected, attempting to reconnect…");
});

// -------------------- CHART.JS SETUP --------------------

/**
 * Creates a line chart with a fixed 10-point buffer.
 * @param {HTMLCanvasElement} canvasEl  – the <canvas> element
 * @param {string[]} labels            – array of dataset labels
 * @param {string[]} colors            – array of border colors, matching labels
 */
function createLineChart(canvasEl, labels, colors) {
  canvasEl.height = 300;

  return new Chart(canvasEl, {
    type: 'line',
    data: {
      labels: Array(10).fill(""),     // placeholder labels
      datasets: labels.map((label, i) => ({
        label,
        borderColor: colors[i],
        data: Array(10).fill(0),
        fill: false,
        tension: 0.1,
        pointRadius: 0
      }))
    },
    options: {
      animation: false,
      maintainAspectRatio: true,
      responsive: true,
      scales: {
        y: {
          beginAtZero: false
        }
      },
      plugins: {
        legend: {
          display: true,
          position: 'bottom'
        }
      }
    }
  });
}

function updateChart(chart, values) {
  chart.data.datasets.forEach((dataset, i) => {
    dataset.data.push(values[i]);
    if (dataset.data.length > 10) {
      dataset.data.shift();
    }
  });
  chart.update('none');
}

// Initialize Temperature and Orientation charts
const tempChart = createLineChart(
  document.getElementById('tempChart'),
  ["Temperature"],
  ["#E67E22"]
);

const oriChart = createLineChart(
  document.getElementById('orientationChart'),
  ["Pitch", "Roll", "Yaw"],
  ["#8E44AD", "#1ABC9C", "#7F8C8D"]
);

// -------------------- THREE.JS SETUP --------------------

const canvas3d = document.getElementById('three-canvas');
const renderer = new THREE.WebGLRenderer({ canvas: canvas3d, antialias: true });
renderer.setSize(canvas3d.clientWidth, canvas3d.clientWidth);
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
camera.position.z = 5;

// Light so normals show
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(0, 0, 10);
scene.add(light);

// Placeholder parent object
const model = new THREE.Object3D();
scene.add(model);

// Load the STL
const loader = new THREE.STLLoader();
loader.load(
  'bugatti.stl',
  function (geometry) {
    geometry.center();
    const material = new THREE.MeshNormalMaterial();
    const mesh = new THREE.Mesh(geometry, material);
    mesh.scale.set(0.01, 0.01, 0.01);
    model.add(mesh);
  },
  undefined,
  function (err) {
    console.error('Error loading STL:', err);
  }
);

// Render loop
function animate() {
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();

// Keep the 3D canvas square on resize
window.addEventListener('resize', () => {
  const size = canvas3d.clientWidth;
  renderer.setSize(size, size);
  camera.aspect = 1;
  camera.updateProjectionMatrix();
});

// -------------------- COMPASS DRAWING --------------------

const compassCanvas = document.getElementById('compassCanvas');
const ctx = compassCanvas.getContext('2d');
const cw = compassCanvas.width;   // 300
const ch = compassCanvas.height;  // 300
const radius = cw / 2 * 0.9;      // leave 10% padding

/**
 * Draws the static compass dial (circle + N/E/S/W labels).
 * This only needs to run once at startup.
 */
function drawCompassDial() {
  ctx.clearRect(0, 0, cw, ch);
  ctx.save();
  ctx.translate(cw / 2, ch / 2);

  // Draw outer circle
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, Math.PI * 2);
  ctx.strokeStyle = "#333";
  ctx.lineWidth = 4;
  ctx.stroke();

  // Draw tick marks and cardinal labels
  ctx.font = "16px Arial";
  ctx.fillStyle = "#333";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";

  // North
  ctx.fillText("N", 0, -radius + 20);
  // East
  ctx.fillText("E", radius - 20, 0);
  // South
  ctx.fillText("S", 0, radius - 20);
  // West
  ctx.fillText("W", -radius + 20, 0);

  // Optional: minor ticks every 30°
  for (let angle = 0; angle < 360; angle += 30) {
    const rad = (angle * Math.PI) / 180;
    const x1 = (radius - 10) * Math.sin(rad);
    const y1 = -(radius - 10) * Math.cos(rad);
    const x2 = radius * Math.sin(rad);
    const y2 = -radius * Math.cos(rad);

    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.strokeStyle = "#555";
    ctx.lineWidth = (angle % 90 === 0) ? 3 : 1; // heavier at N/E/S/W
    ctx.stroke();
  }

  ctx.restore();
}

/**
 * Draws the rotating needle on top of the static dial.
 * @param {number} yawDeg  – yaw in degrees (0 = North)
 */
function drawNeedle(yawDeg) {
  // First, redraw the static dial
  drawCompassDial();

  ctx.save();
  ctx.translate(cw / 2, ch / 2);

  // Rotate so that 0° = pointing straight up
  ctx.rotate((Math.PI / 180) * yawDeg);

  // Draw needle (triangle or line)
  ctx.beginPath();
  ctx.moveTo(0, -radius + 20);    // tip of the needle
  ctx.lineTo(-10, 0);             // left base
  ctx.lineTo(10, 0);              // right base
  ctx.closePath();
  ctx.fillStyle = "red";
  ctx.fill();

  ctx.restore();
}

// Initial draw (assume yaw=0 at startup)
drawCompassDial();

// -------------------- RESPONSIVENESS --------------------------

// Listen for resize and shrink the WebGL canvas on small widths
window.addEventListener('resize', () => {
  const w = window.innerWidth;

  if (w <= 600) {
    // On small screens, force the 3D canvas to 200×200
    renderer.setSize(370, 370);
    camera.aspect = 1;
    camera.updateProjectionMatrix();
  } else {
    // On larger screens, revert to parent-width square
    const size = canvas3d.clientWidth;
    renderer.setSize(size, size);
    camera.aspect = 1;
    camera.updateProjectionMatrix();
  }
});

// -------------------- MQTT MESSAGE HANDLER --------------------

client.on('message', (topic, message) => {
  try {
    const data = JSON.parse(message.toString());
    const { pitch, roll, yaw, temperature } = data;

    // 1) Update the Temperature chart
    updateChart(tempChart, [temperature]);

    // 2) Update the Orientation chart
    updateChart(oriChart, [pitch, roll, yaw]);

    // 3) Rotate the 3D model
    model.rotation.y = yaw * Math.PI / 4;     // intentional yaw/4
    model.rotation.x = pitch * Math.PI / 180; // pitch in radians
    model.rotation.z = roll * Math.PI / 180;  // roll in radians

    // 4) Draw the compass needle using yaw (0 = North)
    //    If you want yaw=0 to point up, but your sensor’s 0 = whatever,
    //    you can offset accordingly. Here we assume yaw=0 → north.
    const sensitivity = 40;   // try 2, 4, 6, etc., until it feels right
    drawNeedle(yaw * sensitivity);
  } catch (err) {
    console.error("MQTT JSON error:", err);
  }
});


import streamlit as st
import threading
import time
import queue
import warnings

warnings.filterwarnings("ignore")

st.set_page_config(
    page_title="ATOM Robot Control",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="collapsed"
)

st.markdown(
    """
<style>
@import url('https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@300;400;500;600&display=swap');

:root {
    --bg-primary: #0a0c0f;
    --bg-secondary: #111318;
    --bg-card: #161a21;
    --border: #1e2530;
    --border-bright: #2a3344;
    --accent-blue: #3b82f6;
    --accent-red: #ef4444;
    --accent-green: #22c55e;
    --accent-amber: #f59e0b;
    --text-primary: #e2e8f0;
    --text-secondary: #64748b;
    --text-mono: #94a3b8;
}

html, body, .stApp {
    background-color: var(--bg-primary) !important;
    font-family: 'IBM Plex Sans', sans-serif;
    color: var(--text-primary);
}

#MainMenu, footer, header {visibility: hidden;}
.block-container {padding: 1.5rem 2rem !important; max-width: 100% !important;}

.atom-header {
    display: flex;
    align-items: center;
    gap: 16px;
    padding: 20px 0 24px 0;
    border-bottom: 1px solid var(--border);
    margin-bottom: 24px;
}

.atom-logo {
    width: 42px;
    height: 42px;
    background: var(--accent-blue);
    border-radius: 8px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 20px;
    font-weight: 700;
    font-family: 'IBM Plex Mono', monospace;
    color: white;
}

.atom-title {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 22px;
    font-weight: 600;
    letter-spacing: -0.5px;
    color: var(--text-primary);
}

.atom-subtitle {
    font-size: 12px;
    color: var(--text-secondary);
    font-family: 'IBM Plex Mono', monospace;
    letter-spacing: 1px;
    text-transform: uppercase;
}

.status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 6px;
}

.status-dot.online { background: var(--accent-green); box-shadow: 0 0 6px var(--accent-green); }
.status-dot.offline { background: var(--text-secondary); }

.card {
    background: var(--bg-card);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 20px;
    margin-bottom: 16px;
}

.card-label {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 1.5px;
    text-transform: uppercase;
    color: var(--text-secondary);
    margin-bottom: 12px;
}

.command-wrapper {
    display: flex;
    gap: 8px;
    align-items: stretch;
}

.stTextInput > div > div > input {
    background: var(--bg-secondary) !important;
    border: 1px solid var(--border-bright) !important;
    border-radius: 8px !important;
    color: var(--text-primary) !important;
    font-family: 'IBM Plex Mono', monospace !important;
    font-size: 14px !important;
    padding: 12px 16px !important;
}

.stTextInput > div > div > input:focus {
    border-color: var(--accent-blue) !important;
    box-shadow: 0 0 0 2px rgba(59,130,246,0.15) !important;
}

.stButton > button {
    font-family: 'IBM Plex Sans', sans-serif !important;
    font-weight: 500 !important;
    border-radius: 8px !important;
    border: none !important;
    transition: all 0.15s ease !important;
    cursor: pointer !important;
}

.send-btn > button {
    background: var(--accent-blue) !important;
    color: white !important;
    width: 100% !important;
    padding: 12px !important;
    font-size: 14px !important;
}

.send-btn > button:hover:not(:disabled) {
    background: #2563eb !important;
    transform: translateY(-1px) !important;
}

.send-btn > button:disabled {
    background: var(--border) !important;
    color: var(--text-secondary) !important;
    cursor: not-allowed !important;
}

.estop-btn > button {
    background: var(--accent-red) !important;
    color: white !important;
    width: 100% !important;
    padding: 18px !important;
    font-size: 16px !important;
    font-weight: 600 !important;
    letter-spacing: 0.5px !important;
    border-radius: 10px !important;
}

.estop-btn > button:hover {
    background: #dc2626 !important;
    box-shadow: 0 0 20px rgba(239,68,68,0.4) !important;
    transform: translateY(-1px) !important;
}

.resume-btn > button {
    background: var(--accent-green) !important;
    color: white !important;
    width: 100% !important;
    padding: 12px !important;
    font-size: 14px !important;
    font-weight: 500 !important;
}

.resume-btn > button:hover {
    background: #16a34a !important;
}

.dock-btn > button {
    background: var(--bg-secondary) !important;
    color: var(--text-primary) !important;
    border: 1px solid var(--border-bright) !important;
    width: 100% !important;
    padding: 12px !important;
    font-size: 13px !important;
}

.dock-btn > button:hover {
    border-color: var(--accent-blue) !important;
    color: var(--accent-blue) !important;
}

.mic-btn > button {
    background: var(--bg-secondary) !important;
    border: 1px solid var(--border-bright) !important;
    color: var(--text-primary) !important;
    padding: 12px 16px !important;
    font-size: 16px !important;
    border-radius: 8px !important;
    min-width: 48px !important;
}

.mic-btn.recording > button {
    background: rgba(239,68,68,0.15) !important;
    border-color: var(--accent-red) !important;
    color: var(--accent-red) !important;
    animation: pulse 1s infinite !important;
}

@keyframes pulse {
    0%, 100% { box-shadow: 0 0 0 0 rgba(239,68,68,0.4); }
    50% { box-shadow: 0 0 0 6px rgba(239,68,68,0); }
}

.status-badge {
    display: inline-flex;
    align-items: center;
    padding: 4px 10px;
    border-radius: 20px;
    font-family: 'IBM Plex Mono', monospace;
    font-size: 11px;
    font-weight: 500;
}

.badge-idle { background: rgba(100,116,139,0.15); color: var(--text-secondary); border: 1px solid var(--border); }
.badge-active { background: rgba(59,130,246,0.15); color: var(--accent-blue); border: 1px solid rgba(59,130,246,0.3); }
.badge-done { background: rgba(34,197,94,0.15); color: var(--accent-green); border: 1px solid rgba(34,197,94,0.3); }
.badge-estop { background: rgba(239,68,68,0.15); color: var(--accent-red); border: 1px solid rgba(239,68,68,0.3); }
.badge-warning { background: rgba(245,158,11,0.15); color: var(--accent-amber); border: 1px solid rgba(245,158,11,0.3); }

.log-container {
    background: var(--bg-secondary);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
    height: 140px;
    overflow-y: auto;
    font-family: 'IBM Plex Mono', monospace;
    font-size: 11px;
    color: var(--text-mono);
    line-height: 1.6;
}

hr { border-color: var(--border) !important; margin: 16px 0 !important; }

.speech-component {
    background: var(--bg-secondary);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 8px 12px;
    font-family: 'IBM Plex Mono', monospace;
    font-size: 12px;
    color: var(--text-secondary);
    margin-top: 6px;
    min-height: 28px;
}
</style>
""",
    unsafe_allow_html=True,
)

@st.cache_resource
def init_ros():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init()

        class ATOMBridge(Node):
            def __init__(self):
                super().__init__("atom_streamlit_bridge")
                self.task_status = "IDLE"
                self.is_ready = True
                self.is_estopped = False
                self.log_queue = queue.Queue(maxsize=50)

                self.create_subscription(String, "/atom/task_status", self._task_status_cb, 10)
                self._cmd_pub = self.create_publisher(String, "/atom/resolved_target", 10)
                self._estop_pub = self.create_publisher(String, "/atom/emergency_stop", 10)
                self._dock_pub = self.create_publisher(String, "/atom/dock_command", 10)

                self._log("ATOM Streamlit bridge initialized")
                self._log("Waiting for robot nodes...")

            def _log(self, msg):
                ts = time.strftime("%H:%M:%S")
                entry = f"[{ts}] {msg}"
                try:
                    self.log_queue.put_nowait(entry)
                except queue.Full:
                    try:
                        self.log_queue.get_nowait()
                    except Exception:
                        pass
                    try:
                        self.log_queue.put_nowait(entry)
                    except Exception:
                        pass

            def _task_status_cb(self, msg):
                status = msg.data
                self.task_status = status
                self._log(f"Status: {status[:60]}")

                if any(s in status for s in ["GOAL COMPLETED", "OBJECT_NOT_FOUND", "IDLE", "EMERGENCY_RESUME"]):
                    self.is_ready = True
                elif any(s in status for s in ["SCANNING", "MEMORY_NAV", "MOVING_TO_SCAN", "CENTERING", "DEPTH_CHECK", "APPROACHING", "DRIVING_1M"]):
                    self.is_ready = False
                elif "EMERGENCY_STOP" in status:
                    self.is_estopped = True
                    self.is_ready = False

            def send_command(self, command: str):
                msg = String()
                msg.data = command
                self._cmd_pub.publish(msg)
                self.is_ready = False
                self._log(f"Command sent: {command}")

            def emergency_stop(self):
                msg = String()
                msg.data = "STOP"
                self._estop_pub.publish(msg)
                self.is_estopped = True
                self.is_ready = False
                self._log("⚠ EMERGENCY STOP ACTIVATED")

            def emergency_resume(self):
                msg = String()
                msg.data = "RESUME"
                self._estop_pub.publish(msg)
                self.is_estopped = False
                self.is_ready = True
                self._log("✓ Emergency stop cleared — IDLE")

            def dock(self):
                msg = String()
                msg.data = "DOCK"
                self._dock_pub.publish(msg)
                self._log("Dock command sent")

            def undock(self):
                msg = String()
                msg.data = "UNDOCK"
                self._dock_pub.publish(msg)
                self._log("Undock command sent")

        node = ATOMBridge()

        def spin():
            import rclpy
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)

        t = threading.Thread(target=spin, daemon=True)
        t.start()
        return node, True

    except Exception:
        return None, False

if "command_text" not in st.session_state:
    st.session_state.command_text = ""
if "logs" not in st.session_state:
    st.session_state.logs = []
if "recording" not in st.session_state:
    st.session_state.recording = False

node, ros_ok = init_ros()

st.markdown(
    """
<div class="atom-header">
    <div class="atom-logo">A</div>
    <div>
        <div class="atom-title">ATOM Control Center</div>
        <div class="atom-subtitle">Autonomous Task & Object Management</div>
    </div>
</div>
""",
    unsafe_allow_html=True,
)

col_left, col_right = st.columns([3, 2], gap="large")

with col_left:
    status_col1, status_col2 = st.columns([1, 1])

    with status_col1:
        status_placeholder = st.empty()
    with status_col2:
        ros_status = st.empty()

    st.markdown('<div style="height:8px"></div>', unsafe_allow_html=True)
    st.markdown('<div class="card-label">Activity Log</div>', unsafe_allow_html=True)
    log_placeholder = st.empty()

with col_right:
    st.markdown('<div class="card-label">Task Command</div>', unsafe_allow_html=True)

    speech_html = """
    <div style="margin-bottom: 8px;">
        <button id="micBtn" onclick="toggleMic()" style="
            background: #161a21;
            border: 1px solid #1e2530;
            border-radius: 8px;
            color: #e2e8f0;
            padding: 10px 16px;
            font-size: 16px;
            cursor: pointer;
            transition: all 0.2s;
            font-family: 'IBM Plex Sans', sans-serif;
            display: flex; align-items: center; gap: 8px;
        " onmouseover="this.style.borderColor='#3b82f6'"
           onmouseout="if(!window.isRecording){this.style.borderColor='#1e2530'}">
            <span id="micIcon">🎤</span>
            <span id="micLabel" style="font-size:13px; font-weight:500;">Start Voice Input</span>
        </button>
        <div id="speechStatus" style="
            margin-top: 8px;
            padding: 8px 12px;
            background: #111318;
            border: 1px solid #1e2530;
            border-radius: 8px;
            font-family: 'IBM Plex Mono', monospace;
            font-size: 11px;
            color: #64748b;
            min-height: 28px;
        ">Ready for voice input...</div>
    </div>

    <script>
    var recognition = null;
    window.isRecording = false;

    function initRecognition() {
        if (!('webkitSpeechRecognition' in window) && !('SpeechRecognition' in window)) {
            document.getElementById('speechStatus').innerHTML = 'Speech recognition not supported in this browser.';
            return null;
        }
        var SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        var rec = new SpeechRecognition();
        rec.continuous = false;
        rec.interimResults = true;
        rec.lang = 'en-US';

        rec.onstart = function() {
            window.isRecording = true;
            document.getElementById('micBtn').style.background = 'rgba(239,68,68,0.15)';
            document.getElementById('micBtn').style.borderColor = '#ef4444';
            document.getElementById('micBtn').style.color = '#ef4444';
            document.getElementById('micIcon').textContent = '⏹';
            document.getElementById('micLabel').textContent = 'Recording... (click to stop)';
            document.getElementById('speechStatus').style.color = '#ef4444';
            document.getElementById('speechStatus').innerHTML = '🔴 Listening...';
        };

        rec.onresult = function(event) {
            var interim = '';
            var final = '';
            for (var i = event.resultIndex; i < event.results.length; i++) {
                if (event.results[i].isFinal) {
                    final += event.results[i][0].transcript;
                } else {
                    interim += event.results[i][0].transcript;
                }
            }
            var display = final || interim;
            document.getElementById('speechStatus').innerHTML =
                (interim ? '<span style="color:#f59e0b">◐ ' : '<span style="color:#22c55e">✓ ') +
                display + '</span>';
            if (final) {
                window.parent.postMessage({type: 'speech_result', text: final.trim()}, '*');
            }
        };

        rec.onerror = function(event) {
            document.getElementById('speechStatus').innerHTML =
                '<span style="color:#ef4444">Error: ' + event.error + '</span>';
            resetMicBtn();
        };

        rec.onend = function() {
            window.isRecording = false;
            resetMicBtn();
        };

        return rec;
    }

    function resetMicBtn() {
        window.isRecording = false;
        document.getElementById('micBtn').style.background = '#161a21';
        document.getElementById('micBtn').style.borderColor = '#1e2530';
        document.getElementById('micBtn').style.color = '#e2e8f0';
        document.getElementById('micIcon').textContent = '🎤';
        document.getElementById('micLabel').textContent = 'Start Voice Input';
    }

    function toggleMic() {
        if (window.isRecording) {
            if (recognition) recognition.stop();
            return;
        }
        recognition = initRecognition();
        if (recognition) recognition.start();
    }

    window.addEventListener('message', function(e) {
        if (e.data && e.data.type === 'speech_result') {
            var inputs = window.parent.document.querySelectorAll('input[type="text"]');
            if (inputs.length > 0) {
                var nativeInputValueSetter = Object.getOwnPropertyDescriptor(
                    window.parent.HTMLInputElement.prototype, 'value').set;
                nativeInputValueSetter.call(inputs[0], e.data.text);
                inputs[0].dispatchEvent(new Event('input', { bubbles: true }));
            }
            document.getElementById('speechStatus').innerHTML =
                '<span style="color:#22c55e">✓ Captured: ' + e.data.text + '</span>';
        }
    });
    </script>
    """

    st.components.v1.html(speech_html, height=120)

    command = st.text_input(
        label="command_input",
        label_visibility="collapsed",
        placeholder="Type a command or use voice input above...",
        key="command_input",
        value=st.session_state.command_text
    )

    st.markdown('<div style="height:8px"></div>', unsafe_allow_html=True)

    is_ready = node.is_ready if node else False
    send_disabled = not is_ready or not command.strip()

    send_col, _ = st.columns([1, 0.01])
    with send_col:
        st.markdown('<div class="send-btn">', unsafe_allow_html=True)
        send_clicked = st.button(
            "▶  Send Command" if is_ready else "⏳  Robot Busy — Wait",
            disabled=send_disabled,
            use_container_width=True,
            key="send_btn"
        )
        st.markdown('</div>', unsafe_allow_html=True)

    if send_clicked and command.strip() and node:
        node.send_command(command.strip())
        st.session_state.command_text = ""
        st.rerun()

    st.markdown('<hr>', unsafe_allow_html=True)
    st.markdown('<div class="card-label">Safety Controls</div>', unsafe_allow_html=True)

    is_estopped = node.is_estopped if node else False

    st.markdown('<div class="estop-btn">', unsafe_allow_html=True)
    if st.button("⛔  EMERGENCY STOP", use_container_width=True, key="estop_btn"):
        if node:
            node.emergency_stop()
            st.rerun()
    st.markdown('</div>', unsafe_allow_html=True)

    st.markdown('<div style="height:8px"></div>', unsafe_allow_html=True)

    if is_estopped:
        st.markdown('<div class="resume-btn">', unsafe_allow_html=True)
        if st.button("✓  Resume Operation", use_container_width=True, key="resume_btn"):
            if node:
                node.emergency_resume()
                st.rerun()
        st.markdown('</div>', unsafe_allow_html=True)

    st.markdown('<hr>', unsafe_allow_html=True)
    st.markdown('<div class="card-label">Dock Controls</div>', unsafe_allow_html=True)

    dcol1, dcol2 = st.columns(2, gap="small")
    with dcol1:
        st.markdown('<div class="dock-btn">', unsafe_allow_html=True)
        if st.button("⬇  Dock Robot", use_container_width=True, key="dock_btn"):
            if node:
                node.dock()
        st.markdown('</div>', unsafe_allow_html=True)

    with dcol2:
        st.markdown('<div class="dock-btn">', unsafe_allow_html=True)
        if st.button("⬆  Undock Robot", use_container_width=True, key="undock_btn"):
            if node:
                node.undock()
        st.markdown('</div>', unsafe_allow_html=True)

if node:
    status = node.task_status
    if "EMERGENCY_STOP" in status:
        badge_class = "badge-estop"
        badge_text = "⛔ EMERGENCY STOP"
    elif "GOAL COMPLETED" in status:
        badge_class = "badge-done"
        badge_text = "✓ GOAL COMPLETED"
    elif "OBJECT_NOT_FOUND" in status:
        badge_class = "badge-warning"
        badge_text = "⚠ OBJECT NOT FOUND"
    elif any(s in status for s in ["SCANNING", "MEMORY_NAV", "MOVING_TO_SCAN", "CENTERING", "DEPTH_CHECK", "APPROACHING"]):
        badge_class = "badge-active"
        badge_text = f"● {status[:40]}"
    else:
        badge_class = "badge-idle"
        badge_text = "○ IDLE — Ready"

    status_placeholder.markdown(
        f'<span class="status-badge {badge_class}">{badge_text}</span>',
        unsafe_allow_html=True
    )

    ros_status.markdown(
        '<span class="status-badge badge-done"><span class="status-dot online"></span>ROS2 Connected</span>',
        unsafe_allow_html=True
    )

    logs = []
    while not node.log_queue.empty():
        try:
            logs.append(node.log_queue.get_nowait())
        except Exception:
            break

    st.session_state.logs.extend(logs)
    st.session_state.logs = st.session_state.logs[-30:]

    log_html = '<div class="log-container">' + "<br>".join(st.session_state.logs[-15:]) + "</div>"
    log_placeholder.markdown(log_html, unsafe_allow_html=True)
else:
    ros_status.markdown(
        '<span class="status-badge badge-estop"><span class="status-dot offline"></span>ROS2 Disconnected</span>',
        unsafe_allow_html=True
    )

time.sleep(0.3)
st.rerun()
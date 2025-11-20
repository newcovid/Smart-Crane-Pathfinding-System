const { createApp, reactive, ref, onMounted, watch, toRefs } = Vue;

import Canvas2d from './components/Canvas2d.js';
import Canvas3d from './components/Canvas3d.js';
import LogPanel from './components/LogPanel.js';
import { ToolBtn, ToggleSwitch, NumberInput } from './components/UIComponents.js';

const socket = io();

const app = createApp({
    components: {
        'canvas-2d': Canvas2d,
        'canvas-3d': Canvas3d,
        'log-panel': LogPanel,
        'tool-btn': ToolBtn,
        'toggle-switch': ToggleSwitch,
        'number-input': NumberInput
    },

    setup() {
        const state = reactive({
            isDark: true,
            connection: { connected: false, statusText: '连接中...', latency: 0 },

            // 完整配置对象 (与 config.py 保持一致)
            settings: {
                // 地图
                MAP_WIDTH_M: 100, MAP_LENGTH_M: 100, MAP_HEIGHT_M: 20, MAP_RESOLUTION_M: 0.5,
                // 物理
                CRANE_FOOTPRINT_SHAPE: 'box',
                CRANE_FOOTPRINT_WIDTH: 5.0,
                CRANE_FOOTPRINT_LENGTH: 5.0,
                CRANE_FOOTPRINT_HEIGHT: 2.0,
                // 安全
                CRANE_Z_SAFETY_MARGIN: 0.5,
                ENABLE_FIXED_HEIGHT_CRUISE: true,
                CRANE_SAFE_TRAVEL_Z_M: 11.0,
                OBSTACLE_INFINITE_HEIGHT: true,
                DEFAULT_OBSTACLE_HEIGHT_M: 2.0,
                // 算法
                PLANNER_ALGORITHM: 'astar',
                USE_3D_OCTILE: false,
                ENABLE_SHORTCUT_OPTIMIZATION: true, // 注意：使用全名
                ENABLE_BEZIER_SMOOTHING: true,
                BEZIER_SMOOTHNESS: 0.3,
                BEZIER_SEGMENTS: 10
            },

            mapState: { width: 0, length: 0, static_obstacles: {}, dynamic_obstacles: {}, resolution: 0.5 },
            pathData: [],
            lastStats: null,
            missionState: { start: { x: 5, y: 5 }, end: { x: 30, y: 20 } },
            ui: { sidebarOpen: true, viewMode: '2d', showLogs: true },
            config: { snapToGrid: true },
            interaction: { mode: 'view', dragging: false, startPos: null, addHeight: 5.0 },
            visuals: { showPathNodes: true, showPathSafety: false },

            logs: []
        });

        const toggleTheme = () => {
            state.isDark = !state.isDark;
            if (state.isDark) document.documentElement.classList.add('dark');
            else document.documentElement.classList.remove('dark');
        };

        const snap = (v) => {
            if (!state.config.snapToGrid) return v;
            const res = state.mapState.resolution || 0.5;
            return Math.round(v / res) * res;
        };

        // --- Socket Events ---
        socket.on('connect', () => { state.connection.connected = true; state.connection.statusText = '在线'; });
        socket.on('disconnect', () => { state.connection.connected = false; state.connection.statusText = '离线'; });
        socket.on('pong', (ms) => { state.connection.latency = ms; });

        socket.on('update_map_state', (d) => {
            state.mapState = {
                width: d.width_m, length: d.length_m,
                static_obstacles: d.static_obstacles || {},
                dynamic_obstacles: d.dynamic_obstacles || {},
                resolution: d.resolution_m
            };

            // 深度合并配置，确保服务端状态覆盖前端默认值
            if (d.system_config) {
                Object.assign(state.settings, d.system_config);
            }

            if (d.mission_state) state.missionState = d.mission_state;
        });

        socket.on('update_path', (p) => { state.pathData = p; });
        socket.on('planning_stats', (s) => { state.lastStats = s; });

        socket.on('server_log', (log) => {
            const timeStr = new Date(log.time * 1000).toLocaleTimeString('en-GB', { hour12: false });
            state.logs.push({ ...log, time: timeStr });
            if (state.logs.length > 200) state.logs.shift();
        });

        socket.on('operation_failed', (d) => { alert('操作失败: ' + d.message); });

        // --- Actions ---
        // 发送完整配置
        const applySettings = () => {
            socket.emit('update_settings', state.settings);
        };

        const requestPlan = () => {
            // 每次请求路径前也尝试同步配置，防止未应用
            socket.emit('update_settings', state.settings);
            socket.emit('request_path', { start: state.missionState.start, end: state.missionState.end });
        };

        const handleMapClick = (e) => {
            const pos = { x: snap(e.x), y: snap(e.y) };
            const mode = state.interaction.mode;
            if (mode === 'view') return;

            if (mode.startsWith('add_')) {
                if (e.type === 'down') {
                    state.interaction.startPos = pos;
                    state.interaction.dragging = true;
                } else if (e.type === 'up' && state.interaction.dragging) {
                    const s = state.interaction.startPos;
                    let w = Math.abs(pos.x - s.x);
                    let h = Math.abs(pos.y - s.y);
                    if (w < 0.1) w = 2;
                    if (h < 0.1) h = 2;
                    const type = mode === 'add_static' ? 'static' : 'dynamic';
                    const finalX = Math.min(s.x, pos.x);
                    const finalY = Math.min(s.y, pos.y);

                    socket.emit('add_obstacle', {
                        type, x: finalX, y: finalY, w, h,
                        z: state.interaction.addHeight
                    });
                    state.interaction.dragging = false;
                }
            } else if (e.type === 'up') {
                if (mode === 'set_start') { state.missionState.start = pos; socket.emit('sync_mission_coordinates', state.missionState); requestPlan(); }
                else if (mode === 'set_end') { state.missionState.end = pos; socket.emit('sync_mission_coordinates', state.missionState); requestPlan(); }
                else if (mode === 'remove') { socket.emit('remove_obstacle_near', { x: e.x, y: e.y }); }
            }
        };

        const handleMapDrag = (e) => {
            if (state.interaction.dragging && state.interaction.mode.startsWith('add_')) {
                const s = state.interaction.startPos;
                const c = { x: snap(e.x), y: snap(e.y) };
                if (e.setPreview) {
                    e.setPreview({
                        x: Math.min(s.x, c.x), y: Math.min(s.y, c.y),
                        w: Math.abs(c.x - s.x) || 1, h: Math.abs(c.y - s.y) || 1
                    });
                }
            }
        };

        // 快捷键支持
        onMounted(() => {
            if (document.documentElement.classList.contains('dark')) state.isDark = true;
            window.addEventListener('keydown', (e) => {
                if ((e.ctrlKey || e.metaKey) && e.key === 's') {
                    e.preventDefault();
                    applySettings();
                }
            });
        });

        return { ...toRefs(state), toggleTheme, applySettings, requestPlan, handleMapClick, handleMapDrag };
    }
});
app.mount('#app');
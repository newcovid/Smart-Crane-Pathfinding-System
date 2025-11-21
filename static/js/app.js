const { createApp, reactive, ref, onMounted, watch, toRefs, markRaw, nextTick, onErrorCaptured } = Vue;

import Canvas2d from './components/Canvas2d.js';
import Canvas3d from './components/Canvas3d.js';
import LogPanel from './components/LogPanel.js';
import {
    ToolBtn,
    ToggleSwitch,
    NumberInput,
    SelectInput,
    SectionGroup,
    StatsPanel
} from './components/UIComponents.js';

const socket = io();

const app = createApp({
    components: {
        'canvas-2d': Canvas2d,
        'canvas-3d': Canvas3d,
        'log-panel': LogPanel,
        'tool-btn': ToolBtn,
        'toggle-switch': ToggleSwitch,
        'number-input': NumberInput,
        'select-input': SelectInput,
        'section-group': SectionGroup,
        'stats-panel': StatsPanel
    },

    setup() {
        const state = reactive({
            isDark: false,
            connection: { connected: false, statusText: '连接中...', latency: 0 },

            settings: {
                MAP_WIDTH_M: 100, MAP_LENGTH_M: 100, MAP_HEIGHT_M: 20, MAP_RESOLUTION_M: 0.5,
                CRANE_FOOTPRINT_SHAPE: 'box',
                CRANE_FOOTPRINT_WIDTH: 5.0,
                CRANE_FOOTPRINT_LENGTH: 5.0,
                CRANE_FOOTPRINT_HEIGHT: 2.0,
                CRANE_Z_SAFETY_MARGIN: 0.5,
                ENABLE_FIXED_HEIGHT_CRUISE: true,
                CRANE_SAFE_TRAVEL_Z_M: 11.0,
                OBSTACLE_INFINITE_HEIGHT: true,
                DEFAULT_OBSTACLE_HEIGHT_M: 2.0,
                PLANNER_ALGORITHM: 'astar',
                USE_3D_OCTILE: false,
                ENABLE_SHORTCUT_OPTIMIZATION: true,
                ENABLE_BEZIER_SMOOTHING: true,
                BEZIER_SMOOTHNESS: 0.3,
                BEZIER_SEGMENTS: 10,
                HEURISTIC_WEIGHT: 1.5
            },

            mapState: {
                width: 0, length: 0,
                static_obstacles: {}, dynamic_obstacles: {},
                resolution: 0.5,
                inflated_grid: null
            },

            pathData: [],
            lastStats: null,
            missionState: {
                start: { x: 5, y: 5, z: 5.0 },
                end: { x: 30, y: 20, z: 5.0 }
            },
            // [UI Layout State]
            ui: {
                sidebarOpen: true,
                viewMode: '2d',
                showLogs: false, // 默认收起
                sidebarWidth: 340, // 初始宽度
                logHeight: 200,    // 初始展开高度
                resizingSidebar: false,
                resizingLog: false
            },
            config: { snapToGrid: true },
            interaction: { mode: 'view', dragging: false, startPos: null, addHeight: 5.0 },
            visuals: { showPathNodes: true, safetyMode: 'footprint' },
            logs: []
        });

        // --- Layout Resizing Logic ---
        const startResizeSidebar = (e) => {
            state.ui.resizingSidebar = true;
            const startX = e.clientX;
            const startWidth = state.ui.sidebarWidth;

            const doDrag = (e) => {
                let newWidth = startWidth + (e.clientX - startX);
                newWidth = Math.max(280, Math.min(newWidth, 500)); // 限制范围
                state.ui.sidebarWidth = newWidth;
            };

            const stopDrag = () => {
                state.ui.resizingSidebar = false;
                window.removeEventListener('mousemove', doDrag);
                window.removeEventListener('mouseup', stopDrag);
            };

            window.addEventListener('mousemove', doDrag);
            window.addEventListener('mouseup', stopDrag);
        };

        const startResizeLog = (e) => {
            state.ui.resizingLog = true;
            const startY = e.clientY;
            const startHeight = state.ui.logHeight;

            const doDrag = (e) => {
                // 向上拖动是增加高度，所以是 start - current
                let newHeight = startHeight + (startY - e.clientY);
                newHeight = Math.max(100, Math.min(newHeight, window.innerHeight - 100));
                state.ui.logHeight = newHeight;
            };

            const stopDrag = () => {
                state.ui.resizingLog = false;
                window.removeEventListener('mousemove', doDrag);
                window.removeEventListener('mouseup', stopDrag);
            };

            window.addEventListener('mousemove', doDrag);
            window.addEventListener('mouseup', stopDrag);
        };

        // --- Error Handling ---
        onErrorCaptured((err, instance, info) => {
            console.error('[Vue Error]', err, info);
            return false;
        });

        const toggleTheme = () => {
            state.isDark = !state.isDark;
            applyTheme();
        };

        const applyTheme = () => {
            if (state.isDark) {
                document.documentElement.classList.add('dark');
            } else {
                document.documentElement.classList.remove('dark');
            }
        };

        const snap = (v) => {
            if (!state.config.snapToGrid) return Number(v.toFixed(2));
            const res = state.mapState.resolution || 0.5;
            return Math.round(v / res) * res;
        };

        // Socket Handlers
        socket.on('connect', () => {
            state.connection.connected = true;
            state.connection.statusText = '在线';
        });
        socket.on('disconnect', () => {
            state.connection.connected = false;
            state.connection.statusText = '离线';
        });
        socket.on('pong', (ms) => { state.connection.latency = ms; });

        socket.on('update_map_state', (d) => {
            try {
                state.mapState.width = d.width_m;
                state.mapState.length = d.length_m;
                state.mapState.resolution = d.resolution_m;
                state.mapState.static_obstacles = d.static_obstacles || {};
                state.mapState.dynamic_obstacles = d.dynamic_obstacles || {};

                if (d.active_inflated_grid) {
                    state.mapState.inflated_grid = markRaw(d.active_inflated_grid);
                }

                if (d.system_config) Object.assign(state.settings, d.system_config);
                if (d.mission_state) {
                    if (d.mission_state.start) Object.assign(state.missionState.start, d.mission_state.start);
                    if (d.mission_state.end) Object.assign(state.missionState.end, d.mission_state.end);
                }
            } catch (e) {
                console.error('[Map] Update Error:', e);
            }
        });

        socket.on('update_path', (p) => {
            try {
                state.pathData = markRaw(p || []);
            } catch (e) {
                console.error('[Path] Update Error:', e);
            }
        });

        socket.on('planning_stats', (s) => { state.lastStats = s; });

        socket.on('server_log', (log) => {
            if (state.logs.length > 100) state.logs.shift();
            const timeStr = new Date(log.time * 1000).toLocaleTimeString('en-GB', { hour12: false });
            state.logs.push({ ...log, time: timeStr });
        });

        socket.on('operation_failed', (d) => { alert('操作失败: ' + d.message); });

        const applySettings = () => {
            socket.emit('update_settings', state.settings);
            socket.emit('sync_mission_coordinates', state.missionState);
        };

        const requestPlan = () => {
            socket.emit('update_settings', state.settings);
            socket.emit('request_path', { start: state.missionState.start, end: state.missionState.end });
        };

        const switchView = (mode) => {
            state.ui.viewMode = mode;
        };

        const handleMapClick = (e) => {
            let x = snap(e.x);
            let y = snap(e.y);
            if (state.mapState.width) x = Math.max(0, Math.min(x, state.mapState.width));
            if (state.mapState.length) y = Math.max(0, Math.min(y, state.mapState.length));

            const pos = { x, y };
            const mode = state.interaction.mode;
            if (mode === 'view') return;

            if (mode.startsWith('add_')) {
                if (e.type === 'down') {
                    state.interaction.startPos = pos;
                    state.interaction.dragging = true;
                } else if (e.type === 'up' && state.interaction.dragging) {
                    const s = state.interaction.startPos;
                    let w = snap(Math.abs(pos.x - s.x));
                    let h = snap(Math.abs(pos.y - s.y));
                    if (w < 0.1) w = state.mapState.resolution * 2 || 1;
                    if (h < 0.1) h = state.mapState.resolution * 2 || 1;

                    const type = mode === 'add_static' ? 'static' : 'dynamic';
                    const finalX = Math.min(s.x, pos.x);
                    const finalY = Math.min(s.y, pos.y);

                    state.interaction.dragging = false;
                    socket.emit('add_obstacle', {
                        type, x: finalX, y: finalY, w, h,
                        z: state.interaction.addHeight
                    });
                }
            } else if (e.type === 'up') {
                if (mode === 'set_start') {
                    state.missionState.start.x = pos.x;
                    state.missionState.start.y = pos.y;
                    socket.emit('sync_mission_coordinates', state.missionState);
                    requestPlan();
                } else if (mode === 'set_end') {
                    state.missionState.end.x = pos.x;
                    state.missionState.end.y = pos.y;
                    socket.emit('sync_mission_coordinates', state.missionState);
                    requestPlan();
                } else if (mode === 'remove') {
                    socket.emit('remove_obstacle_near', { x: e.x, y: e.y });
                }
            }
        };

        const handleMapDrag = (e) => {
            if (state.interaction.dragging && state.interaction.mode.startsWith('add_')) {
                const s = state.interaction.startPos;
                let cx = snap(e.x);
                let cy = snap(e.y);
                if (state.mapState.width) cx = Math.max(0, Math.min(cx, state.mapState.width));
                if (state.mapState.length) cy = Math.max(0, Math.min(cy, state.mapState.length));
                const c = { x: cx, y: cy };

                if (e.setPreview) {
                    e.setPreview({
                        x: Math.min(s.x, c.x), y: Math.min(s.y, c.y),
                        w: Math.abs(c.x - s.x) || 1, h: Math.abs(c.y - s.y) || 1
                    });
                }
            }
        };

        onMounted(() => {
            const sysDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
            state.isDark = sysDark;
            applyTheme();

            window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
                state.isDark = e.matches;
                applyTheme();
            });

            window.addEventListener('keydown', (e) => {
                if ((e.ctrlKey || e.metaKey) && e.key === 's') {
                    e.preventDefault();
                    applySettings();
                }
            });

            if (socket.connected) {
                state.connection.connected = true;
                state.connection.statusText = '在线';
            }
        });

        return {
            ...toRefs(state),
            toggleTheme, applySettings, requestPlan, handleMapClick, handleMapDrag, switchView,
            startResizeSidebar, startResizeLog
        };
    }
});
app.mount('#app');
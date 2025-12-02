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
    StatsPanel,
    NotificationToast
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
        'stats-panel': StatsPanel,
        'notification-toast': NotificationToast
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
                // [新增] Rust 加速开关默认开启
                ENABLE_RUST_CORE: true,
                USE_3D_OCTILE: false,
                ENABLE_SHORTCUT_OPTIMIZATION: true,
                ENABLE_BEZIER_SMOOTHING: true,
                BEZIER_SMOOTHNESS: 0.3,
                BEZIER_SEGMENTS: 10,
                HEURISTIC_WEIGHT: 1.5
            },

            mapState: {
                width: 0, length: 0, height_m: 20,
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
            ui: {
                sidebarOpen: true,
                viewMode: '2d',
                showLogs: false,
                sidebarWidth: 340,
                logHeight: 200,
                resizingSidebar: false,
                resizingLog: false
            },
            config: { snapToGrid: true },
            interaction: { mode: 'view', dragging: false, startPos: null, addHeight: 5.0 },
            visuals: { showPathNodes: true, safetyMode: 'footprint' },
            logs: []
        });

        // --- 通知系统 ---
        const notifications = ref([]);

        const notify = (type, message, title = '') => {
            // [优化] 限制最大弹窗数量为 4 条
            if (notifications.value.length >= 4) {
                // 移除最旧的一条
                notifications.value.shift();
            }

            const id = Date.now() + Math.random();
            // [New] 生成当前时间 HH:mm:ss
            const time = new Date().toLocaleTimeString('en-GB', { hour12: false });

            notifications.value.push({ id, type, message, title, time });
            const duration = type === 'error' ? 6000 : 4000;
            setTimeout(() => {
                removeNotification(id);
            }, duration);
        };

        const removeNotification = (id) => {
            notifications.value = notifications.value.filter(n => n.id !== id);
        };

        // --- Layout Resizing Logic ---
        const startResizeSidebar = (e) => {
            state.ui.resizingSidebar = true;
            const startX = e.clientX;
            const startWidth = state.ui.sidebarWidth;

            const doDrag = (e) => {
                let newWidth = startWidth + (e.clientX - startX);
                newWidth = Math.max(280, Math.min(newWidth, 500));
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

        // Error Boundary
        onErrorCaptured((err, instance, info) => {
            console.error('[Vue Error]', err, info);
            let msg = '未知错误';
            if (err) {
                if (err.message) msg = err.message;
                else if (typeof err === 'string') msg = err;
            }
            if (msg.includes('ResizeObserver loop')) return false;
            notify('error', `${msg}`, '前端组件异常');
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

        // Socket
        socket.on('connect', () => {
            state.connection.connected = true;
            state.connection.statusText = '在线';
            notify('info', '已成功连接到服务器。', '连接恢复');
        });
        socket.on('disconnect', () => {
            state.connection.connected = false;
            state.connection.statusText = '离线';
            notify('error', '与服务器的连接已断开。', '连接丢失');
        });
        socket.on('pong', (ms) => { state.connection.latency = ms; });

        socket.on('update_map_state', (d) => {
            try {
                state.mapState.width = d.width_m;
                state.mapState.length = d.length_m;
                state.mapState.height_m = d.height_m;
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
                const safePath = (p || []).map(pt => {
                    if (Array.isArray(pt) && pt.length >= 2) {
                        return [Number(pt[0]) || 0, Number(pt[1]) || 0, Number(pt[2]) || 0];
                    }
                    return null;
                }).filter(x => x !== null);
                state.pathData = markRaw(safePath);
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

        socket.on('operation_failed', (d) => { notify('error', d.message, '操作失败'); });
        socket.on('operation_success', (d) => { notify('success', d.message, '操作成功'); });

        const applySettings = () => {
            socket.emit('update_settings', state.settings);
            socket.emit('sync_mission_coordinates', state.missionState);
        };

        const requestPlan = () => {
            // [优化] 将 settings 包含在 payload 中，确保后端原子化处理
            const payload = {
                start: state.missionState.start,
                end: state.missionState.end,
                settings: state.settings
            };
            // 移除单独的 update_settings 调用，避免异步竞态条件
            socket.emit('request_path', payload);
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

                    // [修改] 优化拖拽结束后的最小尺寸判定
                    // 原逻辑为 resolution * 2 (强制 2x2)，现改为 resolution (允许 1x1)
                    // 使用 || 0.5 防止 resolution 未加载时出错
                    const minRes = state.mapState.resolution || 0.5;

                    if (w < minRes / 2) w = minRes;
                    if (h < minRes / 2) h = minRes;

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
                    // [修改] 优化拖拽时的视觉反馈
                    // 将默认预览尺寸从硬编码的 1 改为当前网格分辨率
                    // 这样拖拽初期显示的框体大小就和网格完全对齐，手感更"跟手"
                    const minRes = state.mapState.resolution || 0.5;

                    e.setPreview({
                        x: Math.min(s.x, c.x), y: Math.min(s.y, c.y),
                        w: Math.abs(c.x - s.x) || minRes,
                        h: Math.abs(c.y - s.y) || minRes
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
                notify('info', '已成功连接到服务器。', '连接恢复');
            }
        });

        return {
            ...toRefs(state),
            toggleTheme, applySettings, requestPlan, handleMapClick, handleMapDrag, switchView,
            startResizeSidebar, startResizeLog,
            notifications, removeNotification
        };
    }
});
app.mount('#app');
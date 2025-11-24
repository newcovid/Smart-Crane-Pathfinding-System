const { ref, reactive, onMounted, onBeforeUnmount, watch, nextTick } = Vue;

// [核心优化] 模块级全局状态
// 即使组件被销毁（v-if 切换），这些数据也会保留在内存中
const globalViewState = {
    scale: 10.0,
    offsetX: 50,
    offsetY: 50,
    hasInitialized: false // 标记是否已完成首次居中
};

export default {
    props: ['map', 'interaction', 'path', 'mission', 'settings', 'visuals', 'isDark', 'active'],
    emits: ['map-click', 'map-drag'],
    template: `
        <div ref="container" class="w-full h-full relative overflow-hidden cursor-crosshair transition-colors duration-300" :class="isDark ? 'bg-[#0f172a]' : 'bg-slate-100'">
            <canvas ref="canvas" class="block touch-none w-full h-full"></canvas>
            
            <!-- HUD Info Panel -->
            <div class="absolute bottom-6 left-6 pointer-events-none select-none animate-fadeIn">
                <div class="glass-panel px-4 py-2.5 rounded-full text-xs text-slate-600 dark:text-slate-400 flex gap-6 shadow-lg border border-slate-200 dark:border-slate-700 backdrop-blur-md">
                    <span class="flex items-center gap-1.5"><i class="ph-bold ph-mouse-left text-primary text-base"></i> {{ interaction.mode === 'view' ? '平移' : '操作' }}</span>
                    <span class="flex items-center gap-1.5"><i class="ph-bold ph-mouse-middle text-primary text-base"></i> 平移</span>
                    <span class="flex items-center gap-1.5"><i class="ph-bold ph-mouse-scroll text-primary text-base"></i> 缩放</span>
                    <span class="border-l border-slate-300 dark:border-slate-600 pl-4 font-mono font-bold text-slate-700 dark:text-slate-300">
                        {{ map.width }}m × {{ map.length }}m
                    </span>
                </div>
            </div>
        </div>
    `,
    setup(props, { emit }) {
        const container = ref(null);
        const canvas = ref(null);
        const ctx = ref(null);
        const resizeObserver = ref(null);

        // [优化] 使用全局状态初始化本地响应式数据
        const view = reactive({
            scale: globalViewState.scale,
            offsetX: globalViewState.offsetX,
            offsetY: globalViewState.offsetY
        });

        // [优化] 双向同步：本地变化 -> 写入全局
        watch(view, (newVal) => {
            globalViewState.scale = newVal.scale;
            globalViewState.offsetX = newVal.offsetX;
            globalViewState.offsetY = newVal.offsetY;
        });

        const isPanning = ref(false);
        const isInteracting = ref(false);
        const lastMouse = reactive({ x: 0, y: 0 });
        const dragRect = ref(null);
        const hoveredPoint = ref(null);

        // 坐标转换
        const toScreen = (wx, wy) => {
            const mapLen = props.map.length || 100;
            return {
                x: wx * view.scale + view.offsetX,
                y: (mapLen - wy) * view.scale + view.offsetY
            };
        };

        const toWorld = (sx, sy) => {
            const mapLen = props.map.length || 100;
            return {
                x: (sx - view.offsetX) / view.scale,
                y: mapLen - (sy - view.offsetY) / view.scale
            };
        };

        const render = () => {
            if (!props.active) return; // 不活跃时不渲染
            if (!canvas.value || !container.value) return;

            const cw = container.value.clientWidth;
            const ch = container.value.clientHeight;
            if (cw === 0 || ch === 0) return;

            const dpr = window.devicePixelRatio || 1;
            const displayWidth = Math.floor(cw * dpr);
            const displayHeight = Math.floor(ch * dpr);

            if (canvas.value.width !== displayWidth || canvas.value.height !== displayHeight) {
                canvas.value.width = displayWidth;
                canvas.value.height = displayHeight;
            }

            if (!ctx.value) ctx.value = canvas.value.getContext('2d');
            const cx = ctx.value;
            cx.resetTransform();
            cx.scale(dpr, dpr);

            if (!props.map.width) return;

            const w = props.map.width;
            const l = props.map.length;

            // 背景
            cx.fillStyle = props.isDark ? '#0f172a' : '#f1f5f9';
            cx.fillRect(0, 0, cw, ch);

            // Grid
            cx.beginPath();
            const step = Math.max(1, Math.floor(20 / view.scale));
            for (let x = 0; x <= w; x += step) {
                const p0 = toScreen(x, 0); const p1 = toScreen(x, l);
                cx.moveTo(p0.x, p0.y); cx.lineTo(p1.x, p1.y);
            }
            for (let y = 0; y <= l; y += step) {
                const p0 = toScreen(0, y); const p1 = toScreen(w, y);
                cx.moveTo(p0.x, p0.y); cx.lineTo(p1.x, p1.y);
            }
            cx.strokeStyle = props.isDark ? '#1e293b' : '#cbd5e1';
            cx.lineWidth = 1;
            cx.stroke();

            // Map Bounds
            const bl = toScreen(0, 0);
            const tr = toScreen(w, l);
            cx.strokeStyle = props.isDark ? '#475569' : '#94a3b8';
            cx.lineWidth = 2;
            cx.strokeRect(bl.x, tr.y, tr.x - bl.x, bl.y - tr.y);

            // Inflation
            if (props.visuals.safetyMode === 'inflation' && props.map.inflated_grid) {
                const grid = props.map.inflated_grid;
                const res = props.map.resolution;
                cx.fillStyle = 'rgba(239, 68, 68, 0.2)';
                const rows = grid.length;
                const cols = rows > 0 ? grid[0].length : 0;
                cx.beginPath();
                for (let r = 0; r < rows; r++) {
                    for (let c = 0; c < cols; c++) {
                        let isBlocked = Array.isArray(grid[r][c]) ? grid[r][c].some(z => z === 1) : grid[r][c] === 1;
                        if (isBlocked) {
                            const p = toScreen(c * res, (r + 1) * res);
                            cx.rect(p.x, p.y, res * view.scale, res * view.scale);
                        }
                    }
                }
                cx.fill();
            }

            // Obstacles
            let safeZ = 0;
            const isInfinite = props.settings.OBSTACLE_INFINITE_HEIGHT;
            const zMargin = Number(props.settings.CRANE_Z_SAFETY_MARGIN) || 0.5;
            if (props.settings.ENABLE_FIXED_HEIGHT_CRUISE) {
                safeZ = Number(props.settings.CRANE_SAFE_TRAVEL_Z_M) || 100;
            } else {
                if (props.path && props.path.length > 0) {
                    let maxZ = -Infinity;
                    for (const pt of props.path) { if (pt[2] !== undefined && pt[2] > maxZ) maxZ = pt[2]; }
                    safeZ = maxZ;
                } else {
                    const mapH = Number(props.settings.MAP_HEIGHT_M) || 20.0;
                    const craneH = Number(props.settings.CRANE_FOOTPRINT_HEIGHT) || 2.0;
                    safeZ = mapH - (craneH / 2.0);
                }
            }

            Object.values({ ...props.map.static_obstacles, ...props.map.dynamic_obstacles } || {}).forEach(o => {
                const p = toScreen(o.x_m, o.y_m + o.h_m);
                const obsHeight = o.z_m || 100;
                const isIgnored = !isInfinite && (obsHeight <= (safeZ - zMargin));
                cx.fillStyle = o.type === 'dynamic' ? '#ef4444' : (props.isDark ? '#64748b' : '#94a3b8');
                if (o.h_m === undefined) cx.fillStyle = props.isDark ? '#64748b' : '#94a3b8';
                cx.globalAlpha = isIgnored ? 0.15 : 1.0;
                cx.fillRect(p.x, p.y, o.w_m * view.scale, o.h_m * view.scale);
                cx.strokeStyle = props.isDark ? 'rgba(255,255,255,0.3)' : 'rgba(0,0,0,0.3)';
                cx.globalAlpha = isIgnored ? 0.3 : 1.0;
                cx.strokeRect(p.x, p.y, o.w_m * view.scale, o.h_m * view.scale);
                cx.globalAlpha = 1.0;
                if (view.scale > 5) {
                    cx.fillStyle = props.isDark ? '#94a3b8' : '#475569';
                    cx.font = 'bold 13px "Inter", sans-serif';
                    const hText = isInfinite ? '∞' : (o.z_m ? `${o.z_m}m` : '-');
                    cx.fillStyle = isIgnored ? (props.isDark ? '#475569' : '#94a3b8') : (props.isDark ? '#94a3b8' : '#475569');
                    cx.fillText(`H:${hText}`, p.x + 5, p.y + 16);
                }
            });

            // Start/End & Path
            const drawFootprint = (pt, color, isFill = false) => {
                if (!pt) return;
                const p = toScreen(pt.x, pt.y);
                const shape = props.settings.CRANE_FOOTPRINT_SHAPE;
                const width = props.settings.CRANE_FOOTPRINT_WIDTH || 5.0;
                const length = props.settings.CRANE_FOOTPRINT_LENGTH || 5.0;
                cx.fillStyle = color; cx.strokeStyle = color; cx.lineWidth = 1;
                if (shape === 'circle') {
                    const radius = (width / 2) * view.scale;
                    cx.beginPath(); cx.arc(p.x, p.y, radius, 0, Math.PI * 2);
                    if (isFill) { cx.globalAlpha = 0.15; cx.fill(); cx.globalAlpha = 1.0; }
                    cx.stroke();
                } else {
                    const w = width * view.scale; const h = length * view.scale;
                    if (isFill) { cx.globalAlpha = 0.15; cx.fillRect(p.x - w / 2, p.y - h / 2, w, h); cx.globalAlpha = 1.0; }
                    cx.strokeRect(p.x - w / 2, p.y - h / 2, w, h);
                }
            };

            if (props.path && props.path.length > 0 && props.visuals.safetyMode === 'footprint') {
                const step = Math.max(1, Math.floor(props.path.length / 40));
                for (let i = 0; i < props.path.length; i += step) drawFootprint({ x: props.path[i][0], y: props.path[i][1] }, 'rgba(59, 130, 246, 0.4)', true);
            }
            if (props.mission.start) drawFootprint(props.mission.start, '#10b981', true);
            if (props.mission.end) drawFootprint(props.mission.end, '#f43f5e', true);

            if (props.path && props.path.length > 1) {
                cx.beginPath();
                const start = toScreen(props.path[0][0], props.path[0][1]);
                cx.moveTo(start.x, start.y);
                for (let i = 1; i < props.path.length; i++) {
                    const pt = toScreen(props.path[i][0], props.path[i][1]);
                    cx.lineTo(pt.x, pt.y);
                }
                cx.strokeStyle = '#3b82f6'; cx.lineWidth = 3; cx.lineJoin = 'round'; cx.stroke();
                if (props.visuals.showPathNodes) {
                    cx.fillStyle = '#3b82f6';
                    for (let i = 0; i < props.path.length; i++) {
                        const pt = toScreen(props.path[i][0], props.path[i][1]);
                        cx.beginPath(); cx.arc(pt.x, pt.y, props.visuals.safetyMode === 'inflation' ? 4 : 3, 0, Math.PI * 2); cx.fill();
                    }
                }
            }

            // Labels
            const drawLabel = (pt, label, color) => {
                if (!pt) return;
                const p = toScreen(pt.x, pt.y);
                cx.beginPath(); cx.arc(p.x, p.y, 7, 0, Math.PI * 2); cx.fillStyle = color; cx.fill(); cx.strokeStyle = '#fff'; cx.lineWidth = 2.5; cx.stroke();
                cx.fillStyle = props.isDark ? '#fff' : '#0f172a'; cx.font = 'bold 14px "Inter", sans-serif'; cx.strokeStyle = props.isDark ? '#0f172a' : '#fff'; cx.lineWidth = 3;
                cx.strokeText(label, p.x + 12, p.y + 4); cx.fillText(label, p.x + 12, p.y + 4);
            }
            if (props.mission.start) drawLabel(props.mission.start, '起点', '#10b981');
            if (props.mission.end) drawLabel(props.mission.end, '终点', '#f43f5e');

            // Drag Rect & Hover
            if (dragRect.value) {
                const p = toScreen(dragRect.value.x, dragRect.value.y + dragRect.value.h);
                cx.fillStyle = 'rgba(59, 130, 246, 0.3)'; cx.fillRect(p.x, p.y, dragRect.value.w * view.scale, dragRect.value.h * view.scale);
                cx.strokeStyle = '#3b82f6'; cx.strokeRect(p.x, p.y, dragRect.value.w * view.scale, dragRect.value.h * view.scale);
            }
            if (hoveredPoint.value) {
                const p = toScreen(hoveredPoint.value.x, hoveredPoint.value.y);
                const text = `(${hoveredPoint.value.x.toFixed(1)}, ${hoveredPoint.value.y.toFixed(1)}, ${hoveredPoint.value.z.toFixed(1)})`;
                cx.font = '10px "JetBrains Mono", monospace';
                const metrics = cx.measureText(text); const w = metrics.width + 8; const h = 20;
                cx.fillStyle = 'rgba(15, 23, 42, 0.9)'; cx.roundRect(p.x + 10, p.y - h / 2, w, h, 4); cx.fill();
                cx.fillStyle = '#fff'; cx.fillText(text, p.x + 14, p.y + 4);
                cx.beginPath(); cx.arc(p.x, p.y, 5, 0, Math.PI * 2); cx.strokeStyle = '#fff'; cx.stroke();
            }
        };

        // 事件处理保持不变，主要是操作 view 对象
        const handleMouseDown = (e) => {
            isInteracting.value = true;
            const rect = canvas.value.getBoundingClientRect();
            if (e.button === 1 || e.button === 2 || props.interaction.mode === 'view') {
                isPanning.value = true; lastMouse.x = e.clientX; lastMouse.y = e.clientY; return;
            }
            if (e.button === 0) emit('map-click', { ...toWorld(e.clientX - rect.left, e.clientY - rect.top), type: 'down' });
        };
        const handleMouseMove = (e) => {
            const rect = canvas.value.getBoundingClientRect();
            const mouseX = e.clientX - rect.left; const mouseY = e.clientY - rect.top;

            // Hover check
            if (props.path && props.path.length > 0) {
                let found = null;
                for (const pt of props.path) {
                    const sc = toScreen(pt[0], pt[1]);
                    if (Math.hypot(sc.x - mouseX, sc.y - mouseY) < 8) { found = { x: pt[0], y: pt[1], z: pt[2] || 0 }; break; }
                }
                if (found !== hoveredPoint.value) { hoveredPoint.value = found; render(); }
            }

            if (!isInteracting.value && !isPanning.value) return;
            if (isPanning.value) {
                view.offsetX += e.clientX - lastMouse.x;
                view.offsetY += e.clientY - lastMouse.y;
                lastMouse.x = e.clientX; lastMouse.y = e.clientY;
                render();
            } else {
                emit('map-drag', { ...toWorld(mouseX, mouseY), setPreview: (r) => { dragRect.value = r; render(); } });
            }
        };
        const handleMouseUp = (e) => {
            if (!isInteracting.value && !isPanning.value) return;
            isInteracting.value = false; isPanning.value = false; dragRect.value = null;
            const rect = canvas.value.getBoundingClientRect();
            emit('map-click', { ...toWorld(e.clientX - rect.left, e.clientY - rect.top), type: 'up' });
            render();
        };
        const handleWheel = (e) => {
            e.preventDefault();
            const rect = canvas.value.getBoundingClientRect();
            const mouseX = e.clientX - rect.left; const mouseY = e.clientY - rect.top;
            const worldPos = toWorld(mouseX, mouseY);
            const zoomFactor = Math.exp(e.deltaY * -0.001);
            view.scale = Math.max(2, Math.min(200, view.scale * zoomFactor));
            view.offsetX = mouseX - worldPos.x * view.scale;
            view.offsetY = mouseY - (props.map.length - worldPos.y) * view.scale;
            render();
        };

        const tryInitializeView = () => {
            // [核心] 检查全局状态，如果全局已经初始化过，就不再重置视角
            if (globalViewState.hasInitialized) return;
            if (!container.value || container.value.clientWidth === 0) return;

            if (props.map.width) {
                const cw = container.value.clientWidth;
                const ch = container.value.clientHeight;
                const scaleX = (cw - 100) / props.map.width;
                const scaleY = (ch - 100) / props.map.length;
                view.scale = Math.min(scaleX, scaleY);
                view.offsetX = (cw - props.map.width * view.scale) / 2;
                view.offsetY = (ch - props.map.length * view.scale) / 2;

                // 标记全局已初始化
                globalViewState.hasInitialized = true;
                render();
            }
        };

        onMounted(() => {
            resizeObserver.value = new ResizeObserver(() => { tryInitializeView(); render(); });
            if (container.value) resizeObserver.value.observe(container.value);
            canvas.value.addEventListener('mousedown', handleMouseDown);
            canvas.value.addEventListener('contextmenu', e => e.preventDefault());
            window.addEventListener('mousemove', handleMouseMove);
            window.addEventListener('mouseup', handleMouseUp);
            canvas.value.addEventListener('wheel', handleWheel, { passive: false });
            setTimeout(tryInitializeView, 500);
        });

        onBeforeUnmount(() => {
            if (resizeObserver.value) resizeObserver.value.disconnect();
            window.removeEventListener('mousemove', handleMouseMove);
            window.removeEventListener('mouseup', handleMouseUp);
        });

        watch(() => props.active, (val) => {
            if (val) nextTick(() => { tryInitializeView(); render(); });
        });

        watch(() => [props.map, props.path, props.mission, props.isDark, props.settings, props.visuals], () => requestAnimationFrame(render), { deep: true });

        return { container, canvas };
    }
};
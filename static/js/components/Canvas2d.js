const { ref, reactive, onMounted, watch } = Vue;

export default {
    props: ['map', 'interaction', 'path', 'mission', 'settings', 'visuals', 'isDark', 'active'],
    emits: ['map-click', 'map-drag'],
    template: `
        <div ref="container" class="w-full h-full relative overflow-hidden cursor-crosshair bg-slate-100 dark:bg-[#0f172a]">
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

        const view = reactive({ scale: 10.0, offsetX: 50, offsetY: 50 });
        const isPanning = ref(false);
        const isInteracting = ref(false);
        const lastMouse = reactive({ x: 0, y: 0 });
        const dragRect = ref(null);

        const toScreen = (wx, wy) => ({ x: wx * view.scale + view.offsetX, y: wy * view.scale + view.offsetY });
        const toWorld = (sx, sy) => ({ x: (sx - view.offsetX) / view.scale, y: (sy - view.offsetY) / view.scale });

        const render = () => {
            if (!canvas.value || !container.value) return;
            const cw = container.value.clientWidth;
            const ch = container.value.clientHeight;
            if (cw === 0 || ch === 0) return;

            // High DPI 适配
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

            // 1. 背景
            cx.fillStyle = props.isDark ? '#0f172a' : '#f1f5f9';
            cx.fillRect(0, 0, cw, ch);

            // 2. 网格
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

            // 3. 边界
            const origin = toScreen(0, 0);
            const corner = toScreen(w, l);
            cx.strokeStyle = props.isDark ? '#475569' : '#94a3b8';
            cx.lineWidth = 2;
            cx.strokeRect(origin.x, origin.y, corner.x - origin.x, corner.y - origin.y);

            // 4. 障碍物 (带高度)

            // [优化渲染] 计算参考安全高度 (Safe Reference Z)
            // 逻辑: 低于此高度的障碍物将被半透明化处理
            let safeZ = 0;
            const isInfinite = props.settings.OBSTACLE_INFINITE_HEIGHT;
            const zMargin = Number(props.settings.CRANE_Z_SAFETY_MARGIN) || 0.5;

            if (props.settings.ENABLE_FIXED_HEIGHT_CRUISE) {
                // 模式 A: 定高巡航 -> 使用设定的巡航高度
                safeZ = Number(props.settings.CRANE_SAFE_TRAVEL_Z_M) || 100;
            } else {
                // 模式 B: 3D 变高巡航
                if (props.path && props.path.length > 0) {
                    // 情况 B1: 已有规划路径 -> 使用路径中的最大实际高度
                    // 这样可以直观展示当前路径是否真的“飞越”了障碍物
                    let maxZ = -Infinity;
                    for (const pt of props.path) {
                        // pt: [x, y, z]
                        if (pt[2] !== undefined && pt[2] > maxZ) {
                            maxZ = pt[2];
                        }
                    }
                    safeZ = maxZ;
                } else {
                    // 情况 B2: 无路径 -> 计算理论最大起升高度 (预览模式)
                    // MaxZ = 地图总高度(天花板) - 吊具半高(防止穿顶)
                    // 这允许用户在调整参数时实时看到哪些障碍物是“潜在可飞越”的
                    const mapH = Number(props.settings.MAP_HEIGHT_M) || 20.0;
                    const craneH = Number(props.settings.CRANE_FOOTPRINT_HEIGHT) || 2.0;
                    safeZ = mapH - (craneH / 2.0);
                }
            }

            Object.values({ ...props.map.static_obstacles, ...props.map.dynamic_obstacles } || {}).forEach(o => {
                const p = toScreen(o.x_m, o.y_m);

                // 3D 判断: 障碍物高度 < (参考安全高度 - 安全余量) 则忽略(半透明)
                // 这里的逻辑是: 如果障碍物顶部加上安全余量后，仍然比起重机中心点低，那么就是安全的，可以飞越
                const obsHeight = o.z_m || 100;

                // 注意：如果无限高模式开启，isIgnored 恒为 false (实心)
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
                    // 如果是被忽略的障碍物，文字也稍微淡一点
                    cx.fillStyle = isIgnored ? (props.isDark ? '#475569' : '#94a3b8') : (props.isDark ? '#94a3b8' : '#475569');
                    cx.fillText(`H:${hText}`, p.x + 5, p.y + 16);
                }
            });

            // [辅助] 绘制足迹 (支持 Circle/Box)
            const drawFootprint = (pt, color, isFill = false) => {
                if (!pt) return;
                const p = toScreen(pt.x, pt.y);
                const shape = props.settings.CRANE_FOOTPRINT_SHAPE;
                const width = props.settings.CRANE_FOOTPRINT_WIDTH || 5.0;
                const length = props.settings.CRANE_FOOTPRINT_LENGTH || 5.0;

                cx.fillStyle = color;
                cx.strokeStyle = color;
                cx.lineWidth = 1;

                if (shape === 'circle') {
                    const radius = (width / 2) * view.scale;
                    cx.beginPath();
                    cx.arc(p.x, p.y, radius, 0, Math.PI * 2);
                    if (isFill) { cx.globalAlpha = 0.15; cx.fill(); cx.globalAlpha = 1.0; }
                    cx.stroke();
                } else {
                    const w = width * view.scale;
                    const h = length * view.scale;
                    if (isFill) { cx.globalAlpha = 0.15; cx.fillRect(p.x - w / 2, p.y - h / 2, w, h); cx.globalAlpha = 1.0; }
                    cx.strokeRect(p.x - w / 2, p.y - h / 2, w, h);
                }
            };

            // 5. 路径安全范围渲染
            if (props.visuals.showPathSafety && props.path && props.path.length > 0) {
                const step = Math.max(1, Math.floor(props.path.length / 40));
                for (let i = 0; i < props.path.length; i += step) {
                    drawFootprint({ x: props.path[i][0], y: props.path[i][1] }, 'rgba(59, 130, 246, 0.4)', true);
                }
                // 补齐终点
                drawFootprint({ x: props.path[props.path.length - 1][0], y: props.path[props.path.length - 1][1] }, 'rgba(59, 130, 246, 0.4)', true);
            }

            // 6. 路径线与节点
            if (props.path && props.path.length > 1) {
                cx.beginPath();
                const start = toScreen(props.path[0][0], props.path[0][1]);
                cx.moveTo(start.x, start.y);
                for (let i = 1; i < props.path.length; i++) {
                    const pt = toScreen(props.path[i][0], props.path[i][1]);
                    cx.lineTo(pt.x, pt.y);
                }
                cx.strokeStyle = '#3b82f6';
                cx.lineWidth = 3;
                cx.lineJoin = 'round';
                cx.stroke();

                // 节点开关
                if (props.visuals.showPathNodes) {
                    cx.fillStyle = '#3b82f6';
                    for (let i = 0; i < props.path.length; i++) {
                        const pt = toScreen(props.path[i][0], props.path[i][1]);
                        cx.beginPath(); cx.arc(pt.x, pt.y, 3, 0, Math.PI * 2); cx.fill();
                    }
                }
            }

            // 7. 起终点标记 (带足迹预览)
            if (props.mission.start) {
                drawFootprint(props.mission.start, '#10b981');
                const p = toScreen(props.mission.start.x, props.mission.start.y);
                cx.beginPath(); cx.arc(p.x, p.y, 7, 0, Math.PI * 2); cx.fillStyle = '#10b981'; cx.fill(); cx.strokeStyle = '#fff'; cx.lineWidth = 2.5; cx.stroke();
                cx.fillStyle = props.isDark ? '#fff' : '#0f172a'; cx.font = 'bold 14px "Inter", sans-serif'; cx.strokeStyle = props.isDark ? '#0f172a' : '#fff'; cx.lineWidth = 3;
                cx.strokeText('起点', p.x + 12, p.y + 4); cx.fillText('起点', p.x + 12, p.y + 4);
            }
            if (props.mission.end) {
                drawFootprint(props.mission.end, '#f43f5e');
                const p = toScreen(props.mission.end.x, props.mission.end.y);
                cx.beginPath(); cx.arc(p.x, p.y, 7, 0, Math.PI * 2); cx.fillStyle = '#f43f5e'; cx.fill(); cx.strokeStyle = '#fff'; cx.lineWidth = 2.5; cx.stroke();
                cx.fillStyle = props.isDark ? '#fff' : '#0f172a'; cx.font = 'bold 14px "Inter", sans-serif'; cx.strokeStyle = props.isDark ? '#0f172a' : '#fff'; cx.lineWidth = 3;
                cx.strokeText('终点', p.x + 12, p.y + 4); cx.fillText('终点', p.x + 12, p.y + 4);
            }

            // 8. 拖拽预览
            if (dragRect.value) {
                const p = toScreen(dragRect.value.x, dragRect.value.y);
                cx.fillStyle = 'rgba(59, 130, 246, 0.3)';
                cx.fillRect(p.x, p.y, dragRect.value.w * view.scale, dragRect.value.h * view.scale);
                cx.strokeStyle = '#3b82f6'; cx.strokeRect(p.x, p.y, dragRect.value.w * view.scale, dragRect.value.h * view.scale);
            }
        };

        const handleMouseDown = (e) => {
            isInteracting.value = true;
            const rect = canvas.value.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            if (e.button === 1 || e.button === 2 || props.interaction.mode === 'view') {
                isPanning.value = true; lastMouse.x = e.clientX; lastMouse.y = e.clientY; return;
            }
            if (e.button === 0) emit('map-click', { ...toWorld(mouseX, mouseY), type: 'down' });
        };

        const handleMouseMove = (e) => {
            if (!isInteracting.value && !isPanning.value) return;
            const rect = canvas.value.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            if (isPanning.value) {
                view.offsetX += e.clientX - lastMouse.x; view.offsetY += e.clientY - lastMouse.y; lastMouse.x = e.clientX; lastMouse.y = e.clientY; render();
            } else {
                emit('map-drag', { ...toWorld(mouseX, mouseY), setPreview: (r) => { dragRect.value = r; render(); } });
            }
        };

        const handleMouseUp = (e) => {
            if (!isInteracting.value && !isPanning.value) return;
            isInteracting.value = false;
            if (isPanning.value) { isPanning.value = false; return; }
            dragRect.value = null;
            const rect = canvas.value.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            emit('map-click', { ...toWorld(mouseX, mouseY), type: 'up' });
            render();
        };

        const handleWheel = (e) => {
            e.preventDefault();
            const rect = canvas.value.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            const worldX = (mouseX - view.offsetX) / view.scale;
            const worldY = (mouseY - view.offsetY) / view.scale;
            const zoomFactor = Math.exp(e.deltaY * -0.001);
            const newScale = Math.max(2, Math.min(200, view.scale * zoomFactor));
            view.offsetX = mouseX - worldX * newScale; view.offsetY = mouseY - worldY * newScale; view.scale = newScale; render();
        };

        onMounted(() => {
            window.addEventListener('resize', render);
            render();
            canvas.value.addEventListener('mousedown', handleMouseDown);
            canvas.value.addEventListener('contextmenu', e => e.preventDefault());
            window.addEventListener('mousemove', handleMouseMove);
            window.addEventListener('mouseup', handleMouseUp);
            canvas.value.addEventListener('wheel', handleWheel, { passive: false });

            setTimeout(() => {
                if (props.map.width) {
                    const cw = container.value.clientWidth;
                    const ch = container.value.clientHeight;
                    const scaleX = (cw - 100) / props.map.width;
                    const scaleY = (ch - 100) / props.map.length;
                    view.scale = Math.min(scaleX, scaleY);
                    view.offsetX = (cw - props.map.width * view.scale) / 2;
                    view.offsetY = (ch - props.map.length * view.scale) / 2;
                    render();
                }
            }, 500);
        });

        watch(() => [props.map, props.path, props.mission, props.isDark, props.settings, props.visuals], () => {
            requestAnimationFrame(render);
        }, { deep: true });

        watch(() => props.active, (val) => {
            if (val) setTimeout(render, 50);
        });

        return { container, canvas };
    }
};
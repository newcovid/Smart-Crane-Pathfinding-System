import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const { ref, reactive, watch, onMounted, onBeforeUnmount, nextTick } = Vue;

// [核心优化] 3D引擎全局缓存池
const engineCache = {
    renderer: null,
    scene: null,
    camera: null,
    controls: null,
    raycaster: null,
    mouse: null,
    envObjects: { plane: null, grid: null, lights: [] },
    contentMeshes: { obstacles: [], pathLines: [], markers: [], inflationVoxels: null },
    interactionList: [],
    isInitialized: false
};

export default {
    props: ['map', 'path', 'mission', 'active', 'visuals', 'isDark'],
    template: `
    <div class="w-full h-full relative overflow-hidden transition-colors duration-500 select-none" 
         :class="isDark ? 'bg-[#0b1120]' : 'bg-slate-200'"
         @mousemove="onMouseMove">
        
        <!-- 挂载点 -->
        <div ref="mountPoint" class="absolute inset-0 w-full h-full z-0 cursor-pointer"></div>

        <!-- 3D 交互信息浮窗 (Tooltip) -->
        <div v-show="tooltip.visible" 
             class="absolute z-20 pointer-events-none px-3 py-2 bg-white/90 dark:bg-slate-800/90 backdrop-blur shadow-xl rounded-lg border border-slate-200 dark:border-slate-700 text-xs transition-opacity duration-150"
             :style="{ left: tooltip.x + 15 + 'px', top: tooltip.y + 15 + 'px' }">
            <div class="font-bold mb-1 flex items-center gap-1.5" :class="tooltip.headerClass">
                <i :class="tooltip.icon"></i>
                <span>{{ tooltip.title }}</span>
            </div>
            <div class="space-y-0.5 font-mono text-slate-600 dark:text-slate-400 opacity-90">
                <div v-for="(line, idx) in tooltip.lines" :key="idx">{{ line }}</div>
            </div>
        </div>

        <!-- Loading -->
        <transition name="fade">
            <div v-if="!isReady && !initError" class="absolute inset-0 flex items-center justify-center text-slate-500 z-10 transition-colors duration-500" :class="isDark ? 'bg-slate-900/80' : 'bg-slate-100/80'">
                <div class="flex flex-col items-center justify-center gap-2">
                    <i class="ph-bold ph-spinner animate-spin text-2xl"></i>
                    <span>3D 引擎启动中...</span>
                </div>
            </div>
        </transition>
        
        <!-- Error -->
        <div v-if="initError" class="absolute inset-0 flex items-center justify-center text-red-500 z-20 bg-slate-900/90">
            <div class="flex flex-col items-center justify-center gap-2 p-4 text-center">
                <i class="ph-bold ph-warning-circle text-3xl"></i>
                <span class="font-bold">渲染引擎启动失败</span>
                <span class="text-xs opacity-70 font-mono max-w-md break-words">{{ initError }}</span>
                <button @click="forceReInit" class="mt-2 px-3 py-1 bg-white/10 hover:bg-white/20 rounded text-xs text-white transition-colors">重置引擎</button>
            </div>
        </div>
        
        <!-- HUD -->
        <div class="absolute top-4 left-4 z-10 pointer-events-none">
            <div class="backdrop-blur-md text-xs px-3 py-1.5 rounded-lg border shadow-lg transition-all duration-300"
                 :class="isDark ? 'bg-slate-800/80 text-slate-300 border-slate-700' : 'bg-white/80 text-slate-600 border-white/50'">
                <span class="text-primary font-bold">3D 视图</span> | 左键旋转 · 右键平移 · 指向物体查看信息
            </div>
        </div>
    </div>
    `,
    setup(props) {
        const mountPoint = ref(null);
        const isReady = ref(engineCache.isInitialized);
        const initError = ref(null);

        const tooltip = reactive({
            visible: false,
            x: 0,
            y: 0,
            title: '',
            icon: '',
            headerClass: '',
            lines: []
        });

        let hoveredObj = null;
        let animationId = null;
        let resizeObserver = null;
        let isInitializing = false;

        // --- 交互逻辑 ---
        const onMouseMove = (event) => {
            if (!isReady.value || !engineCache.camera || !mountPoint.value) return;

            const rect = mountPoint.value.getBoundingClientRect();
            const mouseX = event.clientX - rect.left;
            const mouseY = event.clientY - rect.top;

            tooltip.x = mouseX;
            tooltip.y = mouseY;

            if (!engineCache.mouse) engineCache.mouse = new THREE.Vector2();
            engineCache.mouse.x = (mouseX / rect.width) * 2 - 1;
            engineCache.mouse.y = -(mouseY / rect.height) * 2 + 1;

            performRaycasting();
        };

        const performRaycasting = () => {
            if (!engineCache.raycaster || !engineCache.camera || engineCache.interactionList.length === 0) return;

            engineCache.raycaster.setFromCamera(engineCache.mouse, engineCache.camera);
            const intersects = engineCache.raycaster.intersectObjects(engineCache.interactionList, false);

            if (intersects.length > 0) {
                const firstHit = intersects[0].object;
                if (hoveredObj !== firstHit) {
                    restoreHoveredObject();
                    hoveredObj = firstHit;
                    highlightObject(hoveredObj);
                    updateTooltip(hoveredObj.userData);
                }
                tooltip.visible = true;
                if (mountPoint.value) mountPoint.value.style.cursor = 'pointer';
            } else {
                if (hoveredObj) {
                    restoreHoveredObject();
                    hoveredObj = null;
                    tooltip.visible = false;
                    if (mountPoint.value) mountPoint.value.style.cursor = 'default';
                }
            }
        };

        const highlightObject = (mesh) => {
            if (!mesh.material) return;
            if (!mesh.userData.originalHex) {
                mesh.userData.originalHex = mesh.material.color.getHex();
                mesh.userData.originalEmissive = mesh.material.emissive ? mesh.material.emissive.getHex() : 0x000000;
            }
            if (mesh.material.emissive) {
                mesh.material.emissive.setHex(0x666666);
            } else {
                mesh.material.color.offsetHSL(0, 0, 0.2);
            }
        };

        const restoreHoveredObject = () => {
            if (!hoveredObj || !hoveredObj.material) return;
            if (hoveredObj.userData.originalHex !== undefined) {
                hoveredObj.material.color.setHex(hoveredObj.userData.originalHex);
            }
            if (hoveredObj.userData.originalEmissive !== undefined && hoveredObj.material.emissive) {
                hoveredObj.material.emissive.setHex(hoveredObj.userData.originalEmissive);
            }
        };

        const updateTooltip = (data) => {
            if (!data) return;
            if (data.type === 'obstacle') {
                const isDynamic = data.info.type === 'dynamic';
                tooltip.title = isDynamic ? '动态障碍物' : '静态障碍物';
                tooltip.icon = isDynamic ? 'ph-bold ph-truck' : 'ph-bold ph-cube';
                tooltip.headerClass = isDynamic ? 'text-red-500' : (props.isDark ? 'text-slate-300' : 'text-slate-700');

                const { w_m, h_m, z_m, x_m, y_m } = data.info;
                const hText = z_m ? `${z_m}m` : (props.map.infinite_height ? '∞' : '-');
                tooltip.lines = [
                    `位置: (${x_m.toFixed(1)}, ${y_m.toFixed(1)})`,
                    `尺寸: ${w_m}m × ${h_m}m`,
                    `高度: ${hText}`
                ];
            } else if (data.type === 'node') {
                const isStart = data.subtype === 'start';
                const isEnd = data.subtype === 'end';
                tooltip.title = isStart ? '起始点 (Start)' : (isEnd ? '终止点 (End)' : '路径节点');
                tooltip.icon = isStart ? 'ph-bold ph-flag-checkered' : (isEnd ? 'ph-bold ph-flag' : 'ph-bold ph-circle');
                tooltip.headerClass = isStart ? 'text-green-500' : (isEnd ? 'text-rose-500' : 'text-blue-500');
                tooltip.lines = [
                    `X: ${data.info.x.toFixed(2)} m`,
                    `Y: ${data.info.y.toFixed(2)} m`,
                    `Z: ${data.info.z.toFixed(2)} m`
                ];
            }
        };

        // --- 引擎逻辑 ---

        const initEngine = async () => {
            if (isInitializing) return;
            const isCanvasAttached = engineCache.renderer && mountPoint.value && mountPoint.value.contains(engineCache.renderer.domElement);
            if (isReady.value && isCanvasAttached) {
                startAnimationLoop();
                return;
            }
            isInitializing = true;
            initError.value = null;

            try {
                await nextTick();
                if (!mountPoint.value) { isInitializing = false; return; }

                const w = mountPoint.value.clientWidth;
                const h = mountPoint.value.clientHeight;

                if (!engineCache.isInitialized || !engineCache.renderer) {
                    if (w === 0) { isInitializing = false; return; }
                    const scene = new THREE.Scene();
                    engineCache.scene = scene;

                    const camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 5000);
                    camera.up.set(0, 0, 1);
                    engineCache.camera = camera;

                    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
                    renderer.setSize(w, h);
                    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
                    renderer.shadowMap.enabled = true;
                    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
                    engineCache.renderer = renderer;

                    engineCache.raycaster = new THREE.Raycaster();
                    engineCache.raycaster.params.Line.threshold = 0.5;
                    engineCache.mouse = new THREE.Vector2();

                    const controls = new OrbitControls(camera, renderer.domElement);
                    controls.enableDamping = true;
                    controls.dampingFactor = 0.08;
                    controls.maxPolarAngle = Math.PI / 2 - 0.05;
                    controls.minDistance = 1;
                    controls.maxDistance = 500;
                    engineCache.controls = controls;

                    engineCache.isInitialized = true;
                }

                if (mountPoint.value && !mountPoint.value.contains(engineCache.renderer.domElement)) {
                    mountPoint.value.appendChild(engineCache.renderer.domElement);
                }

                if (w > 0 && h > 0) {
                    engineCache.camera.aspect = w / h;
                    engineCache.camera.updateProjectionMatrix();
                    engineCache.renderer.setSize(w, h);
                }

                isReady.value = true;
                if (props.map && props.map.width) {
                    updateEnvironment(true);
                    updateSceneContent();
                    applyThemeColors();
                }
                startAnimationLoop();
            } catch (e) {
                console.error('[Canvas3d] Init Error:', e);
                initError.value = e.message;
                engineCache.isInitialized = false;
            } finally {
                isInitializing = false;
            }
        };

        const forceReInit = () => {
            if (engineCache.renderer) {
                try {
                    engineCache.renderer.dispose();
                    engineCache.renderer.forceContextLoss();
                    const gl = engineCache.renderer.domElement.getContext('webgl');
                    if (gl) gl.getExtension('WEBGL_lose_context').loseContext();
                } catch (e) { }
            }
            engineCache.isInitialized = false;
            engineCache.renderer = null;
            engineCache.scene = null;
            engineCache.interactionList = [];
            isReady.value = false;
            initEngine();
        };

        const startAnimationLoop = () => {
            if (animationId) return;
            const loop = () => {
                animationId = requestAnimationFrame(loop);
                if (engineCache.isInitialized && engineCache.renderer && engineCache.scene && engineCache.camera) {
                    if (engineCache.controls) engineCache.controls.update();
                    engineCache.renderer.render(engineCache.scene, engineCache.camera);
                }
            };
            loop();
        };

        const stopAnimationLoop = () => {
            if (animationId) { cancelAnimationFrame(animationId); animationId = null; }
        };

        const updateEnvironment = (resetCamera = false) => {
            const { scene, envObjects, controls, camera } = engineCache;
            if (!scene || !props.map.width) return;

            const mapW = props.map.width;
            const mapL = props.map.length;

            if (envObjects.plane) { scene.remove(envObjects.plane); envObjects.plane.geometry.dispose(); }
            if (envObjects.grid) { scene.remove(envObjects.grid); envObjects.grid.geometry.dispose(); }
            envObjects.lights.forEach(l => scene.remove(l));
            envObjects.lights = [];

            const amb = new THREE.AmbientLight(0xffffff, 0.5);
            scene.add(amb); envObjects.lights.push(amb);

            const dir = new THREE.DirectionalLight(0xffffff, 1.2);
            const center = Math.max(mapW, mapL) / 2;
            dir.position.set(-center, -center, center * 1.5);
            dir.castShadow = true;
            dir.shadow.mapSize.width = 2048; dir.shadow.mapSize.height = 2048;
            const d = Math.max(mapW, mapL) * 1.5;
            dir.shadow.camera.left = -d; dir.shadow.camera.right = d; dir.shadow.camera.top = d; dir.shadow.camera.bottom = -d;
            scene.add(dir); envObjects.lights.push(dir);

            const size = Math.max(mapW, mapL) * 4;
            const planeGeo = new THREE.PlaneGeometry(size, size);
            const planeMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.8, metalness: 0.2 });
            const plane = new THREE.Mesh(planeGeo, planeMat);
            plane.position.set(mapW / 2, mapL / 2, -0.05);
            plane.receiveShadow = true;
            scene.add(plane); envObjects.plane = plane;

            const grid = new THREE.GridHelper(size, Math.floor(size / 5), 0x475569, 0x1e293b);
            grid.rotation.x = Math.PI / 2;
            grid.position.set(mapW / 2, mapL / 2, 0);
            scene.add(grid); envObjects.grid = grid;

            if (resetCamera && camera && controls) {
                const maxDim = Math.max(mapW, mapL);
                camera.position.set(mapW * 0.5, mapL * -0.5, maxDim * 1.2);
                controls.target.set(mapW / 2, mapL / 2, 0);
                controls.update();
            }
            applyThemeColors();
        };

        const updateSceneContent = () => {
            const { scene, contentMeshes } = engineCache;
            if (!scene) return;

            [...contentMeshes.obstacles, ...contentMeshes.pathLines, ...contentMeshes.markers].forEach(obj => {
                scene.remove(obj);
                if (obj.geometry) obj.geometry.dispose();
                if (obj.material) {
                    if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose());
                    else obj.material.dispose();
                }
            });
            contentMeshes.obstacles = []; contentMeshes.pathLines = []; contentMeshes.markers = [];
            engineCache.interactionList = [];

            const obsColorStatic = props.isDark ? 0x94a3b8 : 0x475569;
            const obsColorDynamic = 0xf43f5e;
            const boxGeo = new THREE.BoxGeometry(1, 1, 1);

            const drawObs = (list, type) => {
                Object.values(list || {}).forEach(o => {
                    if (!o || isNaN(o.x_m)) return;
                    const mat = new THREE.MeshStandardMaterial({
                        color: type === 'dynamic' ? obsColorDynamic : obsColorStatic,
                        roughness: 0.4
                    });
                    const mesh = new THREE.Mesh(boxGeo, mat);
                    const h = o.z_m || 2.0;
                    mesh.scale.set(o.w_m, o.h_m, h);
                    mesh.position.set(o.x_m + o.w_m / 2, o.y_m + o.h_m / 2, h / 2);
                    mesh.castShadow = true; mesh.receiveShadow = true;
                    mesh.userData = { type: 'obstacle', info: { ...o, type } };
                    scene.add(mesh);
                    contentMeshes.obstacles.push(mesh);
                    engineCache.interactionList.push(mesh);
                });
            };
            drawObs(props.map.static_obstacles, 'static');
            drawObs(props.map.dynamic_obstacles, 'dynamic');

            if (props.path && props.path.length > 1) {
                const points = props.path.map(p => new THREE.Vector3(p[0], p[1], (p[2] || 0.5)));
                const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: props.isDark ? 0x60a5fa : 0x2563eb, linewidth: 3 }));
                scene.add(line); contentMeshes.pathLines.push(line);

                const sM = new THREE.Mesh(new THREE.SphereGeometry(0.5), new THREE.MeshStandardMaterial({ color: 0x10b981, emissive: 0x064e3b, emissiveIntensity: 0.5 }));
                sM.position.copy(points[0]);
                sM.userData = { type: 'node', subtype: 'start', info: { x: points[0].x, y: points[0].y, z: points[0].z } };
                scene.add(sM);
                contentMeshes.markers.push(sM);
                engineCache.interactionList.push(sM);

                const eM = new THREE.Mesh(new THREE.SphereGeometry(0.5), new THREE.MeshStandardMaterial({ color: 0xf43f5e, emissive: 0x881337, emissiveIntensity: 0.5 }));
                eM.position.copy(points[points.length - 1]);
                eM.userData = { type: 'node', subtype: 'end', info: { x: points[points.length - 1].x, y: points[points.length - 1].y, z: points[points.length - 1].z } };
                scene.add(eM);
                contentMeshes.markers.push(eM);
                engineCache.interactionList.push(eM);

                const step = Math.max(1, Math.floor(points.length / 20));
                for (let i = step; i < points.length - 1; i += step) {
                    const pt = points[i];
                    const nodeM = new THREE.Mesh(new THREE.SphereGeometry(0.3), new THREE.MeshStandardMaterial({ color: 0x3b82f6 }));
                    nodeM.position.copy(pt);
                    nodeM.userData = { type: 'node', subtype: 'waypoint', info: { x: pt.x, y: pt.y, z: pt.z } };
                    scene.add(nodeM);
                    contentMeshes.markers.push(nodeM);
                    engineCache.interactionList.push(nodeM);
                }
            }
        };

        const applyThemeColors = () => {
            // [Fix] 这里的解构赋值中添加了 contentMeshes
            const { scene, envObjects, contentMeshes } = engineCache;
            if (!scene) return;
            const t = props.isDark ? { bg: 0x0b1120, fog: 0x0b1120, plane: 0x1e293b, grid: 0x475569, gridCenter: 0x64748b, ambient: 0.5 } : { bg: 0xe2e8f0, fog: 0xe2e8f0, plane: 0xf1f5f9, grid: 0xcbd5e1, gridCenter: 0x94a3b8, ambient: 0.7 };
            scene.background = new THREE.Color(t.bg);
            scene.fog = new THREE.FogExp2(t.fog, 0.002);
            if (envObjects.plane) envObjects.plane.material.color.setHex(t.plane);
            if (envObjects.grid) {
                scene.remove(envObjects.grid); envObjects.grid.geometry.dispose();
                const size = Math.max(props.map.width, props.map.length) * 4;
                const grid = new THREE.GridHelper(size, Math.floor(size / 5), t.gridCenter, t.grid);
                grid.rotation.x = Math.PI / 2; grid.position.set(props.map.width / 2, props.map.length / 2, 0);
                scene.add(grid); envObjects.grid = grid;
            }
            if (envObjects.lights[0]) envObjects.lights[0].intensity = t.ambient;

            const obsColorStatic = props.isDark ? 0x94a3b8 : 0x475569;
            // [Fix] 现在可以正确访问 contentMeshes.obstacles
            if (contentMeshes && contentMeshes.obstacles) {
                contentMeshes.obstacles.forEach(mesh => {
                    if (mesh.userData.type === 'obstacle' && mesh.userData.info.type !== 'dynamic') {
                        mesh.material.color.setHex(obsColorStatic);
                        delete mesh.userData.originalHex;
                    }
                });
            }
        };

        onMounted(() => {
            resizeObserver = new ResizeObserver(() => {
                if (engineCache.isInitialized && engineCache.renderer && mountPoint.value && mountPoint.value.clientWidth > 0) {
                    const w = mountPoint.value.clientWidth; const h = mountPoint.value.clientHeight;
                    engineCache.camera.aspect = w / h; engineCache.camera.updateProjectionMatrix();
                    engineCache.renderer.setSize(w, h);
                }
            });
            if (mountPoint.value) resizeObserver.observe(mountPoint.value);

            if (props.active) nextTick(initEngine);
        });

        onBeforeUnmount(() => {
            stopAnimationLoop();
            if (resizeObserver) resizeObserver.disconnect();
            tooltip.visible = false;
        });

        watch(() => props.active, (val) => {
            if (val) nextTick(initEngine);
            else stopAnimationLoop();
        });

        watch(() => [props.map, props.path, props.mission, props.visuals, props.isDark], () => {
            if (engineCache.isInitialized) {
                if (props.map.width !== (engineCache.lastMapW || 0)) updateEnvironment(false);
                updateSceneContent();
                applyThemeColors();
            }
        }, { deep: true });

        return { mountPoint, isReady, initError, tooltip, onMouseMove, forceReInit };
    }
};
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const { ref, watch, onMounted, onBeforeUnmount, nextTick } = Vue;

// [核心优化] 3D引擎全局缓存池
const engineCache = {
    renderer: null,
    scene: null,
    camera: null,
    controls: null,
    envObjects: { plane: null, grid: null, lights: [] },
    contentMeshes: { obstacles: [], pathLines: [], markers: [], inflationVoxels: null },
    isInitialized: false // 标记 WebGL 是否已创建
};

export default {
    props: ['map', 'path', 'mission', 'active', 'visuals', 'isDark'],
    template: `
    <div class="w-full h-full relative overflow-hidden transition-colors duration-500" :class="isDark ? 'bg-[#0b1120]' : 'bg-slate-200'">
        
        <!-- 挂载点 -->
        <div ref="mountPoint" class="absolute inset-0 w-full h-full z-0"></div>

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
                <span class="text-primary font-bold">3D 视图</span> | 左键旋转 · 右键平移 · 滚轮缩放
            </div>
        </div>
    </div>
    `,
    setup(props) {
        const mountPoint = ref(null);
        // 如果缓存已初始化，这里初始就是 true，但 DOM 可能还没挂载，所以不能仅靠这个判断
        const isReady = ref(engineCache.isInitialized);
        const initError = ref(null);

        let animationId = null;
        let resizeObserver = null;
        let isInitializing = false;

        // --- 1. 引擎初始化 (支持缓存复用 & 强制重挂载) ---
        const initEngine = async () => {
            if (isInitializing) return;

            // [Bug Fix] 即使 isReady 为 true，也要检查 Canvas 是否真的在当前 mountPoint 里
            const isCanvasAttached = engineCache.renderer && mountPoint.value && mountPoint.value.contains(engineCache.renderer.domElement);

            if (isReady.value && isCanvasAttached) {
                startAnimationLoop();
                return;
            }

            isInitializing = true;
            initError.value = null;

            try {
                await nextTick();
                if (!mountPoint.value) {
                    isInitializing = false;
                    return;
                }

                const w = mountPoint.value.clientWidth;
                const h = mountPoint.value.clientHeight;

                // [关键路径 A] 缓存存在 -> 执行“重挂载 (Re-attach)”
                if (engineCache.isInitialized && engineCache.renderer) {
                    console.log("[Canvas3d] Restoring engine from cache...");

                    // 1. 确保 Canvas 挂载到当前组件 DOM
                    if (!mountPoint.value.contains(engineCache.renderer.domElement)) {
                        mountPoint.value.appendChild(engineCache.renderer.domElement);
                    }

                    // 2. 修正尺寸 (切换视图期间容器尺寸可能变化)
                    if (w > 0 && h > 0) {
                        engineCache.camera.aspect = w / h;
                        engineCache.camera.updateProjectionMatrix();
                        engineCache.renderer.setSize(w, h);
                    }

                    // 3. 状态恢复
                    isReady.value = true;
                    startAnimationLoop();

                    // 4. 确保环境与最新 props 同步
                    updateEnvironment(false);
                    updateSceneContent();
                    applyThemeColors();
                }

                // [关键路径 B] 冷启动 -> 执行“新建 (Create New)”
                else {
                    if (w === 0) { isInitializing = false; return; } // 防止在隐藏状态下初始化

                    console.log("[Canvas3d] Creating new WebGL engine...");

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

                    mountPoint.value.appendChild(renderer.domElement);

                    const controls = new OrbitControls(camera, renderer.domElement);
                    controls.enableDamping = true;
                    controls.dampingFactor = 0.08;
                    controls.maxPolarAngle = Math.PI / 2 - 0.05;
                    controls.minDistance = 1;
                    controls.maxDistance = 500;
                    engineCache.controls = controls;

                    engineCache.isInitialized = true;
                    isReady.value = true;

                    // 初始内容加载
                    if (props.map && props.map.width) {
                        updateEnvironment(true);
                        updateSceneContent();
                        applyThemeColors();
                    }

                    startAnimationLoop();
                }

            } catch (e) {
                console.error('[Canvas3d] Init Error:', e);
                initError.value = e.message || "WebGL Error";
                // 如果出错，标记缓存失效，允许下次重试
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
            if (animationId) {
                cancelAnimationFrame(animationId);
                animationId = null;
            }
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
            });
            if (contentMeshes.inflationVoxels) {
                scene.remove(contentMeshes.inflationVoxels);
                contentMeshes.inflationVoxels.dispose();
                contentMeshes.inflationVoxels = null;
            }
            contentMeshes.obstacles = []; contentMeshes.pathLines = []; contentMeshes.markers = [];

            const obsColorStatic = props.isDark ? 0x94a3b8 : 0x475569;
            const obsColorDynamic = 0xf43f5e;
            const boxGeo = new THREE.BoxGeometry(1, 1, 1);
            const staticMat = new THREE.MeshStandardMaterial({ color: obsColorStatic, roughness: 0.4 });
            const dynamicMat = new THREE.MeshStandardMaterial({ color: obsColorDynamic, roughness: 0.4 });

            const drawObs = (list, mat) => {
                Object.values(list || {}).forEach(o => {
                    if (!o || isNaN(o.x_m)) return;
                    const mesh = new THREE.Mesh(boxGeo, mat);
                    const h = o.z_m || 2.0;
                    mesh.scale.set(o.w_m, o.h_m, h);
                    mesh.position.set(o.x_m + o.w_m / 2, o.y_m + o.h_m / 2, h / 2);
                    mesh.castShadow = true; mesh.receiveShadow = true;
                    scene.add(mesh); contentMeshes.obstacles.push(mesh);
                });
            };
            drawObs(props.map.static_obstacles, staticMat);
            drawObs(props.map.dynamic_obstacles, dynamicMat);

            if (props.path && props.path.length > 1) {
                const points = props.path.map(p => new THREE.Vector3(p[0], p[1], (p[2] || 0.5)));
                const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: props.isDark ? 0x60a5fa : 0x2563eb, linewidth: 3 }));
                scene.add(line); contentMeshes.pathLines.push(line);

                const sM = new THREE.Mesh(new THREE.SphereGeometry(0.5), new THREE.MeshBasicMaterial({ color: 0x10b981 })); sM.position.copy(points[0]); scene.add(sM); contentMeshes.markers.push(sM);
                const eM = new THREE.Mesh(new THREE.SphereGeometry(0.5), new THREE.MeshBasicMaterial({ color: 0xf43f5e })); eM.position.copy(points[points.length - 1]); scene.add(eM); contentMeshes.markers.push(eM);
            }

            if (props.visuals.safetyMode === 'inflation' && props.map.inflated_grid) {
                const grid = props.map.inflated_grid; const res = props.map.resolution || 0.5;
                const instances = [];
                grid.forEach((row, r) => row.forEach((val, c) => {
                    if (val === 1) instances.push(c * res + res / 2, r * res + res / 2, 0.05);
                }));
                if (instances.length > 0) {
                    const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(res * 0.95, res * 0.95, 0.05), new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.2, depthWrite: false }), instances.length / 3);
                    const dummy = new THREE.Object3D();
                    for (let i = 0; i < instances.length / 3; i++) {
                        dummy.position.set(instances[i * 3], instances[i * 3 + 1], instances[i * 3 + 2]); dummy.updateMatrix(); mesh.setMatrixAt(i, dummy.matrix);
                    }
                    scene.add(mesh); contentMeshes.inflationVoxels = mesh;
                }
            }
        };

        const applyThemeColors = () => {
            const { scene, envObjects } = engineCache;
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

            // 如果组件激活，立即初始化（如果是缓存恢复，initEngine 会处理 DOM 挂载）
            if (props.active) nextTick(initEngine);
        });

        onBeforeUnmount(() => {
            stopAnimationLoop();
            if (resizeObserver) resizeObserver.disconnect();
            // 注意：不销毁 renderer，保留在 cache 中
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

        return { mountPoint, isReady, initError, forceReInit };
    }
};
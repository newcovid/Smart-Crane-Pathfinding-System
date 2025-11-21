import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const { ref, watch, onMounted, onBeforeUnmount, nextTick } = Vue;

export default {
    props: ['map', 'path', 'mission', 'active', 'visuals', 'isDark'],
    template: `
    <div ref="container" class="w-full h-full relative overflow-hidden transition-colors duration-500" :class="isDark ? 'bg-[#0b1120]' : 'bg-slate-200'">
        <!-- Loading Overlay -->
        <div v-if="!isReady && !initError" class="absolute inset-0 flex items-center justify-center text-slate-500 z-10 transition-colors duration-500" :class="isDark ? 'bg-slate-900/80' : 'bg-slate-100/80'">
            <div class="flex flex-col items-center justify-center gap-2">
                <i class="ph-bold ph-spinner animate-spin text-2xl"></i>
                <span>3D 引擎启动中...</span>
            </div>
        </div>
        
        <!-- Error Overlay -->
        <div v-if="initError" class="absolute inset-0 flex items-center justify-center text-red-500 z-10 bg-slate-900/90">
            <div class="flex flex-col items-center justify-center gap-2 p-4 text-center">
                <i class="ph-bold ph-warning-circle text-3xl"></i>
                <span class="font-bold">渲染引擎启动失败</span>
                <span class="text-xs opacity-70 font-mono max-w-md break-words">{{ initError }}</span>
                <button @click="retryInit" class="mt-2 px-3 py-1 bg-white/10 hover:bg-white/20 rounded text-xs text-white transition-colors">重试</button>
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
        const container = ref(null);
        const isReady = ref(false);
        const initError = ref(null);

        // Three.js 核心实例
        let scene = null;
        let camera = null;
        let renderer = null;
        let controls = null;
        let animationId = null;
        let resizeObserver = null;

        // 状态锁
        let isInitializing = false;
        let isUnmounted = false;

        // 场景对象引用
        let envObjects = { plane: null, grid: null, lights: [] };
        let contentMeshes = { obstacles: [], pathLines: [], markers: [], inflationVoxels: null };
        let lastMapDim = { w: 0, l: 0 };

        // --- 1. 引擎初始化 (Safe Guarded) ---
        const initEngine = async () => {
            if (isInitializing || isReady.value) return;
            isInitializing = true;
            initError.value = null;

            try {
                await nextTick();
                if (isUnmounted || !container.value) {
                    isInitializing = false;
                    return;
                }

                const w = container.value.clientWidth;
                const h = container.value.clientHeight;

                if (w === 0 || h === 0) {
                    console.warn('[Canvas3d] Container size is 0, deferring init.');
                    isInitializing = false;
                    return;
                }

                // Scene
                scene = new THREE.Scene();

                // Camera
                camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 5000);
                camera.up.set(0, 0, 1);

                // Renderer
                renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
                renderer.setSize(w, h);
                renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
                renderer.shadowMap.enabled = true;
                renderer.shadowMap.type = THREE.PCFSoftShadowMap;

                // 安全挂载 DOM
                container.value.innerHTML = '';
                container.value.appendChild(renderer.domElement);

                // Controls
                controls = new OrbitControls(camera, renderer.domElement);
                controls.enableDamping = true;
                controls.dampingFactor = 0.08;
                controls.maxPolarAngle = Math.PI / 2 - 0.05;
                controls.minDistance = 1;
                controls.maxDistance = 500;

                animate();

                // Initial Data Load
                if (props.map.width) {
                    updateEnvironment(true);
                    updateSceneContent();
                    applyThemeColors();
                }

                // [Fix] 无论数据加载是否完美，只要引擎起来了，就标记 Ready
                isReady.value = true;

            } catch (e) {
                console.error('[Canvas3d] Init Error:', e);
                initError.value = e.message || "Unknown WebGL Error";
            } finally {
                isInitializing = false;
            }
        };

        // --- 2. 环境构建 ---
        const updateEnvironment = (resetCamera = false) => {
            try {
                if (!scene || !props.map.width) return;

                const mapW = props.map.width;
                const mapL = props.map.length;

                if (!resetCamera && Math.abs(mapW - lastMapDim.w) < 0.1 && Math.abs(mapL - lastMapDim.l) < 0.1) {
                    return;
                }
                lastMapDim = { w: mapW, l: mapL };

                // Clean
                if (envObjects.plane) { scene.remove(envObjects.plane); envObjects.plane.geometry.dispose(); }
                if (envObjects.grid) { scene.remove(envObjects.grid); envObjects.grid.geometry.dispose(); }
                envObjects.lights.forEach(l => scene.remove(l));
                envObjects.lights = [];

                // Lights
                const amb = new THREE.AmbientLight(0xffffff, 0.5);
                scene.add(amb);
                envObjects.lights.push(amb);

                const dir = new THREE.DirectionalLight(0xffffff, 1.2);
                const center = Math.max(mapW, mapL) / 2;
                dir.position.set(-center, -center, center * 1.5);
                dir.castShadow = true;
                dir.shadow.mapSize.width = 2048;
                dir.shadow.mapSize.height = 2048;
                // 动态调整阴影范围
                const d = Math.max(mapW, mapL) * 1.5;
                dir.shadow.camera.left = -d;
                dir.shadow.camera.right = d;
                dir.shadow.camera.top = d;
                dir.shadow.camera.bottom = -d;
                scene.add(dir);
                envObjects.lights.push(dir);

                // Ground & Grid
                const size = Math.max(mapW, mapL) * 4;
                const planeGeo = new THREE.PlaneGeometry(size, size);
                const planeMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.8, metalness: 0.2 });
                const plane = new THREE.Mesh(planeGeo, planeMat);
                plane.position.set(mapW / 2, mapL / 2, -0.05);
                plane.receiveShadow = true;
                scene.add(plane);
                envObjects.plane = plane;

                const grid = new THREE.GridHelper(size, Math.floor(size / 5), 0x475569, 0x1e293b);
                grid.rotation.x = Math.PI / 2;
                grid.position.set(mapW / 2, mapL / 2, 0);
                scene.add(grid);
                envObjects.grid = grid;

                // Camera Reset
                if (resetCamera) {
                    const maxDim = Math.max(mapW, mapL);
                    camera.position.set(mapW * 0.5, mapL * -0.5, maxDim * 1.2);
                    controls.target.set(mapW / 2, mapL / 2, 0);
                    controls.update();
                } else {
                    controls.target.set(mapW / 2, mapL / 2, 0);
                    controls.update();
                }

                applyThemeColors();
            } catch (e) {
                console.error('[Canvas3d] Env Update Error:', e);
            }
        };

        // --- 3. 内容更新 (Protected) ---
        const updateSceneContent = () => {
            if (!scene) return;

            try {
                // Clean
                [...contentMeshes.obstacles, ...contentMeshes.pathLines, ...contentMeshes.markers].forEach(obj => {
                    scene.remove(obj);
                    if (obj.geometry) obj.geometry.dispose();
                    if (obj.material) {
                        if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose());
                        else obj.material.dispose();
                    }
                });
                if (contentMeshes.inflationVoxels) {
                    scene.remove(contentMeshes.inflationVoxels);
                    contentMeshes.inflationVoxels.dispose();
                    contentMeshes.inflationVoxels = null;
                }
                contentMeshes.obstacles = []; contentMeshes.pathLines = []; contentMeshes.markers = [];

                // A. Obstacles
                const obsColorStatic = props.isDark ? 0x94a3b8 : 0x475569;
                const obsColorDynamic = 0xf43f5e;
                const boxGeo = new THREE.BoxGeometry(1, 1, 1);
                const staticMat = new THREE.MeshStandardMaterial({ color: obsColorStatic, roughness: 0.4 });
                const dynamicMat = new THREE.MeshStandardMaterial({ color: obsColorDynamic, roughness: 0.4 });

                const drawObs = (list, mat) => {
                    Object.values(list || {}).forEach(o => {
                        // [Safety] 确保坐标是数字
                        if (isNaN(o.x_m) || isNaN(o.y_m) || isNaN(o.w_m) || isNaN(o.h_m)) return;
                        const mesh = new THREE.Mesh(boxGeo, mat);
                        const h = o.z_m || 2.0;
                        mesh.scale.set(o.w_m, o.h_m, h);
                        mesh.position.set(o.x_m + o.w_m / 2, o.y_m + o.h_m / 2, h / 2);
                        mesh.castShadow = true;
                        mesh.receiveShadow = true;
                        scene.add(mesh);
                        contentMeshes.obstacles.push(mesh);
                    });
                };
                drawObs(props.map.static_obstacles, staticMat);
                drawObs(props.map.dynamic_obstacles, dynamicMat);

                // B. Path
                if (props.path && Array.isArray(props.path) && props.path.length > 1) {
                    const points = [];
                    props.path.forEach(p => {
                        if (Array.isArray(p) && !isNaN(p[0]) && !isNaN(p[1])) {
                            points.push(new THREE.Vector3(p[0], p[1], (p[2] !== undefined && !isNaN(p[2])) ? p[2] : 0.5));
                        }
                    });

                    if (points.length > 1) {
                        const geometry = new THREE.BufferGeometry().setFromPoints(points);
                        const material = new THREE.LineBasicMaterial({
                            color: props.isDark ? 0x60a5fa : 0x2563eb,
                            linewidth: 3
                        });
                        const line = new THREE.Line(geometry, material);
                        scene.add(line);
                        contentMeshes.pathLines.push(line);

                        // Markers
                        const sM = new THREE.Mesh(new THREE.SphereGeometry(0.5), new THREE.MeshBasicMaterial({ color: 0x10b981 }));
                        sM.position.copy(points[0]); scene.add(sM); contentMeshes.markers.push(sM);

                        const eM = new THREE.Mesh(new THREE.SphereGeometry(0.5), new THREE.MeshBasicMaterial({ color: 0xf43f5e }));
                        eM.position.copy(points[points.length - 1]); scene.add(eM); contentMeshes.markers.push(eM);
                    }
                }

                // C. Inflation
                if (props.visuals && props.visuals.safetyMode === 'inflation' && props.map.inflated_grid) {
                    const grid = props.map.inflated_grid;
                    // [Safety] 确保 grid 是数组
                    if (Array.isArray(grid) && grid.length > 0) {
                        const res = props.map.resolution || 0.5;
                        const rows = grid.length;
                        const cols = Array.isArray(grid[0]) ? grid[0].length : 0;
                        const instances = [];

                        for (let r = 0; r < rows; r++) {
                            for (let c = 0; c < cols; c++) {
                                if (grid[r][c] === 1) {
                                    instances.push(c * res + res / 2, r * res + res / 2, 0.05);
                                }
                            }
                        }

                        if (instances.length > 0) {
                            const geo = new THREE.BoxGeometry(res * 0.95, res * 0.95, 0.05);
                            const mat = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.2, depthWrite: false });
                            const mesh = new THREE.InstancedMesh(geo, mat, Math.floor(instances.length / 3));
                            const dummy = new THREE.Object3D();

                            for (let i = 0; i < instances.length / 3; i++) {
                                dummy.position.set(instances[i * 3], instances[i * 3 + 1], instances[i * 3 + 2]);
                                dummy.updateMatrix();
                                mesh.setMatrixAt(i, dummy.matrix);
                            }
                            mesh.instanceMatrix.needsUpdate = true;
                            scene.add(mesh);
                            contentMeshes.inflationVoxels = mesh;
                        }
                    }
                }
            } catch (e) {
                console.error("[Canvas3d] Content Update Error:", e);
                // 不抛出错误，防止阻塞渲染循环
            }
        };

        // --- 4. 主题应用 ---
        const applyThemeColors = () => {
            if (!scene) return;
            const t = props.isDark ? {
                bg: 0x0b1120, fog: 0x0b1120, plane: 0x1e293b, grid: 0x475569, gridCenter: 0x64748b, ambient: 0.5
            } : {
                bg: 0xe2e8f0, fog: 0xe2e8f0, plane: 0xf1f5f9, grid: 0xcbd5e1, gridCenter: 0x94a3b8, ambient: 0.7
            };

            scene.background = new THREE.Color(t.bg);
            scene.fog = new THREE.FogExp2(t.fog, 0.002);

            if (envObjects.plane) envObjects.plane.material.color.setHex(t.plane);

            if (envObjects.grid) {
                scene.remove(envObjects.grid);
                envObjects.grid.geometry.dispose();
                const size = Math.max(props.map.width, props.map.length) * 4;
                const grid = new THREE.GridHelper(size, Math.floor(size / 5), t.gridCenter, t.grid);
                grid.rotation.x = Math.PI / 2;
                grid.position.set(props.map.width / 2, props.map.length / 2, 0);
                scene.add(grid);
                envObjects.grid = grid;
            }

            if (envObjects.lights[0]) envObjects.lights[0].intensity = t.ambient;
        };

        const animate = () => {
            if (isUnmounted || !renderer || !scene || !camera) return;
            animationId = requestAnimationFrame(animate);
            if (controls) controls.update();
            renderer.render(scene, camera);
        };

        const retryInit = () => {
            isReady.value = false;
            initEngine();
        };

        onMounted(() => {
            isUnmounted = false;
            resizeObserver = new ResizeObserver(() => {
                if (container.value && renderer && camera) {
                    const w = container.value.clientWidth;
                    const h = container.value.clientHeight;
                    if (w > 0 && h > 0) {
                        camera.aspect = w / h;
                        camera.updateProjectionMatrix();
                        renderer.setSize(w, h);
                    }
                }
            });
            if (container.value) resizeObserver.observe(container.value);

            // 延迟一帧启动，确保 props 已经就绪
            setTimeout(initEngine, 50);
        });

        // Watchers
        watch(() => [props.map.width, props.map.length], () => {
            if (scene) updateEnvironment(false);
            else initEngine();
        });

        watch(() => [props.map.static_obstacles, props.map.dynamic_obstacles, props.path, props.visuals.safetyMode, props.map.inflated_grid], () => {
            updateSceneContent();
        }, { deep: false });

        watch(() => props.isDark, applyThemeColors);

        onBeforeUnmount(() => {
            isUnmounted = true;
            if (resizeObserver) resizeObserver.disconnect();
            if (animationId) cancelAnimationFrame(animationId);
            if (renderer) {
                renderer.dispose();
                renderer.forceContextLoss();
                if (container.value) container.value.innerHTML = '';
            }
            scene = null;
        });

        return { container, isReady, initError, retryInit };
    }
};
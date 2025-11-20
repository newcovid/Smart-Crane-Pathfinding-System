import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const { ref, watch, onMounted, onBeforeUnmount, toRaw, nextTick } = Vue;

export default {
    props: ['map', 'path', 'mission', 'active'],
    template: `
    <div ref="container" class="w-full h-full bg-slate-900 relative overflow-hidden">
        <!-- Loading / Empty State -->
        <div v-if="!map.width" class="absolute inset-0 flex items-center justify-center text-slate-500 z-10 bg-slate-900/80">
            <div class="flex flex-col items-center gap-2">
                <i class="ph-bold ph-spinner animate-spin text-2xl"></i>
                <span>等待地图数据...</span>
            </div>
        </div>
        <!-- Three.js Canvas Container -->
        <div class="absolute top-4 left-4 z-10 pointer-events-none">
            <div class="bg-slate-800/80 backdrop-blur text-slate-300 text-xs px-3 py-1.5 rounded border border-slate-700 shadow-lg">
                <span class="text-primary font-bold">3D 视图</span> | 左键旋转 · 右键平移 · 滚轮缩放
            </div>
        </div>
    </div>
    `,
    setup(props) {
        const container = ref(null);

        // 使用非响应式变量存储 Three.js 实例，避免 Vue 代理带来的性能开销和潜在 BUG
        let scene = null;
        let camera = null;
        let renderer = null;
        let controls = null;
        let animationId = null;
        let resizeObserver = null;

        // 资源缓存，用于更新时清理
        let meshes = {
            obstacles: [],
            pathLines: [],
            markers: []
        };

        /**
         * 初始化 Three.js 场景核心组件
         */
        const init = async () => {
            // 1. 基础检查：等待 DOM 就绪且地图数据有效
            await nextTick();
            if (!container.value || !props.map.width) {
                console.warn("[Canvas3d] 容器或地图数据未就绪，跳过初始化");
                return;
            }

            // 防止重复初始化
            if (renderer) dispose();

            const w = container.value.clientWidth;
            const h = container.value.clientHeight;

            // --- Scene ---
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x0f172a); // 与 Tailwind slate-900 一致
            scene.fog = new THREE.FogExp2(0x0f172a, 0.002); // 远处雾化效果

            // --- Camera ---
            camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 2000);
            // 自动计算相机位置：放置在地图中心上方后退处
            const mapW = props.map.width;
            const mapL = props.map.length;
            const maxDim = Math.max(mapW, mapL);

            // 初始视角：斜向下 45 度
            camera.position.set(mapW * 0.5, mapL * -0.5, maxDim * 1.2);
            camera.up.set(0, 0, 1); // Z 轴向上

            // --- Renderer ---
            renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
            renderer.setSize(w, h);
            renderer.setPixelRatio(window.devicePixelRatio);
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;

            // 挂载 Canvas
            container.value.innerHTML = '';
            container.value.appendChild(renderer.domElement);

            // --- Controls ---
            controls = new OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;
            controls.screenSpacePanning = false;
            // 限制旋转角度，防止穿透地面
            controls.maxPolarAngle = Math.PI / 2 - 0.1;
            // 设置旋转中心为地图中心
            controls.target.set(mapW / 2, mapL / 2, 0);
            controls.update();

            // --- Lights (光照系统) ---
            setupLights(mapW, mapL);

            // --- Environment (环境: 地面, 网格) ---
            setupEnvironment(mapW, mapL);

            // --- Loop ---
            animate();

            // --- Initial Content ---
            updateSceneContent();

            console.log("[Canvas3d] 初始化完成");
        };

        /**
         * 设置光照
         */
        const setupLights = (mapW, mapL) => {
            // 1. 环境光 (基础亮度)
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
            scene.add(ambientLight);

            // 2. 半球光 (模拟天空和地面反光)
            const hemiLight = new THREE.HemisphereLight(0xffffff, 0x0f172a, 0.3);
            hemiLight.position.set(0, 0, 50);
            scene.add(hemiLight);

            // 3. 平行光 (主光源，产生阴影)
            const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
            // 光源位置：从高处斜射
            const center = Math.max(mapW, mapL) / 2;
            dirLight.position.set(-center * 0.5, -center * 0.5, center * 1.5);
            dirLight.castShadow = true;

            // 优化阴影范围 (覆盖整个地图)
            const d = Math.max(mapW, mapL) * 1.5;
            dirLight.shadow.camera.left = -d;
            dirLight.shadow.camera.right = d;
            dirLight.shadow.camera.top = d;
            dirLight.shadow.camera.bottom = -d;
            dirLight.shadow.camera.near = 1;
            dirLight.shadow.camera.far = 500;
            dirLight.shadow.mapSize.width = 2048;
            dirLight.shadow.mapSize.height = 2048;
            dirLight.shadow.bias = -0.0005;

            scene.add(dirLight);
        };

        /**
         * 设置环境 (地面、网格)
         */
        const setupEnvironment = (mapW, mapL) => {
            const centerX = mapW / 2;
            const centerY = mapL / 2;
            const size = Math.max(mapW, mapL) * 4; // 足够大的地面

            // 地面 (接收阴影)
            const planeGeo = new THREE.PlaneGeometry(size, size);
            const planeMat = new THREE.MeshStandardMaterial({
                color: 0x1e293b, // slate-800
                roughness: 0.8,
                metalness: 0.2
            });
            const plane = new THREE.Mesh(planeGeo, planeMat);
            plane.position.set(centerX, centerY, -0.05); // 略微下沉，防止 z-fighting
            plane.receiveShadow = true;
            scene.add(plane);

            // 辅助网格
            const gridHelper = new THREE.GridHelper(size, Math.floor(size / 5), 0x475569, 0x1e293b);
            gridHelper.rotation.x = Math.PI / 2;
            gridHelper.position.set(centerX, centerY, 0);
            scene.add(gridHelper);

            // 坐标轴 (X=红, Y=绿, Z=蓝)
            const axesHelper = new THREE.AxesHelper(5);
            axesHelper.position.set(0, 0, 0.1);
            scene.add(axesHelper);
        };

        /**
         * 更新场景内容 (障碍物, 路径)
         * 使用 Diff 逻辑或全量重建
         */
        const updateSceneContent = () => {
            if (!scene) return;

            // 1. 清理旧物体
            [...meshes.obstacles, ...meshes.pathLines, ...meshes.markers].forEach(obj => {
                scene.remove(obj);
                if (obj.geometry) obj.geometry.dispose();
                if (obj.material) {
                    if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose());
                    else obj.material.dispose();
                }
            });
            meshes.obstacles = [];
            meshes.pathLines = [];
            meshes.markers = [];

            // 2. 添加障碍物
            const boxGeo = new THREE.BoxGeometry(1, 1, 1);
            const staticMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.5, metalness: 0.1 }); // 静态: 灰色
            const dynamicMat = new THREE.MeshStandardMaterial({ color: 0xef4444, roughness: 0.3, metalness: 0.3 }); // 动态: 红色

            const addObs = (dict, mat) => {
                Object.values(dict || {}).forEach(o => {
                    const mesh = new THREE.Mesh(boxGeo, mat);
                    // 尺寸
                    const h = o.z_m || 2.0; // 高度
                    mesh.scale.set(o.w_m, o.h_m, h);
                    // 位置: 转换为中心点坐标
                    // 假设 o.x_m, o.y_m 是左下角，z_m 是总高度
                    mesh.position.set(
                        o.x_m + o.w_m / 2,
                        o.y_m + o.h_m / 2,
                        h / 2
                    );
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;
                    scene.add(mesh);
                    meshes.obstacles.push(mesh);
                });
            };

            addObs(props.map.static_obstacles, staticMat);
            addObs(props.map.dynamic_obstacles, dynamicMat);

            // 3. 添加路径
            if (props.path && props.path.length > 1) {
                // 3.1 路径线
                const points = props.path.map(p => new THREE.Vector3(p[0], p[1], p[2] || 0.5));
                // 使用 TubeGeometry 或 Line 渲染
                // 为简单起见且保证可见性，使用 LineLoop
                const curve = new THREE.CatmullRomCurve3(points, false, 'catmullrom', 0.1); // 稍微平滑一点
                // 如果点太少，直接用 Line
                const geometry = new THREE.BufferGeometry().setFromPoints(points);
                const material = new THREE.LineBasicMaterial({ color: 0x3b82f6, linewidth: 2 });
                const line = new THREE.Line(geometry, material);
                scene.add(line);
                meshes.pathLines.push(line);

                // 3.2 路径节点 (小球)
                const nodeGeo = new THREE.SphereGeometry(0.2, 8, 8);
                const nodeMat = new THREE.MeshBasicMaterial({ color: 0x60a5fa });
                points.forEach(pt => {
                    const node = new THREE.Mesh(nodeGeo, nodeMat);
                    node.position.copy(pt);
                    scene.add(node);
                    meshes.markers.push(node);
                });
            }

            // 4. 起终点标记
            const markerGeo = new THREE.SphereGeometry(0.6, 16, 16);

            if (props.mission.start) {
                const startMesh = new THREE.Mesh(markerGeo, new THREE.MeshStandardMaterial({ color: 0x10b981, emissive: 0x065f46 }));
                startMesh.position.set(props.mission.start.x, props.mission.start.y, 0.6);
                startMesh.castShadow = true;
                scene.add(startMesh);
                meshes.markers.push(startMesh);

                // 起点光柱
                addBeacon(props.mission.start.x, props.mission.start.y, 0x10b981);
            }

            if (props.mission.end) {
                const endMesh = new THREE.Mesh(markerGeo, new THREE.MeshStandardMaterial({ color: 0xf43f5e, emissive: 0x881337 }));
                endMesh.position.set(props.mission.end.x, props.mission.end.y, 0.6);
                endMesh.castShadow = true;
                scene.add(endMesh);
                meshes.markers.push(endMesh);

                // 终点光柱
                addBeacon(props.mission.end.x, props.mission.end.y, 0xf43f5e);
            }
        };

        const addBeacon = (x, y, color) => {
            const geo = new THREE.CylinderGeometry(0.1, 0.1, 10, 8);
            geo.translate(0, 5, 0); // 底部对齐 Z=0
            const mat = new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.3 });
            const mesh = new THREE.Mesh(geo, mat);
            mesh.position.set(x, y, 0);
            mesh.rotation.x = Math.PI / 2; // 竖起来
            scene.add(mesh);
            meshes.markers.push(mesh);
        }

        /**
         * 动画循环
         */
        const animate = () => {
            if (!renderer) return;

            animationId = requestAnimationFrame(animate);

            if (controls) controls.update();
            renderer.render(scene, camera);
        };

        /**
         * 响应式处理
         */
        const onResize = () => {
            if (!container.value || !camera || !renderer) return;
            const w = container.value.clientWidth;
            const h = container.value.clientHeight;
            if (w === 0 || h === 0) return;

            camera.aspect = w / h;
            camera.updateProjectionMatrix();
            renderer.setSize(w, h);
        };

        // --- Lifecycle ---

        onMounted(() => {
            // 核心修复：使用 ResizeObserver 触发首次尺寸计算和初始化
            // 解决 v-if 导致容器尺寸在 mounted 瞬间可能不正确的问题
            resizeObserver = new ResizeObserver((entries) => {
                for (const entry of entries) {
                    if (entry.contentRect.width > 0 && !renderer) {
                        init(); // 尺寸就绪后立即初始化
                    } else if (renderer) {
                        onResize();
                    }
                }
            });

            if (container.value) {
                resizeObserver.observe(container.value);
            }
        });

        // 监听地图数据变化
        watch(() => props.map, (newMap) => {
            if (newMap && newMap.width) {
                if (!scene) {
                    init(); // 如果之前因为没数据没初始化，现在初始化
                } else {
                    updateSceneContent(); // 否则只更新内容
                }
            }
        }, { deep: true });

        // 监听路径变化
        watch(() => [props.path, props.mission], () => {
            updateSceneContent();
        }, { deep: true });

        onBeforeUnmount(() => {
            dispose();
        });

        const dispose = () => {
            if (animationId) cancelAnimationFrame(animationId);
            if (resizeObserver) resizeObserver.disconnect();

            if (renderer) {
                renderer.dispose();
                renderer.forceContextLoss();
                if (container.value) container.value.innerHTML = '';
            }

            // 释放内存
            if (scene) {
                scene.traverse((object) => {
                    if (object.geometry) object.geometry.dispose();
                    if (object.material) {
                        if (Array.isArray(object.material)) object.material.forEach(m => m.dispose());
                        else object.material.dispose();
                    }
                });
            }

            scene = null;
            camera = null;
            renderer = null;
            controls = null;
        };

        return { container };
    }
};
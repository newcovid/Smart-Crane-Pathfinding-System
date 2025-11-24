const { ref, computed, reactive, onMounted, onUnmounted } = Vue;

// 1. 分区卡片组件
export const SectionGroup = {
    props: {
        title: String,
        icon: String,
        defaultOpen: { type: Boolean, default: true },
        noPadding: { type: Boolean, default: false }
    },
    setup(props) {
        const isOpen = ref(props.defaultOpen);
        const toggle = () => isOpen.value = !isOpen.value;
        return { isOpen, toggle };
    },
    template: `
    <div class="bg-white dark:bg-slate-800 rounded-xl border border-slate-200 dark:border-slate-700 shadow-sm overflow-hidden transition-all duration-300 hover:shadow-md">
        <button @click="toggle" class="w-full flex items-center justify-between py-3 px-4 bg-slate-50/50 dark:bg-white/5 hover:bg-slate-100 dark:hover:bg-white/10 transition-colors group select-none border-b border-transparent" :class="{'border-slate-100 dark:border-slate-700/50': isOpen}">
            <div class="flex items-center gap-2.5 text-sm font-bold text-slate-700 dark:text-slate-200 group-hover:text-primary dark:group-hover:text-primary transition-colors">
                <i v-if="icon" :class="icon" class="text-lg text-slate-400 group-hover:text-primary transition-colors"></i>
                <span class="tracking-wide">{{ title }}</span>
            </div>
            <div class="w-6 h-6 rounded-full flex items-center justify-center bg-white dark:bg-slate-700 shadow-sm border border-slate-100 dark:border-slate-600 text-slate-400 group-hover:text-primary transition-all">
                <i class="ph-bold ph-caret-down transition-transform duration-300" :class="{ 'rotate-180': !isOpen }"></i>
            </div>
        </button>
        <div v-show="isOpen" :class="noPadding ? '' : 'p-4 space-y-4'" class="animate-fadeIn bg-white dark:bg-slate-800">
            <slot></slot>
        </div>
    </div>
    `
};

// 2. 数字输入组件
export const NumberInput = {
    props: {
        label: String,
        modelValue: [Number, String],
        unit: String,
        step: { type: Number, default: 1 },
        min: Number,
        max: Number,
        precision: { type: Number, default: 2 }
    },
    emits: ['update:modelValue'],
    setup(props, { emit }) {
        const update = (val) => {
            let num = parseFloat(val);
            if (isNaN(num)) return;
            const p = Math.pow(10, props.precision);
            num = Math.round(num * p) / p;
            if (props.min !== undefined && num < props.min) num = props.min;
            if (props.max !== undefined && num > props.max) num = props.max;
            emit('update:modelValue', num);
        };
        const inc = () => update(Number(props.modelValue) + props.step);
        const dec = () => update(Number(props.modelValue) - props.step);
        return { update, inc, dec };
    },
    template: `
    <div class="flex items-center justify-between gap-4">
        <label class="text-sm text-slate-600 dark:text-slate-300 font-medium shrink-0 select-none">{{ label }}</label>
        <div class="flex items-center bg-slate-100 dark:bg-slate-900/50 rounded-lg overflow-hidden h-9 w-[140px] shrink-0 border border-transparent focus-within:border-primary focus-within:ring-2 focus-within:ring-primary/20 transition-all">
            <button @click="dec" class="w-9 h-full flex items-center justify-center hover:bg-slate-200 dark:hover:bg-slate-700 text-slate-500 dark:text-slate-400 transition-colors active:bg-slate-300 dark:active:bg-slate-600 border-r border-slate-200 dark:border-slate-700">
                <i class="ph-bold ph-minus"></i>
            </button>
            <div class="flex-1 relative h-full group/input bg-white dark:bg-slate-800">
                <input type="number" :value="modelValue" 
                       @change="update($event.target.value)"
                       class="w-full h-full bg-transparent text-center text-sm font-mono font-bold text-slate-800 dark:text-slate-100 focus:outline-none px-1 z-10 relative" />
                <span v-if="unit" class="absolute right-1.5 top-1/2 -translate-y-1/2 text-[10px] text-slate-400 pointer-events-none font-medium">{{ unit }}</span>
            </div>
            <button @click="inc" class="w-9 h-full flex items-center justify-center hover:bg-slate-200 dark:hover:bg-slate-700 text-slate-500 dark:text-slate-400 transition-colors active:bg-slate-300 dark:active:bg-slate-600 border-l border-slate-200 dark:border-slate-700">
                <i class="ph-bold ph-plus"></i>
            </button>
        </div>
    </div>
    `
};

// 3. 下拉选择组件
export const SelectInput = {
    props: ['label', 'modelValue', 'options'],
    emits: ['update:modelValue'],
    template: `
    <div class="flex items-center justify-between gap-4">
        <label class="text-sm text-slate-600 dark:text-slate-300 font-medium shrink-0 select-none">{{ label }}</label>
        <div class="relative w-[140px] h-9 shrink-0 group">
            <select :value="modelValue" 
                    @change="$emit('update:modelValue', $event.target.value)"
                    class="w-full h-full appearance-none bg-white dark:bg-slate-800 border border-slate-200 dark:border-slate-700 rounded-lg px-3 pr-8 text-sm font-medium text-slate-700 dark:text-slate-200 focus:outline-none focus:border-primary focus:ring-2 focus:ring-primary/20 transition-all cursor-pointer hover:border-slate-300 dark:hover:border-slate-600">
                <option v-for="opt in options" :key="opt.value" :value="opt.value">{{ opt.label }}</option>
            </select>
            <div class="absolute right-2.5 top-1/2 -translate-y-1/2 pointer-events-none text-slate-400 group-hover:text-slate-600 dark:group-hover:text-slate-300 transition-colors">
                <i class="ph-bold ph-caret-up-down text-sm"></i>
            </div>
        </div>
    </div>
    `
};

// 4. 开关组件
export const ToggleSwitch = {
    props: ['label', 'modelValue', 'subtext'],
    emits: ['update:modelValue'],
    template: `
    <label class="flex items-start justify-between cursor-pointer group py-1 select-none min-h-[36px]">
        <div class="flex flex-col justify-center h-full pr-4">
            <span class="text-sm text-slate-700 dark:text-slate-200 font-medium group-hover:text-primary dark:group-hover:text-primary transition-colors">{{ label }}</span>
            <span v-if="subtext" class="text-xs text-slate-400 leading-snug mt-0.5">{{ subtext }}</span>
        </div>
        <div class="relative inline-flex items-center cursor-pointer shrink-0 mt-0.5">
            <input type="checkbox" :checked="modelValue" 
                   @change="$emit('update:modelValue', $event.target.checked)" class="sr-only peer">
            <div class="w-11 h-6 bg-slate-200 dark:bg-slate-700 peer-focus:outline-none rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all after:shadow-sm peer-checked:bg-primary peer-checked:after:border-white transition-colors shadow-inner"></div>
        </div>
    </label>
    `
};

// 5. 工具按钮
export const ToolBtn = {
    props: ['icon', 'label', 'value', 'modelValue', 'color'],
    emits: ['update:modelValue'],
    template: `
    <label class="cursor-pointer block relative group select-none" :title="label">
        <input type="radio" name="tool_option_group" :value="value" 
               :checked="modelValue === value" 
               @change="$emit('update:modelValue', value)" class="peer hidden">
        <div class="flex flex-col items-center justify-center h-[72px] rounded-xl border-2 transition-all shadow-sm
                    bg-white dark:bg-slate-800 
                    border-slate-100 dark:border-slate-700 text-slate-400
                    hover:border-slate-300 dark:hover:border-slate-500 hover:text-slate-600 dark:hover:text-slate-200 hover:-translate-y-0.5
                    peer-checked:border-primary peer-checked:bg-primary/5 dark:peer-checked:bg-primary/10 peer-checked:text-primary peer-checked:shadow-md">
            <i :class="[icon, color && modelValue === value ? color : '']" class="text-2xl mb-1.5 ph-bold transition-colors"></i>
            <span class="text-xs font-bold text-center leading-none px-1">{{ label }}</span>
        </div>
    </label>
    `
};

// 6. 综合统计看板
export const StatsPanel = {
    props: ['stats', 'mapState', 'settings'],
    setup(props) {
        const isExpanded = ref(true);

        const pos = reactive({ right: 20, top: 80 });
        const isDragging = ref(false);
        let dragStart = { x: 0, y: 0, initialRight: 0, initialTop: 0 };

        const startDrag = (e) => {
            isDragging.value = true;
            dragStart.x = e.clientX;
            dragStart.y = e.clientY;
            dragStart.initialRight = pos.right;
            dragStart.initialTop = pos.top;
            window.addEventListener('mousemove', handleDrag);
            window.addEventListener('mouseup', stopDrag);
        };

        const handleDrag = (e) => {
            if (!isDragging.value) return;
            const deltaX = e.clientX - dragStart.x;
            const deltaY = e.clientY - dragStart.y;
            let newRight = dragStart.initialRight - deltaX;
            let newTop = dragStart.initialTop + deltaY;
            const currentWidth = isExpanded.value ? 260 : 44;
            const maxRight = window.innerWidth - currentWidth;
            const maxTop = window.innerHeight - 50;
            newRight = Math.max(0, Math.min(newRight, maxRight));
            newTop = Math.max(0, Math.min(newTop, maxTop));
            pos.right = newRight;
            pos.top = newTop;
        };

        const stopDrag = () => {
            isDragging.value = false;
            window.removeEventListener('mousemove', handleDrag);
            window.removeEventListener('mouseup', stopDrag);
        };

        const toggle = () => isExpanded.value = !isExpanded.value;

        const envData = computed(() => {
            if (!props.mapState || !props.mapState.width) return { totalVoxels: 0, dims: '-' };
            const w = Math.ceil(props.mapState.width / props.mapState.resolution);
            const l = Math.ceil(props.mapState.length / props.mapState.resolution);
            const rawH = props.mapState.height_m || props.settings.MAP_HEIGHT_M || 20;
            const h = props.settings.ENABLE_FIXED_HEIGHT_CRUISE ? 1 : Math.ceil(rawH / props.mapState.resolution);
            const total = w * l * h;
            const unit = total > 1000000 ? `${(total / 1000000).toFixed(1)}M` : (total > 1000 ? `${(total / 1000).toFixed(1)}k` : total);
            return { totalVoxels: unit, gridDims: `${w}x${l}x${h}` };
        });

        const gridPrepTime = computed(() => {
            if (props.stats && props.stats.timings && props.stats.timings.grid_prep_ms !== undefined) {
                const t = props.stats.timings.grid_prep_ms;
                if (t < 0.1) return '<0.1';
                return t.toFixed(1);
            }
            return '0.0';
        });

        const algoData = computed(() => {
            if (!props.stats) return {};
            return {
                name: props.settings.PLANNER_ALGORITHM.toUpperCase(),
                nodes: props.stats.nodes_expanded || 0,
                time: props.stats.timings?.pathfinding_ms ? props.stats.timings.pathfinding_ms.toFixed(1) : 0
            };
        });

        const pipelineSteps = computed(() => {
            if (!props.stats || !props.stats.processors_stats) return [];
            return props.stats.processors_stats.map(p => {
                let name = p.processor;
                if (name === 'GreedyShortcut') name = '捷径优化';
                if (name === 'BezierSmoother') name = '贝塞尔平滑';

                const diff = p.output_nodes - p.input_nodes;
                const icon = diff < 0 ? 'ph-arrow-down-right' : (diff > 0 ? 'ph-arrow-up-right' : 'ph-arrow-right');
                const color = diff < 0 ? 'text-emerald-500' : (diff > 0 ? 'text-blue-500' : 'text-slate-400');
                const absRate = Math.abs(p.reduction_rate * 100).toFixed(0);
                const sign = diff > 0 ? '+' : (diff < 0 ? '-' : '');
                const rateText = `${sign}${absRate}%`;

                return {
                    name,
                    time: p.process_time_ms.toFixed(1),
                    in: p.input_nodes,
                    out: p.output_nodes,
                    rate: rateText,
                    diff, icon, color
                };
            });
        });

        const totalTimeColor = computed(() => {
            const t = props.stats && props.stats.timings ? props.stats.timings.total_ms : 0;
            if (t < 50) return 'text-emerald-500';
            if (t < 200) return 'text-amber-500';
            return 'text-rose-500';
        });

        return { isExpanded, toggle, envData, gridPrepTime, algoData, pipelineSteps, totalTimeColor, pos, startDrag };
    },
    template: `
    <div class="fixed z-50 stats-draggable flex flex-col items-end transition-all duration-200 ease-out" 
         :style="{ top: pos.top + 'px', right: pos.right + 'px' }">
         
        <!-- 1. Collapsed Icon -->
        <button v-if="!isExpanded" @mousedown="startDrag" @click="toggle" 
            class="bg-white dark:bg-slate-800 text-slate-500 dark:text-slate-400 p-2.5 rounded-lg shadow-lg border border-slate-200 dark:border-slate-700 hover:text-primary hover:scale-105 transition-all cursor-move active:cursor-grabbing relative z-50">
            <i class="ph-bold ph-chart-polar text-xl"></i>
        </button>

        <!-- 2. Expanded Panel -->
        <div v-else class="w-[260px] bg-white/95 dark:bg-[#1e293b]/95 backdrop-blur-xl border border-slate-200 dark:border-slate-700 rounded-xl shadow-2xl overflow-hidden flex flex-col animate-fadeIn transition-colors">
            <!-- Header -->
            <div @mousedown="startDrag" class="flex items-center justify-between px-3 py-2 border-b border-slate-100 dark:border-slate-700 bg-slate-50/50 dark:bg-white/5 cursor-move active:cursor-grabbing select-none group">
                <div class="flex items-center gap-1.5 text-[10px] font-bold text-slate-500 dark:text-slate-400 uppercase tracking-widest group-hover:text-primary transition-colors">
                    <i class="ph-bold ph-dots-six-vertical text-slate-300"></i>
                    <span>性能监控</span>
                </div>
                <button @mousedown.stop @click="toggle" class="text-slate-400 hover:text-slate-600 dark:hover:text-slate-200 transition-colors p-1 rounded hover:bg-black/5 dark:hover:bg-white/10">
                    <i class="ph-bold ph-minus text-xs"></i>
                </button>
            </div>
            
            <!-- Content -->
            <div class="p-3 space-y-3 overflow-y-auto max-h-[60vh] custom-scrollbar" @mousedown.stop>
                <!-- Environment & Prep -->
                <div class="space-y-1.5">
                    <div class="text-[10px] font-bold text-slate-400 uppercase tracking-wider flex justify-between">
                        <span>环境 (Env)</span>
                        <span class="font-mono text-slate-500 dark:text-slate-400">{{ gridPrepTime }}ms Prep</span>
                    </div>
                    <div class="grid grid-cols-2 gap-2">
                        <div class="bg-slate-50 dark:bg-slate-800/50 p-1.5 rounded border border-slate-100 dark:border-slate-700/50">
                            <div class="text-[10px] text-slate-400 mb-0.5">Total Grid</div>
                            <div class="font-mono text-xs font-bold text-slate-600 dark:text-slate-300">{{ envData.totalVoxels }}</div>
                        </div>
                        <div class="bg-slate-50 dark:bg-slate-800/50 p-1.5 rounded border border-slate-100 dark:border-slate-700/50">
                            <div class="text-[10px] text-slate-400 mb-0.5">Dim</div>
                            <div class="font-mono text-xs font-bold text-slate-600 dark:text-slate-300 truncate" :title="envData.gridDims">{{ envData.gridDims }}</div>
                        </div>
                    </div>
                </div>

                <!-- Planning -->
                <div class="space-y-1.5 relative">
                    <div class="absolute left-[11px] top-5 bottom-[-8px] w-px bg-slate-200 dark:bg-slate-700"></div>
                    <div class="text-[10px] font-bold text-slate-400 uppercase tracking-wider">规划 (Plan)</div>
                    <div class="relative bg-blue-50/50 dark:bg-blue-900/10 border border-blue-100 dark:border-blue-800 rounded p-2">
                        <div class="flex justify-between items-center mb-1">
                            <span class="text-[10px] font-bold text-blue-600 dark:text-blue-400">{{ algoData.name }}</span>
                            <span class="text-[10px] font-mono bg-white dark:bg-slate-800 px-1 rounded text-slate-500 border border-blue-100 dark:border-slate-700 shadow-sm">
                                {{ algoData.time }}ms
                            </span>
                        </div>
                        <div class="flex items-center gap-1.5 text-[10px] text-slate-500 dark:text-slate-400">
                            <i class="ph-bold ph-magnifying-glass"></i>
                            <span>Nodes: <strong class="text-slate-700 dark:text-slate-300 font-mono">{{ algoData.nodes }}</strong></span>
                        </div>
                    </div>
                </div>

                <!-- Pipeline -->
                <div v-if="pipelineSteps.length > 0" class="space-y-2 relative">
                    <div class="absolute left-[11px] top-[-10px] bottom-2 w-px bg-slate-200 dark:bg-slate-700"></div>
                    <div class="text-[10px] font-bold text-slate-400 uppercase tracking-wider pl-6">优化 (Opt)</div>
                    <div v-for="(step, idx) in pipelineSteps" :key="idx" class="relative pl-6">
                        <div class="absolute left-[10px] top-2.5 w-1.5 h-1.5 rounded-full border border-white dark:border-slate-800 bg-slate-300 dark:bg-slate-600 z-10"></div>
                        <div class="bg-slate-50 dark:bg-slate-800/30 border border-slate-100 dark:border-slate-700 rounded p-2 transition-all hover:border-slate-300 dark:hover:border-slate-600">
                            <div class="flex justify-between items-center mb-1">
                                <span class="text-[10px] font-bold text-slate-600 dark:text-slate-300">{{ step.name }}</span>
                                <span class="text-[10px] font-mono text-slate-400">{{ step.time }}ms</span>
                            </div>
                            <div class="flex items-center justify-between">
                                <div class="flex items-center gap-1 text-[10px] text-slate-500 font-mono">
                                    <span>{{ step.in }}</span>
                                    <i :class="[step.icon, step.color]" class="text-[8px]"></i>
                                    <span class="text-slate-700 dark:text-slate-300 font-bold">{{ step.out }}</span>
                                </div>
                                <span class="text-[10px] font-bold" :class="step.color">
                                    {{ step.rate }}
                                </span>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Total -->
                <div class="pt-2 border-t border-slate-200 dark:border-slate-700/50">
                    <div class="flex justify-between items-end">
                        <span class="text-[10px] text-slate-400 uppercase font-bold tracking-wider">TOTAL LATENCY</span>
                        <div class="flex items-baseline gap-0.5">
                            <span class="text-lg font-mono font-bold" :class="totalTimeColor">
                                <!-- [Fix] 安全检查 total_ms，防止 undefined 导致崩溃 -->
                                {{ (stats && stats.timings && stats.timings.total_ms !== undefined) ? stats.timings.total_ms.toFixed(1) : '0.0' }}
                            </span>
                            <span class="text-[10px] font-bold text-slate-400">ms</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    `
};

// 7. 通知 Toast 组件
export const NotificationToast = {
    props: ['notifications'],
    emits: ['remove'],
    template: `
    <!-- [Change] 定位调整至右下角 (bottom-8 right-8) -->
    <div class="fixed bottom-8 right-8 z-[100] flex flex-col gap-2 w-80 pointer-events-none">
        <transition-group name="toast" tag="div" class="flex flex-col gap-2">
            <div v-for="note in notifications" :key="note.id"
                 class="pointer-events-auto flex items-start gap-3 p-3.5 rounded-xl shadow-xl border backdrop-blur-md transition-all transform select-none"
                 :class="getClasses(note.type)">
                
                <!-- Icon -->
                <div class="mt-0.5 shrink-0">
                    <i :class="getIcon(note.type)" class="text-xl"></i>
                </div>
                
                <!-- Content -->
                <div class="flex-1 min-w-0">
                    <h4 v-if="note.title" class="text-sm font-bold opacity-95 mb-0.5 leading-none">{{ note.title }}</h4>
                    <p class="text-xs leading-normal opacity-85 font-medium break-words">{{ note.message }}</p>
                </div>

                <!-- Close -->
                <button @click="$emit('remove', note.id)" class="shrink-0 text-current opacity-50 hover:opacity-100 transition-opacity p-0.5 -mr-1 -mt-1 rounded hover:bg-black/5 dark:hover:bg-white/10">
                    <i class="ph-bold ph-x text-sm"></i>
                </button>
            </div>
        </transition-group>
    </div>
    `,
    setup() {
        const getClasses = (type) => {
            switch (type) {
                case 'error': return 'bg-red-50/95 dark:bg-red-900/95 border-red-200 dark:border-red-800 text-red-700 dark:text-red-200';
                case 'warning': return 'bg-amber-50/95 dark:bg-amber-900/95 border-amber-200 dark:border-amber-800 text-amber-700 dark:text-amber-200';
                case 'success': return 'bg-emerald-50/95 dark:bg-emerald-900/95 border-emerald-200 dark:border-emerald-800 text-emerald-700 dark:text-emerald-200';
                default: return 'bg-blue-50/95 dark:bg-blue-900/95 border-blue-200 dark:border-blue-800 text-blue-700 dark:text-blue-200';
            }
        };
        const getIcon = (type) => {
            switch (type) {
                case 'error': return 'ph-fill ph-warning-circle';
                case 'warning': return 'ph-fill ph-warning';
                case 'success': return 'ph-fill ph-check-circle';
                default: return 'ph-fill ph-info';
            }
        };
        return { getClasses, getIcon };
    }
};
export default {
    props: ['logs'],
    template: `
    <div class="flex flex-col h-full bg-[#1e1e1e] border-t border-slate-700 font-mono text-xs">
        <!-- Toolbar -->
        <div class="flex items-center justify-between px-3 py-1 bg-[#2d2d2d] text-slate-400 border-b border-black/20 select-none">
            <div class="flex items-center gap-2">
                <i class="ph-bold ph-terminal-window text-blue-400"></i>
                <span>系统日志 / 终端</span>
            </div>
            <div class="flex gap-2">
                <button @click="$emit('clear')" class="hover:text-white transition-colors" title="清空日志">
                    <i class="ph-bold ph-prohibit"></i>
                </button>
                <button @click="$emit('close')" class="hover:text-white transition-colors" title="隐藏">
                    <i class="ph-bold ph-caret-down"></i>
                </button>
            </div>
        </div>
        
        <!-- Content -->
        <div ref="logContainer" class="flex-1 overflow-y-auto p-2 space-y-0.5 custom-scrollbar">
            <div v-if="logs.length === 0" class="text-slate-600 italic opacity-50 select-none">
                等待系统日志...
            </div>
            <div v-for="(log, idx) in logs" :key="idx" class="whitespace-pre-wrap break-all flex gap-2 group hover:bg-white/5 leading-5">
                <span class="shrink-0 text-slate-500 select-none w-[60px]">{{ log.time }}</span>
                <span :class="getLevelColor(log.level)" class="font-bold shrink-0 w-[45px]">{{ log.level }}</span>
                <span class="text-slate-300 group-hover:text-white">{{ log.msg }}</span>
            </div>
        </div>
    </div>
    `,

    setup(props) {
        const logContainer = Vue.ref(null);

        const getLevelColor = (level) => {
            switch (level) {
                case 'INFO': return 'text-blue-400';
                case 'WARNING': return 'text-yellow-400';
                case 'ERROR': return 'text-red-400';
                case 'CRITICAL': return 'text-purple-500 bg-purple-500/10 px-1 rounded';
                default: return 'text-slate-400';
            }
        };

        Vue.watch(() => props.logs.length, async () => {
            await Vue.nextTick();
            if (logContainer.value) {
                logContainer.value.scrollTop = logContainer.value.scrollHeight;
            }
        });

        return { logContainer, getLevelColor };
    }
};
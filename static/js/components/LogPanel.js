export default {
    props: ['logs'],
    template: `
    <div class="flex flex-col h-full font-mono text-xs transition-colors duration-300
                bg-slate-50 dark:bg-[#1e1e1e] border-t border-slate-200 dark:border-slate-700">
        
        <!-- Toolbar -->
        <div class="flex items-center justify-between px-4 py-1.5 select-none border-b
                    bg-white dark:bg-[#2d2d2d] border-slate-200 dark:border-black/20 text-slate-500 dark:text-slate-400">
            <div class="flex items-center gap-2">
                <i class="ph-bold ph-terminal-window text-primary"></i>
                <span class="font-bold tracking-wide">SYSTEM TERMINAL</span>
            </div>
            <div class="flex gap-2">
                <button @click="$emit('clear')" class="hover:text-slate-800 dark:hover:text-white transition-colors" title="清空日志">
                    <i class="ph-bold ph-prohibit"></i>
                </button>
                <button @click="$emit('close')" class="hover:text-slate-800 dark:hover:text-white transition-colors" title="隐藏">
                    <i class="ph-bold ph-caret-down"></i>
                </button>
            </div>
        </div>
        
        <!-- Content -->
        <div ref="logContainer" class="flex-1 overflow-y-auto p-3 space-y-1 custom-scrollbar">
            <div v-if="logs.length === 0" class="text-slate-400 dark:text-slate-600 italic opacity-70 select-none pl-1">
                > 等待系统日志...
            </div>
            <div v-for="(log, idx) in logs" :key="idx" class="whitespace-pre-wrap break-all flex gap-3 group leading-relaxed border-b border-transparent hover:bg-slate-200/50 dark:hover:bg-white/5 rounded px-1">
                <span class="shrink-0 text-slate-400 dark:text-slate-500 select-none w-[60px]">{{ log.time }}</span>
                <span :class="getLevelColor(log.level)" class="font-bold shrink-0 w-[50px]">{{ log.level }}</span>
                <span class="text-slate-600 dark:text-slate-300 group-hover:text-slate-900 dark:group-hover:text-white transition-colors">{{ log.msg }}</span>
            </div>
        </div>
    </div>
    `,

    setup(props) {
        const logContainer = Vue.ref(null);

        const getLevelColor = (level) => {
            switch (level) {
                case 'INFO': return 'text-blue-500 dark:text-blue-400';
                case 'WARNING': return 'text-amber-500 dark:text-yellow-400';
                case 'ERROR': return 'text-red-500 dark:text-red-400';
                case 'CRITICAL': return 'text-purple-600 dark:text-purple-500 bg-purple-100 dark:bg-purple-500/10 px-1 rounded';
                default: return 'text-slate-500 dark:text-slate-400';
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
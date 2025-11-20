// 1. 工具按钮组件
export const ToolBtn = {
    props: ['icon', 'label', 'value', 'modelValue', 'color'],
    emits: ['update:modelValue'],
    template: `
    <label class="cursor-pointer block relative group select-none" :title="label">
        <input type="radio" :name="Math.random().toString(36)" :value="value" 
               :checked="modelValue === value" 
               @change="$emit('update:modelValue', value)" class="peer hidden">
        <div class="flex flex-col items-center justify-center p-2.5 rounded-lg border border-slate-200 dark:border-slate-700 bg-white dark:bg-slate-800 text-slate-400 
                    hover:border-slate-300 dark:hover:border-slate-600 hover:text-slate-600 dark:hover:text-slate-200 transition-all shadow-sm
                    peer-checked:border-primary peer-checked:bg-primary/5 dark:peer-checked:bg-primary/10 peer-checked:text-primary peer-checked:shadow-md peer-checked:scale-[1.02]">
            <i :class="[icon, color && modelValue === value ? color : '']" class="text-xl mb-1 ph-bold transition-colors"></i>
            <span class="text-[10px] font-bold">{{ label }}</span>
        </div>
    </label>
    `
};

// 2. 开关组件
export const ToggleSwitch = {
    props: ['label', 'modelValue'],
    emits: ['update:modelValue'],
    template: `
    <label class="flex items-center justify-between text-xs text-slate-600 dark:text-slate-300 cursor-pointer hover:text-slate-900 dark:hover:text-white transition-colors p-1.5 rounded hover:bg-slate-100 dark:hover:bg-slate-700/30 select-none">
        <span>{{ label }}</span>
        <div class="relative inline-flex items-center cursor-pointer">
            <input type="checkbox" :checked="modelValue" 
                   @change="$emit('update:modelValue', $event.target.checked)" class="sr-only peer">
            <div class="w-8 h-4 bg-slate-300 dark:bg-slate-600 peer-focus:outline-none rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-3 after:w-3 after:transition-all peer-checked:bg-primary"></div>
        </div>
    </label>
    `
};

// 3. 数字输入组件
export const NumberInput = {
    props: ['label', 'modelValue', 'unit', 'step', 'title'],
    emits: ['update:modelValue'],
    template: `
    <div :title="title">
        <label class="text-[10px] text-slate-500 dark:text-slate-400 block mb-1 ml-1 font-medium">{{ label }}</label>
        <div class="relative group">
            <input type="number" :step="step||1" :value="modelValue" 
                   @input="$emit('update:modelValue', parseFloat($event.target.value))" 
                   class="w-full bg-white dark:bg-slate-700 border border-slate-200 dark:border-slate-600 rounded-md px-2 py-1.5 text-xs text-right pr-7 
                          focus:outline-none focus:border-primary focus:ring-1 focus:ring-primary transition-all text-slate-700 dark:text-slate-200 font-mono">
            <span class="absolute right-2 top-1.5 text-[10px] text-slate-400 pointer-events-none group-hover:text-slate-500 transition-colors">{{ unit }}</span>
        </div>
    </div>
    `
};
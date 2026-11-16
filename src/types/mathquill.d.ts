declare module '@pie-framework/mathquill' {
  type MQField = {
    latex: (s?: string) => string;
    focus: () => void;
    blur: () => void;
  };

  type MQInterface = {
    StaticMath: (el: HTMLElement) => MQField;
    MathField: (el: HTMLElement, config?: Record<string, unknown>) => MQField;
  };

  type MQFactory = MQInterface & {
    getInterface: (version: number) => MQInterface;
  };

  const MathQuill: MQFactory;
  export default MathQuill;
}

declare module 'jquery' {
  const $: any;
  export default $;
}

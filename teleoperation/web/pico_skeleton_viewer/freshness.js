// Display freshness only; this never changes control input validity.
export function freshnessWarning(state, mode, paused) {
  if (paused) return "已暂停显示；点击“继续”恢复骨架更新。";
  const raw = mode === "raw" || mode === "robot";
  const age = raw ? state?.age_ms : state?.diagnostics_age_ms;
  const frames = raw ? state?.packet?.frames : state?.diagnostics?.frames?.[mode];
  if (!frames || !Number.isFinite(age)) {
    return raw ? "尚未收到 Body 骨架数据。" : "所选计算层尚无数据；启动预览并完成输入准备后才会更新。";
  }
  if (!raw && age > 500) {
    const seconds = (age / 1000).toFixed(1);
    return `所选计算层已 ${seconds} 秒未更新；画面保留最后一帧，原始／固定映射层可单独查看采集。`;
  }
  return "";
}

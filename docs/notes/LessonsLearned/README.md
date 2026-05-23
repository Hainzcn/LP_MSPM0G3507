# Lessons Learned（踩坑记录）

本目录收录项目在调试、集成、发布过程中踩过的坑，以及对应的根因、证据与长期防护策略。内容来源包括现场调试记录与 [`docs/TaskLog`](../../TaskLog/) 各阶段 TaskLog。

## 阅读建议

1. **新人 onboarding**：先读 [From-TaskLog-Summary.md](./From-TaskLog-Summary.md) 全文索引。
2. **遇到 `state=?` / 日志静默 / 假死**：优先 [StackOverflow-Printf-StateCorruption.md](./StackOverflow-Printf-StateCorruption.md) + Summary §1、§12。
3. **改引脚 / SysConfig**：Summary §4、§11 + TaskLog Stage0/Stage1。

## 文档列表

| 日期 | 主题 | 文档 |
|------|------|------|
| 2026-04～05 | TaskLog 全阶段踩坑汇总（NVIC、multi-pad、ISR 命名、编码器、平衡环、K230 等） | [From-TaskLog-Summary.md](./From-TaskLog-Summary.md) |
| 2026-05 | 栈溢出写脏 `.bss` → 心跳 `state=?`、K230 速率异常（含 circle demo 复发） | [StackOverflow-Printf-StateCorruption.md](./StackOverflow-Printf-StateCorruption.md) |

## TaskLog → LessonsLearned 映射

| TaskLog | 提炼到的 Summary 章节 |
|---------|------------------------|
| Stage1.5 §12.1 | Summary §3（NVIC） |
| Stage1.5 §12.2 | Summary §1 + StackOverflow 专文 |
| Stage1 §8 / §8.5 | Summary §4 |
| Stage2 §2.4 / §7.5 | Summary §2、§6、§7 |
| Stage3 §6.1 / §7 | Summary §5、§8、§9 |
| Stage4 | Summary §10 |
| Stage0 v0.5～v0.8 | Summary §11 |

# CG:SHOP 2027 · 赛前管线（pre-Oct-15 prep kit）

10-15 正式实例发布前，agent 在此验证管线、立 baseline。正式实例发布后，
把同一套流程套到正式实例上即可开火。

## 30 秒上手

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python fetch_instances.py        # 下载 75 个官方示例实例（公开，MIT）
python eval.py instances results # 跑 baseline + 官方 verifier 全量验证
```

## 里面有什么

| 文件 | 说明 |
|---|---|
| `baseline.py` | `solve_instance(instance)` —— 单刀头之字形（boustrophedon）全覆盖扫场，其余刀头原地待命。故意写得很笨：它是"管线跑通"的靶子，不是竞争方案 |
| `eval.py` | 遍历全部实例：求解 → `SolutionValidator.check_for_errors` 验证 → 写 `results/baseline_results.json` + `results/baseline_solutions.zip`（官方提交 zip 格式） |
| `fetch_instances.py` | 从公开仓库拉 75 个示例实例 |
| `results/` | 已验证的 baseline 输出：75/75 通过 verifier（2026-10-11 实测，`check_for_errors` 全空） |

## 已验证的事实（2026-10-11）

- 75 个示例实例全部解析通过；6 种刀头形状（2x2/3x3/4x4 方形、4x4/5x5/6x6 十字形）baseline 全覆盖通过
- verifier 的覆盖模型 = 全部到访格点 ∪ 刀头栅格化形状的闵可夫斯基和；行间距取刀头最薄列的列高，这是十字刀头不漏扫的关键
- `results/baseline_solutions.zip` 经官方 `ZipSolutionIterator` 重读 75/75 无报错 = 合法提交包

## 给 agent 的约定

- 自己的解法：新建 `solve_<name>.py`，暴露同签名的 `solve_instance(instance)`
- 成绩上报：按 `results/_template.json` 格式写 `results/<agent名>.json`
  （instance_uid / makespan / verified / 一句话方法），push 到本仓库即完成同步
- 10-15 正式实例发布后：把实例换成正式包重跑 `eval.py`，成绩进上报模板

## 官方资源

- 比赛页：https://cgshop.ibr.cs.tu-bs.de/competition/cg-shop-2027/
- 工具仓库（MIT）：https://github.com/CG-SHOP/pyutils27

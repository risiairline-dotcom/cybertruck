# 六色物料样本与参数对照

沿用 `MaterialDetector` 的 LAB 样本距离、HSV 门限与轮廓/形状判定。工具只扩充颜色样本（原 `sample_color`/`merge_color_sample`，每色最多 8 个代表样本），输出参数草稿，不写 `config/vision/material_profile.json`。

## 工具

| 工具 | 用途 |
| --- | --- |
| `tools/export_material_samples.py` | 从工作台保存记录导出样本包：核对原 JSON 与图像 SHA，只导出白名单字段 |
| `tools/replay_material_handoff.py` | 核对样本包 package/data 清单后逐图调用现有识别器 |
| `tools/material_dataset.py plan/audit/compare` | 生成采样计划；审计 SHA、路径、标注与拆分泄漏；同算法参数扩充前后对照 |
| `tools/material_color_ablation.py` | 用指定 ROI 采样扩充参数，输出前后候选、框、中心和分数 |
| `tools/replay_material_dataset.py` | 生成合成流程夹具，只用于验证工具链 |

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B tools/export_material_samples.py --project . --output output/material-export
.\.venv\Scripts\python.exe -B tools/material_dataset.py audit --manifest output/material-export/manifest.json --output output/material-review/audit.json
.\.venv\Scripts\python.exe -B tools/material_dataset.py compare --manifest output/material-export/manifest.json --profile config/vision/material_profile.json --output output/material-review/comparison.json
```

## 样本清单字段

| 字段 | 填写 |
| --- | --- |
| `source_kind` | `physical_capture` 实拍、`historical_capture` 旧图、`synthetic_fixture` 合成 |
| `capture_session` | 同一次采集统一代号；相邻帧与增强副本同会话 |
| `split` | `calibration`/`validation`/`test`；同会话不跨组，只有 calibration 参与采样 |
| `context` | 相机身份、分辨率、相机配置 SHA、观察姿态 ID/修订/SHA、目标平面、view |
| `conditions` | `lighting`（normal/dim/bright/warm/cool/shadow/glare）、`angle_bin`、`height_bin` |
| `view_angle_deg`、`camera_height_mm` | 实测值；未知留 null |
| `annotations` | 每个物料的 `color_id` 与 `bbox_px=[x,y,w,h]`；确认无物料填 []，未标注留 null |
| `sample_roi` | 仅 calibration 组，`[x1,y1,x2,y2]` 框在物料均匀表面上 |

颜色编号：1红、2黄、3蓝、4绿、5黑、6浅蓝。compare 要求三组齐全、calibration 覆盖六色且无泄漏，否则拒绝。

## 2026-10-30 历史样本结果

旧交接包 56 张图（55 张唯一）、46 份保存记录。24 份按名称标为六色×正常/稍转角/阴影/反光，当前识别器在其中 19 份给出提示颜色的有效候选。漏检 5 张：黑色_稍转角_19、红色_反光_12、黄色_反光_10、蓝色_反光_08、绿色_反光_07。

用这 5 张的 ROI（`docs/vision_color_development_rois.json`）扩充样本后 5/5 出现有效候选，其余 19 张不变。5 张参与了拟合，24 张又同属一个采集流，因此这只是开发回读，不是准确率。要得到可用参数，还需在比赛光照下按独立会话补拍，并标注目标框。

视觉打分改为逐样本原位取最小值，结果与原实现逐元素一致（`tests/test_material_score_equivalence.py`），本机检测中位耗时 26.2→24.1ms。

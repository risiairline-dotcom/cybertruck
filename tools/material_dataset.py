"""六色原算法的采样计划、样本审计和参数扩充前后对照；不启动服务或设备。"""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pi'))
from robot.vision.material_dataset import audit, compare, plan, canonical


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # 对照证据不可覆盖，重复执行请改输出名称。
    with path.open('x', encoding='utf-8') as handle:
        handle.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('plan', 'audit', 'compare'))
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--profile', type=Path, default=ROOT/'config/vision/material_profile.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--full-matrix', action='store_true')
    args = parser.parse_args()
    if args.action == 'plan':
        report = plan(args.full_matrix)
    else:
        if args.manifest is None:
            parser.error('audit/compare需要--manifest')
        if args.action == 'audit':
            _, report = audit(args.manifest, args.full_matrix)
        else:
            candidate_path = args.output.with_name(args.output.stem+'_profile_draft.json')
            if args.output.exists() or candidate_path.exists():
                raise ValueError('输出证据或草稿已存在，请选新名称')
            candidate, report = compare(args.manifest, args.profile)
            # 严格保持原profile格式；applied=false等来源元数据在报告中。
            candidate_path.parent.mkdir(parents=True, exist_ok=True)
            with candidate_path.open('x', encoding='utf-8') as handle:
                handle.write(canonical(candidate))
    write(args.output, report)
    print(json.dumps(dict(action=args.action, status=report.get('status', 'GENERATED'),
                          physical_acceptance=False, output=str(args.output)), ensure_ascii=False))
    return 1 if report.get('status') == 'INVALID' else 0


if __name__ == '__main__':
    raise SystemExit(main())

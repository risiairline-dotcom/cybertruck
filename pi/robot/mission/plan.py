"""比赛任务的线性步骤表：任务顺序固定，每段路径由执行层按工位规划。"""

from ..vision.task_code import parse_task


def item_id(batch, slot):
    return f'b{batch}-{slot}'


def build_steps(code, start_zone, batches=(1, 2)):
    """按任务码生成两批取放步骤；车载料位号即本批搬运顺序号。

    第二批暂存区放置层为2（码垛），执行时若同色第一层缺失再降为平面放置。
    """
    if start_zone not in (1, 2):
        raise ValueError('启停区必须为1或2')
    if not batches or any(batch not in (1, 2) for batch in batches) or list(batches) != sorted(set(batches)):
        raise ValueError('批次只能是1、2的递增子集')
    task = parse_task(code)
    steps = []
    for batch in batches:
        items = task[f'batch{batch}']
        steps.append(dict(kind='goto', station='raw', batch=batch))
        for item in items:
            steps.append(dict(kind='pick_raw', batch=batch, slot=item['order'], color=item['color_id']))
        steps.append(dict(kind='goto', station='rough', batch=batch))
        for item in items:
            steps.append(dict(kind='place', batch=batch, slot=item['order'], area='rough',
                              ring=item['rough_processing_ring'], layer=1))
        for item in items:
            steps.append(dict(kind='pick_ring', batch=batch, slot=item['order'], area='rough',
                              ring=item['rough_processing_ring']))
        steps.append(dict(kind='goto', station='stage', batch=batch))
        for item in items:
            steps.append(dict(kind='place', batch=batch, slot=item['order'], area='stage',
                              ring=item['temporary_storage_ring'], layer=batch))
    steps.append(dict(kind='goto', station=f'start:{start_zone}', batch=0))
    return task, steps

"""Explain existing solver gates without promoting a candidate to calibration."""
import math


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def assess_result(result):
    assessment = {'level': 'NOT_EVALUATED', 'summary': '尚无可判定的配准结果。',
                  'details': [], 'usable_for': [], 'formal_use': False}
    if not isinstance(result, dict):
        return assessment
    details = assessment['details']
    if result.get('kind') == 'manual_correspondence_initial':
        rejected = bool(result.get('rejection_reasons')) or result.get('status') != 'CANDIDATE'
        assessment.update(level='REJECTED' if rejected else 'EXPLORATORY',
            summary=('这组人工对应点未通过现有残差门限，不建议用于融合。' if rejected else
                     '人工拟合通过候选门限，可作离线配准初值；尚未完成独立验证。'),
            usable_for=['检查对应点和残差'] if rejected else ['离线配准初值', '独立场景复核'])
        rmse, maximum = result.get('rmse_m'), result.get('max_residual_m')
        if _number(rmse) and _number(maximum):
            details.append('人工点对 RMSE %.2f cm，最大残差 %.2f cm。' % (100*rmse, 100*maximum))
        limit = result.get('policy', {}).get('max_pair_residual_m')
        if _number(limit):
            residuals = result.get('residuals_m', [])
            count = sum(_number(v) and v > limit for v in residuals)
            details.append('%d/%d 对超过 %.2f cm 候选门限；该门限不是实测精度证明。' % (count, len(residuals), 100*limit))
        if result.get('rejection_reasons'):
            details.append('拒绝原因：' + '、'.join(result['rejection_reasons']))
        details.append('刚体变换保持距离，不能用旋转和平移消除不同点对的形状或尺度不一致。')
    elif 'final_metrics' in result:
        candidate = result.get('status') == 'CANDIDATE'
        manual = result.get('status') == 'MANUAL_UNVALIDATED'
        assessment.update(level='EXPLORATORY' if candidate or manual else 'REJECTED',
            summary=('形成离线候选，可继续检查，尚不能判定正式可用。' if candidate else
                     '当前为手动调整的未验证位置，仅供离线检查。' if manual else
                     '此次配准未通过候选检查，不建议用于融合。'),
            usable_for=['离线叠加检查', '独立场景复核'] if candidate or manual else ['排查配准失败原因'])
        metrics = result.get('final_metrics') or {}
        # Field names are the existing preview_icp contract, not new accuracy gates.
        for key, label in (('forward_overlap_fraction', '右到左重合率'),
                           ('reverse_overlap_fraction', '左到右重合率')):
            if _number(metrics.get(key)):
                details.append('%s %.2f%%。' % (label, 100*metrics[key]))
        for key, label in (('nn_rmse_m', '最近邻 RMSE'), ('nn_p95_m', '最近邻 P95')):
            if _number(metrics.get(key)):
                details.append('%s %.2f cm。' % (label, 100*metrics[key]))
        change = result.get('transform_change') or {}
        if _number(change.get('translation_m')) and _number(change.get('rotation_deg')):
            details.append('相对输入初值改变 %.2f cm、%.2f°。' % (100*change['translation_m'], change['rotation_deg']))
        if result.get('reasons'):
            details.append('算法说明：' + '、'.join(result['reasons']))
        details.append('最近邻指标只统计距离门内的匹配，不是独立对应点误差，也不代表真实定位精度。')
    details.append('计算和保存都不会启用正式外参；实时建图与录包融合配置保持原状。')
    return assessment

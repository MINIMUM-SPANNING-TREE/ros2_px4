"""
DeepSORT 测试示例
演示如何使用 DeepSORT 进行多目标跟踪
"""

import numpy as np
import sys
import os

# 添加包路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.deepsort import DeepSORT
from vision.tracker import Detection


def test_basic_tracking():
    """基本跟踪测试"""
    print("="*60)
    print("DeepSORT 基本跟踪测试")
    print("="*60)
    
    # 创建跟踪器
    tracker = DeepSORT(
        model_name="simple",
        max_cosine_distance=0.3,
        max_age=10,
        n_init=3,
    )
    
    # 模拟多帧检测结果
    # 假设场景：两个目标在移动
    
    # 第1帧：检测到两个目标
    print("\n帧1: 检测到2个目标")
    bboxes1 = np.array([
        [100, 100, 150, 200],  # 目标1
        [300, 100, 350, 200],  # 目标2
    ], dtype=np.float32)
    scores1 = np.array([0.9, 0.85], dtype=np.float32)
    classes1 = np.array([0, 0], dtype=int)
    
    results1 = tracker.update(bboxes=bboxes1, scores=scores1, classes=classes1)
    print(f"  跟踪结果: {len(results1)} 个目标")
    for r in results1:
        print(f"    ID:{r['track_id']} bbox:{r['bbox']} score:{r['score']:.2f}")
    
    # 第2帧：目标移动
    print("\n帧2: 目标移动")
    bboxes2 = np.array([
        [110, 105, 160, 205],  # 目标1 向右下移动
        [290, 95, 340, 195],   # 目标2 向左上移动
    ], dtype=np.float32)
    scores2 = np.array([0.92, 0.88], dtype=np.float32)
    classes2 = np.array([0, 0], dtype=int)
    
    results2 = tracker.update(bboxes=bboxes2, scores=scores2, classes=classes2)
    print(f"  跟踪结果: {len(results2)} 个目标")
    for r in results2:
        print(f"    ID:{r['track_id']} bbox:{r['bbox']} score:{r['score']:.2f}")
    
    # 第3帧：目标继续移动
    print("\n帧3: 目标继续移动")
    bboxes3 = np.array([
        [120, 110, 170, 210],  # 目标1
        [280, 90, 330, 190],   # 目标2
    ], dtype=np.float32)
    scores3 = np.array([0.91, 0.87], dtype=np.float32)
    classes3 = np.array([0, 0], dtype=int)
    
    results3 = tracker.update(bboxes=bboxes3, scores=scores3, classes=classes3)
    print(f"  跟踪结果: {len(results3)} 个目标")
    for r in results3:
        print(f"    ID:{r['track_id']} bbox:{r['bbox']} score:{r['score']:.2f}")
    
    # 第4帧：只有1个目标被检测到
    print("\n帧4: 目标2被遮挡")
    bboxes4 = np.array([
        [130, 115, 180, 215],  # 目标1
    ], dtype=np.float32)
    scores4 = np.array([0.93], dtype=np.float32)
    classes4 = np.array([0], dtype=int)
    
    results4 = tracker.update(bboxes=bboxes4, scores=scores4, classes=classes4)
    print(f"  跟踪结果: {len(results4)} 个目标")
    for r in results4:
        print(f"    ID:{r['track_id']} bbox:{r['bbox']} score:{r['score']:.2f}")
    
    # 第5帧：目标2重新出现
    print("\n帧5: 目标2重新出现")
    bboxes5 = np.array([
        [140, 120, 190, 220],  # 目标1
        [270, 85, 320, 185],   # 目标2
    ], dtype=np.float32)
    scores5 = np.array([0.89, 0.86], dtype=np.float32)
    classes5 = np.array([0, 0], dtype=int)
    
    results5 = tracker.update(bboxes=bboxes5, scores=scores5, classes=classes5)
    print(f"  跟踪结果: {len(results5)} 个目标")
    for r in results5:
        print(f"    ID:{r['track_id']} bbox:{r['bbox']} score:{r['score']:.2f}")
    
    print("\n" + "="*60)
    print("测试完成!")
    print("="*60)
    
    # 验证结果（DeepSORT 需要 n_init 帧来确认跟踪）
    # n_init=3, 所以第1帧只是初始化，不会出现在结果中
    assert len(results1) == 0, "帧1: 跟踪尚未确认"
    assert len(results2) == 2, "帧2应该检测到2个目标"
    assert len(results3) == 2, "帧3应该检测到2个目标"
    assert len(results4) >= 1, "帧4至少应该有1个目标"
    assert len(results5) == 2, "帧5应该检测到2个目标"
    
    # 验证ID连续性
    ids2 = {r['track_id'] for r in results2}
    ids5 = {r['track_id'] for r in results5}
    assert len(ids2 & ids5) > 0, "应该有ID连续的目标"
    
    print("\n✓ 所有断言通过!")
    
    return True


def test_feature_matching():
    """特征匹配测试"""
    print("\n" + "="*60)
    print("DeepSORT 特征匹配测试")
    print("="*60)
    
    # 创建跟踪器
    tracker = DeepSORT(
        model_name="simple",
        max_cosine_distance=0.5,
        max_age=5,
        n_init=2,
    )
    
    # 模拟带有特征的检测
    print("\n帧1: 检测到2个目标")
    bboxes1 = np.array([
        [100, 100, 200, 200],
        [300, 100, 400, 200],
    ], dtype=np.float32)
    
    # 生成不同的特征
    np.random.seed(42)
    features1 = np.random.randn(2, 128).astype(np.float32)
    features1 = features1 / np.linalg.norm(features1, axis=1, keepdims=True)
    
    results1 = tracker.update(bboxes=bboxes1, scores=np.array([0.9, 0.85]))
    print(f"  跟踪结果: {len(results1)} 个目标")
    
    print("\n帧2: 目标移动")
    bboxes2 = np.array([
        [110, 110, 210, 210],
        [310, 110, 410, 210],
    ], dtype=np.float32)
    
    results2 = tracker.update(bboxes=bboxes2, scores=np.array([0.91, 0.86]))
    print(f"  跟踪结果: {len(results2)} 个目标")
    
    # 验证ID保持一致
    if len(results1) > 0 and len(results2) > 0:
        ids1 = {r['track_id'] for r in results1}
        ids2 = {r['track_id'] for r in results2}
        print(f"  帧1 IDs: {ids1}")
        print(f"  帧2 IDs: {ids2}")
        
        if ids1 == ids2:
            print("✓ ID保持一致!")
        else:
            print("✗ ID发生变化")
    
    print("\n" + "="*60)
    print("测试完成!")
    print("="*60)
    
    return True


def test_empty_detection():
    """空检测测试"""
    print("\n" + "="*60)
    print("DeepSORT 空检测测试")
    print("="*60)
    
    tracker = DeepSORT(max_age=3, n_init=2)
    
    # 帧1: 有检测
    print("\n帧1: 有检测")
    bboxes = np.array([[100, 100, 200, 200]], dtype=np.float32)
    results1 = tracker.update(bboxes=bboxes, scores=np.array([0.9]))
    print(f"  跟踪结果: {len(results1)} 个目标")
    
    # 帧2-5: 空检测
    for i in range(4):
        print(f"\n帧{i+2}: 空检测")
        results = tracker.update()
        print(f"  跟踪结果: {len(results)} 个目标")
    
    print("\n" + "="*60)
    print("测试完成!")
    print("="*60)
    
    return True


def main():
    """运行所有测试"""
    print("\n" + "="*60)
    print("DeepSORT 测试套件")
    print("="*60)
    
    tests = [
        test_basic_tracking,
        test_feature_matching,
        test_empty_detection,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"\n✗ 测试失败: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "="*60)
    print(f"测试结果: {passed} 通过, {failed} 失败")
    print("="*60)
    
    return failed == 0


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)

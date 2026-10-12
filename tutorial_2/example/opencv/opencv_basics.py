#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# 一、OpenCV（1.1 cv::Mat、1.3 图像处理基本流程）
#
#   pip install opencv-python numpy
#   python3 opencv_basics.py                    # 默认读 demo/bule_armoe.jpg
#   python3 opencv_basics.py demo/blue_5.png    # 也可以喂自己的图
#   NO_WINDOW=1 python3 opencv_basics.py        # 不开窗口，只存图
#
import os
import sys
import time

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# 几个小工具（真正要讲的 API 在 main() 里）
# ---------------------------------------------------------------------------
def section(index, title):
    print(f"\n【{index}】{title}")


def fit_into(src, area_w, area_h):
    if src.ndim == 2:  # 单通道（灰度 / 掩码）先转成三通道再拼
        src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    s = min(area_w / src.shape[1], area_h / src.shape[0])
    small = cv2.resize(src, (max(1, int(src.shape[1] * s)), max(1, int(src.shape[0] * s))),
                       interpolation=cv2.INTER_AREA)
    canvas = np.zeros((area_h, area_w, 3), np.uint8)
    x = (area_w - small.shape[1]) // 2
    y = (area_h - small.shape[0]) // 2
    canvas[y:y + small.shape[0], x:x + small.shape[1]] = small
    return canvas


def grid(items, cols=3):
    area_w, area_h, bar = 420, 300, 40
    cells = []
    for title, img in items:
        cell = np.full((area_h + bar, area_w, 3), 18, np.uint8)
        cell[bar:, :, :] = fit_into(img, area_w, area_h)
        cv2.putText(cell, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cells.append(cell)
    while len(cells) % cols:
        cells.append(np.full((area_h + bar, area_w, 3), 18, np.uint8))
    rows = [cv2.hconcat(cells[i:i + cols]) for i in range(0, len(cells), cols)]
    return cv2.vconcat(rows)


def white_ratio(m):
    """白像素占比，用来量化二值化 / 边缘的结果"""
    return 100.0 * cv2.countNonZero(m) / m.size


def mean_abs_diff(ref, m):
    """与原图的平均绝对差：越大说明滤波抹得越狠"""
    return float(cv2.mean(cv2.absdiff(ref, m))[0])


class Timer:
    def __init__(self):
        self.t0 = time.perf_counter()

    def ms(self):
        return (time.perf_counter() - self.t0) * 1000.0


def main():
    # -----------------------------------------------------------------------
    section(1, "读图与图像元信息")

    path = sys.argv[1] if len(sys.argv) > 1 else "demo/opencv_basics_1_filter.jpg"
    t_all = Timer()
    img = cv2.imread(path)
    if img is None:  # imread 失败不报错，只返回 None，必须自己判断
        print(f"读不到图片：{path}")
        return 1

    h, w = img.shape[:2]
    print(f"  路径       {path}")
    print(f"  shape      {img.shape}（高, 宽, 通道）")
    print(f"  dtype      {img.dtype}")
    print(f"  size       {img.size} 个元素，占内存 {img.nbytes / 1024:.0f} KB")

    # -----------------------------------------------------------------------
    # ② cvtColor：彩色 → 灰度
    # -----------------------------------------------------------------------
    section(2, "灰度化 cvtColor")

    t = Timer()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(f"  cvtColor(BGR2GRAY) 耗时 {t.ms():.2f} ms，灰度图形状 {gray.shape}")
    print(f"  gray 均值 {gray.mean():.1f}，标准差 {gray.std():.1f}")

    # -----------------------------------------------------------------------
    # ③ 滤波：都用来去噪，区别在「保不保边」
    # -----------------------------------------------------------------------
    section(3, "滤波去噪 blur / GaussianBlur / medianBlur / bilateralFilter")

    t = Timer()
    f_blur = cv2.blur(gray, (5, 5))                       # 均值：最快，边缘一起糊
    f_gauss = cv2.GaussianBlur(gray, (5, 5), 0)           # 高斯：最常用
    f_median = cv2.medianBlur(gray, 5)                    # 中值：专杀椒盐噪声
    f_bilateral = cv2.bilateralFilter(gray, 9, 75, 75)    # 双边：保边去噪，最慢
    print(f"  四种滤波合计 {t.ms():.2f} ms。mean|diff| 越大说明抹得越狠：")
    for name, m in [("blur 5x5", f_blur), ("GaussianBlur 5x5", f_gauss),
                    ("medianBlur 5", f_median), ("bilateralFilter", f_bilateral)]:
        print(f"    {name:<24} mean|diff|={mean_abs_diff(gray, m):5.2f}")

    # -----------------------------------------------------------------------
    # ④ 二值化：自瞄里「找灯条」的第一步
    # -----------------------------------------------------------------------
    section(4, "二值化 threshold / THRESH_OTSU / adaptiveThreshold")

    _, b_fixed = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)  # 阈值手填，换光照就废
    otsu_thr, b_otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    b_adapt_mean = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY, 31, 5)
    b_adapt_gauss = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv2.THRESH_BINARY, 31, 5)

    print(f"  OTSU 自动算出的阈值 = {otsu_thr:.0f}（本次固定阈值用的是 150）")
    print(f"  白像素占比：固定 {white_ratio(b_fixed):.2f}%   OTSU {white_ratio(b_otsu):.2f}%"
          f"   自适应均值 {white_ratio(b_adapt_mean):.2f}%   自适应高斯 {white_ratio(b_adapt_gauss):.2f}%")

    # -----------------------------------------------------------------------
    # ⑤ 颜色分割：BGR 里红色是 (0,0,255)、蓝色是 (255,0,0)，按颜色筛一律走 HSV
    # -----------------------------------------------------------------------
    section(5, "颜色分割 cvtColor(BGR2HSV) + inRange")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.bitwise_or(                              # 红色跨 0 度，要分两段
        cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)),
        cv2.inRange(hsv, (160, 100, 100), (180, 255, 255)))
    blue_mask = cv2.inRange(hsv, (100, 100, 100), (124, 255, 255))
    red_only = cv2.bitwise_and(img, img, mask=red_mask)      # 用掩码把原图抠出来
    blue_only = cv2.bitwise_and(img, img, mask=blue_mask)

    print(f"  red 掩码白像素 {white_ratio(red_mask):.2f}%   blue 掩码白像素 {white_ratio(blue_mask):.2f}%")

    # -----------------------------------------------------------------------
    # ⑥ 形态学：在二值图上修形状
    # -----------------------------------------------------------------------
    section(6, "形态学 morphologyEx：开 / 闭 / 梯度 / 顶帽")

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    m_open = cv2.morphologyEx(b_otsu, cv2.MORPH_OPEN, kernel)       # 去小噪点
    m_close = cv2.morphologyEx(b_otsu, cv2.MORPH_CLOSE, kernel)     # 补空洞
    m_grad = cv2.morphologyEx(b_otsu, cv2.MORPH_GRADIENT, kernel)   # 膨胀-腐蚀 = 轮廓
    m_tophat = cv2.morphologyEx(b_otsu, cv2.MORPH_TOPHAT, kernel)   # 原图-开运算 = 小亮块

    n_otsu, n_open, n_close = (int(cv2.countNonZero(x)) for x in (b_otsu, m_open, m_close))
    print(f"  白像素个数：OTSU {n_otsu} → 开运算 {n_open}（少了 {n_otsu - n_open} 个孤立噪点）"
          f" → 闭运算 {n_close}（多了 {n_close - n_otsu} 个补上的空洞）")

    # -----------------------------------------------------------------------
    # ⑦ 边缘检测：Sobel（方向）/ Laplacian（二阶）/ Canny（最常用）
    # -----------------------------------------------------------------------
    section(7, "边缘检测 Sobel / Laplacian / Canny")

    sx = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3))  # 横向差分 → 竖边缘
    sy = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=3))  # 纵向差分 → 横边缘
    sxy = cv2.addWeighted(sx, 0.5, sy, 0.5, 0)                            # 两个方向合起来看
    lap = cv2.convertScaleAbs(cv2.Laplacian(gray, cv2.CV_16S, ksize=3))
    canny_lo = cv2.Canny(gray, 50, 150)     # 双阈值：小的是起步价，大的是确认线
    canny_hi = cv2.Canny(gray, 100, 200)    # 阈值调高 → 边缘变少但更干净

    print(f"  平均梯度强度：Sobel-|dx| {sx.mean():.1f}  Sobel-|dy| {sy.mean():.1f}"
          f"  Laplacian {lap.mean():.1f}")
    print(f"  二值边缘（Canny）像素占比：阈值(50,150) → {white_ratio(canny_lo):.2f}%"
          f"   阈值(100,200) → {white_ratio(canny_hi):.2f}%")

    # -----------------------------------------------------------------------
    # ⑧ 轮廓：findContours 拿点集，再按面积 / 长宽比筛
    #    ⚠ findContours 会修改输入图，习惯先 copy
    # -----------------------------------------------------------------------
    section(8, "轮廓 findContours + 面积/外接矩形筛选")

    contours, _ = cv2.findContours(m_open.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)   # 按面积从大到小

    min_area = img.shape[0] * img.shape[1] * 0.002    # 面积阈值写成「相对整图」的比例
    contour_vis = img.copy()
    kept = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        kept += 1
        bx, by, bw, bh = cv2.boundingRect(c)              # 正矩形：快
        (cx, cy), (rw, rh), angle = cv2.minAreaRect(c)    # 旋转矩形：贴合斜灯条
        if kept <= 5:                                     # 只打印最大的 5 个
            ratio = max(rw, rh) / max(min(rw, rh), 1.0)
            print(f"    #{kept} 面积={area:.0f} 正矩形={bw}×{bh} 旋转矩形={rw:.0f}×{rh:.0f}"
                  f" 角度={angle:.1f}° 长宽比={ratio:.2f}")
        cv2.drawContours(contour_vis, [c], -1, (0, 255, 0), 2)
        cv2.rectangle(contour_vis, (bx, by), (bx + bw, by + bh), (255, 160, 0), 1)
        cv2.circle(contour_vis, (int(cx), int(cy)), 4, (0, 0, 255), -1)

    print(f"  findContours 共 {len(contours)} 个外轮廓，面积 ≥ {min_area:.0f}px 的留下 {kept} 个")
    print("  提示：minAreaRect 的返回值顺序是 (中心点, (宽, 高), 角度)，角度约定在 4.5 之后变过")

    # -----------------------------------------------------------------------
    # ⑨ 几何变换：改变图的大小 / 角度 / 视角
    # -----------------------------------------------------------------------
    section(9, "几何变换 resize / 旋转 / 透视")

    half = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)  # 缩小用 AREA
    rot_mat = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), 15.0, 1.0)            # 2×3 变换矩阵
    rotated = cv2.warpAffine(img, rot_mat, (w, h))
    src_quad = np.float32([[w * 0.30, h * 0.25], [w * 0.70, h * 0.30],
                           [w * 0.75, h * 0.75], [w * 0.25, h * 0.70]])
    dst_quad = np.float32([[0, 0], [400, 0], [400, 300], [0, 300]])
    persp_mat = cv2.getPerspectiveTransform(src_quad, dst_quad)                 # 3×3 透视矩阵
    warped = cv2.warpPerspective(img, persp_mat, (400, 300))

    print(f"  resize 0.5× → {half.shape[1]}×{half.shape[0]}（原图 {w}×{h}）")
    print("  旋转 15°：getRotationMatrix2D 给 2×3 矩阵，warpAffine 的第 3 个参数决定输出尺寸")
    print("  透视变换：把源图里那个四边形拉正成 400×300（第 ⑩ 步画出来了）")

    # -----------------------------------------------------------------------
    # ⑩ 绘制：调试图一定要画出来才看得懂
    # -----------------------------------------------------------------------
    section(10, "绘制 line / rectangle / circle / putText")

    canvas = img.copy()
    cv2.rectangle(canvas, (60, 60), (360, 260), (0, 255, 0), 3)                     # 绿框
    cv2.circle(canvas, (600, 300), 40, (0, 0, 255), -1)                             # 红实心圆
    cv2.line(canvas, (100, 400), (700, 500), (255, 0, 0), 3)                        # 蓝线
    cv2.arrowedLine(canvas, (900, 300), (1100, 200), (0, 255, 255), 3)              # 黄箭头
    cv2.polylines(canvas, [src_quad.astype(np.int32)], True, (255, 0, 255), 3)      # 洋红四边形
    cv2.putText(canvas, "line / rect / circle / putText", (60, h - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    print("  文字和线宽建议随图尺寸缩放，否则换分辨率就看不见了")

    # -----------------------------------------------------------------------
    # 存三张拼图 + 开窗显示
    # -----------------------------------------------------------------------
    sheet1 = grid([("orig(gray)", gray), ("blur 5x5", f_blur), ("GaussianBlur 5x5", f_gauss),
                   ("medianBlur 5", f_median), ("bilateralFilter", f_bilateral),
                   ("threshold 150", b_fixed), (f"OTSU {otsu_thr:.0f}", b_otsu),
                   ("adaptive MEAN 31", b_adapt_mean), ("adaptive GAUSS 31", b_adapt_gauss)])
    sheet2 = grid([("OTSU", b_otsu), ("MORPH_OPEN", m_open), ("MORPH_CLOSE", m_close),
                   ("MORPH_GRADIENT", m_grad), ("MORPH_TOPHAT", m_tophat),
                   ("Sobel |dx|", sx), ("Sobel |dy|", sy), ("Sobel mix", sxy),
                   ("Laplacian", lap)])
    sheet3 = grid([("red mask", red_mask), ("red only", red_only),
                   ("blue mask", blue_mask), ("blue only", blue_only),
                   ("contours", contour_vis), ("resize 0.5x", half),
                   ("rotate 15deg", rotated), ("perspective", warped), ("draw", canvas)])

    for name, sheet in [("opencv_basics_py_1_filter.png", sheet1),
                        ("opencv_basics_py_2_edge.png", sheet2),
                        ("opencv_basics_py_3_misc.png", sheet3)]:
        if not cv2.imwrite(name, sheet):
            print(f"拼图保存失败：{name}（检查当前目录是否可写）")

    print(f"\n总耗时 {t_all.ms():.2f} ms")
    print("已保存 opencv_basics_py_1_filter.png / _2_edge.png / _3_misc.png")

    if os.environ.get("NO_WINDOW") is None:
        cv2.imshow("1 filter + threshold", sheet1)
        cv2.imshow("2 morphology + edge", sheet2)
        cv2.imshow("3 color + contours + warp", sheet3)
        print("按任意键退出…")
        cv2.waitKey(0)       # 0 = 一直等按键
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())

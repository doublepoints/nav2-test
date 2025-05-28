#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

class HeightRangeAnalyzer(Node):
    """
    分析Livox点云数据的高度分布，帮助确定最佳的min_height和max_height参数
    包含地面检测和过滤功能
    """
    
    def __init__(self):
        super().__init__('height_range_analyzer')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )
        
        self.z_values = []
        self.xy_coordinates = []  # 存储x,y坐标用于地面检测
        self.sample_count = 0
        self.max_samples = 500000  # 减少到50万个点以提高处理速度
        
        # 地面检测参数
        self.lidar_height = 1.24  # 雷达距地面高度(米)
        self.ground_tolerance = 0.1  # 地面检测容差(米)
        
        self.get_logger().info("高度范围分析器启动 (包含地面过滤)")
        self.get_logger().info("正在分析点云高度分布...")
        self.get_logger().info(f"雷达安装高度: {self.lidar_height}m")
        self.get_logger().info("倒挂安装: 地面应该在Z轴负方向")
        self.get_logger().info("将自动检测和排除地面点...")
        
    def pointcloud_callback(self, msg):
        """收集点云高度数据"""
        if len(self.z_values) >= self.max_samples:
            return
            
        try:
            for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                x, y, z = point
                
                # 应用倒挂修正: z' = -z, y' = -y, x不变
                z_corrected = -z
                y_corrected = -y
                x_corrected = x
                
                self.z_values.append(z_corrected)
                self.xy_coordinates.append((x_corrected, y_corrected))
                
                if len(self.z_values) >= self.max_samples:
                    break
            
            self.sample_count += 1
            
            if self.sample_count % 10 == 0:
                self.get_logger().info(f"已收集 {len(self.z_values)} 个点的高度数据...")
                
            # 当收集足够数据时，进行分析
            if len(self.z_values) >= self.max_samples:
                self.analyze_height_distribution()
                
        except Exception as e:
            self.get_logger().error(f"处理点云数据时出错: {str(e)}")
    
    def analyze_height_distribution(self):
        """分析高度分布并给出建议"""
        z_array = np.array(self.z_values)
        xy_array = np.array(self.xy_coordinates)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("高度分布分析结果 (修正后坐标系)")
        self.get_logger().info("="*60)
        
        # 步骤1: 地面检测和过滤
        ground_mask, ground_height = self.detect_ground_points(z_array, xy_array)
        non_ground_z = z_array[~ground_mask]
        
        self.get_logger().info(f"地面检测结果:")
        self.get_logger().info(f"  估计地面高度: {ground_height:.3f}m")
        self.get_logger().info(f"  地面点数量: {np.sum(ground_mask)} ({np.sum(ground_mask)/len(z_array)*100:.1f}%)")
        self.get_logger().info(f"  非地面点数量: {len(non_ground_z)} ({len(non_ground_z)/len(z_array)*100:.1f}%)")
        
        # 步骤2: 分析原始数据和过滤后数据
        self.get_logger().info(f"\n原始数据统计 (包含地面):")
        self.print_basic_statistics(z_array, "原始")
        
        self.get_logger().info(f"\n过滤后数据统计 (排除地面):")
        if len(non_ground_z) > 0:
            self.print_basic_statistics(non_ground_z, "过滤后")
        else:
            self.get_logger().warning("警告: 过滤后没有剩余数据点!")
            return
        
        # 步骤3: 基于过滤后数据进行推荐
        recommended = self.generate_recommendations(non_ground_z, ground_height)
        
        # 步骤4: 保存分析结果
        try:
            self.save_analysis_plot(z_array, non_ground_z, ground_mask, ground_height, recommended)
            self.save_text_report(z_array, non_ground_z, ground_height, recommended)
        except Exception as e:
            self.get_logger().warn(f"无法保存分析图表: {str(e)}")
        
        # 结束节点
        self.get_logger().info("分析完成，节点即将关闭...")
        rclpy.shutdown()
    
    def detect_ground_points(self, z_array, xy_array):
        """
        检测地面点
        返回: (ground_mask, estimated_ground_height)
        """
        # 方法1: 基于高度分布的地面检测
        # 倒挂雷达，地面应该在负Z方向，大约在-1.24m附近
        expected_ground_z = -self.lidar_height
        
        # 寻找最低的数据密集区域作为地面
        hist, bin_edges = np.histogram(z_array, bins=200)
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
        
        # 只考虑预期地面区域附近的直方图
        ground_region_mask = (bin_centers < expected_ground_z + 0.3) & (bin_centers > expected_ground_z - 0.3)
        if np.any(ground_region_mask):
            ground_region_hist = hist[ground_region_mask]
            ground_region_centers = bin_centers[ground_region_mask]
            
            # 找到密度最高的区域作为地面
            max_density_idx = np.argmax(ground_region_hist)
            estimated_ground_height = ground_region_centers[max_density_idx]
        else:
            # 如果没有找到预期区域，使用最低点附近的密集区域
            bottom_10_percent = np.percentile(z_array, 10)
            estimated_ground_height = bottom_10_percent
        
        # 方法2: 结合距离信息的地面检测
        # 远距离的点更可能是地面
        distances = np.sqrt(xy_array[:, 0]**2 + xy_array[:, 1]**2)
        far_points_mask = distances > 3.0  # 距离大于3米的点
        
        if np.any(far_points_mask):
            far_z_values = z_array[far_points_mask]
            far_ground_height = np.percentile(far_z_values, 10)  # 远距离点的低10%高度
            
            # 如果远距离地面高度合理，使用它来精细化地面高度估计
            if abs(far_ground_height - expected_ground_z) < 0.5:
                estimated_ground_height = (estimated_ground_height + far_ground_height) / 2
        
        self.get_logger().info(f"地面检测: 预期地面高度 {expected_ground_z:.3f}m, 估计地面高度 {estimated_ground_height:.3f}m")
        
        # 创建地面点掩码
        ground_tolerance = self.ground_tolerance
        ground_mask = z_array < (estimated_ground_height + ground_tolerance)
        
        # 进一步优化：排除明显的高处地面点（可能是误检）
        # 如果点既在地面高度范围内，又距离较近，更可能是真正的地面
        for i in range(len(z_array)):
            if ground_mask[i]:
                distance = distances[i] if i < len(distances) else 0
                height_diff = abs(z_array[i] - estimated_ground_height)
                
                # 近距离点要求更严格的高度匹配
                if distance < 2.0 and height_diff > 0.05:
                    ground_mask[i] = False
                # 远距离点允许更大的高度误差
                elif distance >= 2.0 and height_diff > 0.15:
                    ground_mask[i] = False
        
        return ground_mask, estimated_ground_height
    
    def print_basic_statistics(self, z_array, label):
        """打印基本统计信息"""
        self.get_logger().info(f"{label}数据基本统计:")
        self.get_logger().info(f"  总点数: {len(z_array):,}")
        self.get_logger().info(f"  高度范围: [{z_array.min():.3f}m, {z_array.max():.3f}m]")
        self.get_logger().info(f"  平均高度: {z_array.mean():.3f}m")
        self.get_logger().info(f"  高度标准差: {z_array.std():.3f}m")
        self.get_logger().info(f"  高度中位数: {np.percentile(z_array, 50):.3f}m")
        
    def generate_recommendations(self, filtered_z_array, ground_height):
        """基于过滤后的数据生成推荐设置"""
        total_points = len(filtered_z_array)
        
        if total_points == 0:
            return None
            
        # 计算关键统计数据 (基于过滤后的数据)
        q5 = np.percentile(filtered_z_array, 5)
        q10 = np.percentile(filtered_z_array, 10)
        q25 = np.percentile(filtered_z_array, 25)
        q50 = np.percentile(filtered_z_array, 50)  # 中位数
        q75 = np.percentile(filtered_z_array, 75)
        q90 = np.percentile(filtered_z_array, 90)
        q95 = np.percentile(filtered_z_array, 95)
        mean = filtered_z_array.mean()
        std = filtered_z_array.std()
        
        self.get_logger().info(f"\n过滤后数据的百分位数:")
        percentiles = [5, 10, 25, 50, 75, 90, 95]
        for p in percentiles:
            value = np.percentile(filtered_z_array, p)
            self.get_logger().info(f"  {p:2d}%: {value:6.3f}m")
        
        # 基于过滤后数据的推荐设置
        self.get_logger().info(f"\n" + "="*60)
        self.get_logger().info("基于过滤后数据的推荐高度范围设置:")
        self.get_logger().info("="*60)
        
        # 方案1: 基于四分位数范围 (IQR) - 排除地面后更准确
        iqr = q75 - q25
        iqr_min = max(ground_height + 0.05, q25 - 0.5 * iqr)  # 确保不包含地面
        iqr_max = q75 + 0.5 * iqr
        iqr_count = np.sum((filtered_z_array >= iqr_min) & (filtered_z_array <= iqr_max))
        iqr_pct = (iqr_count / total_points) * 100
        
        self.get_logger().info(f"方案1 - 基于四分位数范围(IQR)的推荐:")
        self.get_logger().info(f"  数据分析: Q25={q25:.3f}, Q75={q75:.3f}, IQR={iqr:.3f}")
        self.get_logger().info(f"  min_height: {iqr_min:.3f} (确保高于地面)")
        self.get_logger().info(f"  max_height: {iqr_max:.3f}")
        self.get_logger().info(f"  保留点数: {iqr_count} ({iqr_pct:.1f}%)")
        
        # 方案2: 基于标准差 - 以中位数为中心
        std_min = max(ground_height + 0.05, q50 - 1.0 * std)  # 以中位数为中心，避免地面影响
        std_max = q50 + 1.0 * std
        std_count = np.sum((filtered_z_array >= std_min) & (filtered_z_array <= std_max))
        std_pct = (std_count / total_points) * 100
        
        self.get_logger().info(f"\n方案2 - 基于1倍标准差的推荐(以中位数为中心):")
        self.get_logger().info(f"  数据分析: 中位数={q50:.3f}, 标准差={std:.3f}")
        self.get_logger().info(f"  min_height: {std_min:.3f}")
        self.get_logger().info(f"  max_height: {std_max:.3f}")
        self.get_logger().info(f"  保留点数: {std_count} ({std_pct:.1f}%)")
        
        # 方案3: 保守方案 - 主要保留中间80%的数据
        conservative_min = max(ground_height + 0.05, q10)
        conservative_max = q90
        conservative_count = np.sum((filtered_z_array >= conservative_min) & (filtered_z_array <= conservative_max))
        conservative_pct = (conservative_count / total_points) * 100
        
        self.get_logger().info(f"\n方案3 - 保守方案(保留中间80%数据):")
        self.get_logger().info(f"  min_height: {conservative_min:.3f}")
        self.get_logger().info(f"  max_height: {conservative_max:.3f}")
        self.get_logger().info(f"  保留点数: {conservative_count} ({conservative_pct:.1f}%)")
        
        # 方案4: 导航优化方案 - 专注于雷达水平面附近
        # 雷达水平面在Z=0，主要关注±0.2m范围内的障碍物
        nav_min = max(ground_height + 0.05, -0.2)
        nav_max = min(q95, 0.2)  # 不超过95%百分位数
        nav_count = np.sum((filtered_z_array >= nav_min) & (filtered_z_array <= nav_max))
        nav_pct = (nav_count / total_points) * 100
        
        self.get_logger().info(f"\n方案4 - 导航优化方案(雷达水平面±0.2m):")
        self.get_logger().info(f"  min_height: {nav_min:.3f}")
        self.get_logger().info(f"  max_height: {nav_max:.3f}")
        self.get_logger().info(f"  保留点数: {nav_count} ({nav_pct:.1f}%)")
        
        # 自动选择最佳推荐方案
        self.get_logger().info(f"\n" + "="*60)
        self.get_logger().info("自动推荐最佳方案:")
        self.get_logger().info("="*60)
        
        # 选择逻辑：优先考虑数据覆盖率和实用性
        candidates = [
            {
                'name': '导航优化方案',
                'min': nav_min,
                'max': nav_max,
                'count': nav_count,
                'percent': nav_pct,
                'reason': '专注于雷达水平面附近，适合机器人导航',
                'score': nav_pct * 1.2  # 导航方案加权
            },
            {
                'name': '基于四分位数范围',
                'min': iqr_min,
                'max': iqr_max,
                'count': iqr_count,
                'percent': iqr_pct,
                'reason': '基于数据分布，能很好地排除异常值',
                'score': iqr_pct
            },
            {
                'name': '保守方案',
                'min': conservative_min,
                'max': conservative_max,
                'count': conservative_count,
                'percent': conservative_pct,
                'reason': '保留更多数据，适合复杂环境',
                'score': conservative_pct * 0.9  # 保守方案略微降权
            }
        ]
        
        # 选择得分最高的方案
        recommended = max(candidates, key=lambda x: x['score'])
        
        self.get_logger().info(f"推荐方案: {recommended['name']}")
        self.get_logger().info(f"推荐理由: {recommended['reason']}")
        self.get_logger().info(f"推荐设置:")
        self.get_logger().info(f"  min_height: {recommended['min']:.3f}")
        self.get_logger().info(f"  max_height: {recommended['max']:.3f}")
        self.get_logger().info(f"  保留数据: {recommended['count']} 点 ({recommended['percent']:.1f}%)")
        self.get_logger().info(f"  地面高度: {ground_height:.3f}m (已排除)")
        
        # 生成YAML配置建议
        self.get_logger().info(f"\n建议的YAML配置:")
        self.get_logger().info(f"min_height: {recommended['min']:.3f}  # 排除地面后的推荐值")
        self.get_logger().info(f"max_height: {recommended['max']:.3f}  # 基于实际数据分析")
        
        return recommended
    
    def save_analysis_plot(self, z_array_original, z_array_filtered, ground_mask, ground_height, recommended):
        """保存高度分布图表，包含地面过滤和推荐范围标记"""
        plt.figure(figsize=(16, 12))
        
        # 子图1: 原始数据直方图
        plt.subplot(2, 3, 1)
        plt.hist(z_array_original, bins=100, alpha=0.7, color='lightblue', edgecolor='black', label='原始数据')
        ground_points = z_array_original[ground_mask]
        if len(ground_points) > 0:
            plt.hist(ground_points, bins=100, alpha=0.8, color='brown', edgecolor='black', label='地面点')
        
        plt.axvline(ground_height, color='red', linestyle=':', linewidth=2, label=f'估计地面高度: {ground_height:.3f}m')
        plt.axvline(0, color='green', linestyle='-', alpha=0.5, label='雷达水平面')
        plt.xlabel('高度 (m)')
        plt.ylabel('点数')
        plt.title('原始高度分布 (包含地面点)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # 子图2: 过滤后数据直方图
        plt.subplot(2, 3, 2)
        plt.hist(z_array_filtered, bins=50, alpha=0.7, color='blue', edgecolor='black')
        plt.axvline(recommended['min'], color='red', linestyle='--', linewidth=2, 
                   label=f'推荐min: {recommended["min"]:.3f}')
        plt.axvline(recommended['max'], color='red', linestyle='--', linewidth=2,
                   label=f'推荐max: {recommended["max"]:.3f}')
        plt.axvline(0, color='green', linestyle='-', alpha=0.5, label='雷达水平面')
        
        # 添加推荐区域填充
        y_max = plt.ylim()[1]
        plt.axvspan(recommended['min'], recommended['max'], alpha=0.2, color='red', 
                   label=f'推荐区域 ({recommended["percent"]:.1f}%)')
        
        plt.xlabel('高度 (m)')
        plt.ylabel('点数')
        plt.title('过滤后高度分布 (排除地面)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # 子图3: 对比累积分布
        plt.subplot(2, 3, 3)
        
        # 原始数据累积分布
        sorted_z_orig = np.sort(z_array_original)
        y_orig = np.arange(1, len(sorted_z_orig) + 1) / len(sorted_z_orig) * 100
        plt.plot(sorted_z_orig, y_orig, 'b-', linewidth=2, alpha=0.7, label='原始数据')
        
        # 过滤后数据累积分布
        sorted_z_filt = np.sort(z_array_filtered)
        y_filt = np.arange(1, len(sorted_z_filt) + 1) / len(sorted_z_filt) * 100
        plt.plot(sorted_z_filt, y_filt, 'g-', linewidth=2, label='过滤后数据')
        
        plt.axvline(ground_height, color='red', linestyle=':', alpha=0.7, label='地面高度')
        plt.axvline(recommended['min'], color='red', linestyle='--', alpha=0.7)
        plt.axvline(recommended['max'], color='red', linestyle='--', alpha=0.7)
        
        plt.xlabel('高度 (m)')
        plt.ylabel('累积百分比 (%)')
        plt.title('累积分布对比')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # 子图4: 箱线图对比
        plt.subplot(2, 3, 4)
        box_data = [z_array_original, z_array_filtered]
        box_labels = ['原始数据', '过滤后']
        box_plot = plt.boxplot(box_data, labels=box_labels, patch_artist=True)
        box_plot['boxes'][0].set_facecolor('lightblue')
        box_plot['boxes'][1].set_facecolor('lightgreen')
        
        plt.axhline(ground_height, color='red', linestyle=':', linewidth=2, label='地面高度')
        plt.axhline(recommended['min'], color='red', linestyle='--', linewidth=2, label='推荐范围')
        plt.axhline(recommended['max'], color='red', linestyle='--', linewidth=2)
        plt.axhline(0, color='green', linestyle='-', alpha=0.5, label='雷达水平面')
        
        plt.ylabel('高度 (m)')
        plt.title('箱线图对比')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # 子图5: 高度密度热图 (过滤后)
        plt.subplot(2, 3, 5)
        height_bins = np.arange(z_array_filtered.min(), z_array_filtered.max() + 0.05, 0.05)
        hist, bin_edges = np.histogram(z_array_filtered, bins=height_bins)
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
        
        plt.bar(bin_centers, hist, width=0.04, alpha=0.7, color='skyblue', edgecolor='black')
        
        # 高亮推荐区域
        mask = (bin_centers >= recommended['min']) & (bin_centers <= recommended['max'])
        plt.bar(bin_centers[mask], hist[mask], width=0.04, alpha=0.9, 
               color='red', edgecolor='darkred', label='推荐区域')
        
        plt.axvline(0, color='green', linestyle='-', alpha=0.5, label='雷达水平面')
        plt.xlabel('高度 (m)')
        plt.ylabel('点数密度')
        plt.title('高度密度分布 (5cm间隔)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # 子图6: 地面检测效果
        plt.subplot(2, 3, 6)
        
        # 显示地面检测的效果
        plt.hist(z_array_original, bins=100, alpha=0.5, color='lightblue', label='原始数据')
        plt.hist(z_array_original[ground_mask], bins=100, alpha=0.8, color='brown', label='检测到的地面点')
        
        plt.axvline(ground_height, color='red', linestyle='-', linewidth=2, label=f'地面高度: {ground_height:.3f}m')
        plt.axvline(ground_height + self.ground_tolerance, color='orange', linestyle='--', 
                   label=f'地面容差: ±{self.ground_tolerance:.2f}m')
        
        plt.xlabel('高度 (m)')
        plt.ylabel('点数')
        plt.title('地面检测效果')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        plt.tight_layout()
        
        # 添加总标题
        plt.suptitle(f'Livox MID-360 高度分析报告 (地面过滤版)\n推荐方案: {recommended["name"]}', 
                    fontsize=16, y=0.98)
        
        # 保存图片
        filename = 'livox_height_analysis_ground_filtered.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"高度分析报告已保存为: {filename}")
    
    def save_text_report(self, z_array_original, z_array_filtered, ground_height, recommended):
        """保存详细的文本分析报告"""
        filename = 'livox_height_analysis_ground_filtered_report.txt'
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("="*80 + "\n")
            f.write("Livox MID-360 高度分析报告 (地面过滤版)\n")
            f.write("="*80 + "\n\n")
            
            # 地面检测结果
            f.write("地面检测结果:\n")
            f.write("-"*40 + "\n")
            f.write(f"雷达安装高度: {self.lidar_height}m\n")
            f.write(f"估计地面高度: {ground_height:.3f}m\n")
            f.write(f"地面检测容差: ±{self.ground_tolerance:.2f}m\n")
            ground_count = len(z_array_original) - len(z_array_filtered)
            ground_pct = (ground_count / len(z_array_original)) * 100
            f.write(f"检测到地面点: {ground_count:,} ({ground_pct:.1f}%)\n")
            f.write(f"过滤后剩余点: {len(z_array_filtered):,} ({100-ground_pct:.1f}%)\n\n")
            
            # 原始数据统计
            f.write("原始数据统计信息:\n")
            f.write("-"*40 + "\n")
            f.write(f"总点数: {len(z_array_original):,}\n")
            f.write(f"高度范围: [{z_array_original.min():.3f}m, {z_array_original.max():.3f}m]\n")
            f.write(f"平均高度: {z_array_original.mean():.3f}m\n")
            f.write(f"高度标准差: {z_array_original.std():.3f}m\n\n")
            
            # 过滤后数据统计
            f.write("过滤后数据统计信息:\n")
            f.write("-"*40 + "\n")
            f.write(f"高度范围: [{z_array_filtered.min():.3f}m, {z_array_filtered.max():.3f}m]\n")
            f.write(f"平均高度: {z_array_filtered.mean():.3f}m\n")
            f.write(f"高度标准差: {z_array_filtered.std():.3f}m\n")
            f.write(f"高度中位数: {np.percentile(z_array_filtered, 50):.3f}m\n\n")
            
            # 百分位数信息
            f.write("过滤后数据百分位数:\n")
            f.write("-"*40 + "\n")
            percentiles = [5, 10, 25, 50, 75, 90, 95]
            for p in percentiles:
                value = np.percentile(z_array_filtered, p)
                f.write(f"{p:2d}%: {value:8.3f}m\n")
            f.write("\n")
            
            # 推荐设置
            f.write("推荐配置:\n")
            f.write("-"*40 + "\n")
            f.write(f"推荐方案: {recommended['name']}\n")
            f.write(f"推荐理由: {recommended['reason']}\n")
            f.write(f"min_height: {recommended['min']:.3f}\n")
            f.write(f"max_height: {recommended['max']:.3f}\n")
            f.write(f"保留数据: {recommended['count']} 点 ({recommended['percent']:.1f}%)\n")
            f.write(f"地面高度: {ground_height:.3f}m (已排除)\n\n")
            
            # YAML配置
            f.write("YAML配置文件建议:\n")
            f.write("-"*40 + "\n")
            f.write("pointcloud_to_laserscan_node:\n")
            f.write("  ros__parameters:\n")
            f.write("    target_frame: 'livox_frame_corrected'\n")
            f.write("    transform_tolerance: 0.1\n")
            f.write(f"    min_height: {recommended['min']:.3f}  # 排除地面后的推荐值\n")
            f.write(f"    max_height: {recommended['max']:.3f}  # 基于过滤数据分析\n")
            f.write("    angle_min: -3.14159\n")
            f.write("    angle_max: 3.14159\n")
            f.write("    angle_increment: 0.0044\n")
            f.write("    scan_time: 0.033\n")
            f.write("    range_min: 0.2\n")
            f.write("    range_max: 20.0\n")
            f.write("    use_inf: true\n")
            f.write("    inf_epsilon: 1.0\n")
            f.write("    use_sim_time: true\n")
            f.write("    concurrency_level: 1\n")
            f.write("    queue_size: 50\n\n")
            
            # 使用建议
            f.write("使用建议:\n")
            f.write("-"*40 + "\n")
            f.write(f"1. 地面已被自动检测并排除 (高度: {ground_height:.3f}m)\n")
            f.write("2. 推荐的高度范围专注于有效的障碍物检测\n")
            f.write("3. 如果环境中有特殊需求，可以微调min_height和max_height\n")
            f.write("4. 建议在实际环境中测试推荐参数的效果\n")
            
        self.get_logger().info(f"详细分析报告已保存为: {filename}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HeightRangeAnalyzer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
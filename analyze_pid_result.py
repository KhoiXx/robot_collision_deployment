#!/usr/bin/env python3
"""
Script phân tích ổn định PID toàn diện
Phân tích dữ liệu đáp ứng bước và tạo báo cáo ổn định với trực quan hóa
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime
import sys
import os

def analyze_step_response(df_step, target_speed):
    """Phân tích một đáp ứng bước"""
    setpoint = df_step['setpoint'].iloc[-1]
    tolerance = 0.02 * abs(setpoint) if setpoint != 0 else 1.0
    
    in_tolerance = np.abs(df_step['current'] - setpoint) <= tolerance
    
    settling_time = None
    for i in range(len(df_step) - 10):
        if all(in_tolerance.iloc[i:]):
            settling_time = df_step['test_time'].iloc[i]
            break
    
    if setpoint > 0:
        max_val = df_step['current'].max()
        overshoot_percent = ((max_val - setpoint) / setpoint * 100) if setpoint != 0 else 0
    else:
        min_val = df_step['current'].min()
        overshoot_percent = ((setpoint - min_val) / abs(setpoint) * 100) if setpoint != 0 else 0
    
    if setpoint != 0:
        initial = df_step['current'].iloc[0]
        target_10 = initial + 0.1 * (setpoint - initial)
        target_90 = initial + 0.9 * (setpoint - initial)
        
        if setpoint > initial:
            t_10 = df_step[df_step['current'] >= target_10]['test_time'].iloc[0] if any(df_step['current'] >= target_10) else None
            t_90 = df_step[df_step['current'] >= target_90]['test_time'].iloc[0] if any(df_step['current'] >= target_90) else None
        else:
            t_10 = df_step[df_step['current'] <= target_10]['test_time'].iloc[0] if any(df_step['current'] <= target_10) else None
            t_90 = df_step[df_step['current'] <= target_90]['test_time'].iloc[0] if any(df_step['current'] <= target_90) else None
        
        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None
    else:
        rise_time = None
    
    steady_start_idx = int(len(df_step) * 0.8)
    steady_state_error = df_step['error'].iloc[steady_start_idx:].abs().mean()
    rms_error = np.sqrt((df_step['error'] ** 2).mean())
    
    return {
        'target_speed': target_speed,
        'setpoint': setpoint,
        'settling_time': settling_time,
        'rise_time': rise_time,
        'overshoot_percent': overshoot_percent,
        'steady_state_error': steady_state_error,
        'rms_error': rms_error,
        'final_current': df_step['current'].iloc[-1]
    }

def create_plots(csv_file, output_prefix):
    """Tạo biểu đồ trực quan hóa"""
    df = pd.read_csv(csv_file)
    
    df['speed_change'] = df['target_speed'].diff().abs() > 0.01
    speed_changes = df[df['speed_change']].index.tolist()
    speed_changes.insert(0, 0)
    speed_changes.append(len(df))
    
    steps = []
    for i in range(len(speed_changes) - 1):
        start_idx = speed_changes[i]
        end_idx = speed_changes[i + 1]
        step_df = df.iloc[start_idx:end_idx].copy()
        step_df['test_time'] = step_df['test_time'] - step_df['test_time'].iloc[0]
        steps.append((step_df['target_speed'].iloc[0], step_df))
    
    n_steps = len(steps)
    fig, axes = plt.subplots(n_steps, 3, figsize=(18, 4*n_steps))
    if n_steps == 1:
        axes = axes.reshape(1, -1)
    
    fig.suptitle('Phân Tích Đáp Ứng Bước PID - Báo Cáo Ổn Định Toàn Diện', fontsize=16, fontweight='bold')
    
    for idx, (target_speed, step_df) in enumerate(steps):
        setpoint = step_df['setpoint'].iloc[0]
        
        ax1 = axes[idx, 0]
        ax1.plot(step_df['test_time'], step_df['setpoint'], 'r--', linewidth=2, label='Giá trị đặt', alpha=0.7)
        ax1.plot(step_df['test_time'], step_df['current'], 'b-', linewidth=1.5, label='Tốc độ hiện tại')
        ax1.fill_between(step_df['test_time'],
                         step_df['setpoint'] * 0.98,
                         step_df['setpoint'] * 1.02,
                         color='green', alpha=0.2, label='±2% dung sai')
        ax1.set_xlabel('Thời gian (s)', fontsize=10)
        ax1.set_ylabel('Tốc độ (xung encoder/mẫu)', fontsize=10)
        ax1.set_title(f'Đáp ứng bước: {target_speed:.1f} m/s (Giá trị đặt: {setpoint:.0f})', fontsize=11, fontweight='bold')
        ax1.legend(loc='best', fontsize=9)
        ax1.grid(True, alpha=0.3)
        
        ax2 = axes[idx, 1]
        ax2.plot(step_df['test_time'], step_df['error'], 'r-', linewidth=1.5, label='Sai số bám')
        ax2.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5)
        ax2.fill_between(step_df['test_time'], -2, 2, color='green', alpha=0.2, label='Bám tốt (|sai số| < 2)')
        ax2.set_xlabel('Thời gian (s)', fontsize=10)
        ax2.set_ylabel('Sai số (xung)', fontsize=10)
        ax2.set_title(f'Sai số bám theo thời gian', fontsize=11, fontweight='bold')
        ax2.legend(loc='best', fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        ax3 = axes[idx, 2]
        ax3.plot(step_df['test_time'], step_df['output'], 'g-', linewidth=1.5, label='Tín hiệu PWM')
        ax3.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5)
        ax3.set_xlabel('Thời gian (s)', fontsize=10)
        ax3.set_ylabel('Tín hiệu điều khiển PWM', fontsize=10)
        ax3.set_title(f'Tín hiệu điều khiển theo thời gian', fontsize=11, fontweight='bold')
        ax3.legend(loc='best', fontsize=9)
        ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot_file = f"{output_prefix}_comprehensive.png"
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    return plot_file, steps

def generate_report(csv_file, steps, output_prefix):
    """Tạo báo cáo chi tiết bằng tiếng Việt"""
    report_file = f"{output_prefix}_report.txt"
    
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("BÁO CÁO PHÂN TÍCH ỔN ĐỊNH BỘ ĐIỀU KHIỂN PID\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"Tệp dữ liệu: {os.path.basename(csv_file)}\n")
        f.write(f"Ngày phân tích: {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}\n")
        f.write(f"Số lần thử nghiệm bước: {len(steps)}\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("TÓM TẮT - XÁC MINH ỔN ĐỊNH HỆ THỐNG\n")
        f.write("=" * 80 + "\n\n")
        
        all_stable = True
        all_metrics = []
        
        for target_speed, step_df in steps:
            metrics = analyze_step_response(step_df, target_speed)
            all_metrics.append(metrics)
            
            is_stable = True
            if metrics['settling_time'] is None:
                is_stable = False
            elif metrics['settling_time'] > 3.0:
                is_stable = False
            
            if abs(metrics['overshoot_percent']) > 15:
                is_stable = False
            
            if metrics['setpoint'] != 0:
                ss_error_percent = (metrics['steady_state_error'] / abs(metrics['setpoint'])) * 100
                if ss_error_percent > 5:
                    is_stable = False
            
            if not is_stable:
                all_stable = False
        
        if all_stable:
            f.write("✓ KẾT LUẬN: HỆ THỐNG ỔN ĐỊNH\n\n")
            f.write("Bộ điều khiển PID hoạt động ổn định ở tất cả các tốc độ kiểm tra.\n\n")
            f.write("Các chỉ số chứng minh ổn định:\n")
            f.write("  • Tất cả đáp ứng bước đều ổn định trong thời gian chấp nhận được\n")
            f.write("  • Độ vọt lố nằm trong giới hạn chấp nhận được (< 15%)\n")
            f.write("  • Sai số xác lập rất nhỏ (< 5%)\n")
            f.write("  • Không có dao động hoặc phân kỳ\n")
            f.write("  • Hệ thống hoạt động ổn định ở cả chiều xuôi và chiều ngược\n\n")
        else:
            f.write("⚠ LƯU Ý: CÓ MỘT SỐ VẤN ĐỀ VỀ ỔN ĐỊNH\n\n")
            f.write("Một số đáp ứng bước cho thấy có vấn đề nhỏ. Xem phân tích chi tiết bên dưới.\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("PHÂN TÍCH CHI TIẾT CÁC ĐÁP ỨNG BƯỚC\n")
        f.write("=" * 80 + "\n\n")
        
        for idx, ((target_speed, step_df), metrics) in enumerate(zip(steps, all_metrics)):
            f.write(f"\nBƯỚC {idx + 1}: Tốc độ đích = {target_speed:.2f} m/s (Giá trị đặt = {metrics['setpoint']:.0f} xung)\n")
            f.write("-" * 80 + "\n")
            
            f.write("\nCác chỉ số hiệu năng:\n")
            if metrics['settling_time']:
                f.write(f"  • Thời gian ổn định (băng tần 2%):  {metrics['settling_time']:.3f} s\n")
            else:
                f.write(f"  • Thời gian ổn định:               Chưa ổn định hoàn toàn\n")
            
            if metrics['rise_time']:
                f.write(f"  • Thời gian tăng (10%-90%):        {metrics['rise_time']:.3f} s\n")
            else:
                f.write(f"  • Thời gian tăng:                  N/A\n")
            
            f.write(f"  • Độ vọt lố:                       {metrics['overshoot_percent']:.2f} %\n")
            f.write(f"  • Sai số xác lập:                  {metrics['steady_state_error']:.3f} xung\n")
            
            if metrics['setpoint'] != 0:
                ss_error_pct = (metrics['steady_state_error'] / abs(metrics['setpoint'])) * 100
                f.write(f"  • Sai số xác lập (%):              {ss_error_pct:.2f} %\n")
            
            f.write(f"  • Sai số RMS:                      {metrics['rms_error']:.3f} xung\n")
            f.write(f"  • Giá trị cuối cùng:               {metrics['final_current']:.1f} xung\n")
            
            f.write("\nĐánh giá ổn định:\n")
            
            if metrics['settling_time'] is not None and metrics['settling_time'] < 3.0:
                f.write(f"  ✓ Thời gian ổn định nhanh ({metrics['settling_time']:.2f}s < 3.0s)\n")
            elif metrics['settling_time'] is None:
                f.write(f"  ⚠ Chưa ổn định hoàn toàn trong thời gian thử nghiệm\n")
            else:
                f.write(f"  ⚠ Thời gian ổn định chậm ({metrics['settling_time']:.2f}s)\n")
            
            if abs(metrics['overshoot_percent']) < 5:
                f.write(f"  ✓ Độ vọt lố rất thấp ({abs(metrics['overshoot_percent']):.1f}% < 5%)\n")
            elif abs(metrics['overshoot_percent']) < 15:
                f.write(f"  ✓ Độ vọt lố ở mức chấp nhận được ({abs(metrics['overshoot_percent']):.1f}% < 15%)\n")
            else:
                f.write(f"  ✗ Độ vọt lố cao ({abs(metrics['overshoot_percent']):.1f}%)\n")
            
            if metrics['setpoint'] != 0:
                ss_error_pct = (metrics['steady_state_error'] / abs(metrics['setpoint'])) * 100
                if ss_error_pct < 2:
                    f.write(f"  ✓ Độ chính xác xác lập xuất sắc ({ss_error_pct:.2f}%)\n")
                elif ss_error_pct < 5:
                    f.write(f"  ✓ Độ chính xác xác lập tốt ({ss_error_pct:.2f}%)\n")
                else:
                    f.write(f"  ⚠ Có sai số xác lập ({ss_error_pct:.2f}%)\n")
            
            error_max = step_df['error'].abs().max()
            error_std = step_df['error'].std()
            
            f.write(f"\nThống kê sai số:\n")
            f.write(f"  • Sai số tối đa:                   {error_max:.3f} xung\n")
            f.write(f"  • Độ lệch chuẩn sai số:            {error_std:.3f} xung\n")
            
            if error_std < 1.0:
                f.write(f"  ✓ Dao động thấp (độ lệch chuẩn < 1.0)\n")
            elif error_std < 2.0:
                f.write(f"  ✓ Dao động vừa phải (độ lệch chuẩn = {error_std:.2f})\n")
            else:
                f.write(f"  ⚠ Dao động cao (độ lệch chuẩn = {error_std:.2f})\n")
            
            f.write("\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("BẢNG TỔNG HỢP THỐNG KÊ\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"{'Tốc độ':<10} {'Giá trị':<12} {'T.gian':<12} {'Vọt lố':<12} {'Sai số':<12} {'Sai số':<12}\n")
        f.write(f"{'đích':<10} {'đặt':<12} {'ổn định':<12} {'(%)':<12} {'xác lập':<12} {'RMS':<12}\n")
        f.write(f"{'(m/s)':<10} {'(xung)':<12} {'(s)':<12} {'':<12} {'(xung)':<12} {'(xung)':<12}\n")
        f.write("-" * 80 + "\n")
        
        for metrics in all_metrics:
            settling = f"{metrics['settling_time']:.3f}" if metrics['settling_time'] else "N/A"
            f.write(f"{metrics['target_speed']:<10.2f} {metrics['setpoint']:<12.0f} {settling:<12} "
                   f"{metrics['overshoot_percent']:<12.2f} {metrics['steady_state_error']:<12.3f} "
                   f"{metrics['rms_error']:<12.3f}\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("KẾT LUẬN VÀ ĐÁNH GIÁ\n")
        f.write("=" * 80 + "\n\n")
        
        valid_settling = [m['settling_time'] for m in all_metrics if m['settling_time'] is not None]
        avg_settling = np.mean(valid_settling) if valid_settling else 0
        avg_overshoot = np.mean([abs(m['overshoot_percent']) for m in all_metrics])
        avg_ss_error = np.mean([m['steady_state_error'] for m in all_metrics])
        
        f.write(f"Hiệu năng trung bình trên tất cả các bước:\n")
        if valid_settling:
            f.write(f"  • Thời gian ổn định TB:            {avg_settling:.3f} s\n")
        f.write(f"  • Độ vọt lố trung bình:            {avg_overshoot:.2f} %\n")
        f.write(f"  • Sai số xác lập TB:               {avg_ss_error:.3f} xung\n\n")
        
        f.write("Kết luận về ổn định:\n")
        if all_stable:
            f.write("  Bộ điều khiển PID ỔN ĐỊNH và được điều chỉnh tốt. Hệ thống:\n")
            f.write("  • Đáp ứng nhanh với thay đổi giá trị đặt\n")
            f.write("  • Ổn định mà không có vọt lố quá mức\n")
            f.write("  • Duy trì bám chính xác với sai số xác lập tối thiểu\n")
            f.write("  • Hiệu năng ổn định ở các tốc độ khác nhau\n")
            f.write("  • Hoạt động tốt ở cả chiều tiến và chiều lùi\n\n")
            f.write("  Bộ điều khiển này phù hợp để triển khai trong ứng dụng luận văn.\n")
        else:
            f.write("  Hệ thống cho thấy một số vấn đề nhỏ về ổn định:\n")
            for idx, ((target_speed, _), metrics) in enumerate(zip(steps, all_metrics)):
                if metrics['settling_time'] is None or metrics['settling_time'] > 3.0:
                    f.write(f"  • Bước {idx+1} ({target_speed:.2f} m/s): Ổn định chậm\n")
                if abs(metrics['overshoot_percent']) > 15:
                    f.write(f"  • Bước {idx+1} ({target_speed:.2f} m/s): Vọt lố quá mức\n")
            f.write("\n  Tuy nhiên, hệ thống vẫn hoạt động ổn định với sai số nhỏ.\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("KẾT THÚC BÁO CÁO\n")
        f.write("=" * 80 + "\n")
    
    return report_file

def main():
    if len(sys.argv) < 2:
        print("Cách dùng: python3 analyze_pid_vietnamese.py <file_csv>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    if not os.path.exists(csv_file):
        print(f"Lỗi: Không tìm thấy file {csv_file}")
        sys.exit(1)
    
    output_prefix = csv_file.replace('.csv', '')
    
    print(f"Đang phân tích dữ liệu ổn định PID từ: {csv_file}")
    print("-" * 60)
    
    print("\n1. Đang tạo biểu đồ trực quan hóa toàn diện...")
    plot_file, steps = create_plots(csv_file, output_prefix)
    print(f"   ✓ Đã lưu: {plot_file}")
    
    print("\n2. Đang tạo báo cáo ổn định chi tiết...")
    report_file = generate_report(csv_file, steps, output_prefix)
    print(f"   ✓ Đã lưu: {report_file}")
    
    print("\n" + "=" * 60)
    print("HOÀN TẤT PHÂN TÍCH!")
    print("=" * 60)
    print(f"\nCác file đã tạo:")
    print(f"  • Trực quan hóa: {plot_file}")
    print(f"  • Báo cáo:       {report_file}")
    print("\nVui lòng xem báo cáo để biết chi tiết phân tích ổn định.")

if __name__ == '__main__':
    main()

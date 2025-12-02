#!/usr/bin/env python3
"""
Generate a matplotlib plot of the brake law: y = m*x + n
where x = D_T - d_min (distance-to-threshold)
      y = v_b (brake term)
      m = B_max / (D_T - T_min)
      n = 0

Usage:
    conda activate cbio
    python scripts/plot_brake.py --D_T 600 --T_min 100 --B_max 100 --output multimedia/task1/formulas/brake_law.png

Default values are used if arguments are not provided.
"""

import argparse
import matplotlib.pyplot as plt
import numpy as np


def plot_brake_law(D_T, T_min, B_max, output_path):
    """
    Plot the brake law y = m*x + n.
    
    Args:
        D_T (float): Min. distance to obstacle (mm).
        T_min (float): Min. safety threshold (mm).
        B_max (float): Max. allowed braking value (mm/s).
        output_path (str): Path to save the PNG file.
    """
    # Compute slope and intercept
    m = B_max / (D_T - T_min)
    n = 0  # No intercept in this formulation
    
    # x axis: from 0 to D_T - T_min
    x_max = D_T - T_min
    x = np.linspace(0, x_max * 1.1, 100)
    y = m * x + n
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Plot the brake law line
    ax.plot(x, y, 'b-', linewidth=2.5, label=r'$y = mx + n$')
    
    # Threshold markers (vertical dashed lines)
    ax.axvline(x=0, color='gray', linestyle='--', alpha=0.7, linewidth=1)
    ax.text(0, -B_max*0.15, r'$x=0$ ($d_{\min}=D_T=600$ mm)', 
            ha='center', fontsize=10, color='gray')
    
    ax.axvline(x=x_max, color='gray', linestyle='--', alpha=0.7, linewidth=1)
    ax.text(x_max, -B_max*0.15, r'$x=D_T - T_{\min}$', 
            ha='center', fontsize=10, color='gray')
    
    # Range bracket annotation
    ax.annotate('', xy=(x_max, B_max*0.9), xytext=(0, B_max*0.9),
                arrowprops=dict(arrowstyle='<->', color='gray', lw=1))
    ax.text(x_max/2, B_max*0.95, r'$D_T - T_{\min}$', 
            ha='center', fontsize=10, color='gray')
    
    # Axes labels
    ax.set_xlabel(r'$x = D_T - d_{\min}$ (mm)', fontsize=11)
    ax.set_ylabel(r'$y = v_b$ (mm/s)', fontsize=11)
    ax.set_title(r'Brake Law: $y = m \cdot x + n$', fontsize=12, fontweight='bold')
    
    # Grid and legend
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11, loc='upper left')
    
    # Add formula text box
    formula_text = (f'$m = \\frac{{B_{{max}}}}{{D_T - T_{{\\min}}}} = \\frac{{{B_max}}}{{{D_T - T_min}}} = {m:.2f}$\n'
                    f'$n = 0$')
    ax.text(0.98, 0.05, formula_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='bottom', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Set axis limits with padding
    ax.set_xlim(-0.05*x_max, x_max*1.15)
    ax.set_ylim(-B_max*0.25, B_max*1.15)
    
    # Tight layout
    plt.tight_layout()
    
    # Save figure
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✓ Brake law plot saved to: {output_path}")
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generate a matplotlib plot of the brake law y = m*x + n.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Use default values
  python scripts/plot_brake.py --output multimedia/task1/formulas/brake_law.png
  
  # Custom parameters
  python scripts/plot_brake.py --D_T 600 --T_min 100 --B_max 100 --output output.png
        '''
    )
    
    parser.add_argument('--D_T', type=float, default=600,
                        help='Min. distance to obstacle (mm). Default: 600')
    parser.add_argument('--T_min', type=float, default=100,
                        help='Min. safety threshold (mm). Default: 100')
    parser.add_argument('--B_max', type=float, default=100,
                        help='Max. allowed braking value (mm/s). Default: 100')
    parser.add_argument('--output', type=str, default='multimedia/task1/formulas/brake_law.png',
                        help='Output PNG path. Default: multimedia/task1/formulas/brake_law.png')
    
    args = parser.parse_args()
    
    # Validate inputs
    if args.D_T <= args.T_min:
        print("Error: D_T must be greater than T_min")
        exit(1)
    if args.B_max <= 0:
        print("Error: B_max must be positive")
        exit(1)
    
    # Generate plot
    plot_brake_law(args.D_T, args.T_min, args.B_max, args.output)

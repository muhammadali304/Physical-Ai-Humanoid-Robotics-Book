#!/usr/bin/env python3
"""
Navigation Results Analyzer
Analyzes the results from navigation validation tests
"""

import json
import os
import sys
from datetime import datetime


def analyze_results(results_file):
    """Analyze the results from a validation run"""
    with open(results_file, 'r') as f:
        data = json.load(f)

    total_tests = data.get('total_tests', 0)
    successful_tests = data.get('successful_tests', 0)
    failed_tests = data.get('failed_tests', 0)
    success_rate = data.get('success_rate', 0)
    target_rate = data.get('target_rate', 90.0)
    met_target = data.get('met_target', False)

    print("="*60)
    print("NAVIGATION VALIDATION RESULTS ANALYSIS")
    print("="*60)
    print(f"Results file: {results_file}")
    print(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("-"*60)
    print(f"Total tests:    {total_tests}")
    print(f"Successful:     {successful_tests}")
    print(f"Failed:         {failed_tests}")
    print(f"Success rate:   {success_rate:.2f}%")
    print(f"Target rate:    {target_rate:.2f}%")
    print(f"Target met:     {'YES' if met_target else 'NO'}")
    print("-"*60)

    if 'results' in data:
        successful_waypoints = [r['waypoint'] for r in data['results'] if r['success']]
        failed_waypoints = [r['waypoint'] for r in data['results'] if not r['success']]

        print(f"Successful waypoints ({len(successful_waypoints)}):")
        for i, wp in enumerate(successful_waypoints, 1):
            print(f"  {i:2d}. ({wp[0]:6.2f}, {wp[1]:6.2f}, {wp[2]:6.2f})")

        print(f"\nFailed waypoints ({len(failed_waypoints)}):")
        for i, wp in enumerate(failed_waypoints, 1):
            print(f"  {i:2d}. ({wp[0]:6.2f}, {wp[1]:6.2f}, {wp[2]:6.2f})")

    print("="*60)

    return met_target


def find_latest_results(results_dir):
    """Find the most recent results file"""
    if not os.path.exists(results_dir):
        return None

    json_files = [f for f in os.listdir(results_dir) if f.endswith('.json')]
    if not json_files:
        return None

    # Sort by modification time (most recent first)
    json_files.sort(key=lambda x: os.path.getmtime(os.path.join(results_dir, x)), reverse=True)
    return os.path.join(results_dir, json_files[0])


def main():
    if len(sys.argv) > 1:
        # Use specified results file
        results_file = sys.argv[1]
        if not os.path.exists(results_file):
            print(f"Error: Results file {results_file} not found")
            sys.exit(1)
    else:
        # Find the most recent results file
        results_dir = os.path.join(os.path.dirname(__file__), "results")
        results_file = find_latest_results(results_dir)

        if results_file is None:
            print("No results files found. Run navigation validation first.")
            sys.exit(1)

    success = analyze_results(results_file)

    # Exit with appropriate code
    if success:
        print("\n✅ VALIDATION PASSED: Navigation system meets requirements")
        sys.exit(0)
    else:
        print("\n❌ VALIDATION FAILED: Navigation system does not meet requirements")
        sys.exit(1)


if __name__ == '__main__':
    main()
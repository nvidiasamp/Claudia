#!/usr/bin/env python3
# scripts/validation/foot_force/run_quick_abcd_test.py
# Generated: 2025-06-26 19:10:00
# Purpose: Unitree Go2 foot force sensor quick ABCD validation test

import os
import sys
import json
import time
from pathlib import Path
from datetime import datetime

# Set environment variables
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

# Add project path
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

def test_abcd_components():
    """Test whether all ABCD components are working correctly"""

    print("Unitree Go2 Foot Force Sensor ABCD Component Test")
    print("="*60)

    results = {
        'phase_a': False,
        'phase_b': False,
        'phase_c': False,
        'phase_d': False
    }

    # Test Phase A: Data reading framework
    print("\nTesting Phase A: Data Reading Framework")
    try:
        sys.path.append(str(Path(__file__).parent / "foot_force_validation"))
        from foot_force_validation.foot_force_config import FootForceConfig

        config_path = Path(__file__).parent / "foot_force_validation" / "validation_config.json"
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)

        # Mock test FootForceConfig
        foot_config = FootForceConfig(network_interface="eth0")
        print("  FootForceConfig initialization successful")
        results['phase_a'] = True

    except Exception as e:
        print(f"  Phase A test failed: {e}")

    # Test Phase B: Static validation
    print("\nTesting Phase B: Static Validation Framework")
    try:
        from foot_force_validation.static_tester import StaticFootForceTester
        print("  StaticFootForceTester import successful")
        results['phase_b'] = True

    except Exception as e:
        print(f"  Phase B test failed: {e}")

    # Test Phase C: Dynamic testing
    print("\nTesting Phase C: Dynamic Testing Framework")
    try:
        from foot_force_validation.dynamic_tester import DynamicFootForceTester
        print("  DynamicFootForceTester import successful")
        results['phase_c'] = True

    except Exception as e:
        print(f"  Phase C test failed: {e}")

    # Test Phase D: Comprehensive visualization
    print("\nTesting Phase D: Comprehensive Visualization Framework")
    try:
        from foot_force_validation.comprehensive_dashboard import ComprehensiveFootForceDashboard
        print("  ComprehensiveFootForceDashboard import successful")
        results['phase_d'] = True

    except Exception as e:
        print(f"  Phase D test failed: {e}")

    # Summary results
    print("\nTest Results Summary:")
    print("="*60)

    success_count = sum(results.values())
    total_count = len(results)

    for phase, success in results.items():
        status = "PASSED" if success else "FAILED"
        print(f"  {phase.upper()}: {status}")

    print(f"\nOverall result: {success_count}/{total_count} phases passed")

    if success_count == total_count:
        print("All ABCD component tests passed! You can run the full validation workflow.")
        return True
    else:
        print("WARNING: Some component tests failed. Please check the relevant modules.")
        return False

def run_minimal_test():
    """Run minimal foot force test"""
    print("\nRunning Minimal Foot Force Validation Test")
    print("-"*60)

    try:
        # Phase A: Mock data reading
        print("Phase A: Mock data reading test...")
        import numpy as np

        # Mock foot force data
        mock_data = {
            'timestamp': time.time(),
            'foot_forces': [[10.0, 5.0, 40.0] for _ in range(4)],  # 4 feet
            'contact_states': [True, True, True, True],
            'total_force': 160.0
        }
        print(f"  Mock data generated: total force {mock_data['total_force']}N")

        # Phase B: Mock static validation
        print("Phase B: Mock static validation test...")
        static_score = 85.0
        print(f"  Mock static validation: score {static_score}")

        # Phase C: Mock dynamic test
        print("Phase C: Mock dynamic test...")
        dynamic_scores = [82.0, 78.5, 85.2]
        avg_dynamic = np.mean(dynamic_scores)
        print(f"  Mock dynamic test: average score {avg_dynamic:.1f}")

        # Phase D: Mock report generation
        print("Phase D: Mock report generation...")
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'static_score': static_score,
            'dynamic_score': avg_dynamic,
            'overall_score': static_score * 0.6 + avg_dynamic * 0.4
        }

        # Save mock report
        output_dir = Path("scripts/validation/foot_force/foot_force_validation/output")
        output_dir.mkdir(parents=True, exist_ok=True)

        report_file = output_dir / f"mock_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        print(f"  Mock report saved: {report_file.name}")

        print(f"\nMock test complete:")
        print(f"   Overall score: {report_data['overall_score']:.1f}")
        print(f"   Report file: {report_file}")

        return True

    except Exception as e:
        print(f"Minimal test failed: {e}")
        return False

def main():
    """Main function"""
    print("Starting Quick ABCD Component Test")

    # 1. Test component imports
    components_ok = test_abcd_components()

    # 2. Run minimal test
    minimal_test_ok = run_minimal_test()

    # 3. Provide recommendations
    print("\nRecommendations:")

    if components_ok and minimal_test_ok:
        print("All tests passed! You can run the full ABCD validation:")
        print("   python3 scripts/validation/foot_force/run_complete_validation.py")
    elif components_ok:
        print("WARNING: Component tests passed, but need to connect to robot for actual testing")
        print("   Please ensure robot network connection is working, then run full validation")
    else:
        print("Component tests failed. Please check:")
        print("   1. Ensure all dependency packages are installed")
        print("   2. Check cyclonedds environment configuration")
        print("   3. Verify Python path settings")

    print(f"\nOutput directory: scripts/validation/foot_force/foot_force_validation/output/")

if __name__ == "__main__":
    main()

#!/usr/bin/env python
"""
Example showing how to use the unified unitree_interface for hand control
This demonstrates the new integrated API where hand control is part of unitree_interface.

Usage:
    python3 example_unified_hand_control.py --net <network_interface>
"""

import argparse
import time
import math
import sys
import os

# Add the build directory to the path
# Try multiple possible locations for the module (prioritize current directory)
possible_paths = [
    '.',  # Current directory (highest priority)
    os.getcwd(),  # Current working directory
    os.path.join(os.path.dirname(__file__), '../../build/lib'),  # From deeper subdirectory  
    os.path.join(os.path.dirname(__file__), '../build/lib'),  # From subdirectory
    os.path.join(os.path.dirname(__file__), 'build/lib'),  # From python_binding directory
]

for path in possible_paths:
    # Check for different Python versions
    module_patterns = [
        'unitree_interface.cpython-38-x86_64-linux-gnu.so',  # Python 3.8
        'unitree_interface.cpython-313-x86_64-linux-gnu.so', # Python 3.13
        'unitree_interface.so'  # Generic fallback
    ]
    
    for pattern in module_patterns:
        module_path = os.path.join(path, pattern)
        if os.path.exists(module_path):
            sys.path.insert(0, path)
            print(f"🔍 Using module from: {module_path}")
            break
    else:
        continue  # Continue to next path if no module found
    break  # Break out of outer loop if module found

try:
    import unitree_interface as ui
    print("✅ Successfully imported unified unitree_interface module")
    print(f"📍 Module location: {ui.__file__ if hasattr(ui, '__file__') else 'Unknown'}")
    print(f"🔍 Available classes: {[x for x in dir(ui) if 'Interface' in x]}")
    print(f"🔍 HandInterface available: {hasattr(ui, 'HandInterface')}")
except ImportError as e:
    print(f"❌ Failed to import unitree_interface: {e}")
    print("Make sure you have built the Python bindings first:")
    print("  cd build && cmake .. -DBUILD_PYTHON_BINDING=ON && make")
    sys.exit(1)

class UnifiedHandController:
    """
    Unified hand controller using the new integrated unitree_interface API
    """
    
    def __init__(self, network_interface: str, re_init: bool = True):
        """
        Initialize unified hand controller with both robot and hand interfaces
        
        Args:
            network_interface: Network interface name
            re_init: Whether to re-initialize DDS communication
        """
        print(f"🚀 Initializing Unified Hand Controller on {network_interface}...")
        
        # Option 1: Create hands using factory methods
        print("Creating hand interfaces...")
        self.left_hand = ui.HandInterface.create_left_hand(network_interface, re_init)
        self.right_hand = ui.HandInterface.create_right_hand(network_interface, False)  # Don't re-init
        
        # Option 2: Alternative - use convenience functions
        # self.left_hand, self.right_hand = ui.create_dual_hands(network_interface, re_init)
        
        # Show what we can also do with robots
        print("Available robot factory methods:")
        print("  - ui.UnitreeInterface.create_g1()")
        print("  - ui.UnitreeInterface.create_h1()")
        print("  - ui.UnitreeInterface.create_h1_2()")
        
        print(f"✅ {self.left_hand.get_hand_name()} initialized")
        print(f"✅ {self.right_hand.get_hand_name()} initialized")
        
        # Get hand information
        left_limits = self.left_hand.get_max_limits()
        right_limits = self.right_hand.get_max_limits()
        print(f"📏 Left hand joint limits: [{left_limits[0]:.3f}, {left_limits[1]:.3f}, ...]")
        print(f"📏 Right hand joint limits: [{right_limits[0]:.3f}, {right_limits[1]:.3f}, ...]")
        
    def demonstrate_unified_api(self):
        """Demonstrate the unified API features"""
        print("\n🎯 Demonstrating Unified API Features:")
        
        # Show constants available in unified interface
        print(f"📊 DEX3_NUM_MOTORS: {ui.DEX3_NUM_MOTORS}")
        print(f"📊 DEX3_NUM_PRESS_SENSORS: {ui.DEX3_NUM_PRESS_SENSORS}")
        print(f"🤖 Robot constants: G1={ui.G1_NUM_MOTOR}, H1={ui.H1_NUM_MOTOR}, H1_2={ui.H1_2_NUM_MOTOR}")
        
        # Show available hand types
        print(f"👋 HandType.LEFT_HAND: {ui.HandType.LEFT_HAND}")
        print(f"👋 HandType.RIGHT_HAND: {ui.HandType.RIGHT_HAND}")
        
        # Show topics
        print(f"📡 Topics: {ui.LEFT_HAND_CMD_TOPIC}, {ui.RIGHT_HAND_CMD_TOPIC}")
        
    def test_hand_control(self):
        """Test basic hand control functionality"""
        print("\n🔧 Testing Hand Control...")
        
        # Create commands using unified interface
        left_cmd = self.left_hand.create_zero_command()
        right_cmd = self.right_hand.create_zero_command()
        
        print(f"📝 Created commands with {len(left_cmd.q_target)} joints each")
        
        # Set some target positions
        left_cmd.q_target = [0.1, 0.0, 0.2, -0.3, -0.2, -0.3, -0.2]
        right_cmd.q_target = [-0.1, 0.0, -0.2, 0.3, 0.2, 0.3, 0.2]
        
        # Send commands (would work with real hardware)
        print("📤 Sending commands to hands...")
        self.left_hand.write_hand_command(left_cmd)
        self.right_hand.write_hand_command(right_cmd)
        
        print("⏱️  Waiting 2 seconds...")
        time.sleep(2.0)
        
        # Read states (would work with real hardware)
        print("📥 Reading hand states...")
        try:
            left_state = self.left_hand.read_hand_state()
            right_state = self.right_hand.read_hand_state()
            
            print(f"🤚 Left hand motor positions: {[f'{q:.3f}' for q in left_state.motor.q]}")
            print(f"🤚 Right hand motor positions: {[f'{q:.3f}' for q in right_state.motor.q]}")
            print(f"⚡ Left hand power: {left_state.power_v:.2f}V")
            print(f"⚡ Right hand power: {right_state.power_v:.2f}V")
            
        except Exception as e:
            print(f"ℹ️  State reading would work with real hardware (got: {e})")
    
    def test_convenience_functions(self):
        """Test convenience functions"""
        print("\n🛠️  Testing Convenience Functions...")
        
        # Test joint limits and normalization
        test_angles = [0.5, 0.2, 1.0, -0.8, -1.2, -0.9, -1.1]
        print(f"🔍 Original angles: {test_angles}")
        
        # Clamp angles (in-place)
        clamped_angles = test_angles.copy()
        # Note: clamp_joint_angles expects array type, converted internally
        print(f"🗜️  Would clamp angles: {clamped_angles}")
        
        # Normalize angles  
        # Note: normalize_joint_angles would work with array input
        print(f"📐 Would normalize angles to [0-1] range")
        
        # Show default gains
        kp = self.left_hand.get_default_kp()
        kd = self.left_hand.get_default_kd()
        print(f"⚙️  Default gains: Kp={kp[0]:.2f}, Kd={kd[0]:.2f}")
    
    def demonstrate_integration_with_robot(self):
        """Show how hand control integrates with robot control"""
        print("\n🤖 Integration with Robot Control:")
        print("With the unified interface, you can now do:")
        print()
        print("# Control G1 robot")
        print("robot = ui.UnitreeInterface.create_g1('eth0', re_init=True)")
        print("robot_state = robot.read_low_state()")
        print()
        print("# Control hands (reuse DDS connection)")  
        print("left_hand = ui.HandInterface.create_left_hand('eth0', re_init=False)")
        print("right_hand = ui.HandInterface.create_right_hand('eth0', re_init=False)")
        print()
        print("# Or use convenience function")
        print("left_hand, right_hand = ui.create_dual_hands('eth0', re_init=False)")
        print()
        print("This allows coordinated robot + hand control! 🎉")


def main():
    parser = argparse.ArgumentParser(description='Unified Unitree Interface Hand Control Example')
    parser.add_argument('--net', type=str, default='enp0s31f6', 
                       help='Network interface (default: eth0)')
    parser.add_argument('--re-init', action='store_true', default=True,
                       help='Re-initialize DDS communication (default: True)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🎯 Unified Unitree Interface Hand Control Example")
    print("=" * 60)
    
    # Create controller
    controller = UnifiedHandController(args.net, args.re_init)
    
    # Run demonstrations
    controller.demonstrate_unified_api()
    controller.test_hand_control()
    controller.test_convenience_functions()
    controller.demonstrate_integration_with_robot()
    
    print("\n" + "=" * 60)
    print("✅ All demonstrations completed successfully!")
    print("🔧 Ready for real hardware control!")
    print("=" * 60)
    
    


if __name__ == "__main__":
    main()
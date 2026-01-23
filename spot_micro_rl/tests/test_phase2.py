#!/usr/bin/env python3
"""
Test Script - Phase 2 Integration Validation
Vérifie que tous les composants Phase 2 fonctionnent correctement
"""

import os
import sys
import pickle
import numpy as np

# Add package to path
script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.join(script_dir, '..', 'src')
sys.path.insert(0, package_dir)

def test_imports():
    """Test 1: Vérifier imports Phase 1 + Phase 2"""
    print("\n" + "="*70)
    print("TEST 1: Imports")
    print("="*70)
    
    try:
        # Phase 1
        from spot_micro_rl import MotorModel
        from spot_micro_rl import SpotEnvRandomizer, MinimalRandomizer, AggressiveRandomizer
        from spot_micro_rl import HeightField, FlatTerrain, GentleTerrain, RoughTerrain
        
        # Core
        from spot_micro_rl import SpotMicroEnv
        from spot_micro_rl import Policy, Normalizer, ARSAgent
        
        print("✅ All imports successful!")
        print("   - Phase 1 components: MotorModel, Randomizers, HeightField")
        print("   - Core components: SpotMicroEnv, Policy, Normalizer, ARSAgent")
        return True
    except Exception as e:
        print(f"❌ Import failed: {e}")
        return False


def test_environment_creation():
    """Test 2: Créer environment avec Phase 1 features"""
    print("\n" + "="*70)
    print("TEST 2: Environment Creation")
    print("="*70)
    
    try:
        from spot_micro_rl import SpotMicroEnv, SpotEnvRandomizer
        
        # Environment avec toutes les features
        env = SpotMicroEnv(
            render=False,
            motor_model_enabled=True,
            env_randomizer=SpotEnvRandomizer(),
            terrain_type='flat',
            max_timesteps=100
        )
        
        print("✅ Environment created with:")
        print("   - Motor models: True")
        print("   - Randomization: SpotEnvRandomizer")
        print("   - Terrain: flat")
        
        # Test reset
        obs = env.reset()
        print(f"✅ Environment reset successful")
        print(f"   - Observation shape: {obs.shape}")
        print(f"   - Expected: (46,)")  # Updated from 33 to 46
        
        env.close()
        
        return obs.shape == (46,)  # Updated from 33 to 46
    except Exception as e:
        print(f"❌ Environment creation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_save_load_policy():
    """Test 3: Save/Load policy (pickle format)"""
    print("\n" + "="*70)
    print("TEST 3: Save/Load Policy (Pickle Format)")
    print("="*70)
    
    try:
        from spot_micro_rl import Policy, Normalizer, ARSAgent, SpotMicroEnv
        
        # Create simple agent
        env = SpotMicroEnv(render=False, max_timesteps=50)
        policy = Policy(state_dim=33, action_dim=14)
        normalizer = Normalizer(33)
        agent = ARSAgent(normalizer, policy, env)
        
        # Set some random weights
        agent.policy.theta = np.random.randn(14, 33)
        original_theta = agent.policy.theta.copy()
        
        # Save
        test_dir = os.path.join(script_dir, '..', 'models', 'checkpoints')
        os.makedirs(test_dir, exist_ok=True)
        test_file = os.path.join(test_dir, 'test_policy')
        
        agent.save_policy(test_file)
        print(f"✅ Policy saved to: {test_file}")
        
        # Modify weights
        agent.policy.theta = np.zeros((14, 33))
        
        # Load
        agent.load_policy(test_file)
        print(f"✅ Policy loaded from: {test_file}")
        
        # Verify
        if np.allclose(agent.policy.theta, original_theta):
            print("✅ Loaded weights match original!")
            
            # Cleanup
            if os.path.exists(test_file):
                os.remove(test_file)
                print("✅ Test file cleaned up")
            
            env.close()
            return True
        else:
            print("❌ Loaded weights don't match!")
            env.close()
            return False
            
    except Exception as e:
        print(f"❌ Save/Load test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_survival_data_format():
    """Test 4: Survival data format (pour gmbc_data.py)"""
    print("\n" + "="*70)
    print("TEST 4: Survival Data Format")
    print("="*70)
    
    try:
        # Create test survival data
        test_survival_data = [850, 920, 780, 1000, 650, 890, 950, 820, 990, 880]
        
        # Save
        results_dir = os.path.join(script_dir, '..', 'results', 'survival_data')
        os.makedirs(results_dir, exist_ok=True)
        
        test_file = os.path.join(results_dir, 'test_survival_10')
        
        with open(test_file, 'wb') as f:
            pickle.dump(test_survival_data, f)
        
        print(f"✅ Survival data saved: {test_file}")
        print(f"   - Episodes: {len(test_survival_data)}")
        print(f"   - Format: pickle")
        
        # Load
        with open(test_file, 'rb') as f:
            loaded_data = pickle.load(f)
        
        print(f"✅ Survival data loaded")
        print(f"   - Mean survival: {np.mean(loaded_data):.2f}")
        print(f"   - Std: {np.std(loaded_data):.2f}")
        
        # Cleanup
        if os.path.exists(test_file):
            os.remove(test_file)
            print("✅ Test file cleaned up")
        
        return loaded_data == test_survival_data
        
    except Exception as e:
        print(f"❌ Survival data test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_directory_structure():
    """Test 5: Vérifier structure models/ et results/"""
    print("\n" + "="*70)
    print("TEST 5: Directory Structure")
    print("="*70)
    
    try:
        base_dir = os.path.join(script_dir, '..')
        
        required_dirs = [
            'models',
            'models/urdf',
            'models/checkpoints',
            'results',
            'results/survival_data',
            'results/training_logs',
            'results/plots'
        ]
        
        all_exist = True
        for dir_path in required_dirs:
            full_path = os.path.join(base_dir, dir_path)
            exists = os.path.exists(full_path)
            
            if exists:
                print(f"✅ {dir_path}/")
            else:
                print(f"❌ {dir_path}/ (missing)")
                all_exist = False
        
        # Check README files
        readme_files = [
            'models/README.md',
            'results/README.md',
            'PHASE2_README.md',
            'PHASES_RECAP.md'
        ]
        
        for readme in readme_files:
            full_path = os.path.join(base_dir, readme)
            exists = os.path.exists(full_path)
            
            if exists:
                print(f"✅ {readme}")
            else:
                print(f"❌ {readme} (missing)")
                all_exist = False
        
        return all_exist
        
    except Exception as e:
        print(f"❌ Directory structure test failed: {e}")
        return False


def test_scripts_exist():
    """Test 6: Vérifier scripts Phase 2"""
    print("\n" + "="*70)
    print("TEST 6: Scripts Existence")
    print("="*70)
    
    try:
        scripts_dir = os.path.join(os.path.dirname(script_dir), 'scripts')
        
        required_scripts = [
            'spot_rl_train_ars.py',
            'gmbc_data.py',
            'spot_rl_eval.py',
            'spot_rl_train_sac.py',
            'spot_rl_train_td3.py'
        ]
        
        all_exist = True
        for script in required_scripts:
            full_path = os.path.join(scripts_dir, script)
            exists = os.path.exists(full_path)
            
            if exists:
                # Check if executable
                try:
                    with open(full_path, 'r', encoding='utf-8', errors='ignore') as f:
                        first_line = f.readline()
                        has_shebang = first_line.startswith('#!')
                    
                    status = "✅" if has_shebang else "⚠️"
                    print(f"{status} {script} {'(executable)' if has_shebang else '(not executable)'}")
                except Exception as e:
                    print(f"✅ {script} (could not check shebang: {e})")
            else:
                print(f"❌ {script} (missing)")
                all_exist = False
        
        return all_exist
        
    except Exception as e:
        print(f"❌ Scripts test failed: {e}")
        return False


def main():
    """Run all tests"""
    print("\n" + "#"*70)
    print("#  PHASE 2 INTEGRATION VALIDATION")
    print("#"*70)
    
    tests = [
        ("Imports", test_imports),
        ("Environment Creation", test_environment_creation),
        ("Save/Load Policy", test_save_load_policy),
        ("Survival Data Format", test_survival_data_format),
        ("Directory Structure", test_directory_structure),
        ("Scripts Existence", test_scripts_exist)
    ]
    
    results = []
    
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n❌ Test '{name}' crashed: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status:10s} {name}")
    
    print(f"\n{'='*70}")
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 ALL TESTS PASSED! Phase 2 integration is VALID!")
        print("\nReady for:")
        print("  1. Training with spot_rl_train_ars.py")
        print("  2. Analysis with gmbc_data.py")
        print("  3. Phase 3 integration (SAC/TD3)")
    else:
        print(f"\n⚠️  {total - passed} test(s) failed. Please fix before proceeding.")
    
    print("="*70)
    
    return passed == total


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)

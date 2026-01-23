#!/usr/bin/env python3
"""
PHASE 1 INTEGRATION VALIDATION
================================

Tests pour Motor Models, Domain Randomization et Terrain:
1. Imports des modules Phase 1
2. Création motor models
3. Domain randomization
4. Terrain generation
5. Environnement avec Phase 1 features
"""

import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Test results
test_results = []


def test_imports():
    """Test 1: Import all Phase 1 components"""
    print("\n" + "=" * 70)
    print("TEST 1: Imports")
    print("=" * 70)
    
    try:
        # Core imports
        from src.spot_micro_rl.spot_env import SpotMicroEnv
        
        # Phase 1 imports
        from src.spot_micro_rl.motor import MotorModel
        from src.spot_micro_rl.env_randomizer_base import EnvRandomizerBase
        from src.spot_micro_rl.spot_env_randomizer import (
            SpotEnvRandomizer,
            MinimalRandomizer,
            AggressiveRandomizer
        )
        from src.spot_micro_rl.heightfield import (
            HeightField,
            FlatTerrain,
            GentleTerrain,
            RoughTerrain
        )
        
        print("✅ All imports successful!")
        print("   - Core: SpotMicroEnv")
        print("   - Motor: MotorModel")
        print("   - Randomizers: SpotEnvRandomizer, MinimalRandomizer, AggressiveRandomizer")
        print("   - Terrain: HeightField, FlatTerrain, GentleTerrain, RoughTerrain")
        
        return True, "All imports successful"
    except Exception as e:
        print(f"❌ Import failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_motor_models():
    """Test 2: Create and test motor models"""
    print("\n" + "=" * 70)
    print("TEST 2: Motor Models")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.motor import MotorModel
        
        # Create motor model
        motor = MotorModel(
            kt=0.05,           # Torque constant
            r=1.0,             # Resistance
            l=0.001,           # Inductance
            friction_coeff=0.01,
            damping=0.001,
            backlash=0.01
        )
        
        # Test torque calculation
        voltage = 12.0
        position = 0.0
        velocity = 1.0
        desired_position = 0.5
        
        torque = motor.get_torque(voltage, position, velocity, desired_position)
        
        assert isinstance(torque, (int, float)), "Torque should be numeric"
        assert -100 < torque < 100, f"Torque {torque} out of reasonable range"
        
        # Create 12 motors
        motors = {
            f'motor_{i}': MotorModel(
                kt=0.05, r=1.0, l=0.001,
                friction_coeff=0.01, damping=0.001, backlash=0.01
            )
            for i in range(12)
        }
        
        print("✅ Motor models working correctly")
        print(f"   - Single motor torque: {torque:.4f} N.m")
        print(f"   - Created {len(motors)} motors")
        print(f"   - Parameters: kt={motor.kt}, R={motor.r}, L={motor.l}")
        
        return True, "Motor models functional"
    except Exception as e:
        print(f"❌ Motor models test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_randomizers():
    """Test 3: Domain randomization"""
    print("\n" + "=" * 70)
    print("TEST 3: Domain Randomization")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.spot_env_randomizer import (
            SpotEnvRandomizer,
            MinimalRandomizer,
            AggressiveRandomizer
        )
        
        # Test all 3 randomizers
        randomizers = [
            ("SpotEnvRandomizer", SpotEnvRandomizer()),
            ("MinimalRandomizer", MinimalRandomizer()),
            ("AggressiveRandomizer", AggressiveRandomizer())
        ]
        
        for name, randomizer in randomizers:
            # Test randomization
            params = randomizer.randomize_env(None)
            
            assert 'base_mass_range' in params, "Missing base_mass_range"
            assert 'motor_kp_range' in params, "Missing motor_kp_range"
            assert 'ground_friction_range' in params, "Missing ground_friction_range"
            
        print("✅ Domain randomization working correctly")
        print(f"   - Tested {len(randomizers)} randomizer profiles")
        print(f"   - Parameters randomized: mass, friction, motor gains, latency, noise")
        
        return True, "Domain randomization functional"
    except Exception as e:
        print(f"❌ Randomization test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_terrain():
    """Test 4: Terrain generation"""
    print("\n" + "=" * 70)
    print("TEST 4: Terrain Generation")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.heightfield import (
            FlatTerrain,
            GentleTerrain,
            RoughTerrain
        )
        
        # Test all terrain types
        terrains = [
            ("Flat", FlatTerrain()),
            ("Gentle", GentleTerrain(resolution=0.01, size=5.0)),
            ("Rough", RoughTerrain(resolution=0.01, size=5.0, max_height=0.05))
        ]
        
        for name, terrain in terrains:
            heightfield = terrain.generate()
            
            assert heightfield is not None, f"{name} terrain returned None"
            assert isinstance(heightfield, np.ndarray), f"{name} terrain not numpy array"
            assert heightfield.ndim == 2, f"{name} terrain not 2D"
            
        print("✅ Terrain generation working correctly")
        print(f"   - Tested {len(terrains)} terrain types")
        print(f"   - Flat: {terrains[0][1].generate().shape}")
        print(f"   - Gentle: {terrains[1][1].generate().shape}")
        print(f"   - Rough: {terrains[2][1].generate().shape}")
        
        return True, "Terrain generation functional"
    except Exception as e:
        print(f"❌ Terrain test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_environment_with_phase1():
    """Test 5: Environment with Phase 1 features"""
    print("\n" + "=" * 70)
    print("TEST 5: Environment with Phase 1 Features")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.spot_env import SpotMicroEnv
        from src.spot_micro_rl.motor import MotorModel
        from src.spot_micro_rl.spot_env_randomizer import SpotEnvRandomizer
        from src.spot_micro_rl.heightfield import GentleTerrain
        
        # Create motor models
        motor_models = {
            f'motor_{i}': MotorModel(
                kt=0.05, r=1.0, l=0.001,
                friction_coeff=0.01, damping=0.001, backlash=0.01
            )
            for i in range(12)
        }
        
        # Create randomizer
        randomizer = SpotEnvRandomizer()
        
        # Create terrain
        terrain = GentleTerrain(resolution=0.01, size=5.0)
        
        # Create environment with all Phase 1 features
        env = SpotMicroEnv(
            render=False,
            motor_models=motor_models,
            env_randomizer=randomizer,
            height_field=terrain
        )
        
        # Test reset
        obs = env.reset()
        
        assert obs is not None, "Observation is None"
        assert obs.shape == (46,), f"Wrong observation shape: {obs.shape}"
        
        # Test step
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        
        assert obs.shape == (46,), f"Wrong observation shape after step: {obs.shape}"
        assert isinstance(reward, (int, float)), "Reward should be numeric"
        assert isinstance(done, bool), "Done should be boolean"
        
        env.close()
        
        print("✅ Environment with Phase 1 features working correctly")
        print(f"   - Motor models: {len(motor_models)} motors")
        print(f"   - Domain randomization: SpotEnvRandomizer")
        print(f"   - Terrain: GentleTerrain")
        print(f"   - Observation shape: {obs.shape}")
        print(f"   - Action space: {env.action_space.shape}")
        
        return True, "Environment with Phase 1 functional"
    except Exception as e:
        print(f"❌ Environment test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def main():
    """Run all tests"""
    print("#" * 70)
    print("#  PHASE 1 INTEGRATION VALIDATION")
    print("#" * 70)
    
    tests = [
        ("Imports", test_imports),
        ("Motor Models", test_motor_models),
        ("Domain Randomization", test_randomizers),
        ("Terrain Generation", test_terrain),
        ("Environment with Phase 1", test_environment_with_phase1),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            passed, message = test_func()
            results.append((test_name, passed, message))
        except Exception as e:
            results.append((test_name, False, str(e)))
    
    # Print summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    
    for test_name, passed, message in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status:12s} {test_name}")
    
    # Final result
    passed_count = sum(1 for _, passed, _ in results if passed)
    total_count = len(results)
    
    print("=" * 70)
    print(f"Results: {passed_count}/{total_count} tests passed")
    print()
    
    if passed_count == total_count:
        print("🎉 ALL TESTS PASSED! Phase 1 integration is VALID!")
        print()
        print("Phase 1 features:")
        print("  ✅ Motor models réalistes")
        print("  ✅ Domain randomization")
        print("  ✅ Terrain varié")
    else:
        print("❌ SOME TESTS FAILED. Please fix errors above.")
    
    print("=" * 70)
    
    return passed_count == total_count


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)

#!/usr/bin/env python3
"""
PHASE 3 INTEGRATION VALIDATION
================================

Tests pour SAC et TD3:
1. Imports des nouveaux modules
2. Création des agents SAC et TD3
3. Save/Load des agents
4. Replay buffer functionality
5. Training step (court)
6. Scripts d'entraînement existence
"""

import sys
import os
import numpy as np
import torch

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Test results
test_results = []


def test_imports():
    """Test 1: Import all Phase 3 components"""
    print("\n" + "=" * 70)
    print("TEST 1: Imports")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.networks import (
            Actor, Critic, GaussianActor, SoftCritic,
            soft_update, hard_update
        )
        from src.spot_micro_rl.replay_buffer import ReplayBuffer, PrioritizedReplayBuffer
        from src.spot_micro_rl.sac import SACAgent, SACPolicy
        from src.spot_micro_rl.td3 import TD3Agent, TD3Policy
        
        print("✅ All imports successful!")
        print("   - Networks: Actor, Critic, GaussianActor, SoftCritic")
        print("   - Replay buffers: ReplayBuffer, PrioritizedReplayBuffer")
        print("   - Agents: SACAgent, TD3Agent")
        print("   - Policies: SACPolicy, TD3Policy")
        
        return True, "All imports successful"
    except Exception as e:
        print(f"❌ Import failed: {e}")
        return False, str(e)


def test_replay_buffer():
    """Test 2: Replay buffer functionality"""
    print("\n" + "=" * 70)
    print("TEST 2: Replay Buffer")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.replay_buffer import ReplayBuffer
        
        # Create buffer
        state_dim = 46
        action_dim = 14
        buffer = ReplayBuffer(state_dim, action_dim, max_size=1000)
        
        # Add some transitions
        for i in range(100):
            state = np.random.randn(state_dim)
            action = np.random.randn(action_dim)
            reward = np.random.randn()
            next_state = np.random.randn(state_dim)
            done = np.random.rand() > 0.9
            
            buffer.add(state, action, reward, next_state, done)
        
        # Sample batch
        batch = buffer.sample(32)
        states, actions, rewards, next_states, dones = batch
        
        # Verify shapes
        assert states.shape == (32, state_dim), f"State shape mismatch: {states.shape}"
        assert actions.shape == (32, action_dim), f"Action shape mismatch: {actions.shape}"
        assert rewards.shape == (32, 1), f"Reward shape mismatch: {rewards.shape}"
        
        print("✅ Replay buffer working correctly")
        print(f"   - Buffer size: {len(buffer)}/1000")
        print(f"   - Sampled batch: 32 transitions")
        print(f"   - State shape: {states.shape}")
        print(f"   - Action shape: {actions.shape}")
        
        return True, "Replay buffer functional"
    except Exception as e:
        print(f"❌ Replay buffer test failed: {e}")
        return False, str(e)


def test_td3_agent():
    """Test 3: TD3 Agent creation and basic operations"""
    print("\n" + "=" * 70)
    print("TEST 3: TD3 Agent")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.td3 import TD3Agent
        
        state_dim = 46
        action_dim = 14
        max_action = 1.0
        
        # Create agent
        agent = TD3Agent(
            state_dim=state_dim,
            action_dim=action_dim,
            max_action=max_action,
            hidden_dims=[64, 64],
            buffer_size=10000,
            device='cpu'
        )
        
        # Test action selection
        state = np.random.randn(state_dim)
        action = agent.select_action(state, noise=0.1)
        
        assert action.shape == (action_dim,), f"Action shape mismatch: {action.shape}"
        assert np.all(np.abs(action) <= max_action + 0.1), "Action exceeds bounds"
        
        # Add some data to buffer
        for i in range(50):
            s = np.random.randn(state_dim)
            a = np.random.randn(action_dim)
            r = np.random.randn()
            ns = np.random.randn(state_dim)
            d = False
            agent.replay_buffer.add(s, a, r, ns, d)
        
        print("✅ TD3 agent created successfully")
        print(f"   - State dim: {state_dim}")
        print(f"   - Action dim: {action_dim}")
        print(f"   - Action range: [-{max_action}, {max_action}]")
        print(f"   - Buffer size: {len(agent.replay_buffer)}")
        
        return True, "TD3 agent functional"
    except Exception as e:
        print(f"❌ TD3 agent test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_sac_agent():
    """Test 4: SAC Agent creation and basic operations"""
    print("\n" + "=" * 70)
    print("TEST 4: SAC Agent")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.sac import SACAgent
        
        state_dim = 46
        action_dim = 14
        max_action = 1.0
        
        # Create agent with auto entropy tuning
        agent = SACAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            max_action=max_action,
            hidden_dims=[64, 64],
            auto_entropy_tuning=True,
            buffer_size=10000,
            device='cpu'
        )
        
        # Test action selection (stochastic)
        state = np.random.randn(state_dim)
        action_stochastic = agent.select_action(state, deterministic=False)
        action_deterministic = agent.select_action(state, deterministic=True)
        
        assert action_stochastic.shape == (action_dim,), "Stochastic action shape mismatch"
        assert action_deterministic.shape == (action_dim,), "Deterministic action shape mismatch"
        
        # Add some data to buffer
        for i in range(50):
            s = np.random.randn(state_dim)
            a = np.random.randn(action_dim)
            r = np.random.randn()
            ns = np.random.randn(state_dim)
            d = False
            agent.replay_buffer.add(s, a, r, ns, d)
        
        print("✅ SAC agent created successfully")
        print(f"   - State dim: {state_dim}")
        print(f"   - Action dim: {action_dim}")
        print(f"   - Auto entropy tuning: {agent.auto_entropy_tuning}")
        print(f"   - Initial alpha: {agent.alpha.item():.4f}")
        print(f"   - Buffer size: {len(agent.replay_buffer)}")
        
        return True, "SAC agent functional"
    except Exception as e:
        print(f"❌ SAC agent test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_save_load():
    """Test 5: Save and load agents"""
    print("\n" + "=" * 70)
    print("TEST 5: Save/Load Agents")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.td3 import TD3Agent
        from src.spot_micro_rl.sac import SACAgent
        
        state_dim = 46
        action_dim = 14
        
        # Test TD3 save/load
        td3_agent = TD3Agent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=[32, 32],
            device='cpu'
        )
        
        td3_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'checkpoints', 'test_td3.pth')
        td3_agent.save(td3_path)
        
        td3_agent_loaded = TD3Agent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=[32, 32],
            device='cpu'
        )
        td3_agent_loaded.load(td3_path)
        
        # Cleanup
        os.remove(td3_path)
        buffer_file = td3_path.replace('.pth', '_buffer.npz')
        if os.path.exists(buffer_file):
            os.remove(buffer_file)
        
        # Test SAC save/load
        sac_agent = SACAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=[32, 32],
            device='cpu'
        )
        
        sac_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'checkpoints', 'test_sac.pth')
        sac_agent.save(sac_path)
        
        sac_agent_loaded = SACAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=[32, 32],
            device='cpu'
        )
        sac_agent_loaded.load(sac_path)
        
        # Cleanup
        os.remove(sac_path)
        buffer_file = sac_path.replace('.pth', '_buffer.npz')
        if os.path.exists(buffer_file):
            os.remove(buffer_file)
        
        print("✅ Save/Load successful")
        print("   - TD3 agent saved and loaded")
        print("   - SAC agent saved and loaded")
        print("   - Test files cleaned up")
        
        return True, "Save/Load functional"
    except Exception as e:
        print(f"❌ Save/Load test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_training_step():
    """Test 6: Execute training step"""
    print("\n" + "=" * 70)
    print("TEST 6: Training Step")
    print("=" * 70)
    
    try:
        from src.spot_micro_rl.td3 import TD3Agent
        from src.spot_micro_rl.sac import SACAgent
        
        state_dim = 46
        action_dim = 14
        
        # Test TD3 training
        td3_agent = TD3Agent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=[32, 32],
            device='cpu'
        )
        
        # Fill buffer
        for i in range(300):
            s = np.random.randn(state_dim)
            a = np.random.randn(action_dim)
            r = np.random.randn()
            ns = np.random.randn(state_dim)
            d = False
            td3_agent.replay_buffer.add(s, a, r, ns, d)
        
        # Train
        critic_loss, actor_loss = td3_agent.train(batch_size=64)
        
        assert critic_loss is not None, "TD3 critic loss is None"
        
        # Test SAC training
        sac_agent = SACAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=[32, 32],
            device='cpu'
        )
        
        # Fill buffer
        for i in range(300):
            s = np.random.randn(state_dim)
            a = np.random.randn(action_dim)
            r = np.random.randn()
            ns = np.random.randn(state_dim)
            d = False
            sac_agent.replay_buffer.add(s, a, r, ns, d)
        
        # Train
        critic_loss_sac, actor_loss_sac, alpha_loss = sac_agent.train(batch_size=64)
        
        assert critic_loss_sac is not None, "SAC critic loss is None"
        assert actor_loss_sac is not None, "SAC actor loss is None"
        
        print("✅ Training steps successful")
        print(f"   - TD3 critic loss: {critic_loss:.4f}")
        print(f"   - SAC critic loss: {critic_loss_sac:.4f}")
        print(f"   - SAC actor loss: {actor_loss_sac:.4f}")
        if alpha_loss:
            print(f"   - SAC alpha loss: {alpha_loss:.4f}")
        
        return True, "Training functional"
    except Exception as e:
        print(f"❌ Training test failed: {e}")
        import traceback
        traceback.print_exc()
        return False, str(e)


def test_training_scripts():
    """Test 7: Check training scripts exist"""
    print("\n" + "=" * 70)
    print("TEST 7: Training Scripts")
    print("=" * 70)
    
    try:
        scripts_dir = os.path.dirname(__file__)
        
        td3_script = os.path.join(scripts_dir, 'spot_rl_train_td3.py')
        sac_script = os.path.join(scripts_dir, 'spot_rl_train_sac.py')
        
        assert os.path.exists(td3_script), f"TD3 script not found: {td3_script}"
        assert os.path.exists(sac_script), f"SAC script not found: {sac_script}"
        
        # Check if executable
        assert os.access(td3_script, os.R_OK), "TD3 script not readable"
        assert os.access(sac_script, os.R_OK), "SAC script not readable"
        
        # Check content (contains main training logic)
        with open(td3_script, 'r', encoding='utf-8', errors='ignore') as f:
            td3_content = f.read()
            assert 'TD3Agent' in td3_content, "TD3 script doesn't use TD3Agent"
            assert 'train_td3' in td3_content, "TD3 script missing train function"
        
        with open(sac_script, 'r', encoding='utf-8', errors='ignore') as f:
            sac_content = f.read()
            assert 'SACAgent' in sac_content, "SAC script doesn't use SACAgent"
            assert 'train_sac' in sac_content, "SAC script missing train function"
        
        print("✅ Training scripts validated")
        print(f"   - {os.path.basename(td3_script)} (executable)")
        print(f"   - {os.path.basename(sac_script)} (executable)")
        
        return True, "Scripts exist and valid"
    except Exception as e:
        print(f"❌ Scripts test failed: {e}")
        return False, str(e)


def main():
    """Run all tests"""
    print("#" * 70)
    print("#  PHASE 3 INTEGRATION VALIDATION")
    print("#" * 70)
    
    tests = [
        ("Imports", test_imports),
        ("Replay Buffer", test_replay_buffer),
        ("TD3 Agent", test_td3_agent),
        ("SAC Agent", test_sac_agent),
        ("Save/Load Agents", test_save_load),
        ("Training Step", test_training_step),
        ("Training Scripts", test_training_scripts),
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
        print("🎉 ALL TESTS PASSED! Phase 3 integration is VALID!")
        print()
        print("Ready for:")
        print("  1. Training with SAC: python scripts/spot_rl_train_sac.py")
        print("  2. Training with TD3: python scripts/spot_rl_train_td3.py")
        print("  3. Compare ARS vs SAC vs TD3 performance")
    else:
        print("❌ SOME TESTS FAILED. Please fix errors above.")
    
    print("=" * 70)
    
    return passed_count == total_count


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)

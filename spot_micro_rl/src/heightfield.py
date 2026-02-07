#!/usr/bin/env python3
"""Heightfield terrain generator for SpotMicro training.

Generates varied terrain (hills, slopes, obstacles) to train robust locomotion.
Without this, robot only learns to walk on perfectly flat ground.

Adapted from spot_mini_mini for RL_app
"""

import pybullet as p
import pybullet_data
import numpy as np
import os

# Terrain generation modes
USE_PROGRAMMATIC = 0   # Procedurally generated random terrain
USE_TERRAIN_PNG = 1    # Load from PNG heightmap
USE_DEEPLO_CSV = 2     # Load from CSV file

# Heightfield parameters
NUM_HEIGHTFIELD_ROWS = 256
NUM_HEIGHTFIELD_COLUMNS = 256


class HeightField:
    """Generates varied terrain for robust locomotion training.
    
    Provides three terrain generation modes:
    1. **Programmatic**: Random hills and valleys
    2. **PNG**: Load from grayscale heightmap image
    3. **CSV**: Load from text file
    
    Benefits of varied terrain:
    - Prevents overfitting to flat ground
    - Trains adaptive foot placement
    - Improves robustness to uneven surfaces
    - Essential for outdoor deployment
    """
    
    def __init__(self, mode=USE_PROGRAMMATIC):
        """Initialize heightfield generator.
        
        Args:
            mode: Terrain generation mode:
                  - USE_PROGRAMMATIC: Random procedural generation
                  - USE_TERRAIN_PNG: Load from PNG file
                  - USE_DEEPLO_CSV: Load from CSV file
        """
        self.mode = mode
        self.hf_id = 0
        self.terrain_shape = 0
        self.terrain_body = 0
        self.heightfield_data = [0] * NUM_HEIGHTFIELD_ROWS * NUM_HEIGHTFIELD_COLUMNS
        
        # Texture (optional, for visualization)
        self.texture_id = -1
    
    def generate_terrain(self, env, height_perturbation_range=0.08):
        """Generate terrain in the environment.
        
        Args:
            env: SpotMicroEnv instance
            height_perturbation_range: Maximum height variation (meters)
                                      Typical: 0.05-0.15m
                                      Higher = more difficult terrain
        
        Returns:
            int: PyBullet body ID of created terrain
        """
        # Disable rendering for faster generation
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        
        if self.mode == USE_PROGRAMMATIC:
            self.terrain_body = self._generate_programmatic_terrain(
                env, height_perturbation_range
            )
        
        elif self.mode == USE_TERRAIN_PNG:
            self.terrain_body = self._generate_png_terrain(env)
        
        elif self.mode == USE_DEEPLO_CSV:
            self.terrain_body = self._generate_csv_terrain(env)
        
        else:
            raise ValueError(f"Unknown terrain mode: {self.mode}")
        
        # Re-enable rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        
        return self.terrain_body
    
    def _generate_programmatic_terrain(self, env, height_perturbation_range):
        """Generate random procedural terrain.
        
        Creates a grid of random height values with 2x2 blocks
        for smoother appearance.
        
        Args:
            env: Environment
            height_perturbation_range: Max height variation (m)
        
        Returns:
            int: Terrain body ID
        """
        np.random.seed()  # Different terrain each time
        
        # Generate heightfield with 2x2 blocks for smoother terrain
        for j in range(int(NUM_HEIGHTFIELD_COLUMNS / 2)):
            for i in range(int(NUM_HEIGHTFIELD_ROWS / 2)):
                # Random height for this block
                height = np.random.uniform(0, height_perturbation_range)
                
                # Apply to 2x2 block
                self.heightfield_data[2*i + 2*j * NUM_HEIGHTFIELD_ROWS] = height
                self.heightfield_data[2*i + 1 + 2*j * NUM_HEIGHTFIELD_ROWS] = height
                self.heightfield_data[2*i + (2*j + 1) * NUM_HEIGHTFIELD_ROWS] = height
                self.heightfield_data[2*i + 1 + (2*j + 1) * NUM_HEIGHTFIELD_ROWS] = height
        
        # Create collision shape
        self.terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[0.06, 0.06, 0.8],  # X, Y, Z scaling
            heightfieldTextureScaling=(NUM_HEIGHTFIELD_ROWS - 1) / 2,
            heightfieldData=self.heightfield_data,
            numHeightfieldRows=NUM_HEIGHTFIELD_ROWS,
            numHeightfieldColumns=NUM_HEIGHTFIELD_COLUMNS
        )
        
        # Create multibody
        terrain = p.createMultiBody(0, self.terrain_shape)
        
        # Position at origin
        p.resetBasePositionAndOrientation(
            terrain,
            [0, 0, 0.0],
            [0, 0, 0, 1]
        )
        
        # Set friction
        p.changeDynamics(
            terrain,
            -1,
            lateralFriction=1.0
        )
        
        # Set color (grayish)
        p.changeVisualShape(
            terrain,
            -1,
            rgbaColor=[0.8, 0.8, 0.8, 1]
        )
        
        self.hf_id = self.terrain_shape
        
        return terrain
    
    def _generate_png_terrain(self, env):
        """Generate terrain from PNG heightmap.
        
        PNG should be:
        - Grayscale image
        - Black (0) = low elevation
        - White (255) = high elevation
        - Size: any (will be scaled)
        
        Args:
            env: Environment
        
        Returns:
            int: Terrain body ID
        """
        # Path to heightmap (relative to package)
        heightmap_path = "heightmaps/terrain_height.png"
        texture_path = "heightmaps/terrain_texture.png"
        
        # Check if files exist
        if not os.path.exists(heightmap_path):
            print(f"  Heightmap not found: {heightmap_path}")
            print("   Falling back to programmatic terrain")
            return self._generate_programmatic_terrain(env, 0.08)
        
        # Create collision shape from PNG
        self.terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[0.05, 0.05, 1.8],
            fileName=heightmap_path
        )
        
        # Create multibody
        terrain = p.createMultiBody(0, self.terrain_shape)
        
        # Load texture if available
        if os.path.exists(texture_path):
            self.texture_id = p.loadTexture(texture_path)
            p.changeVisualShape(
                terrain,
                -1,
                textureUniqueId=self.texture_id
            )
        
        # Position
        p.resetBasePositionAndOrientation(
            terrain,
            [0, 0, 0.1],
            [0, 0, 0, 1]
        )
        
        # Friction
        p.changeDynamics(
            terrain,
            -1,
            lateralFriction=1.0
        )
        
        self.hf_id = self.terrain_shape
        
        return terrain
    
    def _generate_csv_terrain(self, env):
        """Generate terrain from CSV heightmap file.
        
        CSV should contain space or comma-separated height values.
        
        Args:
            env: Environment
        
        Returns:
            int: Terrain body ID
        """
        heightmap_path = "heightmaps/terrain.txt"
        
        if not os.path.exists(heightmap_path):
            print(f"  Heightmap not found: {heightmap_path}")
            print("   Falling back to programmatic terrain")
            return self._generate_programmatic_terrain(env, 0.08)
        
        # Create collision shape from CSV
        self.terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[0.5, 0.5, 2.5],
            fileName=heightmap_path,
            heightfieldTextureScaling=128
        )
        
        # Create multibody
        terrain = p.createMultiBody(0, self.terrain_shape)
        
        # Position
        p.resetBasePositionAndOrientation(
            terrain,
            [0, 0, 0],
            [0, 0, 0, 1]
        )
        
        # Friction
        p.changeDynamics(
            terrain,
            -1,
            lateralFriction=1.0
        )
        
        self.hf_id = self.terrain_shape
        
        return terrain
    
    def update_terrain(self, env, new_height_range=0.08):
        """Update existing terrain with new random heights.
        
        Useful for curriculum learning - start with flat terrain,
        gradually increase difficulty.
        
        Args:
            env: Environment
            new_height_range: New max height variation
        """
        if self.mode != USE_PROGRAMMATIC:
            print("  Terrain update only supported for programmatic mode")
            return
        
        # Generate new heights
        for j in range(int(NUM_HEIGHTFIELD_COLUMNS / 2)):
            for i in range(int(NUM_HEIGHTFIELD_ROWS / 2)):
                height = np.random.uniform(0, new_height_range)
                
                self.heightfield_data[2*i + 2*j * NUM_HEIGHTFIELD_ROWS] = height
                self.heightfield_data[2*i + 1 + 2*j * NUM_HEIGHTFIELD_ROWS] = height
                self.heightfield_data[2*i + (2*j + 1) * NUM_HEIGHTFIELD_ROWS] = height
                self.heightfield_data[2*i + 1 + (2*j + 1) * NUM_HEIGHTFIELD_ROWS] = height
        
        # Remove old terrain
        if self.terrain_body > 0:
            p.removeBody(self.terrain_body)
        
        # Create new terrain
        self.terrain_body = self._generate_programmatic_terrain(env, new_height_range)


# Preset terrain configurations

class FlatTerrain(HeightField):
    """Perfectly flat terrain for initial training."""
    def __init__(self):
        super().__init__(mode=USE_PROGRAMMATIC)
    
    def generate_terrain(self, env, height_perturbation_range=0.0):
        return super().generate_terrain(env, 0.0)


class GentleTerrain(HeightField):
    """Gentle hills for intermediate training."""
    def __init__(self):
        super().__init__(mode=USE_PROGRAMMATIC)
    
    def generate_terrain(self, env, height_perturbation_range=0.05):
        return super().generate_terrain(env, 0.05)


class RoughTerrain(HeightField):
    """Rough terrain for advanced training."""
    def __init__(self):
        super().__init__(mode=USE_PROGRAMMATIC)
    
    def generate_terrain(self, env, height_perturbation_range=0.15):
        return super().generate_terrain(env, 0.15)

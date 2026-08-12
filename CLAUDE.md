# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**MoSynth** is a motion synthesis system for character animation that combines motion matching with neural motion fields. It's a Unity project (v6000.4.4) that synthesizes realistic character locomotion by blending traditional motion matching with learned motion fields. The system uses a pipeline architecture where pose data flows through multiple "stages" that transform it, with interop to Python via PythonNET for neural network inference.

### Code Style
- Do not write comments in generated code that depend on the context of the conversation that produced them (e.g. referencing the task, a fix, a prior approach, or "why we're changing this now"). Comments should only explain non-obvious WHY that a reader with just the code in front of them would need.
- Use `var` to declare variables unless an explicit type makes the code clearer (IDEs surface type info well).
- Naming:
  - Private instance/static fields: `_camelCase` (e.g. `_skeleton`, `_poseSet`)
  - Inspector-serialized fields (`[SerializeField]` private, or public fields that aren't `[NonSerialized]`): `camelCase` (e.g. `poseSkeleton`, `synthesisFrameRate`)
  - Public non-serialized fields (e.g. `[NonSerialized] public`): `PascalCase` (e.g. `CurrentPose`, `SkeletonTransforms`)
  - Types, methods, properties, events, constants: `PascalCase`
  - Locals and parameters: `camelCase`

### Tech Stack
- **Unity**: 6000.4.4f1 (LTS)
- **Language**: C# / Python (CPython 3.13 via PythonNET)
- **Key Dependencies**: 
  - Barracuda (3.0.2) - neural network inference in Unity
  - Mathematics, Collections, Jobs (for performance)
  - PyThonNET - C#/Python interoperability
  - Scipy, NumPy, PyTorch (Python side)

## Architecture

### Core Pipeline: MoSynthStage System

The motion synthesis pipeline is built on an abstract stage architecture:

```
MotionSynthesisComponent
  └── Stages[] (pipeline that transforms pose data each frame)
      ├── MotionMatchingStage (searches animation database for best pose)
      ├── MotionFieldStage (applies neural motion field transformations)
      ├── RetargetingStage (optional skeleton retargeting)
      └── Other stages (RootMotionCorrection, PoseSetVisualizer, etc.)
```

**Key classes:**
- `MoSynthStage` (abstract base): interface for all stages with `Init()`, `Apply(PoseVector, deltaTime)`, `GetSkeleton()`, `OnDestroy()`
- `MotionSynthesisComponent`: main orchestrator that runs stages each frame; manages skeleton transforms and pose updates
- `PoseVector`: the mutable pose data structure carrying joint positions, rotations, velocities, contact states
- `Skeleton`: immutable skeleton structure with joint hierarchy used for FK/IK

### Data Flow: Pose Representation

Poses are packed into flat arrays for efficiency with Python interop:
- **Format**: `(..., num_bones + 2, 4)` array
  - Index 0: root position (xyz) + padding (1 float)
  - Index 1: hips position (xyz) + padding (1 float)
  - Indices 2+: joint rotations as quaternions (xyzw per joint)
- **Velocities**: parallel arrays using same format but represent per-second rates (scaled by `frame_time` to get one-frame deltas)
- **Python classes**: `Pose` (immutable), `PoseDelta` (velocity), `Skeleton` (FK/IK)

### MotionFieldStage (Current Feature Branch)

Integrates a neural motion field into the pipeline via PythonNET:

**C# side (`Assets/MotionField/MotionFieldStage.cs`):**
- Initializes Python engine, loads `MotionField.py` and `action_predictor.py` modules
- Each frame `StepPolicy()` calls one of the four Python policy methods, chosen by the stage's
  `Policy` enum. The dispatch lives in C# so the policy never crosses the boundary as a value
- Converts Python arrays back to `PoseVector` for Unity

**Python side (`Python/MotionField.py`):**
- KNN search over motion states (positions + velocities as features)
- Blends nearest neighbor poses and velocities
- Policy methods: `optimal_action()` (value function, falling back to greedy when none is loaded),
  `greedy_action()`, and the debug pair `get_next_pose()` / `get_next_pose_from_field()`
- Supporting: `get_knn()`, `get_batched_knn()`, `build_motion_states()`, `load_value_function()`

**⚠️ Known Issues:**
- Python paths are hard-coded (`MotionFieldStage:40`, `MotionFieldStage:47`) — should be made configurable
- Requires specific Python 3.13 DLL location
- Requires virtual environment setup at a specific path
- Should use `Application.streamingAssetsPath` pattern like the animation data does

## Key Directories & Files

```
Assets/
├── MotionField/
│   ├── MotionFieldStage.cs        [stage implementing neural field]
│   ├── MfConnector.cs              [data connector for field]
│   └── MotionFieldData.cs           [serializable config]
├── MotionMatching/
│   ├── Runtime/Core/
│   │   ├── MoSynthStage.cs          [abstract base for all stages]
│   │   ├── MotionSynthesisComponent.cs [main orchestrator]
│   │   ├── MotionMatchingStage.cs   [database search stage]
│   │   ├── RetargetingStage.cs
│   │   └── RootMotionCorrectionStage.cs
│   ├── Runtime/Pose/
│   │   ├── PoseVector.cs            [mutable pose in motion synthesis]
│   │   └── Skeleton.cs              [skeleton structure]
│   └── Editor/                      [tools and visualization]
├── Scenes/
│   ├── ExampleSimpleMMStages.unity  [demo scene with motion matching]
│   └── (animation/scene test assets)
└── Animation/
    └── MotionMatching/              [animation database assets]

Python/
├── MotionField.py                   [neural motion field implementation]
├── Pose.py                          [pose data classes]
├── Skeleton.py                      [skeleton FK/IK]
├── action_predictor.py              [animation loading & conversion]
├── motion_field_io.py               [.mffield.npz value function format]
├── motion_field_trainer.py          [fitted value iteration]
├── motion_field_embedding.py        [UMAP projection for the debug visualizer]
├── utils/
│   └── quaternions.py               [quaternion utilities]
└── debugging/                       [debug scripts, not in builds]

Packages/manifest.json               [Unity package dependencies]
ProjectSettings/ProjectVersion.txt   [Unity version: 6000.4.4f1]
```

## Common Development Tasks

### Opening the Project
```powershell
# Open in Unity (requires Unity 6 LTS installed)
& "C:\Program Files\Unity\Hub\Editor\6000.4.4f1\Editor\Unity.exe" -projectPath "E:\UnityProjects\MoSynth"

# Or open the solution in Visual Studio/Rider
start MoSynth.sln
```

### Running a Demo Scene
1. Open `Assets/Scenes/ExampleSimpleMMStages.unity` in the Unity Editor
2. Press Play in the Editor
3. Use WASD/mouse for character control (if MoSynthControlInput is configured)

### Python Environment Setup
```bash
# The MotionFieldStage expects a venv at a specific path (currently hard-coded)
# Python 3.13 is required for PythonNET compatibility
python -m venv D:\iitbpg\MoSynth\AnimationTech\.anim_env
D:\iitbpg\MoSynth\AnimationTech\.anim_env\Scripts\activate

# Install dependencies
pip install numpy scipy torch

# Test Python modules
python Python/MotionField.py
```

### Building for Distribution
The project uses standard Unity build pipeline:
1. `File → Build Settings` in Unity Editor
2. Select target platform and scenes
3. Configure player settings (scripts-only for development)

### Testing & Validation
- **Editor play mode**: test motion synthesis visually
- **Python validation**: `Python/test_server.py` (if it exists) or manual import tests
- **Motion database**: verify animation data loads in `StreamingAssets/MMDatabases/MotionMatchingData`

## Important Patterns & Conventions

### Stage Development
When adding a new `MoSynthStage`:
1. Inherit from `MoSynthStage` abstract class
2. Override `Init(MotionSynthesisComponent)` for setup (called once at startup)
3. Override `Apply(PoseVector pose, float deltaTime)` to transform pose (called every frame)
4. Override `GetSkeleton(in Skeleton)` if the stage modifies skeleton structure
5. Implement `IDisposable.Dispose()` if holding unmanaged resources (e.g., Python state)
6. Call `stage.OnDestroy()` happens automatically when scene unloads

### Python Interop with PythonNET
- Always wrap Python calls in `using (Py.GIL()) { ... }` to acquire the Global Interpreter Lock
- Use `dynamic` types for Python objects in C#
- Python modules should be added to `sys.path` or placed in `Assets/../Python/`
- For module hot-reloading during development, use `importlib.reload()` (see `MotionFieldStage:63-65`)

### Pose Data Handling
- `PoseVector` is mutable and used during synthesis (Unity animations apply to it)
- `Pose` (Python class) is immutable; unpack with `.from_array()`, pack with `.pack()`
- Always use consistent `frame_time` when scaling velocities (stored in metadata)
- Root velocity is in local space (rotated by root bone before applying); hips use world space

### File Paths
- Use `Application.dataPath` for Assets folder (e.g., `Path.Combine(Application.dataPath, "../Python")`)
- Use `Application.streamingAssetsPath` for packaged read-only data (e.g., animation databases)
- Avoid hard-coded absolute paths like `D:\iitbpg\...` and `C:\Users\...` — these break on other machines

## Known Issues & TODOs

1. **Hard-coded Python paths** (`MotionFieldStage:40`, `:47`) — should be configurable via Inspector or config file
2. **Module reloading** uses `importlib.reload()` for development convenience but should be removed in production
3. **Unused code** in `MotionField.py` (see dead branches in `get_pose()` method lines 169-171)
4. **PoseVector vs Pose confusion**: dual representation exists; unify or document the split
5. **MotionSynthesisComponent TODO** at line 11: should decouple from `MotionMatchingData` dependency

## Testing & Debugging

- **Unity Console** shows Python errors with proper stack traces (thanks to PythonNET error propagation)
- **Python print() statements** flush to Unity's Debug output when wrapped in `Py.GIL()`
- **Profiler**: Enable in Player settings to monitor stage performance; each stage's `Apply()` runs in sequence
- **Scene playback**: ExampleSimpleMMStages is the main test scene; use it to validate pose quality and blending

## Before Your First Commit

- Verify hard-coded paths are not leaking (search for `D:\`, `C:\Users\`)
- Test on a fresh Unity project if adding dependencies
- Update `Assets/MotionField/MotionFieldData.cs` or config if adding configurable parameters
- Run Python linting on any modified `.py` files (PEP8 style preferred)


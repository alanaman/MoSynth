using System;
using System.IO;
using MotionMatching;
using Python.Runtime;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionField
{
/// <summary>
/// Drives the character from a neural motion field evaluated in Python.
///
/// Each frame the field is asked for the action with the best long-term value given where the
/// player wants to go. That look-ahead is what a value function buys: a one-step-greedy policy only
/// asks which action points closest to the goal on the very next frame, so it oscillates rather
/// than committing to a turn that takes a few steps to set up. Without a trained value function the
/// stage falls back to exactly that greedy policy.
/// </summary>
[Serializable]
public class MotionFieldStage : MoSynthStage, IDisposable
{
    private bool _isInitialized;
    private bool _initializationFailed;

    private dynamic _actionPredictor;
    private dynamic _motionField;
    private dynamic _skeleton;
    private dynamic _currentX;
    private dynamic _currentV;
    private dynamic _currentContacts;

    private Transform _rootTransform;

    [Tooltip("Animation database and motion field hyperparameters. Train the value function from " +
             "this asset's inspector.")]
    [SerializeField] public MotionFieldConfig config;

    [Tooltip("Database state the character starts from.")]
    [SerializeField] [Min(0)] public int startStateIndex = 700;

    [Tooltip("Re-execute the Python modules on start so .py edits apply without restarting Unity.")]
    [SerializeField] public bool reloadPythonModules = true;

    public enum Policy
    {
        /// <summary>Trained value function when available, greedy otherwise.</summary>
        Automatic,
        /// <summary>Force the one-step-greedy policy, ignoring any trained value function.</summary>
        Greedy,
        /// <summary>Debug: play the database back frame by frame.</summary>
        Playback,
        /// <summary>Debug: snap to the successor of the nearest database state.</summary>
        NearestNeighbour
    }

    [SerializeField] public Policy policy = Policy.Automatic;

    public enum SteeringSource
    {
        /// <summary>Hold a fixed heading in world space.</summary>
        FixedDirection,
        /// <summary>Walk toward a target transform.</summary>
        TowardsTarget,
        /// <summary>WASD / left stick, interpreted relative to the camera when one is set.</summary>
        PlayerInput
    }

    [Header("Steering")]
    [SerializeField] public SteeringSource steering = SteeringSource.PlayerInput;

    [Tooltip("World-space heading for FixedDirection, and the fallback when input is idle.")]
    [SerializeField] public Vector3 fixedDirection = Vector3.forward;

    [SerializeField] public Transform target;

    [Tooltip("Input is taken relative to this transform's facing. Falls back to the main camera.")]
    [SerializeField] public Transform inputReference;

    /// <summary>Last desired heading in world space, for gizmos and debugging.</summary>
    public Vector3 DesiredWorldDirection { get; private set; } = Vector3.forward;

    /// <summary>Last goal heading handed to Python, in radians, in the character's frame.</summary>
    public float Theta { get; private set; }

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        if (_isInitialized || _initializationFailed) return;

        if (config == null)
        {
            Debug.LogError("[MotionField] MotionFieldStage has no MotionFieldConfig assigned.");
            _initializationFailed = true;
            return;
        }

        _rootTransform = motionSynthesisComponent.SkeletonTransforms is { Length: > 0 }
            ? motionSynthesisComponent.SkeletonTransforms[0]
            : motionSynthesisComponent.transform;

        try
        {
            PythonRuntime.EnsureInitialized(config.pythonDllPath, config.pythonVenvPath);

            using (Py.GIL())
            {
                _actionPredictor = PythonRuntime.Import("action_predictor", reloadPythonModules);
                dynamic motionFieldModule = PythonRuntime.Import("MotionField", reloadPythonModules);

                string dataPath = config.GetAssetPath();
                var animData = _actionPredictor.load_animations(dataPath, config.name);
                _skeleton = animData[0];
                dynamic poseX = animData[1];
                dynamic poseV = animData[2];
                dynamic poseY = animData[3];
                dynamic poseContacts = animData[4];
                dynamic frameTime = animData[5];

                _motionField = motionFieldModule.MotionField(
                    poseX, poseV, poseY, _skeleton, frameTime,
                    device: config.DeviceName,
                    pos_weight: config.posWeight,
                    vel_weight: config.velWeight,
                    k_neighbors: config.kNeighbors,
                    tug_ratio: config.tugRatio,
                    knn_chunk: config.knnChunk);

                // A stale or absent value function degrades to greedy control rather than throwing.
                // Failing hard here would surface as an opaque managed exception in a player build.
                string valuePath = config.GetValueFunctionPath();
                if (File.Exists(valuePath))
                {
                    _motionField.load_value_function(valuePath, dataPath, config.name);
                }
                else
                {
                    Debug.LogWarning(
                        $"[MotionField] No trained value function at '{valuePath}'. Using the " +
                        "one-step-greedy policy. Press Train Motion Field on the config to fix this.");
                }

                int stateCount = (int)poseX.shape[0];
                int index = Mathf.Clamp(startStateIndex, 0, Mathf.Max(0, stateCount - 1));
                _currentX = poseX[index].copy();
                _currentV = poseV[index].copy();
                _currentContacts = poseContacts[index];
            }

            _isInitialized = true;
        }
        catch (Exception e)
        {
            _initializationFailed = true;
            Debug.LogError("[MotionField] Failed to initialize the motion field. " +
                           "Check the Python DLL and venv paths on the MotionFieldConfig.");
            Debug.LogException(e);
        }
    }

    public override bool Apply(PoseVector pose, float deltaTime)
    {
        // Never return false. MotionSynthesisComponent breaks the stage chain but still applies the
        // pose it built from the current transforms, whose angular velocities are Quaternion.euler
        // angles in degrees wrapped to 0..360 -- which kicks the root by hundreds of rad/s.
        if (!_isInitialized) return true;

        try
        {
            UpdateSteering();

            using (Py.GIL())
            {
                var nextPose = _motionField.get_pose(_currentX, _currentV, deltaTime,
                    theta: Theta, mode: PolicyName());
                _currentX = nextPose[0];
                _currentV = nextPose[1];

                dynamic poseArrays = _actionPredictor.get_pose_arrays(
                    _skeleton, _currentX, _currentV, _currentContacts);

                float[] posArray = (float[])poseArrays[0];
                float[] quatArray = (float[])poseArrays[1];
                float[] lvArray = (float[])poseArrays[2];
                float[] lavArray = (float[])poseArrays[3];

                pose.leftFootContact = (bool)poseArrays[4];
                pose.rightFootContact = (bool)poseArrays[5];

                int numJoints = pose.jointLocalPositions.Length;

                for (int i = 0; i < numJoints; i++)
                {
                    pose.jointLocalPositions[i] = new Unity.Mathematics.float3(
                        posArray[i * 3], posArray[i * 3 + 1], posArray[i * 3 + 2]);

                    pose.jointLocalRotations[i] = new Unity.Mathematics.quaternion(
                        quatArray[i * 4], quatArray[i * 4 + 1], quatArray[i * 4 + 2], quatArray[i * 4 + 3]);

                    pose.jointLocalVelocities[i] = new Unity.Mathematics.float3(
                        lvArray[i * 3], lvArray[i * 3 + 1], lvArray[i * 3 + 2]);

                    pose.jointLocalAngularVelocities[i] = new Unity.Mathematics.float3(
                        lavArray[i * 3], lavArray[i * 3 + 1], lavArray[i * 3 + 2]);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogException(e);
            _isInitialized = false; // stop spamming the console every frame
        }

        return true;
    }

    private string PolicyName() => policy switch
    {
        Policy.Greedy => "greedy",
        Policy.Playback => "playback",
        Policy.NearestNeighbour => "nearest",
        _ => null // let Python pick: value function if loaded, greedy otherwise
    };

    /// <summary>
    /// Convert the desired world heading into the goal angle the value function was trained on.
    ///
    /// Theta is the character's own heading expressed in the goal frame, which is what makes it
    /// frame-agnostic: Python accumulates its root pose independently of Unity's transform, and the
    /// only thing that couples them is the per-step yaw. Measuring the goal relative to the
    /// character's current facing means the two accumulators never have to be synchronised.
    ///
    /// The negation matches the training convention, where reward is -|theta + delta_yaw|: the
    /// action that cancels theta is the one that turns the character onto the goal.
    /// </summary>
    private void UpdateSteering()
    {
        Vector3 desired = ResolveDesiredDirection();
        desired.y = 0f;

        if (desired.sqrMagnitude > 1e-6f)
        {
            DesiredWorldDirection = desired.normalized;
        }

        // Root-local +Z is the character's facing: PoseExtractor builds the simulation bone as
        // LookRotation(hips forward projected on the ground).
        Vector3 forward = _rootTransform != null ? _rootTransform.forward : Vector3.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 1e-6f) forward = Vector3.forward;

        Theta = -Vector3.SignedAngle(forward, DesiredWorldDirection, Vector3.up) * Mathf.Deg2Rad;
    }

    private Vector3 ResolveDesiredDirection()
    {
        switch (steering)
        {
            case SteeringSource.TowardsTarget:
                return target != null && _rootTransform != null
                    ? target.position - _rootTransform.position
                    : fixedDirection;

            case SteeringSource.PlayerInput:
                Vector2 stick = ReadMoveInput();
                if (stick.sqrMagnitude < 1e-4f) return DesiredWorldDirection;

                Transform reference = inputReference != null ? inputReference
                    : (Camera.main != null ? Camera.main.transform : null);
                if (reference == null) return new Vector3(stick.x, 0f, stick.y);

                Vector3 right = reference.right;
                Vector3 ahead = reference.forward;
                right.y = 0f;
                ahead.y = 0f;
                return right.normalized * stick.x + ahead.normalized * stick.y;

            case SteeringSource.FixedDirection:
                return fixedDirection;
            
            default:
                throw new NotImplementedException();
        }
    }

    private static Vector2 ReadMoveInput()
    {
        Vector2 move = Vector2.zero;

        var gamepad = Gamepad.current;
        if (gamepad != null)
        {
            move = gamepad.leftStick.ReadValue();
            if (move.sqrMagnitude > 1e-4f) return move;
        }

        var keyboard = Keyboard.current;
        if (keyboard != null)
        {
            if (keyboard.aKey.isPressed) move.x -= 1f;
            if (keyboard.dKey.isPressed) move.x += 1f;
            if (keyboard.sKey.isPressed) move.y -= 1f;
            if (keyboard.wKey.isPressed) move.y += 1f;
        }

        return move;
    }

    public void Dispose()
    {
        if (!_isInitialized) return;

        using (Py.GIL())
        {
            _actionPredictor = null;
            _motionField = null;
            _skeleton = null;
            _currentX = null;
            _currentV = null;
            _currentContacts = null;
        }

        _isInitialized = false;
    }

    public override void OnDestroy()
    {
        Dispose();
    }
}
}

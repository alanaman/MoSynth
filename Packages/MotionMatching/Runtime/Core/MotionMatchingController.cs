using System;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Serialization;

namespace MotionMatching
{
using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

// Simulation bone is the transform
public class MotionMatchingController : MonoBehaviour
{
    public event Action OnSkeletonTransformUpdated;

    [Header("General")] public MotionMatchingCharacterController characterController;
    [FormerlySerializedAs("MMData")] public MotionMatchingData mmData;

    [SerializeReference] [SubclassSelector]
    public MotionMatchingSearch search;

    [FormerlySerializedAs("LockFPS")] public bool lockFPS = true;

    /// <summary>
    /// The interval in seconds between two Motion Matching searches when there are no sudden input changes.
    /// </summary>
    [FormerlySerializedAs("searchTime")] public float searchInterval = 10.0f / 60.0f;


    [FormerlySerializedAs("Inertialize")]
    public bool inertialize = true; // Should inertialize transitions after a big change of the pose

    [FormerlySerializedAs("FootLock")]
    public bool footLock = true; // Should lock the feet to the ground when contact information is true

    [FormerlySerializedAs("FootUnlockDistance")]
    public float footUnlockDistance = 0.2f; // Distance from actual pose to IK target to unlock the feet

    [FormerlySerializedAs("InertializeHalfLife")] [Range(0.0f, 1.0f)]
    public float
        inertializeHalfLife = 0.1f; // Time needed to move half of the distance between the source to the target pose

    [FormerlySerializedAs("Responsiveness")]
    [Tooltip("How important is the trajectory (future positions + future directions)")]
    [Range(0.0f, 1.0f)]
    public float responsiveness = 1.0f;

    [FormerlySerializedAs("Quality")] [Tooltip("How important is the current pose")] [Range(0.0f, 1.0f)]
    public float quality = 1.0f;

    [FormerlySerializedAs("FeatureWeights")] [HideInInspector]
    public float[] featureWeights;


    public float3 Velocity { get; private set; }
    public float3 AngularVelocity { get; private set; }
    public float DatabaseFrameTime { get; private set; }
    public int DatabaseFrameRate { get; private set; }
    public PoseSet PoseSet { get; private set; }
    public FeatureSet FeatureSet { get; private set; }
    public float SearchTimeLeft { get; private set; }
    public Transform[] SkeletonTransforms { get; private set; }
    public NativeArray<float> QueryFeature { get; private set; }
    public NativeArray<float> FeaturesWeightsNativeArray { get; private set; }
    public int CurrentFrame { get; private set; } // Current frame index in the pose/feature set
    public int LastMmSearchFrame { get; private set; } // Frame before the last Motion Matching Search
    public NativeArray<bool> TagMask { get; private set; }

    private float3 _animationSpaceOriginPos;
    private quaternion _inverseAnimationSpaceOriginRot;
    private float3 _mmTransformOriginPose; // Position of the transform right after motion matching search
    private quaternion _mmTransformOriginRot; // Rotation of the transform right after motion matching search
    private float _currentFrameTime; // Current frame index as float to keep track of variable frame rate

    private Inertialization _inertialization;

    // Foot Lock
    public bool IsLeftFootContact { get; private set; }

    public bool IsRightFootContact { get; private set; }

    // Target position of the toes
    public float3 LeftToesContactTarget { get; private set; }
    public float3 RightToesContactTarget { get; private set; }
    private float3 _leftFootContact, _rightFootContact; // Position of the foot
    private float3 _leftFootPoleContact, _rightFootPoleContact; // Forward vector of the knee
    private float3 _leftLowerLegLocalForward, _rightLowerLegLocalForward;
    public int LeftToesIndex { get; private set; }
    private int _leftFootIndex;
    private int _leftLowerLegIndex;
    private int _leftUpperLegIndex;

    public int RightToesIndex { get; private set; }
    private int _rightFootIndex;
    private int _rightLowerLegIndex;
    private int _rightUpperLegIndex;

    // Other
    private bool _isDestroyed;
    private MotionMatchingSearch _searchInstance;

    private void Awake()
    {
        PoseSet = mmData.GetOrImportPoseSet();
        FeatureSet = mmData.GetOrImportFeatureSet();

        // Skeleton
        SkeletonTransforms = new Transform[PoseSet.Skeleton.Joints.Count];
        SkeletonTransforms[0] = transform; // Simulation Bone
        for (int j = 1; j < PoseSet.Skeleton.Joints.Count; j++)
        {
            // Joints
            Skeleton.Joint joint = PoseSet.Skeleton.Joints[j];
            Transform t = new GameObject().transform;
            t.name = joint.name;
            t.SetParent(SkeletonTransforms[joint.parentIndex], false);
            t.localPosition = joint.localOffset;
            SkeletonTransforms[j] = t;
        }

        // Inertialization
        _inertialization = new Inertialization(PoseSet.Skeleton);

        // FPS
        DatabaseFrameTime = PoseSet.FrameTime;
        DatabaseFrameRate = Mathf.RoundToInt(1.0f / DatabaseFrameTime);
        if (lockFPS)
        {
            Application.targetFrameRate = DatabaseFrameRate;
            Debug.Log("[Motion Matching] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
            Debug.LogWarning(
                "[Motion Matching] LockFPS is not set. Motion Matching will malfunction if the application frame rate is higher than the animation database.");
        }

        // Other initialization
        int numberFeatures = mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count +
                             mmData.EnvironmentFeatures.Count;
        if (featureWeights == null || featureWeights.Length != numberFeatures)
        {
            float[] newWeights = new float[numberFeatures];
            for (int i = 0; i < newWeights.Length; ++i) newWeights[i] = 1.0f;
            for (int i = 0; i < Mathf.Min(featureWeights.Length, newWeights.Length); i++)
                newWeights[i] = featureWeights[i];
            featureWeights = newWeights;
        }

        FeaturesWeightsNativeArray = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);
        QueryFeature = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);

        // Tags
        TagMask = new NativeArray<bool>(FeatureSet.NumberFeatureVectors, Allocator.Persistent);
        DisableQueryTag();

        // Foot Lock
        var skeleton = PoseSet.Skeleton;
        LeftToesIndex = skeleton.GetJointIndex(HumanBodyBones.LeftToes);
        _leftFootIndex = skeleton.GetJointIndex(HumanBodyBones.LeftFoot);
        _leftLowerLegIndex = skeleton.GetJointIndex(HumanBodyBones.LeftLowerLeg);
        _leftUpperLegIndex = skeleton.GetJointIndex(HumanBodyBones.LeftUpperLeg);
        RightToesIndex = skeleton.GetJointIndex(HumanBodyBones.RightToes);
        _rightFootIndex = skeleton.GetJointIndex(HumanBodyBones.RightFoot);
        _rightLowerLegIndex = skeleton.GetJointIndex(HumanBodyBones.RightLowerLeg);
        _rightUpperLegIndex = skeleton.GetJointIndex(HumanBodyBones.RightUpperLeg);
        _leftLowerLegLocalForward = mmData.GetLocalForward(_leftLowerLegIndex);
        _rightLowerLegLocalForward = mmData.GetLocalForward(_rightLowerLegIndex);

        // Init Pose
        SkeletonTransforms[0].SetPositionAndRotation(characterController.GetWorldInitPosition(),
            quaternion.LookRotation(characterController.GetWorldInitDirection(), Vector3.up));

        // Search first Frame valid (to start with a valid pose)
        for (int i = 0; i < FeatureSet.NumberFeatureVectors; i++)
        {
            if (FeatureSet.IsValidFeature(i) && TagMask[i])
            {
                LastMmSearchFrame = i;
                CurrentFrame = i;
                UpdateAnimationSpaceOrigin();
                break;
            }
        }

        // Search Strategy
        search ??= MotionMatchingSearch.Default;
        _searchInstance.Initialize(this);
    }

    private void OnEnable()
    {
        characterController.OnUpdated += OnCharacterControllerUpdated;
        characterController.OnInputChangedQuickly += OnInputChangedQuickly;
    }

    private void OnDisable()
    {
        characterController.OnUpdated -= OnCharacterControllerUpdated;
        characterController.OnInputChangedQuickly -= OnInputChangedQuickly;
    }

    private void OnCharacterControllerUpdated(float deltaTime)
    {
        PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Total");
        if (SearchTimeLeft <= 0)
        {
            // Motion Matching
            float currentDistance = PrepareQueryVector(out bool isCurrentValid);
            PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Search");
            int bestFrame = _searchInstance.FindBestFrame(this, currentDistance);
            PROFILE.END_SAMPLE_PROFILING("Motion Matching Search");
            // Check if use current or best
            if (isCurrentValid && bestFrame == -1) bestFrame = CurrentFrame;
            Debug.Assert(bestFrame != -1,
                "Motion Matching is not able to find any valid pose. Maybe the motion database is empty or the query tag used produces an empty set of poses?");
            const int ignoreSurrounding = 20; // ignore near frames
            if (math.abs(bestFrame - CurrentFrame) > ignoreSurrounding)
            {
                // Inertialize
                if (inertialize)
                {
                    _inertialization.PoseTransition(PoseSet, CurrentFrame, bestFrame);
                }

                LastMmSearchFrame = CurrentFrame;
                _currentFrameTime =
                    bestFrame + math.frac(
                        _currentFrameTime); // the fractional part is the error accumulated, add it to the current to avoid drifting
                CurrentFrame = bestFrame;
                UpdateAnimationSpaceOrigin();
            }

            SearchTimeLeft = searchInterval;
        }
        else
        {
            SearchTimeLeft -= deltaTime;
        }

        // Always advance one (bestFrame from motion matching is the best match to the current frame, but we want to move to the next frame)
        // Ideally the applications runs at 1.0f/FrameTime fps (to match the database) however, as this may not happen, we may need to skip some frames
        // from the database, e.g., if 1.0f/FrameTime = 60 and our game runes at 30, we need to advance 2 frames at each update
        // However, as we are using Application.targetFrameRate=1.0f/FrameTime, we do not consider the case where the application runs faster than the database
        _currentFrameTime += DatabaseFrameRate * deltaTime; // DatabaseFrameRate / (1.0f / deltaTime)
        CurrentFrame = (int)math.floor(_currentFrameTime);

        UpdateTransformAndSkeleton(CurrentFrame);
        PROFILE.END_SAMPLE_PROFILING("Motion Matching Total");

        _searchInstance.OnSearchCompleted(this);
    }

    private void UpdateAnimationSpaceOrigin()
    {
        PoseSet.GetPose(CurrentFrame, out PoseVector mmPose);
        _animationSpaceOriginPos = mmPose.JointLocalPositions[0];
        _inverseAnimationSpaceOriginRot = math.inverse(mmPose.JointLocalRotations[0]);
        _mmTransformOriginPose = SkeletonTransforms[0].position;
        _mmTransformOriginRot = SkeletonTransforms[0].rotation;
    }

    private void OnInputChangedQuickly()
    {
        SearchTimeLeft = 0; // Force search
    }

    private float PrepareQueryVector(out bool isCurrentValid)
    {
        // Weights
        UpdateAndGetFeatureWeights();

        // Init Query Vector
        FeatureSet.GetFeature(QueryFeature, CurrentFrame);
        FillQueryVector();

        // Get next feature vector (when doing motion matching search, they need less error than this)
        float currentDistance = float.MaxValue;
        isCurrentValid = FeatureSet.IsValidFeature(CurrentFrame) && TagMask[CurrentFrame];
        if (isCurrentValid)
        {
            currentDistance = 0.0f;
            // the pose is the same... the distance is only the trajectory
            for (int j = 0; j < FeatureSet.PoseOffset; j++)
            {
                float diff = FeatureSet.GetFeatures()[CurrentFrame * FeatureSet.FeatureSize + j] - QueryFeature[j];
                currentDistance += diff * diff * FeaturesWeightsNativeArray[j];
            }
        }

        return currentDistance;
    }

    public void FillQueryVector()
    {
        NativeArray<float> queryFeature = QueryFeature;
        int offset = 0;
        for (int i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            TrajectoryFeature feature = mmData.TrajectoryFeatures[i];
            for (int p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                int featureSize = feature.GetSize();
                Debug.Assert(featureSize > 0, "Trajectory feature size must be larger than 0");
                NativeArray<float> featureVector = new(featureSize, Allocator.Temp);
                characterController.GetTrajectoryFeature(feature, p, SkeletonTransforms[0], featureVector);
                for (int j = 0; j < featureSize; j++)
                {
                    queryFeature[offset + j] = featureVector[j];
                }

                offset += featureSize;
            }
        }

        // Normalize (only trajectory... because current FeatureVector is already normalized)
        FeatureSet.NormalizeTrajectory(queryFeature);

        if (FeatureSet.EnvironmentOffset.Length > 0)
        {
            offset = FeatureSet.EnvironmentOffset[0];
            for (int i = 0; i < mmData.EnvironmentFeatures.Count; i++)
            {
                TrajectoryFeature feature = mmData.EnvironmentFeatures[i];
                for (int p = 0; p < feature.FramesPrediction.Length; p++)
                {
                    int featureSize = feature.GetSize();
                    Debug.Assert(featureSize > 0, "Environment feature size must be larger than 0");
                    NativeArray<float> featureVector = new(featureSize, Allocator.Temp);
                    characterController.GetEnvironmentFeature(feature, p, SkeletonTransforms[0], featureVector);
                    for (int j = 0; j < featureSize; j++)
                    {
                        queryFeature[offset + j] = featureVector[j];
                    }

                    offset += featureSize;
                }
            }
        }
    }

    private void UpdateTransformAndSkeleton(int frameIndex)
    {
        PoseSet.GetPose(frameIndex, out PoseVector pose);
        // Update Inertialize if enabled
        if (inertialize)
        {
            _inertialization.Update(pose, inertializeHalfLife, Time.deltaTime);
        }

        // Simulation Bone
        float3 previousPosition = SkeletonTransforms[0].position;
        quaternion previousRotation = SkeletonTransforms[0].rotation;
        // animation space to local space
        float3 localSpacePos = math.mul(_inverseAnimationSpaceOriginRot,
            pose.JointLocalPositions[0] - _animationSpaceOriginPos);
        quaternion localSpaceRot = math.mul(_inverseAnimationSpaceOriginRot, pose.JointLocalRotations[0]);
        // local space to world space
        SkeletonTransforms[0].SetPositionAndRotation(
            math.mul(_mmTransformOriginRot, localSpacePos) + _mmTransformOriginPose,
            math.mul(_mmTransformOriginRot, localSpaceRot));
        // update velocity and angular velocity
        Velocity = ((float3)SkeletonTransforms[0].position - previousPosition) / Time.deltaTime;
        AngularVelocity =
            MathExtensions.AngularVelocity(previousRotation, SkeletonTransforms[0].rotation, Time.deltaTime);
        // Joints
        if (inertialize)
        {
            for (int i = 1; i < _inertialization.InertializedRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = _inertialization.InertializedRotations[i];
            }
        }
        else
        {
            for (int i = 1; i < pose.JointLocalRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }
        }

        // Hips Position
        SkeletonTransforms[1].localPosition =
            inertialize ? _inertialization.InertializedHips : pose.JointLocalPositions[1];
        // Foot Lock
        UpdateFootLock(pose);
        // Post-processing the transforms
        OnSkeletonTransformUpdated?.Invoke();
    }

    private void UpdateFootLock(PoseVector pose)
    {
        float3 currentLeftToesPosition = SkeletonTransforms[LeftToesIndex].position;
        float3 currentRightToesPosition = SkeletonTransforms[RightToesIndex].position;
        // Compute input contact position velocity
        float3 currentLeftToesVelocity = (currentLeftToesPosition - (float3)LeftToesContactTarget) / Time.deltaTime;
        float3 currentRightToesVelocity = (currentRightToesPosition - (float3)RightToesContactTarget) / Time.deltaTime;
        LeftToesContactTarget = currentLeftToesPosition;
        RightToesContactTarget = currentRightToesPosition;

        // Update Inertializer
        _inertialization.UpdateContact(IsLeftFootContact ? _leftFootContact : currentLeftToesPosition,
            IsLeftFootContact ? float3.zero : currentLeftToesVelocity,
            IsRightFootContact ? _rightFootContact : currentRightToesPosition,
            IsRightFootContact ? float3.zero : currentRightToesVelocity,
            inertializeHalfLife, Time.deltaTime);
        float3 leftContactPosition = _inertialization.InertializedLeftContact;
        float3 leftContactVelocity = _inertialization.InertializedLeftContactVelocity;
        float3 rightContactPosition = _inertialization.InertializedRightContact;
        float3 rightContactVelocity = _inertialization.InertializedRightContactVelocity;

        // If the contact point is too far from the current input position
        // unlock the contact
        bool unlockLeftContact = IsLeftFootContact &&
                                 (math.length(_leftFootContact - currentLeftToesPosition) > footUnlockDistance);
        bool unlockRightContact = IsRightFootContact &&
                                  (math.length(_rightFootContact - currentRightToesPosition) > footUnlockDistance);

        // If the contact was previously inactive and now it is active,
        // transition to the locked contact state
        // Also, make sure the inertialization returns an almost 0 velocity before locking
        if (!IsLeftFootContact && pose.LeftFootContact &&
            math.length(leftContactVelocity) < mmData.ContactVelocityThreshold)
        {
            // Contact point is the current position of the foot
            // projected onto the ground + foot height
            IsLeftFootContact = true;
            _leftFootContact = leftContactPosition;
            // LeftFootContact.y =  // TODO: Add foot height
            Transform leftLowerLeg = SkeletonTransforms[_leftLowerLegIndex];
            _leftFootPoleContact = math.mul(leftLowerLeg.rotation, _leftLowerLegLocalForward);

            if (inertialize)
            {
                _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
                    _leftFootContact, float3.zero);
            }
            else
            {
                _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
                    currentLeftToesPosition, currentLeftToesVelocity);
            }
        }
        // If we need to unlock or previously in contact but now not
        // we transition to the input position
        else if (unlockLeftContact || (IsLeftFootContact && !pose.LeftFootContact))
        {
            IsLeftFootContact = false;

            if (inertialize)
            {
                _inertialization.LeftContactTransition(_leftFootContact, float3.zero, currentLeftToesPosition,
                    currentLeftToesVelocity);
            }
            else
            {
                _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
                    currentLeftToesPosition, currentLeftToesVelocity);
            }
        }

        // Same for Right Foot
        if (!IsRightFootContact && pose.RightFootContact &&
            math.length(rightContactVelocity) < mmData.ContactVelocityThreshold)
        {
            IsRightFootContact = true;
            _rightFootContact = rightContactPosition;
            // RightFootContact.y = 0.0f;
            Transform rightLowerLeg = SkeletonTransforms[_rightLowerLegIndex];
            _rightFootPoleContact = math.mul(rightLowerLeg.rotation, _rightLowerLegLocalForward);

            if (inertialize)
            {
                _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
                    _rightFootContact, float3.zero);
            }
            else
            {
                _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
                    currentRightToesPosition, currentRightToesVelocity);
            }
        }
        else if (unlockRightContact || (IsRightFootContact && !pose.RightFootContact))
        {
            IsRightFootContact = false;

            if (inertialize)
            {
                _inertialization.RightContactTransition(_rightFootContact, float3.zero, currentRightToesPosition,
                    currentRightToesVelocity);
            }
            else
            {
                _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
                    currentRightToesPosition, currentRightToesVelocity);
            }
        }

        // IK to place the foot
        if (footLock)
        {
            // Left Foot IK
            TwoJointIK.Solve(
                (Vector3)leftContactPosition + (SkeletonTransforms[_leftFootIndex].position -
                                                SkeletonTransforms[LeftToesIndex].position),
                SkeletonTransforms[_leftUpperLegIndex],
                SkeletonTransforms[_leftLowerLegIndex],
                SkeletonTransforms[_leftFootIndex],
                _leftFootPoleContact);
            // Right Foot IK
            TwoJointIK.Solve(
                (Vector3)rightContactPosition + (SkeletonTransforms[_rightFootIndex].position -
                                                 SkeletonTransforms[RightToesIndex].position),
                SkeletonTransforms[_rightUpperLegIndex],
                SkeletonTransforms[_rightLowerLegIndex],
                SkeletonTransforms[_rightFootIndex],
                _rightFootPoleContact);
        }
    }

    /// <summary>
    /// Disables any previous set tag or query so searches are performed over the entire pose set
    /// </summary>
    public void DisableQueryTag()
    {
        var job = new DisableTagBurst
        {
            TagMask = TagMask,
        };
        job.Schedule().Complete();
        OnInputChangedQuickly();
    }

    /// <summary>
    /// Motion Matching will only search over those poses belonging to the tag
    /// </summary>
    public void SetQueryTag(string tagName)
    {
        PoseSet.AnimationTag animTag = PoseSet.GetTag(tagName);
        // TODO: cache results to avoid duplicated computations...
        var job = new SetTagBurst
        {
            MaximumFramesPrediction = PoseSet.MaximumFramesPrediction,
            TagMask = TagMask,
            StartRanges = animTag.GetStartRanges(),
            EndRanges = animTag.GetEndRanges(),
        };
        job.Schedule().Complete();
        OnInputChangedQuickly();
    }

    /// <summary>
    /// Motion Matching will only search over those poses belonging to the query tag
    /// </summary>
    public void SetQueryTag(QueryTag query)
    {
        query.ComputeRanges(PoseSet);
        var job = new SetTagBurst
        {
            MaximumFramesPrediction = PoseSet.MaximumFramesPrediction,
            TagMask = TagMask,
            StartRanges = query.GetStartRanges(),
            EndRanges = query.GetEndRanges(),
        };
        job.Schedule().Complete();
        OnInputChangedQuickly();
    }

    /// <summary>
    /// Adds an offset to the current transform space (useful to move the character to a different position)
    /// Simply changing the transform won't work because motion matching applies root motion based on the current motion matching search space
    /// </summary>
    public void SetPosAdjustment(float3 posAdjustment)
    {
        _mmTransformOriginPose += posAdjustment;
    }

    /// <summary>
    /// Adds a rot offset to the current transform space (useful to rotate the character to a different direction)
    /// Simply changing the transform won't work because motion matching applies root motion based on the current motion matching search space
    /// </summary>
    public void SetRotAdjustment(quaternion rotAdjustment)
    {
        _mmTransformOriginRot = math.mul(rotAdjustment, _mmTransformOriginRot);
    }

    public void SetCurrentFrame(int frame)
    {
        CurrentFrame = frame;
    }

    public NativeArray<float> UpdateAndGetFeatureWeights()
    {
        NativeArray<float> featuresWeightsNativeArray = FeaturesWeightsNativeArray;
        int offset = 0;
        for (int i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            TrajectoryFeature feature = mmData.TrajectoryFeatures[i];
            int featureSize = feature.GetSize();
            float weight = featureWeights[i] * responsiveness;
            for (int p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                for (int f = 0; f < featureSize; f++)
                {
                    featuresWeightsNativeArray[offset + f] = weight;
                }

                offset += featureSize;
            }
        }

        for (int i = 0; i < mmData.PoseFeatures.Count; i++)
        {
            float weight = featureWeights[i + mmData.TrajectoryFeatures.Count] * quality;
            featuresWeightsNativeArray[offset + 0] = weight;
            featuresWeightsNativeArray[offset + 1] = weight;
            featuresWeightsNativeArray[offset + 2] = weight;
            offset += 3;
        }

        for (int i = 0; i < mmData.EnvironmentFeatures.Count; i++)
        {
            TrajectoryFeature feature = mmData.EnvironmentFeatures[i];
            int featureSize = feature.GetSize();
            float baseWeight = featureWeights[i + mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count];
            float weight = _searchInstance.OnUpdateEnvironmentFeatureWeight(this, feature, baseWeight);
            for (int p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                for (int f = 0; f < featureSize; f++)
                {
                    featuresWeightsNativeArray[offset + f] = weight;
                }

                offset += featureSize;
            }
        }

        return featuresWeightsNativeArray;
    }

    public float3 GetMainPositionFeature(int trajectoryIndex)
    {
        float3 characterOrigin = SkeletonTransforms[0].position;
        float3 characterForward = SkeletonTransforms[0].forward;
        quaternion characterRot = quaternion.LookRotation(characterForward, math.up());
        // Find Main Position Trajectory Index
        int t = -1;
        for (int i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            if (mmData.TrajectoryFeatures[i].IsMainPositionFeature)
            {
                t = i;
                break;
            }
        }

        if (t == -1)
        {
            Debug.LogError("[Motion Matching] No Main Position Trajectory Feature found.");
            return characterOrigin;
        }

        float3 value = FeatureSet.Get3DValuePositionOrDirectionFeature(mmData.TrajectoryFeatures[t], CurrentFrame, t,
            trajectoryIndex, isEnvironment: false);
        value = characterOrigin + math.mul(characterRot, value);
        return value;
    }

    public float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
    {
        float3 characterForward = SkeletonTransforms[0].forward;
        quaternion characterRot = quaternion.LookRotation(characterForward, math.up());
        int t = -1;
        for (int i = 0; i < mmData.EnvironmentFeatures.Count; i++)
        {
            if (mmData.EnvironmentFeatures[i].Name == featureName)
            {
                t = i;
                break;
            }
        }

        if (t == -1)
        {
            Debug.LogError("[Motion Matching] No Environment Feature with name " + featureName + " found.");
            return float4.zero;
        }

        float4 value = FeatureSet.Get4DEnvironmentFeature(CurrentFrame, t, trajectoryIndex);
        float primaryDistance = value.x;
        float secondaryDistance = value.y;
        float3 primaryAxisUnitCharacterSpace = new(value.z, 0.0f, value.w);
        float3 primaryAxisUnitWorldSpace = math.mul(characterRot, primaryAxisUnitCharacterSpace);
        float2 primaryAxisUnit = new(primaryAxisUnitWorldSpace.x, primaryAxisUnitWorldSpace.z);
        float2 secondaryAxisUnit = new(-primaryAxisUnit.y, primaryAxisUnit.x);
        return new float4(primaryAxisUnit * primaryDistance, secondaryAxisUnit * secondaryDistance);
    }

    private void OnDestroy()
    {
        if (_isDestroyed) return;
        _isDestroyed = true;
        mmData.Dispose();
        _searchInstance.Dispose();
        if (QueryFeature.IsCreated) QueryFeature.Dispose();
        if (FeaturesWeightsNativeArray.IsCreated) FeaturesWeightsNativeArray.Dispose();
        if (TagMask.IsCreated) TagMask.Dispose();
    }

    private void OnApplicationQuit()
    {
        OnDestroy();
    }

#if UNITY_EDITOR

    public Matrix4x4 GetSimulationBoneWorldSpaceTransform(PoseVector futurePose)
    {
        // animation space to local space
        float3 localSpacePos = math.mul(_inverseAnimationSpaceOriginRot,
            futurePose.JointLocalPositions[0] - _animationSpaceOriginPos);
        quaternion localSpaceRot = math.mul(_inverseAnimationSpaceOriginRot, futurePose.JointLocalRotations[0]);
        // local space to world space
        float3 simulationBonePos = math.mul(_mmTransformOriginRot, localSpacePos) + _mmTransformOriginPose;
        quaternion simulationBoneRot = math.mul(_mmTransformOriginRot, localSpaceRot);

        var simulationBoneTransform = Matrix4x4.TRS(simulationBonePos, simulationBoneRot, Vector3.one);
        return simulationBoneTransform;
    }
#endif
}
}
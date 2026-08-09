using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System.IO;
using UnityEngine.Serialization;

namespace MotionMatching
{
/// <summary>
/// Defines all data used for Motion Matching in one avatar
/// Contains animation clips, mapping between the skeleton and Mecanim, and other data
/// </summary>
[CreateAssetMenu(fileName = "MotionMatchingData", menuName = "MotionMatching/MotionMatchingData")]
public class MotionMatchingData : ScriptableObject, IPoseSetSource
{
    // TODO: DefaultHipsForward... detect/suggest automatically? try to fix automatically at BVHAnimation level? 
    // (if it is fixed some code can be deleted... all code related to DefaultHipsForward and in the UpdateTransform() when correcting the hips forward)

    [FormerlySerializedAs("AnimationDatas")] [SerializeField]
    public List<AnnotatedAnimationClip> animationClips = new();

    [Tooltip("Animation with T-Pose, Animation with a T-Pose in the first frame, used for retargeting")]
    [FormerlySerializedAs("AnimationDataTPose")]
    public AnnotatedAnimationClip tPoseAnimationClip; // Animation with a TPose in the first frame, used for retargeting

    [FormerlySerializedAs("HipsForwardLocalVector")]
    public float3
        hipsForwardLocalVector = new(0, 0, 1); // Local vector (axis) pointing in the forward direction of the hips

    [FormerlySerializedAs("HipsUpLocalVector")]
    public float3 hipsUpLocalVector = new(0, 1, 0); // Local vector (axis) pointing in the up direction of the hips

    // TODO: Implement Savitzky-Golay filter or similar low-pass filter in Unity (before I was using Python implementation)
    //public bool SmoothSimulationBone; // Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter
    [FormerlySerializedAs("ContactVelocityThreshold")]
    public float
        contactVelocityThreshold =
            0.15f; // Minimum velocity of the foot to be considered in movement and not in contact with the ground

    [FormerlySerializedAs("SkeletonToMecanim")]
    public List<JointToMecanim> animationChannelToMecanim = new();

    // TODO: these should prolly be inside FeatureSet
    [FormerlySerializedAs("TrajectoryFeatures")]
    public List<TrajectoryFeature> trajectoryFeatures = new();

    [FormerlySerializedAs("PoseFeatures")] public List<PoseFeature> poseFeatures = new();

    [FormerlySerializedAs("EnvironmentFeatures")]
    public List<TrajectoryFeature>
        environmentFeatures =
            new(); // Features used for dynamic computations and not used during the standard distance check in the Motion Matching search

    private PoseSet _poseSet;

    public FeatureSet FeatureSet
    {
        get => _featureSet;
        private set => _featureSet = value;
    }

    // Information extracted form T-Pose
    [FormerlySerializedAs("JointsLocalForward")] [SerializeField]
    private float3[] jointsLocalForward; // Local forward vector of each joint 

    private FeatureSet _featureSet;
    public bool JointsLocalForwardError => jointsLocalForward == null;

    // IPoseSetSource --- the subset of this asset the pose-database pipeline actually reads.
    // Exposed as properties so a consumer that only wants a pose database (MotionField's config)
    // can supply one without also being a full Motion Matching asset.
    public List<AnnotatedAnimationClip> AnimationClips => animationClips;
    public float3 HipsForwardLocalVector => hipsForwardLocalVector;
    public float ContactVelocityThreshold => contactVelocityThreshold;

    /// <summary>
    /// Frames of lookahead the longest trajectory feature needs. Poses closer than this to the end
    /// of a clip cannot be used for prediction.
    /// </summary>
    public int MaximumFramesPrediction
    {
        get
        {
            int maximum = 0;
            foreach (var t in trajectoryFeatures)
            {
                if (t.framesPrediction.Length > 0 && t.framesPrediction[^1] > maximum)
                {
                    maximum = t.framesPrediction[^1];
                }
            }

            return maximum;
        }
    }

    private void ImportAnimations()
    {
        PROFILE.BEGIN_SAMPLE_PROFILING("BVH Import");
        foreach (var animData in animationClips)
        {
            // Add Mecanim mapping information
            animData.UpdateMecanimInformation(this);
        }

        PROFILE.END_AND_PRINT_SAMPLE_PROFILING("BVH Import");
    }

    public PoseSet GetOrImportPoseSet()
    {
        if (_poseSet == null)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Import");
            PoseSerializer serializer = new PoseSerializer();
            if (!serializer.Deserialize(GetAssetPath(), name, this, out PoseSet poseSet))
            {
                Debug.LogWarning("Failed to read pose set. Creating it in runtime instead.");
                ImportPoseSet();
#if UNITY_EDITOR
                PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize");
                PoseSerializer poseSerializer = new PoseSerializer();
                poseSerializer.Serialize(_poseSet, GetAssetPath(), this.name);
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize");
#endif
            }
            else
            {
                _poseSet = poseSet;
            }

            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Import");
        }

        return _poseSet;
    }

    public void ImportPoseSet()
    {
        ImportAnimations();
        _poseSet = new PoseSet(this);
        _poseSet.SetSkeletonFromBvh(animationClips[0].Skeleton);
        for (int i = 0; i < animationClips.Count; i++)
        {
            // Extract poses
            if (!PoseExtractor.Extract(animationClips[i], _poseSet, this))
            {
                Debug.LogWarning("[FeatureDebug] Failed to extract poseSet from AnimationDat. Animation Index: " + i);
            }
        }

        _poseSet.ConvertTagsToNativeArrays();
        Debug.Log("Number of poses: " + _poseSet.NumberPoses);
    }

    public FeatureSet GetOrImportFeatureSet()
    {
        if (FeatureSet == null)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Import");
            FeatureSerializer serializer = new FeatureSerializer();
            if (!serializer.Deserialize(GetAssetPath(), name, this, out FeatureSet featureSet))
            {
                Debug.LogWarning("Failed to read feature set. Creating it in runtime instead.");
                ImportFeatureSet();
#if UNITY_EDITOR
                PROFILE.BEGIN_SAMPLE_PROFILING("Feature Serialize");
                FeatureSerializer featureSerializer = new FeatureSerializer();
                featureSerializer.Serialize(FeatureSet, this, GetAssetPath(), this.name);
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Serialize");
#endif
            }
            else
            {
                FeatureSet = featureSet;
            }

            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Import");
        }

        return FeatureSet;
    }

    public void ImportFeatureSet()
    {
        FeatureSet = new FeatureSet(this, _poseSet.NumberPoses);
        FeatureSet.Extract(_poseSet, this);
        FeatureSet.NormalizeFeatures();
    }

    public void ComputeJointsLocalForward()
    {
        // Import T-Pose
        jointsLocalForward = new float3[tPoseAnimationClip.Skeleton.Joints.Count + 1]; // +1 for the simulation bone
        // Find forward character vector by projecting hips forward vector onto the ground
        Quaternion[] localRotations = tPoseAnimationClip.Frames[0].localRotations;
        float3 hipsWorldForwardProjected = math.mul(localRotations[0], hipsForwardLocalVector);
        hipsWorldForwardProjected.y = 0;
        hipsWorldForwardProjected = math.normalize(hipsWorldForwardProjected);
        // Find right character vector by rotating Y-Axis 90 degrees (Unity is Left-Handed and Y-Axis is Up)
        float3 hipsWorldRightProjected =
            math.mul(quaternion.AxisAngle(math.up(), math.radians(90.0f)), hipsWorldForwardProjected);
        // Compute JointsLocalForward based on the T-Pose
        jointsLocalForward[0] = math.forward();
        for (int i = 1; i < jointsLocalForward.Length; i++)
        {
            quaternion worldRot = quaternion.identity;
            int joint = i - 1;
            while (joint != 0) // while not root
            {
                worldRot = math.mul(localRotations[joint], worldRot);
                joint = tPoseAnimationClip.Skeleton.Joints[joint].parentIndex;
            }

            worldRot = math.mul(localRotations[0], worldRot); // root
            joint = i - 1;
            // Change to Local
            if (!TryGetMecanimBone(tPoseAnimationClip.Skeleton.Joints[joint].name, out HumanBodyBones bone))
            {
                Debug.LogWarning("[FeatureDebug] Failed to find Mecanim bone for joint " +
                                 tPoseAnimationClip.Skeleton.Joints[joint].name);
            }

            float3 worldForward = hipsWorldForwardProjected;
            if (HumanBodyBonesExtensions.IsLeftArmBone(bone))
            {
                worldForward = -hipsWorldRightProjected;
            }
            else if (HumanBodyBonesExtensions.IsRightArmBone(bone))
            {
                worldForward = hipsWorldRightProjected;
            }

            jointsLocalForward[i] = math.mul(math.inverse(worldRot), worldForward);
        }
    }

    /// <summary>
    /// Returns the local forward vector of the iven joint index (after adding simulation bone)
    /// Vector computed from the T-Pose BVH and HipsForwardLocalVector
    /// </summary>
    public float3 GetLocalForward(int jointIndex)
    {
        Debug.Assert(!JointsLocalForwardError, "JointsLocalForward is not initialized");
        return jointsLocalForward[jointIndex];
    }

    public bool TryGetMecanimBone(string jointName, out HumanBodyBones bone)
    {
        for (int i = 0; i < animationChannelToMecanim.Count; i++)
        {
            if (animationChannelToMecanim[i].name != jointName) continue;
            bone = animationChannelToMecanim[i].mecanimBone;
            return true;
        }

        bone = HumanBodyBones.LastBone;
        return false;
    }

    public bool TryGetJointName(HumanBodyBones bone, out string jointName)
    {
        for (int i = 0; i < animationChannelToMecanim.Count; i++)
        {
            if (animationChannelToMecanim[i].mecanimBone == bone)
            {
                jointName = animationChannelToMecanim[i].name;
                return true;
            }
        }

        jointName = "";
        return false;
    }

    public string GetAssetPath()
    {
        string path = Path.Combine(Application.streamingAssetsPath, "MMDatabases", name);
#if UNITY_EDITOR
        if (!Directory.Exists(path))
        {
            Directory.CreateDirectory(path);
        }
#endif
        return path;
    }

    [Serializable]
    public struct JointToMecanim
    {
        [FormerlySerializedAs("Name")] public string name;
        [FormerlySerializedAs("MecanimBone")] public HumanBodyBones mecanimBone;

        public JointToMecanim(string name, HumanBodyBones mecanimBone)
        {
            this.name = name;
            this.mecanimBone = mecanimBone;
        }
    }

    [Serializable]
    public class TrajectoryFeature
    {
        public enum Type
        {
            Position,
            Direction,
            Custom1D,
            Custom2D,
            Custom3D,
            Custom4D
        }

        [FormerlySerializedAs("Name")] public string name;
        [FormerlySerializedAs("FeatureType")] public Type featureType;

        [FormerlySerializedAs("FramesPrediction")]
        public int[] framesPrediction = new int[0]; // Number of frames in the future for each point of the trajectory

        [FormerlySerializedAs("SimulationBone")]
        public bool
            simulationBone; // Use the simulation bone (articial root added during pose extraction) instead of a bone

        [FormerlySerializedAs("Bone")]
        public HumanBodyBones bone; // Bone used to compute the trajectory in the feature set

        [FormerlySerializedAs("ZeroX")] public bool zeroX; // Zero the X, Y and/or Z component of the trajectory feature
        [FormerlySerializedAs("ZeroY")] public bool zeroY; // Zero the X, Y and/or Z component of the trajectory feature
        [FormerlySerializedAs("ZeroZ")] public bool zeroZ; // Zero the X, Y and/or Z component of the trajectory feature

        [FormerlySerializedAs("FeatureExtractor")]
        public ScriptableObject featureExtractor; // Custom feature extractor for user-defined types

        [FormerlySerializedAs("IsMainPositionFeature")]
        public bool
            isMainPositionFeature; // Only for position feature type. Used for visualizing gizmos of other trajectory features colocated with this position feature.

        public int GetSize()
        {
            if (featureType == Type.Position || featureType == Type.Direction)
            {
                return 3 - (zeroX ? 1 : 0) - (zeroY ? 1 : 0) - (zeroZ ? 1 : 0);
            }
            else if (featureType == Type.Custom1D)
            {
                return 1;
            }
            else if (featureType == Type.Custom2D)
            {
                return 2;
            }
            else if (featureType == Type.Custom3D)
            {
                return 3;
            }
            else if (featureType == Type.Custom4D)
            {
                return 4;
            }

            Debug.Assert(false, "Size not defined for FeatureType: " + featureType.ToString());
            return -1;
        }
    }

    [Serializable]
    public class PoseFeature
    {
        public enum Type
        {
            Position,
            Velocity
        }

        [FormerlySerializedAs("Name")] public string name;
        [FormerlySerializedAs("FeatureType")] public Type featureType;
        [FormerlySerializedAs("Bone")] public HumanBodyBones bone;
    }
}
}
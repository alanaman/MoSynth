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
    public class MotionMatchingData : ScriptableObject
    {
        // TODO: DefaultHipsForward... detect/suggest automatically? try to fix automatically at BVHAnimation level? 
        // (if it is fixed some code can be deleted... all code related to DefaultHipsForward and in the UpdateTransform() when correcting the hips forward)

        public List<AnimationData> AnimationDatas;
        public AnimationData AnimationDataTPose; // Animation with a TPose in the first frame, used for retargeting
        public float3 HipsForwardLocalVector = new(0, 0, 1); // Local vector (axis) pointing in the forward direction of the hips
        public float3 HipsUpLocalVector = new(0, 1, 0); // Local vector (axis) pointing in the up direction of the hips
        // TODO: Implement Savitzky-Golay filter or similar low-pass filter in Unity (before I was using Python implementation)
        //public bool SmoothSimulationBone; // Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter
        public float ContactVelocityThreshold = 0.15f; // Minimum velocity of the foot to be considered in movement and not in contact with the ground
        [FormerlySerializedAs("SkeletonToMecanim")] public List<JointToMecanim> animationChannelToMecanim = new();
        
        // TODO: these should prolly be inside FeatureSet
        public List<TrajectoryFeature> TrajectoryFeatures = new();
        public List<PoseFeature> PoseFeatures = new();
        public List<TrajectoryFeature> EnvironmentFeatures = new(); // Features used for dynamic computations and not used during the standard distance check in the Motion Matching search

        private PoseSet _poseSet;

        public FeatureSet FeatureSet
        {
            get => _featureSet;
            private set => _featureSet = value;
        }

        private int PoseSetUserCount = 0;
        private int FeatureSetUserCount = 0;

        // Information extracted form T-Pose
        [FormerlySerializedAs("JointsLocalForward")] 
        [SerializeField] private float3[] jointsLocalForward; // Local forward vector of each joint 
        private FeatureSet _featureSet;
        public bool JointsLocalForwardError => jointsLocalForward == null;

        private void ImportAnimations()
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("BVH Import");
            foreach (var animData in AnimationDatas)
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
            PoseSetUserCount += 1;
            return _poseSet;
        }

        public void ImportPoseSet()
        {
            ImportAnimations();
            _poseSet = new PoseSet(this);
            _poseSet.SetSkeletonFromBvh(AnimationDatas[0].GetAnimation().Skeleton);
            for (int i = 0; i < AnimationDatas.Count; i++)
            {
                // Extract poses
                if (!PoseExtractor.Extract(AnimationDatas[i], _poseSet, this))
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
            FeatureSetUserCount += 1;
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
            BvhAnimation tPoseAnimation = AnimationDataTPose.GetAnimation();
            jointsLocalForward = new float3[tPoseAnimation.Skeleton.Joints.Count + 1]; // +1 for the simulation bone
            // Find forward character vector by projecting hips forward vector onto the ground
            Quaternion[] localRotations = tPoseAnimation.Frames[0].localRotations;
            float3 hipsWorldForwardProjected = math.mul(localRotations[0], HipsForwardLocalVector);
            hipsWorldForwardProjected.y = 0;
            hipsWorldForwardProjected = math.normalize(hipsWorldForwardProjected);
            // Find right character vector by rotating Y-Axis 90 degrees (Unity is Left-Handed and Y-Axis is Up)
            float3 hipsWorldRightProjected = math.mul(quaternion.AxisAngle(math.up(), math.radians(90.0f)), hipsWorldForwardProjected);
            // Compute JointsLocalForward based on the T-Pose
            jointsLocalForward[0] = math.forward();
            for (int i = 1; i < jointsLocalForward.Length; i++)
            {
                quaternion worldRot = quaternion.identity;
                int joint = i - 1;
                while (joint != 0) // while not root
                {
                    worldRot = math.mul(localRotations[joint], worldRot);
                    joint = tPoseAnimation.Skeleton.Joints[joint].parentIndex;
                }
                worldRot = math.mul(localRotations[0], worldRot); // root
                joint = i - 1;
                // Change to Local
                if (!TryGetMecanimBone(tPoseAnimation.Skeleton.Joints[joint].name, out HumanBodyBones bone))
                {
                    Debug.LogWarning("[FeatureDebug] Failed to find Mecanim bone for joint " + tPoseAnimation.Skeleton.Joints[joint].name);
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

        [System.Serializable]
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
            public string Name;
            public Type FeatureType;
            public int[] FramesPrediction = new int[0]; // Number of frames in the future for each point of the trajectory
            public bool SimulationBone; // Use the simulation bone (articial root added during pose extraction) instead of a bone
            public HumanBodyBones Bone; // Bone used to compute the trajectory in the feature set
            public bool ZeroX, ZeroY, ZeroZ; // Zero the X, Y and/or Z component of the trajectory feature
            public ScriptableObject FeatureExtractor; // Custom feature extractor for user-defined types
            public bool IsMainPositionFeature; // Only for position feature type. Used for visualizing gizmos of other trajectory features colocated with this position feature.

            public int GetSize()
            {
                if (FeatureType == Type.Position || FeatureType == Type.Direction)
                {
                    return 3 - (ZeroX ? 1 : 0) - (ZeroY ? 1 : 0) - (ZeroZ ? 1 : 0);
                }
                else if (FeatureType == Type.Custom1D)
                {
                    return 1;
                }
                else if (FeatureType == Type.Custom2D)
                {
                    return 2;
                }
                else if (FeatureType == Type.Custom3D)
                {
                    return 3;
                }
                else if (FeatureType == Type.Custom4D)
                {
                    return 4;
                }
                Debug.Assert(false, "Size not defined for FeatureType: " + FeatureType.ToString());
                return -1;
            }
        }

        [System.Serializable]
        public class PoseFeature
        {
            public enum Type
            {
                Position,
                Velocity
            }
            public string Name;
            public Type FeatureType;
            public HumanBodyBones Bone;
        }

        public void Dispose()
        {
            PoseSetUserCount = math.max(0, PoseSetUserCount - 1);
            FeatureSetUserCount = math.max(0, FeatureSetUserCount - 1);
            if (_poseSet != null && PoseSetUserCount == 0)
            {
                _poseSet.Dispose();
                _poseSet = null;
            }
            if (FeatureSet != null && FeatureSetUserCount == 0)
            {
                FeatureSet.Dispose();
                FeatureSet = null;
            }
        }
    }
}
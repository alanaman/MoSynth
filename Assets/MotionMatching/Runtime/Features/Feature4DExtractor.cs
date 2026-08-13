using AnimationTools;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    public abstract class Feature4DExtractor : ScriptableObject
    {
        /// <summary>
        /// Called once at the beginning of the feature extraction process.
        /// </summary
        public abstract void StartExtracting(SkeletonAsset skeleton);
        /// <summary>
        /// Called for each pose in the motion clip.
        /// StartExtracting(...) is always called once before ExtractFeature(...) is called multiple times.
        /// </summary
        public abstract float4 ExtractFeature(PoseVector pose, int poseIndex, PoseVector nextPose, int animationClip, SkeletonAsset skeleton, float3 characterOrigin, float3 characterForward);
        /// <summary>
        /// Called when Gizmos are drawn to debug features
        /// StartExtracting(...) is not called before DrawGizmos(...)
        /// </summary>
        public abstract void DrawGizmos(float4 feature, float radius,
                                        float3 characterOrigin, float3 characterForward,
                                        Transform[] joints, SkeletonAsset skeleton, float3 posOffset);
    }
}

using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{

/// <summary>
/// A temporary parent class for a component that does motion matching
/// TODO: remove 
/// </summary>
public abstract class MotionSynthesizer : MonoBehaviour 
{

    public abstract MotionMatchingData MmData { get; }

    public float DatabaseFrameTime => MmData.GetOrImportPoseSet().FrameTime;
    
    public abstract float3 RootVelocity { get; protected set; }
    public abstract float3 RootAngularVelocity { get; protected set; }
    
    public abstract float3 RootPosition { get; protected set; }
    public abstract quaternion RootRotation { get; protected set; }
    public abstract PoseVector CurrentPose { get; }

    public abstract void SetRotAdjustment(quaternion adjustmentRotation);
    public abstract void SetPosAdjustment(float3 adjustmentPosition);

    public abstract float3 GetMainPositionFeature(int trajectoryIndex);
    public abstract float4 GetEnvironmentFeature(string featureName, int trajectoryIndex);

}
}
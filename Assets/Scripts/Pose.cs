using System;
using System.Collections.Generic;
using System.Numerics;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Splines;

namespace AnimationTools
{
class PoseElement
{
}

abstract class PositionPoseElement : PoseElement
{
    public abstract float3 GetPosition(PoseDefinition def, PoseArray);
}

class BonePositionPoseElement : PositionPoseElement
{
    private Bone bone;

    public BonePositionPoseElement(Bone bone)
    {
        this.bone = bone;
    }

    public override float3 GetPosition(PoseDefinition def, PoseArray array)
    {
        var range = def.ranges[this];
        var slice = array.values.AsSpan().Slice(range.Start.Value,
            range.End.Value - range.Start.Value);
        return new float3(slice[0], slice[1], slice[2]);
    }
}
class PoseDefinition
{
    public Dictionary<PoseElement, Range> ranges = new Dictionary<PoseElement, Range>();
}

public class PoseArray
{
    public NativeArray<float> values;
}
}
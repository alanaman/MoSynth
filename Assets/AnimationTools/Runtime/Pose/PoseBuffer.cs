using System.Diagnostics;
using Unity.Collections;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

namespace AnimationTools
{
/// <summary>
/// A flat float pose buffer described by a <see cref="PoseLayoutData"/>.
/// </summary>
/// <remarks>
/// <see cref="PoseBuffer"/> is a VIEW struct, exactly like <see cref="NativeArray{T}"/>
/// itself: copying a PoseBuffer copies the view, not the data — every copy aliases the
/// same underlying <see cref="Data"/> array. Ownership follows the usual Native* rule:
/// whoever calls <see cref="Allocate"/> is responsible for calling <see cref="Dispose"/>.
/// Component-lifetime buffers should use <see cref="Allocator.Persistent"/> and dispose in
/// OnDestroy; short-lived scratch buffers should use <see cref="Allocator.Temp"/>.
/// </remarks>
public struct PoseBuffer
{
    public NativeArray<float> Data;
    public PoseLayoutData Layout;

    public bool IsCreated => Data.IsCreated;

    public NativeSlice<float3> Positions =>
        new NativeSlice<float>(Data, Layout.PositionStart, Layout.PositionCount * 3).SliceConvert<float3>();

    public NativeSlice<quaternion> Rotations
    {
        get
        {
            Debug.Assert(Layout.RotationStride == 4, "Rotations slice requires a Quaternion (stride 4) representation.");
            return new NativeSlice<float>(Data, Layout.RotationStart, Layout.RotationCount * 4).SliceConvert<quaternion>();
        }
    }

    public NativeSlice<float3> Scales =>
        new NativeSlice<float>(Data, Layout.ScaleStart, Layout.ScaleCount * 3).SliceConvert<float3>();

    public NativeSlice<float3> Velocities =>
        new NativeSlice<float>(Data, Layout.VelocityStart, Layout.VelocityCount * 3).SliceConvert<float3>();

    public NativeSlice<float3> AngularVelocities =>
        new NativeSlice<float>(Data, Layout.AngularVelocityStart, Layout.AngularVelocityCount * 3).SliceConvert<float3>();

    public float3 GetPosition(PositionHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "PositionHandle");
        return new float3(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2]);
    }

    public void SetPosition(PositionHandle h, float3 value)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "PositionHandle");
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
        Data[h.Index + 2] = value.z;
    }

    public quaternion GetRotation(RotationHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 4, "RotationHandle");
        Debug.Assert(Layout.RotationStride == 4, "GetRotation requires a Quaternion (stride 4) representation.");
        return new quaternion(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2], Data[h.Index + 3]);
    }

    public void SetRotation(RotationHandle h, quaternion value)
    {
        AssertHandle(h.Index, h.LayoutHash, 4, "RotationHandle");
        Debug.Assert(Layout.RotationStride == 4, "SetRotation requires a Quaternion (stride 4) representation.");
        var v = value.value;
        Data[h.Index] = v.x;
        Data[h.Index + 1] = v.y;
        Data[h.Index + 2] = v.z;
        Data[h.Index + 3] = v.w;
    }

    public float3 GetScale(ScaleHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "ScaleHandle");
        return new float3(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2]);
    }

    public void SetScale(ScaleHandle h, float3 value)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "ScaleHandle");
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
        Data[h.Index + 2] = value.z;
    }

    public float3 GetVelocity(VelocityHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "VelocityHandle");
        return new float3(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2]);
    }

    public void SetVelocity(VelocityHandle h, float3 value)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "VelocityHandle");
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
        Data[h.Index + 2] = value.z;
    }

    public float3 GetAngularVelocity(AngularVelocityHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "AngularVelocityHandle");
        return new float3(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2]);
    }

    public void SetAngularVelocity(AngularVelocityHandle h, float3 value)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "AngularVelocityHandle");
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
        Data[h.Index + 2] = value.z;
    }

    public bool GetBool(BoolHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 1, "BoolHandle");
        return Data[h.Index] > 0.5f;
    }

    public void SetBool(BoolHandle h, bool value)
    {
        AssertHandle(h.Index, h.LayoutHash, 1, "BoolHandle");
        Data[h.Index] = value ? 1f : 0f;
    }

    public float GetFloat(ChannelHandle h, int offset = 0)
    {
        AssertHandle(h.Index, h.LayoutHash, h.Count, "ChannelHandle");
        Debug.Assert(offset >= 0 && offset < h.Count, "Offset is outside the channel.");
        return Data[h.Index + offset];
    }

    public void SetFloat(ChannelHandle h, int offset, float value)
    {
        AssertHandle(h.Index, h.LayoutHash, h.Count, "ChannelHandle");
        Debug.Assert(offset >= 0 && offset < h.Count, "Offset is outside the channel.");
        Data[h.Index + offset] = value;
    }

    public float2 GetFloat2(ChannelHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 2, "ChannelHandle");
        return new float2(Data[h.Index], Data[h.Index + 1]);
    }

    public void SetFloat2(ChannelHandle h, float2 value)
    {
        AssertHandle(h.Index, h.LayoutHash, 2, "ChannelHandle");
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
    }

    public float3 GetFloat3(ChannelHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "ChannelHandle");
        return new float3(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2]);
    }

    public void SetFloat3(ChannelHandle h, float3 value)
    {
        AssertHandle(h.Index, h.LayoutHash, 3, "ChannelHandle");
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
        Data[h.Index + 2] = value.z;
    }

    public NativeSlice<float> GetSlice(ChannelHandle h)
    {
        AssertHandle(h.Index, h.LayoutHash, h.Count, "ChannelHandle");
        return new NativeSlice<float>(Data, h.Index, h.Count);
    }

    public void CopyFrom(in PoseBuffer other)
    {
        Debug.Assert(Layout.LayoutHash == other.Layout.LayoutHash, "CopyFrom requires matching layouts.");
        NativeArray<float>.Copy(other.Data, Data, Layout.FloatCount);
    }

    public static void Lerp(in PoseBuffer a, in PoseBuffer b, float t, PoseBuffer destination)
    {
        Debug.Assert(a.Layout.LayoutHash == b.Layout.LayoutHash && a.Layout.LayoutHash == destination.Layout.LayoutHash,
            "Lerp requires all three buffers to share the same layout.");

        var layout = a.Layout;

        for (var i = 0; i < layout.PositionCount * 3; i++)
        {
            var idx = layout.PositionStart + i;
            destination.Data[idx] = math.lerp(a.Data[idx], b.Data[idx], t);
        }

        if (layout.RotationStride == 4)
        {
            for (var e = 0; e < layout.RotationCount; e++)
            {
                var idx = layout.RotationStart + e * 4;
                var qa = new float4(a.Data[idx], a.Data[idx + 1], a.Data[idx + 2], a.Data[idx + 3]);
                var qb = new float4(b.Data[idx], b.Data[idx + 1], b.Data[idx + 2], b.Data[idx + 3]);
                if (math.dot(qa, qb) < 0f) qb = -qb;
                var lerped = math.normalizesafe(math.lerp(qa, qb, t));
                destination.Data[idx] = lerped.x;
                destination.Data[idx + 1] = lerped.y;
                destination.Data[idx + 2] = lerped.z;
                destination.Data[idx + 3] = lerped.w;
            }
        }
        else
        {
            for (var i = 0; i < layout.RotationCount * layout.RotationStride; i++)
            {
                var idx = layout.RotationStart + i;
                destination.Data[idx] = math.lerp(a.Data[idx], b.Data[idx], t);
            }
        }

        for (var i = 0; i < layout.ScaleCount * 3; i++)
        {
            var idx = layout.ScaleStart + i;
            destination.Data[idx] = math.lerp(a.Data[idx], b.Data[idx], t);
        }

        for (var i = 0; i < layout.VelocityCount * 3; i++)
        {
            var idx = layout.VelocityStart + i;
            destination.Data[idx] = math.lerp(a.Data[idx], b.Data[idx], t);
        }

        for (var i = 0; i < layout.AngularVelocityCount * 3; i++)
        {
            var idx = layout.AngularVelocityStart + i;
            destination.Data[idx] = math.lerp(a.Data[idx], b.Data[idx], t);
        }

        for (var e = 0; e < layout.BoolCount; e++)
        {
            var idx = layout.BoolStart + e;
            destination.Data[idx] = t < 0.5f ? a.Data[idx] : b.Data[idx];
        }

        // Sections past the built-in six have no per-kind blend rule, so they interpolate linearly.
        for (var i = 0; i < layout.ExtraCount; i++)
        {
            var idx = layout.ExtraStart + i;
            destination.Data[idx] = math.lerp(a.Data[idx], b.Data[idx], t);
        }
    }

    public static PoseBuffer Allocate(PoseLayout layout, Allocator allocator)
    {
        return new PoseBuffer
        {
            Data = new NativeArray<float>(layout.FloatCount, allocator, NativeArrayOptions.ClearMemory),
            Layout = layout.Data
        };
    }

    public void Dispose()
    {
        if (Data.IsCreated) Data.Dispose();
    }

    [Conditional("UNITY_ASSERTIONS")]
    private void AssertHandle(int index, int layoutHash, int width, string handleName)
    {
        Debug.Assert(index >= 0, $"{handleName} is invalid.");
        Debug.Assert(layoutHash == Layout.LayoutHash, $"{handleName} was bound against a different layout.");
        Debug.Assert(index + width <= Data.Length, $"{handleName} is out of range for this buffer.");
    }
}
}

using System.Diagnostics;
using Unity.Collections;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

namespace AnimationTools
{
/// <summary>
/// A flat float buffer of per-frame state described by a <see cref="StateBufferLayoutData"/>,
/// addressed through <see cref="ChannelHandle"/>s bound against the matching
/// <see cref="StateBufferLayout"/>.
/// </summary>
/// <remarks>
/// <see cref="StateBuffer"/> is a VIEW struct, exactly like <see cref="NativeArray{T}"/> itself:
/// copying a StateBuffer copies the view, not the data — every copy aliases the same underlying
/// <see cref="Data"/> array. Ownership follows the usual Native* rule: whoever calls
/// <see cref="Allocate"/> is responsible for calling <see cref="Dispose"/>. Component-lifetime
/// buffers should use <see cref="Allocator.Persistent"/> and dispose in OnDestroy; short-lived
/// scratch buffers should use <see cref="Allocator.Temp"/>.
/// </remarks>
public struct StateBuffer
{
    public NativeArray<float> Data;
    public StateBufferLayoutData Layout;

    public bool IsCreated => Data.IsCreated;

    public float GetFloat(ChannelHandle h, int offset = 0)
    {
        AssertHandle(h, h.Count);
        Debug.Assert(offset >= 0 && offset < h.Count, "Offset is outside the channel.");
        return Data[h.Index + offset];
    }

    public void SetFloat(ChannelHandle h, int offset, float value)
    {
        AssertHandle(h, h.Count);
        Debug.Assert(offset >= 0 && offset < h.Count, "Offset is outside the channel.");
        Data[h.Index + offset] = value;
    }

    public float2 GetFloat2(ChannelHandle h)
    {
        AssertHandle(h, 2);
        return new float2(Data[h.Index], Data[h.Index + 1]);
    }

    public void SetFloat2(ChannelHandle h, float2 value)
    {
        AssertHandle(h, 2);
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
    }

    public float3 GetFloat3(ChannelHandle h)
    {
        AssertHandle(h, 3);
        return new float3(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2]);
    }

    public void SetFloat3(ChannelHandle h, float3 value)
    {
        AssertHandle(h, 3);
        Data[h.Index] = value.x;
        Data[h.Index + 1] = value.y;
        Data[h.Index + 2] = value.z;
    }

    /// <summary>
    /// A width-4 handle can only belong to a quaternion-valued channel, so the width assert also
    /// covers the representation.
    /// </summary>
    public quaternion GetQuaternion(ChannelHandle h)
    {
        AssertHandle(h, 4);
        return new quaternion(Data[h.Index], Data[h.Index + 1], Data[h.Index + 2], Data[h.Index + 3]);
    }

    public void SetQuaternion(ChannelHandle h, quaternion value)
    {
        AssertHandle(h, 4);
        var v = value.value;
        Data[h.Index] = v.x;
        Data[h.Index + 1] = v.y;
        Data[h.Index + 2] = v.z;
        Data[h.Index + 3] = v.w;
    }

    public bool GetBool(ChannelHandle h)
    {
        AssertHandle(h, 1);
        return Data[h.Index] > 0.5f;
    }

    public void SetBool(ChannelHandle h, bool value)
    {
        AssertHandle(h, 1);
        Data[h.Index] = value ? 1f : 0f;
    }

    public NativeSlice<float> GetSlice(ChannelHandle h)
    {
        AssertHandle(h, h.Count);
        return new NativeSlice<float>(Data, h.Index, h.Count);
    }

    public void CopyFrom(in StateBuffer other)
    {
        Debug.Assert(Layout.LayoutHash == other.Layout.LayoutHash, "CopyFrom requires matching layouts.");
        NativeArray<float>.Copy(other.Data, Data, Layout.FloatCount);
    }

    public static StateBuffer Allocate(StateBufferLayout layout, Allocator allocator)
    {
        return new StateBuffer
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
    private void AssertHandle(ChannelHandle h, int width)
    {
        Debug.Assert(h.Index >= 0, "ChannelHandle is invalid.");
        Debug.Assert(h.LayoutHash == Layout.LayoutHash, "ChannelHandle was bound against a different layout.");
        Debug.Assert(h.Count == width, $"ChannelHandle is {h.Count} floats wide where {width} are expected.");
        Debug.Assert(h.Index + width <= Data.Length, "ChannelHandle is out of range for this buffer.");
    }
}
}

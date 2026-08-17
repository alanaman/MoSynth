using System;
using System.IO;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Loads a MotionRecorder recording (.json manifest + .bin float32 frames) back into memory for analysis.
/// </summary>
public sealed class RecordingReader
{
    public RecordingManifest Manifest { get; }

    /// <summary>Whole frames actually present in the data (file may be shorter than the manifest claims after a crash).</summary>
    public int FrameCount { get; }

    private readonly float[] _data;
    private readonly int _frameFloatCount;

    public RecordingReader(RecordingManifest manifest, float[] data)
    {
        if (manifest == null)
            throw new ArgumentException("manifest must not be null.", nameof(manifest));
        if (manifest.frameFloatCount <= 0)
            throw new ArgumentException(
                $"manifest.frameFloatCount must be positive, was {manifest.frameFloatCount}.", nameof(manifest));

        Manifest = manifest;
        _data = data ?? Array.Empty<float>();
        _frameFloatCount = manifest.frameFloatCount;
        FrameCount = _data.Length / _frameFloatCount;
    }

    public static RecordingReader Load(string manifestPath)
    {
        var manifest = JsonUtility.FromJson<RecordingManifest>(File.ReadAllText(manifestPath));
        var binPath = Path.Combine(Path.GetDirectoryName(manifestPath), manifest.dataFile);

        var bytes = File.ReadAllBytes(binPath);
        var floats = new float[bytes.Length / 4];
        Buffer.BlockCopy(bytes, 0, floats, 0, floats.Length * 4);

        return new RecordingReader(manifest, floats);
    }

    public RecordingManifestChannel GetChannel(string name)
    {
        if (TryGetChannel(name, out var channel)) return channel;

        var available = Manifest.channels == null
            ? ""
            : string.Join(", ", Array.ConvertAll(Manifest.channels, c => c.name));
        throw new ArgumentException(
            $"No channel named \"{name}\" in this recording. Available channels: {available}", nameof(name));
    }

    public bool TryGetChannel(string name, out RecordingManifestChannel channel)
    {
        var channels = Manifest.channels;
        if (channels != null)
        {
            for (var i = 0; i < channels.Length; i++)
            {
                if (channels[i].name != name) continue;
                channel = channels[i];
                return true;
            }
        }

        channel = null;
        return false;
    }

    public float GetFloat(int frame, RecordingManifestChannel channel, int component)
    {
        return _data[frame * _frameFloatCount + channel.floatOffset + component];
    }

    public float3 GetFloat3(int frame, RecordingManifestChannel channel)
    {
        return new float3(
            GetFloat(frame, channel, 0),
            GetFloat(frame, channel, 1),
            GetFloat(frame, channel, 2));
    }
}
}

using UnityEditor;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using AnimationTools;
using MotionMatching;
using System.Collections.Generic;
using UnityEditor.AnimatedValues;
using UnityEditor.SceneManagement;

namespace MotionMatching.Editor
{
    [CustomEditor(typeof(BvhAnimation))]
    public class BvhAnimationEditor : UnityEditor.Editor
    {
        private PreviewRenderUtility _previewRenderUtility;
        private BvhAnimation _bvhAnimation;
        private bool _isPlaying = false;
        private float _currentTime = 0f;
        private float _previousTime = 0f;

        private Vector2 _drag;
        private float _distance = 5f;
        private Vector3 _targetPos = Vector3.zero;

        private Material _lineMaterial;
        private Mesh _skeletonMesh;
        private List<Vector3> _lineVertices = new List<Vector3>();
        private List<int> _lineIndices = new List<int>();

        private Material _gridMaterial;
        private Mesh _gridMesh;

        private SkeletonAsset _skeletonAsset;
        private SkeletonData _skeletonData;
        private PoseBuffer _poseBuffer;
        private NativeArray<float3> _fkPositions;
        private NativeArray<quaternion> _fkRotations;

        private void OnEnable()
        {
            _bvhAnimation = (BvhAnimation)target;
            EditorApplication.update += UpdateSimulation;
            _previousTime = (float)EditorApplication.timeSinceStartup;

            InitPoseState();

            Shader shader = Shader.Find("Hidden/Internal-Colored");
            if (shader != null)
            {
                _lineMaterial = new Material(shader);
                _lineMaterial.hideFlags = HideFlags.HideAndDontSave;
                _lineMaterial.SetInt("_ZWrite", 1);
                _lineMaterial.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.LessEqual);
                _lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            }

            _skeletonMesh = new Mesh();
            _skeletonMesh.hideFlags = HideFlags.HideAndDontSave;
            _skeletonMesh.MarkDynamic();

            if (shader != null)
            {
                _gridMaterial = new Material(shader);
                _gridMaterial.hideFlags = HideFlags.HideAndDontSave;
                _gridMaterial.SetInt("_ZWrite", 1);
                _gridMaterial.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.LessEqual);
                _gridMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            }
            CreateGridMesh();
        }

        private void InitPoseState()
        {
            var skeleton = _bvhAnimation.Skeleton;
            if (skeleton?.Joints == null || skeleton.Joints.Count == 0) return;

            _skeletonAsset = ScriptableObject.CreateInstance<SkeletonAsset>();
            _skeletonAsset.hideFlags = HideFlags.HideAndDontSave;
            SkeletonAssetFromMmData.Build(skeleton, _skeletonAsset);

            var layout = PoseLayout.CreateFullPose(_skeletonAsset, false, false);
            _skeletonData = _skeletonAsset.GetSkeletonData();
            _poseBuffer = PoseBuffer.Allocate(layout, Allocator.Persistent);

            var positions = _poseBuffer.Positions;
            for (var i = 0; i < _skeletonAsset.BoneCount; i++)
            {
                positions[i] = _skeletonAsset.GetBone(i).restLocalPosition;
            }

            _fkPositions = new NativeArray<float3>(_skeletonAsset.BoneCount, Allocator.Persistent);
            _fkRotations = new NativeArray<quaternion>(_skeletonAsset.BoneCount, Allocator.Persistent);
        }

        private void CreateGridMesh()
        {
            _gridMesh = new Mesh();
            _gridMesh.hideFlags = HideFlags.HideAndDontSave;

            List<Vector3> verts = new List<Vector3>();
            List<int> indices = new List<int>();
            List<Color> colors = new List<Color>();

            int gridSize = 10;
            float step = 1f;
            Color darkGray = new Color(0.3f, 0.3f, 0.3f, 1f);
            Color lightGray = new Color(0.5f, 0.5f, 0.5f, 1f);

            for (int i = -gridSize; i <= gridSize; i++)
            {
                verts.Add(new Vector3(i * step, 0, -gridSize * step));
                verts.Add(new Vector3(i * step, 0, gridSize * step));

                verts.Add(new Vector3(-gridSize * step, 0, i * step));
                verts.Add(new Vector3(gridSize * step, 0, i * step));

                Color c = (i % 5 == 0) ? lightGray : darkGray;
                colors.Add(c); colors.Add(c);
                colors.Add(c); colors.Add(c);

                int count = indices.Count;
                indices.Add(count); indices.Add(count + 1);
                indices.Add(count + 2); indices.Add(count + 3);
            }

            _gridMesh.SetVertices(verts);
            _gridMesh.SetColors(colors);
            _gridMesh.SetIndices(indices, MeshTopology.Lines, 0);
        }

        private void OnDisable()
        {
            EditorApplication.update -= UpdateSimulation;

            if (_previewRenderUtility != null)
            {
                _previewRenderUtility.Cleanup();
                _previewRenderUtility = null;
            }

            if (_lineMaterial != null)
            {
                DestroyImmediate(_lineMaterial);
            }
            if (_skeletonMesh != null)
            {
                DestroyImmediate(_skeletonMesh);
            }
            if (_gridMaterial != null)
            {
                DestroyImmediate(_gridMaterial);
            }
            if (_gridMesh != null)
            {
                DestroyImmediate(_gridMesh);
            }

            if (_poseBuffer.IsCreated)
            {
                _poseBuffer.Dispose();
            }
            if (_fkPositions.IsCreated)
            {
                _fkPositions.Dispose();
            }
            if (_fkRotations.IsCreated)
            {
                _fkRotations.Dispose();
            }
            if (_skeletonAsset != null)
            {
                DestroyImmediate(_skeletonAsset);
            }
        }

        private void UpdateSimulation()
        {
            float time = (float)EditorApplication.timeSinceStartup;
            float deltaTime = time - _previousTime;
            _previousTime = time;

            if (_isPlaying && _bvhAnimation != null && _bvhAnimation.Frames != null && _bvhAnimation.Frames.Length > 0)
            {
                _currentTime += deltaTime;
                float duration = _bvhAnimation.Frames.Length * _bvhAnimation.FrameTime;
                if (_currentTime >= duration)
                {
                    _currentTime = _currentTime % duration;
                }
                Repaint();
            }
        }

        public override bool HasPreviewGUI()
        {
            return _bvhAnimation != null && _bvhAnimation.Frames != null && _bvhAnimation.Frames.Length > 0;
        }

        public override GUIContent GetPreviewTitle()
        {
            return new GUIContent("BvhAnimation Preview");
        }

        public override void OnPreviewSettings()
        {
            GUIStyle buttonStyle = new GUIStyle(EditorStyles.toolbarButton);

            if (GUILayout.Button(_isPlaying ? "Pause" : "Play", buttonStyle))
            {
                _isPlaying = !_isPlaying;
                if (_isPlaying)
                {
                    _previousTime = (float)EditorApplication.timeSinceStartup;
                }
            }

            if (_bvhAnimation != null && _bvhAnimation.Frames != null && _bvhAnimation.Frames.Length > 0)
            {
                float duration = _bvhAnimation.Frames.Length * _bvhAnimation.FrameTime;
                EditorGUI.BeginChangeCheck();
                _currentTime = GUILayout.HorizontalSlider(_currentTime, 0f, duration, GUILayout.Width(150));
                if (EditorGUI.EndChangeCheck())
                {
                    // Update preview if scrubbed manually
                    Repaint();
                }
            }
        }

        public override void OnInteractivePreviewGUI(Rect r, GUIStyle background)
        {
            if (_previewRenderUtility == null)
            {
                _previewRenderUtility = new PreviewRenderUtility();
                _previewRenderUtility.camera.fieldOfView = 30f;
                _previewRenderUtility.camera.nearClipPlane = 0.01f;
                _previewRenderUtility.camera.farClipPlane = 1000f;
            }

            HandleCameraControls(r);

            _previewRenderUtility.BeginPreview(r, background);

            if (_gridMaterial != null && _gridMesh != null)
            {
                _previewRenderUtility.DrawMesh(_gridMesh, Matrix4x4.identity, _gridMaterial, 0);
            }

            DrawSkeleton();

            _previewRenderUtility.camera.Render();
            Texture resultRender = _previewRenderUtility.EndPreview();
            GUI.DrawTexture(r, resultRender, ScaleMode.StretchToFill, false);
        }

        private void HandleCameraControls(Rect r)
        {
            Event e = Event.current;

            if (r.Contains(e.mousePosition))
            {
                if (e.type == EventType.MouseDrag && e.button == 0)
                {
                    _drag.x += e.delta.x * 0.5f;
                    _drag.y += e.delta.y * 0.5f;
                    e.Use();
                }
                else if (e.type == EventType.MouseDrag && e.button == 2)
                {
                    // Pan
                    Vector3 right = _previewRenderUtility.camera.transform.right;
                    Vector3 up = _previewRenderUtility.camera.transform.up;
                    _targetPos -= (right * e.delta.x - up * e.delta.y) * (0.01f * _distance);
                    e.Use();
                }
                else if (e.type == EventType.ScrollWheel)
                {
                    _distance += e.delta.y * 0.1f;
                    _distance = Mathf.Max(0.1f, _distance);
                    e.Use();
                }
            }
        }

        private void DrawSkeleton()
        {
            if (_bvhAnimation == null || _bvhAnimation.Frames == null || _bvhAnimation.Frames.Length == 0) return;

            int frameIndex = Mathf.FloorToInt(_currentTime / _bvhAnimation.FrameTime);
            frameIndex = Mathf.Clamp(frameIndex, 0, _bvhAnimation.Frames.Length - 1);

            var frame = _bvhAnimation.Frames[frameIndex];
            var boneCount = _skeletonData.BoneCount;

            if (!_poseBuffer.IsCreated) return;
            if (frame.localRotations == null || frame.localRotations.Length != boneCount) return;

            var positions = _poseBuffer.Positions;
            positions[0] = frame.rootMotion;
            var rotations = _poseBuffer.Rotations;
            for (var i = 0; i < boneCount; i++)
            {
                rotations[i] = frame.localRotations[i];
            }

            PoseFK.LocalToCharacter(_poseBuffer, _skeletonData, _fkPositions, _fkRotations);

            _lineVertices.Clear();
            _lineIndices.Clear();

            // Root to children
            for (var i = 1; i < boneCount; i++)
            {
                var parentIndex = _skeletonData.ParentIndices[i];

                _lineVertices.Add(_fkPositions[parentIndex]);
                _lineVertices.Add(_fkPositions[i]);

                var indexCount = _lineIndices.Count;
                _lineIndices.Add(indexCount);
                _lineIndices.Add(indexCount + 1);
            }

            _skeletonMesh.Clear();
            _skeletonMesh.SetVertices(_lineVertices);
            _skeletonMesh.SetIndices(_lineIndices, MeshTopology.Lines, 0);

            // Setup Camera
            Vector3 rootPos = _fkPositions[0];

            // Adjust target if needed, maybe slowly lerp?
            // For now, center around rootPos + _targetPos (pan offset)
            Vector3 camTarget = rootPos + _targetPos;

            Quaternion camRotation = Quaternion.Euler(_drag.y, _drag.x, 0);
            _previewRenderUtility.camera.transform.position = camTarget - camRotation * Vector3.forward * _distance;
            _previewRenderUtility.camera.transform.rotation = camRotation;

            Color[] colors = new Color[_lineVertices.Count];
            for(int i = 0; i < colors.Length; i++) colors[i] = Color.green;
            _skeletonMesh.SetColors(colors);

            Matrix4x4 matrix = Matrix4x4.identity;

            if (_lineMaterial != null)
            {
                _previewRenderUtility.DrawMesh(_skeletonMesh, matrix, _lineMaterial, 0);
            }
        }
    }
}
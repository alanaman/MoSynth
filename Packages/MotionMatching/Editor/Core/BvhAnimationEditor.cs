using UnityEditor;
using UnityEngine;
using Unity.Mathematics;
using MotionMatching;
using System.Collections.Generic;
using UnityEditor.AnimatedValues;
using UnityEditor.SceneManagement;

namespace MotionMatching.Editor
{
    [CustomEditor(typeof(BvhAnimation))]
    public class BvhAnimationEditor : UnityEditor.Editor
    {
        private PreviewRenderUtility previewRenderUtility;
        private BvhAnimation bvhAnimation;
        private bool isPlaying = false;
        private float currentTime = 0f;
        private float previousTime = 0f;

        private Vector2 drag;
        private float distance = 5f;
        private Vector3 targetPos = Vector3.zero;

        private Material lineMaterial;
        private Mesh skeletonMesh;
        private List<Vector3> lineVertices = new List<Vector3>();
        private List<int> lineIndices = new List<int>();

        private Material gridMaterial;
        private Mesh gridMesh;

        private void OnEnable()
        {
            bvhAnimation = (BvhAnimation)target;
            EditorApplication.update += UpdateSimulation;
            previousTime = (float)EditorApplication.timeSinceStartup;

            Shader shader = Shader.Find("Hidden/Internal-Colored");
            if (shader != null)
            {
                lineMaterial = new Material(shader);
                lineMaterial.hideFlags = HideFlags.HideAndDontSave;
                lineMaterial.SetInt("_ZWrite", 1);
                lineMaterial.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.LessEqual);
                lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            }

            skeletonMesh = new Mesh();
            skeletonMesh.hideFlags = HideFlags.HideAndDontSave;
            skeletonMesh.MarkDynamic();

            if (shader != null)
            {
                gridMaterial = new Material(shader);
                gridMaterial.hideFlags = HideFlags.HideAndDontSave;
                gridMaterial.SetInt("_ZWrite", 1);
                gridMaterial.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.LessEqual);
                gridMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            }
            CreateGridMesh();
        }

        private void CreateGridMesh()
        {
            gridMesh = new Mesh();
            gridMesh.hideFlags = HideFlags.HideAndDontSave;

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

            gridMesh.SetVertices(verts);
            gridMesh.SetColors(colors);
            gridMesh.SetIndices(indices, MeshTopology.Lines, 0);
        }

        private void OnDisable()
        {
            EditorApplication.update -= UpdateSimulation;

            if (previewRenderUtility != null)
            {
                previewRenderUtility.Cleanup();
                previewRenderUtility = null;
            }

            if (lineMaterial != null)
            {
                DestroyImmediate(lineMaterial);
            }
            if (skeletonMesh != null)
            {
                DestroyImmediate(skeletonMesh);
            }
            if (gridMaterial != null)
            {
                DestroyImmediate(gridMaterial);
            }
            if (gridMesh != null)
            {
                DestroyImmediate(gridMesh);
            }
        }

        private void UpdateSimulation()
        {
            float time = (float)EditorApplication.timeSinceStartup;
            float deltaTime = time - previousTime;
            previousTime = time;

            if (isPlaying && bvhAnimation != null && bvhAnimation.Frames != null && bvhAnimation.Frames.Length > 0)
            {
                currentTime += deltaTime;
                float duration = bvhAnimation.Frames.Length * bvhAnimation.FrameTime;
                if (currentTime >= duration)
                {
                    currentTime = currentTime % duration;
                }
                Repaint();
            }
        }

        public override bool HasPreviewGUI()
        {
            return bvhAnimation != null && bvhAnimation.Frames != null && bvhAnimation.Frames.Length > 0;
        }

        public override GUIContent GetPreviewTitle()
        {
            return new GUIContent("BvhAnimation Preview");
        }

        public override void OnPreviewSettings()
        {
            GUIStyle buttonStyle = new GUIStyle(EditorStyles.toolbarButton);

            if (GUILayout.Button(isPlaying ? "Pause" : "Play", buttonStyle))
            {
                isPlaying = !isPlaying;
                if (isPlaying)
                {
                    previousTime = (float)EditorApplication.timeSinceStartup;
                }
            }

            if (bvhAnimation != null && bvhAnimation.Frames != null && bvhAnimation.Frames.Length > 0)
            {
                float duration = bvhAnimation.Frames.Length * bvhAnimation.FrameTime;
                EditorGUI.BeginChangeCheck();
                currentTime = GUILayout.HorizontalSlider(currentTime, 0f, duration, GUILayout.Width(150));
                if (EditorGUI.EndChangeCheck())
                {
                    // Update preview if scrubbed manually
                    Repaint();
                }
            }
        }

        public override void OnInteractivePreviewGUI(Rect r, GUIStyle background)
        {
            if (previewRenderUtility == null)
            {
                previewRenderUtility = new PreviewRenderUtility();
                previewRenderUtility.camera.fieldOfView = 30f;
                previewRenderUtility.camera.nearClipPlane = 0.01f;
                previewRenderUtility.camera.farClipPlane = 1000f;
            }

            HandleCameraControls(r);

            previewRenderUtility.BeginPreview(r, background);

            if (gridMaterial != null && gridMesh != null)
            {
                previewRenderUtility.DrawMesh(gridMesh, Matrix4x4.identity, gridMaterial, 0);
            }

            DrawSkeleton();

            previewRenderUtility.camera.Render();
            Texture resultRender = previewRenderUtility.EndPreview();
            GUI.DrawTexture(r, resultRender, ScaleMode.StretchToFill, false);
        }

        private void HandleCameraControls(Rect r)
        {
            Event e = Event.current;

            if (r.Contains(e.mousePosition))
            {
                if (e.type == EventType.MouseDrag && e.button == 0)
                {
                    drag.x += e.delta.x * 0.5f;
                    drag.y += e.delta.y * 0.5f;
                    e.Use();
                }
                else if (e.type == EventType.MouseDrag && e.button == 2)
                {
                    // Pan
                    Vector3 right = previewRenderUtility.camera.transform.right;
                    Vector3 up = previewRenderUtility.camera.transform.up;
                    targetPos -= (right * e.delta.x - up * e.delta.y) * 0.01f * distance;
                    e.Use();
                }
                else if (e.type == EventType.ScrollWheel)
                {
                    distance += e.delta.y * 0.1f;
                    distance = Mathf.Max(0.1f, distance);
                    e.Use();
                }
            }
        }

        private void DrawSkeleton()
        {
            if (bvhAnimation == null || bvhAnimation.Frames == null || bvhAnimation.Frames.Length == 0) return;

            int frameIndex = Mathf.FloorToInt(currentTime / bvhAnimation.FrameTime);
            frameIndex = Mathf.Clamp(frameIndex, 0, bvhAnimation.Frames.Length - 1);

            BvhAnimation.Frame frame = bvhAnimation.Frames[frameIndex];
            Skeleton skeleton = bvhAnimation.Skeleton;

            if (skeleton == null || skeleton.Joints == null || skeleton.Joints.Count == 0) return;

            Vector3[] jointPositions = new Vector3[skeleton.Joints.Count];
            Quaternion[] jointRotations = new Quaternion[skeleton.Joints.Count];

            // Compute global positions and rotations
            for (int i = 0; i < skeleton.Joints.Count; i++)
            {
                Skeleton.Joint joint = skeleton.Joints[i];
                if (i == 0) // Root
                {
                    jointPositions[0] = frame.rootMotion;
                    jointRotations[0] = frame.localRotations[0];
                }
                else
                {
                    int parentIndex = joint.parentIndex;
                    jointRotations[i] = jointRotations[parentIndex] * frame.localRotations[i];
                    jointPositions[i] = jointPositions[parentIndex] + jointRotations[parentIndex] * joint.localOffset;
                }
            }

            lineVertices.Clear();
            lineIndices.Clear();

            // Root to children
            for (int i = 1; i < skeleton.Joints.Count; i++)
            {
                Skeleton.Joint joint = skeleton.Joints[i];
                int parentIndex = joint.parentIndex;

                lineVertices.Add(jointPositions[parentIndex]);
                lineVertices.Add(jointPositions[i]);

                int indexCount = lineIndices.Count;
                lineIndices.Add(indexCount);
                lineIndices.Add(indexCount + 1);
            }

            // End sites
            if (bvhAnimation.EndSites != null)
            {
                for (int i = 0; i < bvhAnimation.EndSites.Count; i++)
                {
                    BvhAnimation.EndSite endSite = bvhAnimation.EndSites[i];
                    Vector3 endPos = jointPositions[endSite.ParentIndex] + jointRotations[endSite.ParentIndex] * endSite.Offset;

                    lineVertices.Add(jointPositions[endSite.ParentIndex]);
                    lineVertices.Add(endPos);

                    int indexCount = lineIndices.Count;
                    lineIndices.Add(indexCount);
                    lineIndices.Add(indexCount + 1);
                }
            }

            skeletonMesh.Clear();
            skeletonMesh.SetVertices(lineVertices);
            skeletonMesh.SetIndices(lineIndices, MeshTopology.Lines, 0);

            // Setup Camera
            Vector3 rootPos = jointPositions[0];

            // Adjust target if needed, maybe slowly lerp?
            // For now, center around rootPos + targetPos (pan offset)
            Vector3 camTarget = rootPos + targetPos;

            Quaternion camRotation = Quaternion.Euler(drag.y, drag.x, 0);
            previewRenderUtility.camera.transform.position = camTarget - camRotation * Vector3.forward * distance;
            previewRenderUtility.camera.transform.rotation = camRotation;

            Color[] colors = new Color[lineVertices.Count];
            for(int i = 0; i < colors.Length; i++) colors[i] = Color.green;
            skeletonMesh.SetColors(colors);

            Matrix4x4 matrix = Matrix4x4.identity;

            if (lineMaterial != null)
            {
                previewRenderUtility.DrawMesh(skeletonMesh, matrix, lineMaterial, 0);
            }
        }
    }
}
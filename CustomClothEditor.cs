using UnityEngine;
using UnityEditor;
using System;

namespace Libraries
{
    [CustomEditor(typeof(CustomCloth))]
    public class CustomClothEditor : Editor
    {
        CustomCloth clothScript;
        [Range(0, 1)]
        float currentEditorResistance = 0;
        protected int translatingVertexPos = -1;
        protected bool canHandleEvents = true;
        protected Transform modelTransform;
        private GUIStyle selectedButton = null;
        private EditState state;
        private EditSpringsState editSpringsState;
        private ToolsState toolsState;
        private Triangle[] meshTriangles;
        private Gradient myGradient;
        private float maxDistance = 1;
        private int selectedVertex = -1;
        protected int selectedWatcher = -1;
        protected bool unConstrained = true;
        protected bool startedString;
        protected float lastCutsceneTime;

        protected bool interactiveObjsFoldout;
        protected bool[] interactiveObjsChildFoldouts;
        enum EditState
        {
            None,
            EditSprings,
            EditMaxDistance,
            AddClothColliders,
            Translating,
            Cutscene
        }

        enum EditSpringsState
        {
            Select,
            ShowAllConnections,
            Edit
        }

        enum ToolsState
        {
            Move,
            Rotate,
            Scale
        }

        delegate void Callback(bool i, int pos);
        delegate Color HandleColor(int pos);
        delegate bool ShowVertex(int pos);

        void Awake()
        {
            state = new EditState();
            clothScript = (CustomCloth)target;

            if (clothScript.vertexDatas == null) clothScript.InitializeVertexDatas();

            modelTransform = clothScript.transform;
            meshTriangles = new Triangle[clothScript.triangles.Length / 3];

            if (clothScript.clothColliders == null)
                clothScript.clothColliders = new ClothCollider[0];
          
            for (int i = 0; i < clothScript.triangles.Length; i = i + 3)
            {
                meshTriangles[i / 3] = new Triangle(clothScript.triangles[i], clothScript.triangles[i + 1], clothScript.triangles[i + 2]);
            }

            myGradient = new Gradient();
            myGradient.colorKeys = new GradientColorKey[] { new GradientColorKey(Color.green, 1), new GradientColorKey(Color.yellow, 0.5f), new GradientColorKey(Color.red, 0) };

            if (clothScript.windowRect == null || clothScript.windowRect.width == 0 || clothScript.windowRect.height == 0)
                clothScript.windowRect = new Rect(18, 30, 250, 200);

            if (clothScript.interactiveObjs != null)
            {
                interactiveObjsChildFoldouts = new bool[clothScript.interactiveObjs.Length];
            }
        }

        void OnDisable()
        {
            Tools.hidden = false;
        }


        public override void OnInspectorGUI()
        {
            ClothType temp = (ClothType)EditorGUILayout.EnumPopup("Cloth Type", clothScript.clothData.clothType);
            if (temp != clothScript.clothData.clothType)
            {
                clothScript.clothData.clothType = temp;
                for (int i = 0; i < clothScript.vertexDatas.Length; i++)
                {
                    clothScript.vertexDatas[i].bendingSprings = new int[0];
                    clothScript.vertexDatas[i].bendingSpringsDistancesP2 = new float[0];

                    clothScript.vertexDatas[i].springs = new int[0];
                    clothScript.vertexDatas[i].springsDistancesP2 = new float[0];
                }
                state = EditState.Cutscene;
                lastCutsceneTime = Time.realtimeSinceStartup;
                clothScript.GetVertexConnections();
                Repaint();
                EditorUtility.SetDirty(target);
            }

            clothScript.clothData.gravity = EditorGUILayout.Vector3Field("Gravity", clothScript.clothData.gravity);
            clothScript.clothData.wind = EditorGUILayout.Vector3Field("Wind", clothScript.clothData.wind);
            clothScript.clothData.damp = EditorGUILayout.Slider("Damping", clothScript.clothData.damp, 0, 1);
            clothScript.clothData.deltaPosInfluence = EditorGUILayout.Slider("World Acceleration Scale", clothScript.clothData.deltaPosInfluence, 0, 1);
            clothScript.clothData.stiffness = EditorGUILayout.Slider("Stiffness", clothScript.clothData.stiffness, 0, 1);
            if (clothScript.clothData.clothType == ClothType.Struct_Shear_Bend)
                clothScript.clothData.bending = EditorGUILayout.Slider("Bending", clothScript.clothData.bending, 0, 1);

            clothScript.clothData.volumePressure = EditorGUILayout.FloatField("Volume Pressure", clothScript.clothData.volumePressure);
            clothScript.clothData.sleepThreshold = EditorGUILayout.FloatField("Sleep Threshold", clothScript.clothData.sleepThreshold);
            clothScript.clothData.impactDampening = EditorGUILayout.FloatField("Impact Dampening", clothScript.clothData.impactDampening);


            interactiveObjsFoldout = EditorGUILayout.Foldout(interactiveObjsFoldout, "Interactive Objects");
            if (interactiveObjsFoldout)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Space(15);
                GUILayout.BeginVertical();
                int tempSize = EditorGUILayout.IntField("Size", clothScript.interactiveObjs != null ? clothScript.interactiveObjs.Length : 0);
                if (clothScript.interactiveObjs == null || tempSize != clothScript.interactiveObjs.Length)
                {
                    Array.Resize<InteractiveObj>(ref clothScript.interactiveObjs, tempSize);
                    Array.Resize<bool>(ref interactiveObjsChildFoldouts, tempSize);

                    for (int i = 0; i < clothScript.interactiveObjs.Length; i++)
                    {
                        clothScript.interactiveObjs[i] = new InteractiveObj();

                    }

                    EditorUtility.SetDirty(target);
                }
                for (int l = 0; l < clothScript.interactiveObjs.Length; l++)
                {
                    interactiveObjsChildFoldouts[l] = EditorGUILayout.Foldout(interactiveObjsChildFoldouts[l], "Element " + l);
                    if (interactiveObjsChildFoldouts[l])
                    {
                        clothScript.interactiveObjs[l].interactiveObjType = (ObjType)EditorGUILayout.EnumPopup("Object Type", clothScript.interactiveObjs[l].interactiveObjType);
                        clothScript.interactiveObjs[l].paramsType = (ObjParamsType)EditorGUILayout.EnumPopup("Collider Params Type", clothScript.interactiveObjs[l].paramsType);

                        if (clothScript.interactiveObjs[l].interactiveObjType == ObjType.Sphere)
                        {
                            if (clothScript.interactiveObjs[l].paramsType == ObjParamsType.UnityCollider)
                            {
                                clothScript.interactiveObjs[l].sphereCollider = EditorGUILayout.ObjectField("Sphere Collider", clothScript.interactiveObjs[l].sphereCollider, typeof(SphereCollider), true) as SphereCollider;
                                clothScript.interactiveObjs[l].transform = clothScript.interactiveObjs[l].sphereCollider != null ? clothScript.interactiveObjs[l].sphereCollider.transform : null;

                                if (clothScript.interactiveObjs[l].sphereCollider != null)
                                    clothScript.interactiveObjs[l].radius = clothScript.interactiveObjs[l].sphereCollider.radius * clothScript.interactiveObjs[l].transform.lossyScale.y;
                            }
                            else
                            {
                                clothScript.interactiveObjs[l].transform = EditorGUILayout.ObjectField("Transform", clothScript.interactiveObjs[l].transform, typeof(Transform), true) as Transform;
                                clothScript.interactiveObjs[l].radius = EditorGUILayout.FloatField("Radius", clothScript.interactiveObjs[l].radius);
                            }

                            clothScript.interactiveObjs[l].localRadius =  clothScript.interactiveObjs[l].radius / clothScript.transform.lossyScale.y;
                        }
                        //else if (clothScript.interactiveObjs[l].interactiveObjType == ObjType.Box)
                        //{
                        //    if (clothScript.interactiveObjs[l].paramsType == ObjParamsType.UnityCollider)
                        //    {
                        //        clothScript.interactiveObjs[l].boxCollider = EditorGUILayout.ObjectField("Box Collider", clothScript.interactiveObjs[l].sphereCollider, typeof(BoxCollider), true) as BoxCollider;
                        //        clothScript.interactiveObjs[l].transform = clothScript.interactiveObjs[l].boxCollider != null ? clothScript.interactiveObjs[l].boxCollider.transform : null;

                        //        if (clothScript.interactiveObjs[l].boxCollider != null)
                        //            clothScript.interactiveObjs[l].radius = clothScript.interactiveObjs[l].sphereCollider.radius * clothScript.interactiveObjs[l].transform.lossyScale.y;
                        //    }
                        //    else
                        //    {
                        //        clothScript.interactiveObjs[l].transform = EditorGUILayout.ObjectField("Transform", clothScript.interactiveObjs[l].transform, typeof(Transform), true) as Transform;
                        //        clothScript.interactiveObjs[l].size = EditorGUILayout.Vector3Field("Size", clothScript.interactiveObjs[l].size);
                        //    }
                        //    //clothScript.interactiveObjs[l].localSize = clothScript.interactiveObjs[l].size / clothScript.transform.lossyScale.y;  to do when make box collision
                        //}

                        if (clothScript.interactiveObjs[l].transform != null && GUI.changed)
                        {
                            clothScript.interactiveObjs[l].prevPos = clothScript.interactiveObjs[l].transform.position;
                        }

                        clothScript.interactiveObjs[l].affectCloth = EditorGUILayout.Toggle("Affect Cloth Vertices", clothScript.interactiveObjs[l].affectCloth);

                        if (clothScript.interactiveObjs[l].affectCloth)
                        {
                            clothScript.interactiveObjs[l].vertForceMult = EditorGUILayout.FloatField("Vertex force multiplyer", clothScript.interactiveObjs[l].vertForceMult);
                        }

                        clothScript.interactiveObjs[l].colPenetration = EditorGUILayout.Toggle("Can penetrate cloth", clothScript.interactiveObjs[l].colPenetration);

                        clothScript.interactiveObjs[l].bounce = EditorGUILayout.Toggle("Can receive force", clothScript.interactiveObjs[l].bounce);
                        if (clothScript.interactiveObjs[l].bounce)
                        {
                            clothScript.interactiveObjs[l].bounceForceMult = EditorGUILayout.FloatField("Bounce force multiplyer", clothScript.interactiveObjs[l].bounceForceMult);

                            if (clothScript.interactiveObjs[l].transform == null || clothScript.interactiveObjs[l].transform.GetComponent<Rigidbody>() == null)
                            {
                                EditorGUILayout.HelpBox("To be able to bounce from the cloth the interactive object needs a rigidbody", MessageType.Warning);
                            }
                            if (clothScript.interactiveObjs[l].transform != null)
                            {
                                clothScript.interactiveObjs[l].rigidBody = clothScript.interactiveObjs[l].transform.GetComponent<Rigidbody>();
                            }
                        }
                    }
                 
                }
                if(GUI.changed)
                    EditorUtility.SetDirty(target);
                GUILayout.EndVertical();
                GUILayout.EndHorizontal();
            }


            if (selectedButton == null)
            {
                selectedButton = new GUIStyle(GUI.skin.button);
                selectedButton.normal.background = selectedButton.active.background;
                selectedButton.normal.textColor = selectedButton.active.textColor;
            }

            EditorGUILayout.BeginVertical();
            {
                //EditorGUILayout.BeginHorizontal();
                //{
                //    clothScript.gridSize = EditorGUILayout.Vector3Field("Grid Size", clothScript.gridSize);
                //    GUILayout.Button("Resize");
                //}
                //EditorGUILayout.EndHorizontal();

                GUILayout.Label("Tools");
                EditorGUILayout.BeginHorizontal();
                {
                    if (GUILayout.Button("Edit Max Distance", state == EditState.EditMaxDistance ? selectedButton : GUI.skin.button, GUILayout.Width(110), GUILayout.Height(30)))
                    {
                        state = EditState.EditMaxDistance;
                    }
                    if (GUILayout.Button("Vertex Move", state == EditState.Translating ? selectedButton : GUI.skin.button, GUILayout.Width(100), GUILayout.Height(30)))
                    {
                        state = EditState.Translating;
                    }
                }

                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndVertical();

        }

        void OnSceneGUI()
        {
            if (state == EditState.Translating)
            {
                ShowVertexPoints(false, false, (int i) => true, (int i) => Color.white, (bool k, int i) =>
                {
                    if (k)
                    {
                        translatingVertexPos = i;
                    }
                });
                if (translatingVertexPos != -1)
                {
                    Vector3 newPos = modelTransform.InverseTransformPoint(Handles.PositionHandle(modelTransform.TransformPoint(clothScript.vertexDatas[translatingVertexPos].position), Quaternion.identity));
                    clothScript.vertexDatas[translatingVertexPos].position = newPos;
                }
                Tools.hidden = true;
            }
            else if (state == EditState.EditMaxDistance)
            {
                ShowVertexPoints(false, false, (int i) => true, (int i) => clothScript.vertexDatas[i].maxDistance == -1 ? Color.black : myGradient.Evaluate(maxDistance > 0 ? clothScript.vertexDatas[i].maxDistance / maxDistance : maxDistance), (bool k, int i) =>
                {
                    if (k)
                    {
                        clothScript.vertexDatas[i].maxDistance = unConstrained ? -1 : currentEditorResistance;
                        EditorUtility.SetDirty(target);
                    }
                });
                Handles.BeginGUI();
                clothScript.windowRect.width = 210;

                clothScript.windowRect = GUILayout.Window(0, clothScript.windowRect, DrawVertexPaintToolBox, "Vertex Resistance");
                Handles.EndGUI();
                Tools.hidden = true;
            }
            else if (state == EditState.EditSprings)
            {
                for (int i = 0; i < clothScript.vertexDatas.Length; i++)
                {
                    if (!clothScript.vertexDatas[i].clone)
                    {
                        Handles.color = selectedVertex == i ? Color.green : Color.black;
                        bool pressed = Handles.Button(clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position), Quaternion.identity, 0.05f, 0.05f, Handles.CubeCap);
                        if (pressed)
                        {
                            if (editSpringsState == EditSpringsState.Select)
                            {
                                selectedVertex = i;
                            }
                            else if (editSpringsState == EditSpringsState.Edit)
                            {
                                if (i == selectedVertex)
                                {
                                    startedString = !startedString;
                                }
                                else
                                {
                                    if (startedString)
                                    {
                                        bool alreadyExistant = false;
                                        for (int l = 0; l < clothScript.vertexDatas[selectedVertex].springs.Length; l++)
                                        {
                                            if (clothScript.vertexDatas[selectedVertex].springs[l] == i)
                                            {
                                                alreadyExistant = true;
                                                break;
                                            }
                                        }
                                        if (!alreadyExistant)
                                        {
                                            int index = clothScript.vertexDatas[selectedVertex].springs.Length + 1;
                                            Array.Resize(ref clothScript.vertexDatas[selectedVertex].springs, index);
                                            Array.Resize(ref clothScript.vertexDatas[selectedVertex].springsDistancesP2, index);
                                            clothScript.vertexDatas[selectedVertex].springs[index - 1] = i;
                                            clothScript.vertexDatas[selectedVertex].springsDistancesP2[index - 1] = (clothScript.vertexDatas[selectedVertex].position - clothScript.vertexDatas[i].position).sqrMagnitude;
                                            startedString = false;
                                        }
                                    }else
                                    {
                                        for (int l = 0; l < clothScript.vertexDatas[selectedVertex].springs.Length; l++)
                                        {
                                            if (clothScript.vertexDatas[selectedVertex].springs[l] == i)
                                            {
                                                ArrayUtility.RemoveAt(ref clothScript.vertexDatas[selectedVertex].springs, l);
                                                ArrayUtility.RemoveAt(ref clothScript.vertexDatas[selectedVertex].springsDistancesP2, l);
                                                startedString = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            else if (editSpringsState == EditSpringsState.ShowAllConnections)
                            {
                                selectedVertex = i;
                                startedString = false;
                                editSpringsState = EditSpringsState.Select;
                            }
                        }
                    }

                    if (i == selectedVertex || editSpringsState == EditSpringsState.ShowAllConnections)
                    {
                        for (int j = 0; j < clothScript.vertexDatas[i].springs.Length; j++)
                        {
                            int second = clothScript.vertexDatas[i].springs[j];
                            Vector3 firstPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position);
                            Vector3 secondPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[second].position);

                            Handles.color = Color.green;
                            Handles.DrawLine(firstPos, secondPos);
                        }
                        for (int j = 0; j < clothScript.vertexDatas[i].bendingSprings.Length; j++)
                        {
                            int second = clothScript.vertexDatas[i].bendingSprings[j];
                            Vector3 firstPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position) + new Vector3(0.02f, 0, 0.02f);
                            Vector3 secondPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[second].position) + new Vector3(0.02f, 0, 0.02f);

                            Handles.color = Color.red;
                            Handles.DrawLine(firstPos, secondPos);
                        }
                    }
                }

                if (startedString)
                {
                    Handles.color = Color.green;
                    Vector3 firstPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[selectedVertex].position);
                    Handles.DrawLine(firstPos, HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(50));
                    HandleUtility.Repaint();
                }

                if (Event.current.type == EventType.MouseDown && Event.current.button == 0 || Event.current.type == EventType.KeyDown && Event.current.keyCode == KeyCode.Escape)
                {
                    startedString = false;
                }
               
                Handles.BeginGUI();
                clothScript.windowRect.width = 210;
                clothScript.windowRect.height = 0;
                clothScript.windowRect = GUILayout.Window(0, clothScript.windowRect, DrawEditSpringToolBox, "Vertex Resistance");
                Handles.EndGUI();
            }
            else if (state == EditState.AddClothColliders)
            {
                for (int i = 0; i < clothScript.clothColliders.Length; i++)
                {
                    Handles.color = i == selectedWatcher ? Color.yellow : Color.black;
                    bool pressed = Handles.Button(clothScript.clothColliders[i].bounds.center, Quaternion.identity, 0.1f, 0.1f, Handles.CubeCap);

                    if (pressed) selectedWatcher = i;

                    Handles.color = Color.green;
                    DrawWireCube(clothScript.clothColliders[i].bounds.center, clothScript.clothColliders[i].bounds.extents * 2);
                }

                if (clothScript.clothColliders != null && clothScript.clothColliders.Length > 0 && selectedWatcher != -1) { 

                    ClothCollider collider = clothScript.clothColliders[selectedWatcher];

                    for (int k = 0; k < meshTriangles.Length; k++)
                    {
                        Vector3 vertex1Pos = clothScript.transform.TransformPoint(clothScript.vertexDatas[meshTriangles[k].index1].position);
                        Vector3 vertex2Pos = clothScript.transform.TransformPoint(clothScript.vertexDatas[meshTriangles[k].index2].position);
                        Vector3 vertex3Pos = clothScript.transform.TransformPoint(clothScript.vertexDatas[meshTriangles[k].index3].position);

                        Vector3 center = (vertex1Pos + vertex2Pos + vertex3Pos) / 3;

                       
                        if (Array.IndexOf(collider.trisIndexes, k) != -1)
                        {
                            Handles.color = Color.green;
                            bool pressed = Handles.Button(center, Quaternion.identity, 0.1f, 0.1f, Handles.CubeCap);
                            if (pressed)
                            {
                                ArrayUtility.Remove(ref clothScript.clothColliders[selectedWatcher].trisIndexes, k);
                                ArrayUtility.Remove(ref clothScript.clothColliders[selectedWatcher].connectedTris, meshTriangles[k]);

                                clothScript.clothColliders[selectedWatcher].RecalculateBounds(clothScript.transform, clothScript.vertexDatas);
                            }

                            Handles.color = new Color(0, 1, 0, 0.2f);
                            Handles.DrawAAConvexPolygon(new Vector3[] { vertex1Pos, vertex2Pos, vertex3Pos });
                        }
                        else
                        {
                            Handles.color = Color.white;
                            bool pressed = Handles.Button(center, Quaternion.identity, 0.1f, 0.1f, Handles.CubeCap);
                            if (pressed)
                            {
                                Array.Resize(ref clothScript.clothColliders[selectedWatcher].trisIndexes, collider.trisIndexes.Length + 1);
                                Array.Resize(ref clothScript.clothColliders[selectedWatcher].connectedTris, collider.connectedTris.Length + 1);

                                clothScript.clothColliders[selectedWatcher].trisIndexes[collider.trisIndexes.Length] = k;
                                clothScript.clothColliders[selectedWatcher].connectedTris[collider.connectedTris.Length] = meshTriangles[k];

                                clothScript.clothColliders[selectedWatcher].RecalculateBounds(clothScript.transform, clothScript.vertexDatas);
                            }
                        }
                    } 
                }

                Handles.BeginGUI();
                int height = clothScript.clothColliders.Length * 20 + 45;
                clothScript.windowRect.width = 250;
                clothScript.windowRect.height = height;
                clothScript.windowRect = GUILayout.Window(0, clothScript.windowRect, DrawWatchersToolBox, "Cloth Colliders");
                Handles.EndGUI();
            }
            else if (state == EditState.Cutscene)
            {
                float remaining = lastCutsceneTime + 2 - Time.realtimeSinceStartup;

                if (remaining > 0)
                {
                    float alpha = remaining < 0.5f ? remaining * 2 : 1;

                    for (int i = 0; i < clothScript.vertexDatas.Length; i++)
                    {
                        for (int j = 0; j < clothScript.vertexDatas[i].springs.Length; j++)
                        {
                            int second = clothScript.vertexDatas[i].springs[j];
                            Vector3 firstPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position);
                            Vector3 secondPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[second].position);

                            Handles.color = new Color32(0, 255, 0, (byte)(alpha * 255));
                            Handles.DrawLine(firstPos, secondPos);
                        }
                        for (int j = 0; j < clothScript.vertexDatas[i].bendingSprings.Length; j++)
                        {
                            int second = clothScript.vertexDatas[i].bendingSprings[j];
                            Vector3 firstPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position) + new Vector3(0.03f, 0, 0.03f);
                            Vector3 secondPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[second].position) + new Vector3(0.03f, 0, 0.03f);

                            Handles.color = new Color32(255, 0, 0, (byte)(alpha * 255));
                            Handles.DrawLine(firstPos, secondPos);
                        }
                    }

                    HandleUtility.Repaint();
                }
                else
                {
                    state = EditState.None;
                }
            }
            if (clothScript.interactiveObjs != null)
            {
                for (int l = 0; l < clothScript.interactiveObjs.Length; l++)
                {
                    if (clothScript.interactiveObjs[l].paramsType == ObjParamsType.Custom)
                    {
                        if (clothScript.interactiveObjs[l].interactiveObjType == ObjType.Sphere && clothScript.interactiveObjs[l].transform != null)
                        {
                            Handles.color = new Color(0, 1, 0, 0.2f);
                            Handles.DrawSphere(0, clothScript.interactiveObjs[l].transform.position, Quaternion.identity, clothScript.interactiveObjs[l].radius * 2);
                            Handles.color = new Color(0, 1, 0, 0.8f);
                            //Handles.DrawWireDisc(clothScript.interactiveObjs[l].transform.position, clothScript.transform.up, clothScript.interactiveObjs[l].radius);
                            //Handles.DrawWireDisc(clothScript.interactiveObjs[l].transform.position, clothScript.transform.forward, clothScript.interactiveObjs[l].radius);
                            //Handles.DrawWireDisc(clothScript.interactiveObjs[l].transform.position, clothScript.transform.right, clothScript.interactiveObjs[l].radius);
                        }
                    }
                }
            }
           
            HandleEvents();
        }
        
        // lower unity version hack
        public static void DrawWireCube(Vector3 position, Vector3 size)
        {
            var half = size / 2;
            // draw front
            Handles.DrawLine(position + new Vector3(-half.x, -half.y, half.z), position + new Vector3(half.x, -half.y, half.z));
            Handles.DrawLine(position + new Vector3(-half.x, -half.y, half.z), position + new Vector3(-half.x, half.y, half.z));
            Handles.DrawLine(position + new Vector3(half.x, half.y, half.z), position + new Vector3(half.x, -half.y, half.z));
            Handles.DrawLine(position + new Vector3(half.x, half.y, half.z), position + new Vector3(-half.x, half.y, half.z));
            // draw back
            Handles.DrawLine(position + new Vector3(-half.x, -half.y, -half.z), position + new Vector3(half.x, -half.y, -half.z));
            Handles.DrawLine(position + new Vector3(-half.x, -half.y, -half.z), position + new Vector3(-half.x, half.y, -half.z));
            Handles.DrawLine(position + new Vector3(half.x, half.y, -half.z), position + new Vector3(half.x, -half.y, -half.z));
            Handles.DrawLine(position + new Vector3(half.x, half.y, -half.z), position + new Vector3(-half.x, half.y, -half.z));
            // draw corners
            Handles.DrawLine(position + new Vector3(-half.x, -half.y, -half.z), position + new Vector3(-half.x, -half.y, half.z));
            Handles.DrawLine(position + new Vector3(half.x, -half.y, -half.z), position + new Vector3(half.x, -half.y, half.z));
            Handles.DrawLine(position + new Vector3(-half.x, half.y, -half.z), position + new Vector3(-half.x, half.y, half.z));
            Handles.DrawLine(position + new Vector3(half.x, half.y, -half.z), position + new Vector3(half.x, half.y, half.z));
        }

        void DrawWatchersToolBox(int windowID)
        {
            GUILayout.BeginVertical();
            {
                for (int i = 0; i < clothScript.clothColliders.Length; i++)
                {
                    GUILayout.BeginHorizontal();
                    {
                        clothScript.clothColliders[i].name = GUILayout.TextField(clothScript.clothColliders[i].name, GUILayout.Width(120));

                        if (GUILayout.Button("Select", GUILayout.Height(16)))
                            selectedWatcher = i;

                        if (GUILayout.Button("Clear", GUILayout.Height(16)))
                        {
                            clothScript.clothColliders[i].bounds = new Bounds();
                            clothScript.clothColliders[i].connectedTris = new Triangle[0];
                            clothScript.clothColliders[i].trisIndexes = new int[0];
                        }

                        if (GUILayout.Button("Dell", GUILayout.Height(16)))
                        {
                            if (selectedWatcher == clothScript.clothColliders.Length - 1)
                                selectedWatcher = -1;

                            ArrayUtility.RemoveAt(ref clothScript.clothColliders, i);
                        }
                    }
                    GUILayout.EndHorizontal();
                }
               
                if (GUILayout.Button("Add", GUILayout.Height(18)))
                {
                    ClothCollider collider = new ClothCollider();
                    Mesh sharedMesh = clothScript.GetComponent<MeshFilter>().sharedMesh;
                    collider.bounds.center = clothScript.transform.TransformPoint(sharedMesh.bounds.center);
                    collider.bounds.extents = clothScript.transform.TransformVector(sharedMesh.bounds.extents);
                    collider.connectedTris = meshTriangles;
                    collider.trisIndexes = new int[meshTriangles.Length];
                    collider.name = "Collider " + clothScript.clothColliders.Length;

                    for (int i = 0; i < meshTriangles.Length; i++)
                        collider.trisIndexes[i] = i;

                    int index = clothScript.clothColliders.Length;
                    Array.Resize(ref clothScript.clothColliders, index + 1);
                    clothScript.clothColliders[index] = collider;
                } 
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
        void DrawVertexPaintToolBox(int windowID)
        {
            GUI.enabled = !unConstrained;
            GUIStyle style = new GUIStyle();
            Texture2D tex = new Texture2D(40, 30);

            GUILayout.BeginHorizontal(GUIStyle.none, new GUILayoutOption[] { GUILayout.Width(0), GUILayout.Height(0) });

            GUI.color = Color.red;
            style.alignment = TextAnchor.LowerLeft;
            style.margin.left = 5;
            GUILayout.Label(tex, style, GUILayout.Width(20), GUILayout.Height(15));
            GUILayout.Label("0", style);

            GUI.color = Color.green;
            style.alignment = TextAnchor.LowerRight;
            style.margin.right = 5;
            GUILayout.Label(maxDistance.ToString("F1"), style);
            GUILayout.Label(tex, style, GUILayout.Width(20), GUILayout.Height(15));

            GUILayout.EndHorizontal();

            maxDistance = 0;
            for (int a = 0; a < clothScript.vertexDatas.Length; a++)
            {
                if (clothScript.vertexDatas[a].maxDistance > maxDistance) maxDistance = clothScript.vertexDatas[a].maxDistance;
            }

            GUILayout.BeginHorizontal();
            GUI.color = Color.white;
            float tempDistance;
            currentEditorResistance = GUILayout.HorizontalSlider(currentEditorResistance, 0, maxDistance, GUILayout.Width(190), GUILayout.Height(15));
            tempDistance = EditorGUILayout.FloatField(currentEditorResistance);

            if (tempDistance > maxDistance)
            {
                currentEditorResistance = maxDistance = tempDistance;
            }
            GUILayout.EndHorizontal();

            GUI.enabled = true;
            unConstrained = GUILayout.Toggle(unConstrained, "Unconstrained");

            if (GUILayout.Button("Set All"))
            {
                for (int i = 0; i < clothScript.vertexDatas.Length; i++)
                {
                    if (!clothScript.vertexDatas[i].clone)
                        clothScript.vertexDatas[i].maxDistance = unConstrained ? -1 : currentEditorResistance;
                }
            }

            GUI.DragWindow();
        }

        void DrawEditSpringToolBox(int windowID)
        {
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Select", editSpringsState == EditSpringsState.Select ? selectedButton : GUI.skin.button))
            {
                editSpringsState = EditSpringsState.Select;
            }
            GUI.enabled = selectedVertex != -1;
            if (GUILayout.Button("Edit", editSpringsState == EditSpringsState.Edit ? selectedButton : GUI.skin.button))
            {
                editSpringsState = editSpringsState == EditSpringsState.Edit ? EditSpringsState.Select : EditSpringsState.Edit;
            }
            GUI.enabled = true;
            if (GUILayout.Button("Show All", editSpringsState == EditSpringsState.ShowAllConnections ? selectedButton : GUI.skin.button))
            {
                editSpringsState = editSpringsState == EditSpringsState.ShowAllConnections ? EditSpringsState.Select : EditSpringsState.ShowAllConnections;
                selectedVertex = -1;
            }
            GUILayout.EndHorizontal();
            GUI.DragWindow();
        }

        static string GetColumnName(int index)
        {
            const int alphabetsCount = 26;

            if (index > alphabetsCount)
            {
                int mod = index % alphabetsCount;
                int columnIndex = index / alphabetsCount;

                // if mod is 0 (clearly divisible) we reached end of one combination. Something like AZ
                if (mod == 0)
                {
                    // reducing column index as index / alphabetsCount will give the next value and we will miss one column.
                    columnIndex -= 1;
                    // passing 0 to the function will return character '@' which is invalid
                    // mod should be the alphabets count. So it takes the last char in the alphabet.
                    mod = alphabetsCount;
                }
                return GetColumnName(columnIndex) + GetColumnName(mod);
            }
            else
            {
                int code = (index - 1) + (int)'A';
                return char.ConvertFromUtf32(code);
            }
        }


        void ShowVertexPoints(bool showLabel, bool showConnections, ShowVertex showVertex, HandleColor color, Callback callback)
        {
            for (int i = 0; i < clothScript.vertexDatas.Length; i++)
            {
                if (showVertex(i))
                {
                    if (!clothScript.vertexDatas[i].clone)
                    {
                        Handles.color = color(i);
                        if (showLabel) Handles.Label(clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position), GetColumnName(i));

                        bool pressed = Handles.Button(clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position), Quaternion.identity, 0.05f, 0.05f, Handles.CubeCap);
                        callback(pressed, i);
                    }

                    if (showConnections)
                    {
                        for (int j = 0; j < clothScript.vertexDatas[i].springs.Length; j++)
                        {
                            int second = clothScript.vertexDatas[i].springs[j];
                            Vector3 firstPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[i].position);
                            Vector3 secondPos = clothScript.transform.TransformPoint(clothScript.vertexDatas[second].position);

                            //Vector3 halfDistance = (firstPos - secondPos) / 2;

                            //Handles.color = myGradient.Evaluate(clothScript.vertexDatas[i].maxDistance / maxDistance);
                            //Handles.DrawLine(firstPos, firstPos - halfDistance);

                            //Handles.color = myGradient.Evaluate(clothScript.vertexDatas[j].maxDistance / maxDistance);
                            //Handles.DrawLine(secondPos, secondPos + halfDistance);
                            Handles.color = color(i);
                            Handles.DrawLine(firstPos, secondPos);
                        }
                    }
                }
            }
        }

        void HandleEvents()
        {
            Event e = Event.current;
            if (e.type == EventType.KeyDown && canHandleEvents)
            {
                if (e.keyCode == (KeyCode.U))
                {
                    state = EditState.EditMaxDistance;
                }
                else if (e.keyCode == (KeyCode.O))
                {
                    state = EditState.Translating;
                }
                else if (e.keyCode == (KeyCode.I))
                {
                    state = EditState.AddClothColliders;
                }
                else if (e.keyCode == (KeyCode.P))
                {
                    state = EditState.EditSprings;
                }
                else if (state == EditState.AddClothColliders)
                {
                    if (e.keyCode == KeyCode.E)
                    {
                        toolsState = ToolsState.Rotate;
                    }
                    else if (e.keyCode == KeyCode.W)
                    {
                        toolsState = ToolsState.Move;
                    }
                    else if (e.keyCode == KeyCode.R)
                    {
                        toolsState = ToolsState.Scale;
                    }
                }

            }
            if (e.type == EventType.MouseDown)
            {
                canHandleEvents = false;
            }
            else if (e.type == EventType.MouseUp)
            {
                canHandleEvents = true;
            }
        }

        Vector4 QuaternionToVector3(Quaternion rot)
        {
            return rot.eulerAngles;
        }
        Quaternion Vector3ToQuaternion(Vector3 four)
        {
            return Quaternion.Euler(four);
        }
    }
    
}

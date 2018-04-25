using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;

namespace Libraries
{
    public enum ClothType
    {
        Struct,
        Struct_Shear,
        Struct_Shear_Bend
    }

    [Serializable]
    public class VertexData
    {
        public Vector3 position;
        public Vector3 prevPosition;
        public Vector3 initialPosition;
        public Vector3 centetrDisplace;
        public Vector3 velocity;
        public float maxDistance;
        public float centetrDisplaceMag;
        public bool clone;
        public int[] springs;
        public int[] bendingSprings;
        public int[] clones;
        public float[] springsDistancesP2;
        public float[] bendingSpringsDistancesP2;

        public float tension;
        public VertexData() { }

        public VertexData(Vector3 position, bool clone, int maxDistance)
        {
            this.position = position;
            this.prevPosition = position;
            this.initialPosition = position;
            this.maxDistance = maxDistance;
            this.centetrDisplace = Vector3.zero;
            this.velocity = Vector3.zero;
            this.clone = clone;
            centetrDisplaceMag = 0;
            springs = new int[0];
            bendingSprings = new int[0];
            clones = new int[0];
            springsDistancesP2 = new float[0];
            bendingSpringsDistancesP2 = new float[0];
        }
        public void SetCenterDisplace(Vector3 center)
        {
            this.centetrDisplace = position - center;
            this.centetrDisplaceMag = centetrDisplace.sqrMagnitude;
        }
        public void AddConnection(int vertexPos, VertexData[] vertexDatas, bool bending)
        {
            Vector3 heading = position - vertexDatas[vertexPos].position;

            if (bending)
            {
                int index = bendingSprings.Length + 1;

                Array.Resize(ref bendingSprings, index);
                Array.Resize(ref bendingSpringsDistancesP2, index);

                bendingSprings[index - 1] = vertexPos;
                bendingSpringsDistancesP2[index - 1] = (float)Math.Pow(heading.magnitude, 2);
            }
            else
            {
                int index = springs.Length + 1;

                Array.Resize(ref springs, index);
                Array.Resize(ref springsDistancesP2, index);

                springs[index - 1] = vertexPos;
                springsDistancesP2[index - 1] = (float)Math.Pow(heading.magnitude, 2);
            }

        }

        public void Update(Transform transform, ClothData clothData, VertexData[] vertexDatas, float dTimeP2, ref float moveSqrMagnitude)
        {
            if (clone)
            {
                moveSqrMagnitude = 0;
                return;
            }
            float var = dTimeP2 * 500;

            if (springs != null && springs.Length > 0)
            {
                for (int i = 0; i < springs.Length; i++)
                {
                    UpdateConnection(springs[i], vertexDatas, clothData, springsDistancesP2[i], var, false);
                }
            }

            if (bendingSprings != null && bendingSprings.Length > 0)
            {
                for (int i = 0; i < bendingSprings.Length; i++)
                {
                    UpdateConnection(bendingSprings[i], vertexDatas, clothData, bendingSpringsDistancesP2[i], var, true);
                }
            }

            if (!clone)
            {
                velocity = position - prevPosition;

                prevPosition = position;

                velocity *= (1 - clothData.damp);

                velocity += clothData.localGravity * dTimeP2;

                position += velocity;

               
                if (maxDistance == 0)
                {
                    position = initialPosition;
                    prevPosition = initialPosition;
                }
                else if (maxDistance != -1)
                {
                    Vector3 deltaPos = initialPosition - position;
                    float deltaPosMag = deltaPos.x * deltaPos.x + deltaPos.y * deltaPos.y + deltaPos.z * deltaPos.z;
                    if (deltaPosMag > maxDistance)
                        position += deltaPos / deltaPosMag * (deltaPosMag - maxDistance);
                }


                if (clones != null)
                {
                    for (int i = 0; i < clones.Length; i++)
                    {
                        vertexDatas[clones[i]].position = position;
                        vertexDatas[clones[i]].prevPosition = prevPosition;
                    }
                }

                moveSqrMagnitude = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
            }
        }

        private void UpdateConnection(int index, VertexData[] vertexDatas, ClothData clothData, float origirnalSqrMag, float dTimeP2, bool bending)
        {
            Vector3 heading = vertexDatas[index].position - position;
            float headingSqrMag = heading.x * heading.x + heading.y * heading.y + heading.z * heading.z;
            heading *= (origirnalSqrMag / (headingSqrMag + origirnalSqrMag) - 0.5f) * (bending ? clothData.bending : clothData.stiffness) * dTimeP2;

            //float currentDistance = heading.magnitude;
            //float diffDist = currentDistance - Mathf.Sqrt(originalDistanceP2);
            //Vector3 segment = heading * diffDist / currentDistance * (bending ? clothData.bending : clothData.stiffness) * dTimeP2;

            if(maxDistance == 0)
            {
                if(vertexDatas[index].maxDistance != 0)
                {
                    vertexDatas[index].tension += bending ? 0 : headingSqrMag * 2;
                    vertexDatas[index].Move(vertexDatas, heading * 2);
                }
            }
            else if(vertexDatas[index].maxDistance == 0)
            {
                vertexDatas[index].tension += bending ? 0 : headingSqrMag * 2;
                Move(vertexDatas, -heading * 2);
            }
            else
            {
                vertexDatas[index].tension += bending ? 0 : headingSqrMag;
                tension += bending ? 0 : headingSqrMag;

                //float diffTension = Mathf.Clamp01(((tension / (springs.Length + bendingSprings.Length)) / (vertexDatas[index].tension / (vertexDatas[index].springs.Length + vertexDatas[index].bendingSprings.Length))));
                //if (tension == 0 && vertexDatas[index].tension == 0)
                //    diffTension = 0.5f;
                //else if (tension == 0)
                //    diffTension = 0;
                //else if (vertexDatas[index].tension == 0)
                //    diffTension = 1;
               
                //{
                //    //Move(vertexDatas, -heading * (2 * (1 - diffTension)));
                //    //vertexDatas[index].Move(vertexDatas, heading * (2 * diffTension));
                //}
                Move(vertexDatas, -heading );
                vertexDatas[index].Move(vertexDatas, heading);

            }
        }

        public void Move(VertexData[] vertexDatas, Vector3 deltaPos)
        {
            if(!clone)
                position += deltaPos;
            if (clones != null && clones.Length > 0)
            {
                for (int i = 0; i < clones.Length; i++)
                {
                    vertexDatas[clones[i]].position = position;
                    vertexDatas[clones[i]].prevPosition = position;
                }
            }
        }

    }

    public struct ClothActive
    {
        public float maxVertDeltaPos;
        public int index;
    }

    public enum ObjType
    {
        Sphere,
        //Box
    }

    public enum ObjParamsType
    {
        UnityCollider,
        Custom
    }
    [Serializable]
    public class OnCollision : UnityEngine.Events.UnityEvent<CustomCloth, Vector3, Vector3> { };

    [Serializable]
    public class InteractiveObj
    {
        public ObjType interactiveObjType;
        public ObjParamsType paramsType;

        public SphereCollider sphereCollider;
        public BoxCollider boxCollider;

        public float radius;
        public float localRadius;

        public Vector3 size;
        public Vector3 localSize;

        public Vector3 prevPos;
        public Vector3 localPos;
        public Vector3 localPrevPos;

        public Transform transform;
        public Rigidbody rigidBody;

        public bool affectCloth;
        public float vertForceMult;
        public bool colPenetration;
        public bool bounce;
        public float bounceForceMult;
        public float friction;

        [NonSerialized]
        public int framesNotColliding = 25;
        public bool justCollided;
        public Triangle lastColTriangle;
        [NonSerialized]
        public int lastColSide;
        public Vector3 lastColRelativePos;

        public OnCollision onCollision;
    }


    [Serializable]
    public class ClothData
    {
        public ClothType clothType;
        public Vector3 gravity;
        [HideInInspector]
        public Vector3 localGravity;
        public Vector3 wind;
        [Range(0, 1)]
        public float damp;
        [Range(0, 1)]
        public float deltaPosInfluence;
        [Range(0, 1)]
        public float stiffness;
        [Range(0, 1)]
        public float bending;
        [Range(0, 1)]
        public float volumePressure;
        public float sleepThreshold;
        public float impactDampening;

        public ClothData()
        {
            gravity = new Vector3(0, -9.81f, 0);
            localGravity = Vector3.zero;
            wind = Vector3.zero;
            damp = 0.1f;
            deltaPosInfluence = 0.005f;
            stiffness = 0.1f;
            bending = 0.1f;
            sleepThreshold = 0.00005f;
        }
    }


    [Serializable]
    public class Triangle
    {
        public int index1;
        public int index2;
        public int index3;

        public Triangle(int index1, int index2, int index3)
        {
            this.index1 = index1;
            this.index2 = index2;
            this.index3 = index3;
        }
    }

    [Serializable]
    public struct ClothCollider
    {
        public Bounds bounds;
        public Triangle[] connectedTris;
        public int[] trisIndexes;
        public String name;

        int middleIndex;
        Vector3 min, max;
        int left, top, front, right, bottom, back;
        float averageDenominator;

        public void Initialise(Transform transform, VertexData[] vertexDatas)
        {
            Vector3 sum = Vector3.zero;
            for (int k = 0; k < connectedTris.Length; k++)
                sum = sum + vertexDatas[connectedTris[k].index1].position + vertexDatas[connectedTris[k].index2].position + vertexDatas[connectedTris[k].index3].position;

            Vector3 center = transform.TransformPoint(sum / (connectedTris.Length * 3));

            middleIndex = connectedTris[0].index1;
            float lastCenterDistance = Vector3.Distance(center, vertexDatas[connectedTris[0].index1].position);

            for (int k = 0; k < connectedTris.Length; k++)
            {
                float currentDistance = Vector3.Distance(center, vertexDatas[connectedTris[k].index1].position);
                if (currentDistance < lastCenterDistance)
                {
                    lastCenterDistance = currentDistance;
                    middleIndex = connectedTris[k].index1;
                }

                currentDistance = Vector3.Distance(center, vertexDatas[connectedTris[k].index2].position);
                if (currentDistance < lastCenterDistance)
                {
                    lastCenterDistance = currentDistance;
                    middleIndex = connectedTris[k].index2;
                }

                currentDistance = Vector3.Distance(center, vertexDatas[connectedTris[k].index3].position);
                if (currentDistance < lastCenterDistance)
                {
                    lastCenterDistance = currentDistance;
                    middleIndex = connectedTris[k].index3;
                }
            }

        }

        public void RecalculateBounds(Transform transform, VertexData[] vertexDatas)
        {
            min = transform.TransformPoint(new Vector3(vertexDatas[left].position.x, vertexDatas[bottom].position.y, vertexDatas[back].position.z));
            max = transform.TransformPoint(new Vector3(vertexDatas[right].position.x, vertexDatas[top].position.y, vertexDatas[front].position.z));

            for (int i = 0; i < connectedTris.Length; i++)
            {
                CalculateMargins(transform.TransformPoint(vertexDatas[connectedTris[i].index1].position), connectedTris[i].index1);
                CalculateMargins(transform.TransformPoint(vertexDatas[connectedTris[i].index2].position), connectedTris[i].index2);
                CalculateMargins(transform.TransformPoint(vertexDatas[connectedTris[i].index3].position), connectedTris[i].index3);
            }

            bounds.center = transform.TransformPoint(vertexDatas[middleIndex].position);

            bounds.min = min;
            bounds.max = max;
        }

        public void CalculateMargins(Vector3 vector, int index)
        {
            if (vector.x < min.x)
            {
                left = index;
                min.x = vector.x;
            }
            if (vector.x > max.x)
            {
                right = index;
                max.x = vector.x;
            }

            if (vector.y < min.y)
            {
                bottom = index;
                min.y = vector.y;
            }
            if (vector.y > max.y)
            {
                top = index;
                max.y = vector.y;
            }

            if (vector.z < min.z)
            {
                back = index;
                min.z = vector.z;
            }
            if (vector.z > max.z)
            {
                front = index;
                max.z = vector.z;
            }
        }
    }

    public class CustomCloth : MonoBehaviour
    {

        [HideInInspector]
        public VertexData[] vertexDatas;
        public ClothData clothData = new ClothData();
        [HideInInspector]
        public ClothCollider[] clothColliders;
        public InteractiveObj[] interactiveObjs;
        [HideInInspector]
        public bool showBounds;
        [HideInInspector]
        public Rect windowRect;

        protected ClothActive clothActive;
        protected Mesh mesh;
        [HideInInspector]
        public Vector3[] vertices;
        [HideInInspector]
        public Vector2[] uvs;
        [HideInInspector]
        public int[] triangles;
        protected Renderer clothRenderer;
        protected Bounds bounds;
        protected Ray ray;
        protected float lastCollisionDistance;
        protected float startTime;

        protected Vector3 prevPos;
        protected Quaternion prevRot;

        protected float vertDenom;

        protected VertexData center;

      
        void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(transform.TransformPoint(centers), 0.05f);

            if (showBounds)
            {

                Gizmos.color = Color.red;
                if (clothRenderer == null)
                    clothRenderer = GetComponent<Renderer>();

                Gizmos.DrawWireCube(clothRenderer.bounds.center, clothRenderer.bounds.size);
            }
        }

        void Start()
        {
            mesh = GetComponent<MeshFilter>().mesh;

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;

            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            mesh.Optimize();

            mesh.MarkDynamic();
            mesh.UploadMeshData(false);
            bounds = GetComponent<Renderer>().bounds;

            clothRenderer = GetComponent<Renderer>();

            clothData.localGravity = transform.InverseTransformDirection(clothData.gravity);

            for (int i = 0; i < clothColliders.Length; i++)
            {
                clothColliders[i].Initialise(transform, vertexDatas);
            }

            prevPos = transform.position;
            prevRot = transform.rotation;

            clothActive.index = -1;

            vertDenom = 1.0f / vertexDatas.Length;



            Vector3 sum = Vector3.zero;
            for (int i = 0; i < vertexDatas.Length; i++)
            {
                sum += vertexDatas[i].position;
            }

            center = new VertexData(sum / vertexDatas.Length, true, -1);

            startTime = Time.time;
        }
        Vector3 centers = Vector3.zero; 

        void FixedUpdate()
        {
            UpdateCollisions();

            Profiler.BeginSample("Verlet");
            float dTimeP2 = Time.deltaTime * Time.deltaTime;
            //verlet integration
            if (clothActive.maxVertDeltaPos > clothData.sleepThreshold || clothActive.index == -1 || Time.time - startTime < 5 || clothData.wind != Vector3.zero)
            {
                for (int i = 0; i < vertexDatas.Length; i++)
                {
                    vertexDatas[i].tension *= 0.85f;
                }
                for (int i = 0; i < vertexDatas.Length; i++)
                {
                    float moveMagnitude = 0;

                    vertexDatas[i].Update(transform, clothData, vertexDatas, dTimeP2, ref moveMagnitude);

                    if (moveMagnitude > clothActive.maxVertDeltaPos || clothActive.index == i || clothActive.index == -1)
                    {
                        clothActive.maxVertDeltaPos = moveMagnitude;
                        clothActive.index = i;
                    }
                    centers += vertexDatas[i].position;
                }
                for (int i = 0; i < vertexDatas.Length; i++)
                {
                    vertices[i] = vertexDatas[i].position;
                }
                centers /= vertexDatas.Length;
                if (clothData.volumePressure != 0)
                {
                    for (int i = 0; i < clothColliders.Length; i++)
                    {
                        for (int j = 0; j < clothColliders[i].connectedTris.Length; j++)
                        {
                            VertexData v1 = vertexDatas[clothColliders[i].connectedTris[j].index1].clone ? vertexDatas[vertexDatas[clothColliders[i].connectedTris[j].index1].clones[0]] : vertexDatas[clothColliders[i].connectedTris[j].index1];
                            VertexData v2 = vertexDatas[clothColliders[i].connectedTris[j].index2].clone ? vertexDatas[vertexDatas[clothColliders[i].connectedTris[j].index2].clones[0]] : vertexDatas[clothColliders[i].connectedTris[j].index2];
                            VertexData v3 = vertexDatas[clothColliders[i].connectedTris[j].index3].clone ? vertexDatas[vertexDatas[clothColliders[i].connectedTris[j].index3].clones[0]] : vertexDatas[clothColliders[i].connectedTris[j].index3];
                            Plane plane = new Plane(v1.position, v2.position, v3.position);
                            int value = plane.GetSide(centers) ? -1 : 1;
                            v1.Move(vertexDatas, plane.normal * clothData.volumePressure * value);
                            v3.Move(vertexDatas, plane.normal * clothData.volumePressure * value);
                            v2.Move(vertexDatas, plane.normal * clothData.volumePressure * value);
                        }
                    }
                    
                }
            }
            Profiler.EndSample();

            Profiler.BeginSample("Wind");
            //wind
            if (clothData.wind != Vector3.zero)
            {
                for (int l = 0; l < triangles.Length; l = l + 3)
                {
                    Vector3 normal = Vector3.Cross(vertexDatas[triangles[l + 1]].position - vertexDatas[triangles[l]].position, vertexDatas[triangles[l + 2]].position - vertexDatas[triangles[l ]].position);
                    float dot = Vector3.Dot(normal.normalized, clothData.wind + clothData.wind * Mathf.Sin(Time.time * 2 *vertexDatas[triangles[l + 1]].position.y));
                    Vector3 force = normal * dot * dTimeP2;
                 
                    //if (Mathf.Abs(dot) > 0.05)
                    {
                        vertexDatas[triangles[l]].Move(vertexDatas, force);
                        vertexDatas[triangles[l + 1]].Move(vertexDatas, force);
                        vertexDatas[triangles[l + 2]].Move(vertexDatas, force);
                    }
                }

            }
            Profiler.EndSample();

            for (int i = 0; i < interactiveObjs.Length; i++)
            {
                if (interactiveObjs[i].bounce)
                {
                    VertexData v1 = vertexDatas[interactiveObjs[i].lastColTriangle.index1].clone ? vertexDatas[vertexDatas[interactiveObjs[i].lastColTriangle.index1].clones[0]] : vertexDatas[interactiveObjs[i].lastColTriangle.index1];
                    VertexData v2 = vertexDatas[interactiveObjs[i].lastColTriangle.index2].clone ? vertexDatas[vertexDatas[interactiveObjs[i].lastColTriangle.index2].clones[0]] : vertexDatas[interactiveObjs[i].lastColTriangle.index2];
                    VertexData v3 = vertexDatas[interactiveObjs[i].lastColTriangle.index3].clone ? vertexDatas[vertexDatas[interactiveObjs[i].lastColTriangle.index3].clones[0]] : vertexDatas[interactiveObjs[i].lastColTriangle.index3];
                    Plane plane = new Plane(v1.position, v2.position, v3.position);
                    int side = plane.GetSide(interactiveObjs[i].localPos + plane.normal * interactiveObjs[i].localRadius * interactiveObjs[i].lastColSide) ? 1 : -1;
                    if (side != interactiveObjs[i].lastColSide && interactiveObjs[i].lastColSide != 0 && interactiveObjs[i].framesNotColliding < 5)
                    {
                        var dist = plane.GetDistanceToPoint(interactiveObjs[i].localPos);
                        if (Mathf.Abs(dist) < interactiveObjs[i].localRadius * 10)
                            interactiveObjs[i].transform.position += transform.TransformDirection(plane.normal * interactiveObjs[i].lastColSide * (dist + interactiveObjs[i].localRadius) * 1.05f);
                    }
                    else if (interactiveObjs[i].justCollided)
                    {
                        var center = (v1.position + v2.position + v3.position) * 0.333333f;
                        interactiveObjs[i].transform.position = transform.TransformPoint(center - interactiveObjs[i].lastColRelativePos);
                    }
                }

                interactiveObjs[i].prevPos = interactiveObjs[i].transform.position;
            }

        }


        void UpdateCollisions()
        {
            Profiler.BeginSample("Collision");
            //collision

            for (int i = 0; i < interactiveObjs.Length; i++)
            {
               
                interactiveObjs[i].localPos = transform.InverseTransformPoint(interactiveObjs[i].transform.position);
                interactiveObjs[i].localPrevPos = transform.InverseTransformPoint(interactiveObjs[i].prevPos);
                interactiveObjs[i].justCollided = false;

                if (!interactiveObjs[i].transform.gameObject.activeSelf)
                {
                    interactiveObjs[i].framesNotColliding++;
                    continue;
                }
                if (interactiveObjs[i].interactiveObjType == ObjType.Sphere)
                {
                    for (int k = 0; k < clothColliders.Length; k++)
                    {
                        if (interactiveObjs[i].transform == null || interactiveObjs[i].radius <= 0)
                            continue;
                        if (CubeIntersectSphere(clothColliders[k].bounds.min, clothColliders[k].bounds.max, interactiveObjs[i].transform.position, interactiveObjs[i].radius)
                            || CheckLineBox(clothColliders[k].bounds.min, clothColliders[k].bounds.max, interactiveObjs[i].prevPos, interactiveObjs[i].transform.position))
                        {
                            if (CheckClothCollision2(interactiveObjs[i], clothColliders[k].connectedTris))
                            {
                                interactiveObjs[i].framesNotColliding = 0;
                                interactiveObjs[i].justCollided = true;
                            }
                            else
                            {
                                interactiveObjs[i].framesNotColliding++;
                            }
                        }else
                        {
                            interactiveObjs[i].framesNotColliding++;
                        }
                    }
                }
            }
            Profiler.EndSample();
        }

        public void Reset()
        {
            foreach (var item in vertexDatas)
            {
                item.position = item.prevPosition = item.initialPosition;
            }
        }

        void Update()
        {
            if (clothActive.maxVertDeltaPos > clothData.sleepThreshold || clothActive.index == -1 || Time.time - startTime < 5 || clothData.wind != Vector3.zero)
            {
                mesh.vertices = vertices;
                mesh.UploadMeshData(false);
                mesh.RecalculateBounds();
                //mesh.RecalculateNormals();
                mesh.RecalculateNormals(360);
                for (int k = 0; k < clothColliders.Length; k++)
                {
                    clothColliders[k].RecalculateBounds(transform, vertexDatas);
                }
            }

            if (transform.position != prevPos)
            {
                if (clothData.deltaPosInfluence > 0)
                {
                    Vector3 deltaPos = transform.InverseTransformDirection(transform.position - prevPos);
                    for (int k = 0; k < vertexDatas.Length; k++)
                    {
                        if (!vertexDatas[k].clone)
                        {
                            vertexDatas[k].Move(vertexDatas, -deltaPos * clothData.deltaPosInfluence);
                        }
                    }
                }
                clothActive.index = -1;
            }

            if (transform.rotation != prevRot)
            {
                clothData.localGravity = transform.InverseTransformDirection(clothData.gravity);
                clothActive.index = -1;
            }

            prevPos = transform.position;
            prevRot = transform.rotation;
        }
        public void InitializeVertexDatas()
        {
            mesh = GetComponent<MeshFilter>().sharedMesh;
            vertices = mesh.vertices;
            triangles = mesh.triangles;
            uvs = mesh.uv;

            vertexDatas = new VertexData[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                if (vertexDatas[i] == null || !vertexDatas[i].clone)
                {
                    vertexDatas[i] = new VertexData(vertices[i], false, -1);

                    for (int j = i + 1; j < vertices.Length; j++)
                    {
                        if (Mathf.Abs(Vector3.Distance(vertices[i], vertices[j])) < 0.05f)
                        {
                            vertexDatas[j] = new VertexData(vertices[j], true, -1);

                            Array.Resize(ref vertexDatas[j].clones, vertexDatas[j].clones.Length + 1);
                            vertexDatas[j].clones[vertexDatas[j].clones.Length - 1] = i;

                            Array.Resize(ref vertexDatas[i].clones, vertexDatas[i].clones.Length + 1);
                            vertexDatas[i].clones[vertexDatas[i].clones.Length - 1] = j;
                        }
                    }
                }
            }
            Vector3 sum = Vector3.zero;
            for (int i = 0; i < vertexDatas.Length; i++)
                sum += vertexDatas[i].position;

            for (int i = 0; i < vertexDatas.Length; i++)
            {
                vertexDatas[i].SetCenterDisplace(sum / vertexDatas.Length);
            }

            GetVertexConnections();
        }

        public void GetVertexConnections()
        {
            for (int i = 0; i < vertexDatas.Length; i++)
            {
                if (!vertexDatas[i].clone)
                {
                    int last = -1;
                    int last2 = -1;
                    int last3 = -1;
                    for (int k = 0; k < triangles.Length; k++)
                    {
                        if ((triangles[k] == i || vertexDatas[triangles[k]].clone && vertexDatas[triangles[k]].clones[0] == i) && k % 3 == 0 && k + 2 < triangles.Length)
                        {
                            int first = vertexDatas[triangles[k + 1]].clone ? vertexDatas[triangles[k + 1]].clones[0] : triangles[k + 1];
                            int second = vertexDatas[triangles[k + 2]].clone ? vertexDatas[triangles[k + 2]].clones[0] : triangles[k + 2];
                            int third = vertexDatas[triangles[k]].clone ? vertexDatas[triangles[k]].clones[0] : triangles[k];

                            if (Array.IndexOf(vertexDatas[first].springs, second) == -1 && Array.IndexOf(vertexDatas[second].springs, first) == -1)
                                AddConectionToVertex(second, first);

                            if (last != -1)
                            {
                                if (last2 == first || last2 == second)
                                {
                                    if (Array.IndexOf(vertexDatas[last3].springs, last) == -1 && Array.IndexOf(vertexDatas[last].springs, last3) == -1)
                                        AddConectionToVertex(last3, last);
                                    if (last2 == first && Array.IndexOf(vertexDatas[triangles[k]].springs, second) == -1 && Array.IndexOf(vertexDatas[second].springs, triangles[k]) == -1)
                                        AddConectionToVertex(triangles[k], second);
                                    if (last2 == second && Array.IndexOf(vertexDatas[triangles[k]].springs, first) == -1 && Array.IndexOf(vertexDatas[first].springs, triangles[k]) == -1)
                                        AddConectionToVertex(triangles[k], first);
                                }

                                if (last3 == second || last3 == first)
                                {
                                    if (Array.IndexOf(vertexDatas[last2].springs, last) == -1 && Array.IndexOf(vertexDatas[last].springs, last2) == -1)
                                        AddConectionToVertex(last, last2);
                                    if (last3 == first && Array.IndexOf(vertexDatas[triangles[k]].springs, second) == -1 && Array.IndexOf(vertexDatas[second].springs, triangles[k]) == -1)
                                        AddConectionToVertex(triangles[k], second);
                                    if (last3 == second && Array.IndexOf(vertexDatas[triangles[k]].springs, first) == -1 && Array.IndexOf(vertexDatas[first].springs, triangles[k]) == -1)
                                        AddConectionToVertex(triangles[k], first);
                                }

                            }

                            last = third;
                            last2 = first;
                            last3 = second;
                        }
                    }
                }
            }

            if (clothData.clothType == ClothType.Struct_Shear_Bend)
            {
                for (int l = 0; l < vertexDatas.Length; l++)
                {
                    if (!vertexDatas[l].clone)
                    {
                        for (int k = 0; k < triangles.Length; k++)
                        {
                            if ((triangles[k] == l || vertexDatas[triangles[k]].clone && vertexDatas[triangles[k]].clones[0] == l) && k % 3 == 0 && k + 2 < triangles.Length)
                            {

                                int first = vertexDatas[triangles[k]].clone ? vertexDatas[triangles[k]].clones[0] : triangles[k];
                                int second = vertexDatas[triangles[k + 1]].clone ? vertexDatas[triangles[k + 1]].clones[0] : triangles[k + 1];
                                int third = vertexDatas[triangles[k + 2]].clone ? vertexDatas[triangles[k + 2]].clones[0] : triangles[k + 2];

                                for (int j = 0; j < triangles.Length - 2; j++)
                                {

                                    int firstJ = vertexDatas[triangles[j]].clone ? vertexDatas[triangles[j]].clones[0] : triangles[j];
                                    int secondJ = vertexDatas[triangles[j + 1]].clone ? vertexDatas[triangles[j + 1]].clones[0] : triangles[j + 1];
                                    int thirdJ = vertexDatas[triangles[j + 2]].clone ? vertexDatas[triangles[j + 2]].clones[0] : triangles[j + 2];

                                    if (triangles[j] == triangles[k + 1] && j % 3 == 0)
                                    {
                                        bool one = Array.IndexOf(vertexDatas[first].springs, second) > -1 || Array.IndexOf(vertexDatas[second].springs, first) > -1;
                                        bool two = Array.IndexOf(vertexDatas[firstJ].springs, secondJ) > -1 || Array.IndexOf(vertexDatas[secondJ].springs, firstJ) > -1;

                                        if (one && two)
                                        {
                                            AddConectionToVertex(secondJ, first, true);
                                        }
                                    }

                                    if (triangles[j] == triangles[k + 2] && j % 3 == 0)
                                    {
                                        bool one = Array.IndexOf(vertexDatas[third].springs, first) > -1 || Array.IndexOf(vertexDatas[first].springs, third) > -1;
                                        bool two = Array.IndexOf(vertexDatas[thirdJ].springs, firstJ) > -1 || Array.IndexOf(vertexDatas[firstJ].springs, thirdJ) > -1;

                                        if (one && two)
                                        {
                                            AddConectionToVertex(first, thirdJ, true);
                                        }
                                    }

                                    if (firstJ == third && (j - 1) % 3 == 0 && j + 2 < triangles.Length && j - 2 > 0)
                                    {
                                        int minusJ = vertexDatas[triangles[j - 1]].clone ? vertexDatas[triangles[j - 1]].clones[0] : triangles[j - 1];
                                        int minusJ2 = vertexDatas[triangles[j - 2]].clone ? vertexDatas[triangles[j - 2]].clones[0] : triangles[j - 2];

                                        bool one = Array.IndexOf(vertexDatas[first].springs, second) == -1 && Array.IndexOf(vertexDatas[second].springs, first) == -1;
                                        bool two = Array.IndexOf(vertexDatas[minusJ].springs, first) != -1 || Array.IndexOf(vertexDatas[first].springs, minusJ) != -1;

                                        if (first != thirdJ && one && two)
                                        {
                                            AddConectionToVertex(triangles[k + 1], triangles[j + 1], true);
                                        }
                                    }

                                    if (third == secondJ && j % 3 == 0 && j - 2 > 0)
                                    {
                                        bool one = Array.IndexOf(vertexDatas[second].springs, third) != -1 || Array.IndexOf(vertexDatas[third].springs, second) != -1;
                                        bool two = Array.IndexOf(vertexDatas[firstJ].springs, thirdJ) != -1 || Array.IndexOf(vertexDatas[thirdJ].springs, secondJ) != -1;
                                        bool three = Array.IndexOf(vertexDatas[first].springs, third) == -1 && Array.IndexOf(vertexDatas[third].springs, first) == -1;
                                        bool four = Array.IndexOf(vertexDatas[firstJ].springs, thirdJ) == -1 && Array.IndexOf(vertexDatas[thirdJ].springs, firstJ) == -1;

                                        if (first != firstJ && one && three && four)
                                        {
                                            AddConectionToVertex(second, thirdJ, true);
                                        }
                                    }
                                }
                            }
                        }
                    }

                }
            }

            if (clothData.clothType == ClothType.Struct_Shear_Bend || clothData.clothType == ClothType.Struct_Shear)
            {
                for (int i = 0; i < vertexDatas.Length; i++)
                {
                    int last = -1;
                    int last2 = -1;
                    int last3 = -1;
                    for (int k = 0; k < triangles.Length; k++)
                    {
                        if (triangles[k] == i && k % 3 == 0 && k + 2 < triangles.Length)
                        {
                            int first = triangles[k + 1];
                            int second = triangles[k + 2];
                            AddConectionToVertex(i, first);
                            AddConectionToVertex(i, second);

                            if (last != -1 && last != first && last != second && last != i && last2 != last)
                                AddConectionToVertex(second, last);

                            if (last2 != -1 && last2 != last && (last2 == first || last == second))
                                AddConectionToVertex(first, last2);



                            last = first;
                            last2 = second;
                            last3 = triangles[k];
                        }
                    }
                }
            }
        }

        void AddConectionToVertex(int index, int index2, bool bending = false)
        {
            if (index == index2)
                return;

            if (vertexDatas[index].clone && Array.IndexOf(vertexDatas[index].clones, index2) != -1)
                return;

            if (vertexDatas[index2].clone && Array.IndexOf(vertexDatas[index2].clones, index) != -1)
                return;


            if (vertexDatas[index].clone && vertexDatas[index2].clone)
            {
                if (Array.IndexOf(vertexDatas[vertexDatas[index].clones[0]].springs, vertexDatas[index2].clones[0]) == -1 && Array.IndexOf(vertexDatas[vertexDatas[index2].clones[0]].springs, vertexDatas[index].clones[0]) == -1)
                    vertexDatas[vertexDatas[index].clones[0]].AddConnection(index2, vertexDatas, bending);
            }
            if (vertexDatas[index].clone)
            {
                if (Array.IndexOf(vertexDatas[vertexDatas[index].clones[0]].springs, index2) == -1 && Array.IndexOf(vertexDatas[index2].springs, index) == -1)
                    vertexDatas[vertexDatas[index].clones[0]].AddConnection(index2, vertexDatas, bending);
            }
            else if (vertexDatas[index2].clone)
            {
                if (Array.IndexOf(vertexDatas[index].springs, vertexDatas[index2].clones[0]) == -1 && Array.IndexOf(vertexDatas[vertexDatas[index2].clones[0]].springs, index) == -1)
                    vertexDatas[index].AddConnection(vertexDatas[index2].clones[0], vertexDatas, bending);
            }
            else
            {
                if (Array.IndexOf(vertexDatas[index].springs, index2) == -1 && Array.IndexOf(vertexDatas[index2].springs, index) == -1)
                    vertexDatas[index].AddConnection(index2, vertexDatas, bending);
            }


        }


        //colision params
        Vector3 rayHeading;
        Vector3 rayDircetion;
        Vector3 rayOrigin;
        Vector3 deltaRadius;
        float rayMagnitude;

        const float kRestitution = 0.1f;
        const float kGoalNetParticleMass = 0.08f;
        const float kGoalNetParticleInvMass = 1.0f / kGoalNetParticleMass;


        public bool CheckClothCollision2(InteractiveObj interactiveObj, Triangle[] triangles)
        {
            bool collided = false;
            rayHeading = interactiveObj.localPos - interactiveObj.localPrevPos;
            rayMagnitude = rayHeading.magnitude;
            rayDircetion = rayHeading / rayMagnitude;

            deltaRadius = rayDircetion * interactiveObj.localRadius;
            rayOrigin = interactiveObj.localPrevPos - deltaRadius * 0.99f;

            bool firstCollision = interactiveObj.framesNotColliding > 5;
            bool isSlow = rayMagnitude < interactiveObj.radius * 2; 

            float colDistance = 0;
            bool addedForce = false;
            for (int k = 0; k < triangles.Length; k++)
            {
                VertexData v1 = vertexDatas[triangles[k].index1].clone ? vertexDatas[vertexDatas[triangles[k].index1].clones[0]] : vertexDatas[triangles[k].index1];
                VertexData v2 = vertexDatas[triangles[k].index2].clone ? vertexDatas[vertexDatas[triangles[k].index2].clones[0]] : vertexDatas[triangles[k].index2];
                VertexData v3 = vertexDatas[triangles[k].index3].clone ? vertexDatas[vertexDatas[triangles[k].index3].clones[0]] : vertexDatas[triangles[k].index3];

                var center = (v1.position + v2.position + v3.position) * 0.333333f;
                var prevCenter = (v1.prevPosition + v2.prevPosition + v3.prevPosition) * 0.333333f;
                var netDisplacement = center - prevCenter;

                if ((firstCollision || !isSlow) && Intersect(v1.position, v2.position, v3.position, rayOrigin, rayDircetion, rayMagnitude, interactiveObj.localRadius * 2, out colDistance))
                {
                    Plane plane = new Plane(v1.position, v2.position, v3.position);

                    if (clothData.impactDampening > 0 && !addedForce )
                    {
                        addedForce = true;
                        interactiveObj.rigidBody.velocity = transform.TransformDirection(rayHeading * (1 - clothData.impactDampening) / Time.deltaTime);
                    }
                    Vector3 relativePos = rayOrigin + rayDircetion * (colDistance );
                    Debug.LogError("enters");


                    Vector3 netImpulse = rayDircetion * (rayMagnitude + interactiveObj.localRadius * 2 - colDistance);

                    //interactiveObj.transform.position = transform.TransformPoint(newCenter - lastColRelativePos);
                    v1.Move(vertexDatas, rayHeading * (1 - clothData.impactDampening));
                    v2.Move(vertexDatas, rayHeading * (1 - clothData.impactDampening));
                    v3.Move(vertexDatas, rayHeading * (1 - clothData.impactDampening));

                    var newCenter = (v1.position + v2.position + v3.position) * 0.333333f;
                    interactiveObj.lastColTriangle = triangles[k];
                    interactiveObj.lastColSide = plane.GetSide(rayOrigin) ? 1 : -1;
                    interactiveObj.lastColRelativePos = newCenter - relativePos;

                    clothActive.index = -1;
                    collided = true;
                }
                else if ((!firstCollision || rayMagnitude < 0.000001f || !interactiveObj.bounce) && isSlow)
                {
                    Vector3 colisionPoint = ClosestPointOnTriangle(interactiveObj.localPos, v1.position, v2.position, v3.position);
                    Vector3 heading = interactiveObj.localPos - colisionPoint;
                    float headingMag = heading.magnitude;
                    Vector3 direction = heading / headingMag;

                    if (headingMag < (interactiveObj.bounce ? interactiveObj.localRadius : interactiveObj.localRadius * 1.1f))
                    {
                        var normal = Vector3.Cross(v2.position - v1.position, v3.position - v1.position).normalized;

                        var ballRelVel = (interactiveObj.localPos - interactiveObj.localPrevPos) - netDisplacement;

                        var nv = Vector3.Dot(ballRelVel, normal);
                        var j = ((1.0f + kRestitution) * nv / (0.3f + kGoalNetParticleInvMass)) * normal; // 0.3 should be 3
                        var netImpulse = j * kGoalNetParticleInvMass;

                        if (firstCollision && clothData.impactDampening > 0 && !addedForce)
                        {
                            addedForce = true;
                            interactiveObj.rigidBody.velocity = transform.TransformDirection(rayHeading * (1 - clothData.impactDampening) / Time.deltaTime);
                        }

                        if (!addedForce && interactiveObj.bounce)
                        {
                            interactiveObj.rigidBody.velocity -= transform.TransformDirection(j * 3.333f) / Time.deltaTime;
                            interactiveObj.rigidBody.velocity *= 0.99f;
                            addedForce = true;
                        }

                        //interactiveObj.transform.position += transform.TransformDirection(-direction * (headingMag - interactiveObj.localRadius)) * (0.05f + 0.95f * (Mathf.Clamp(Mathf.Abs(headingMag - interactiveObj.localRadius), 0, 0.2f) * 5f));
                        if (!interactiveObj.bounce)
                        {
                            netImpulse = heading - direction * interactiveObj.localRadius * 1.1f;
                        }

                   
                        v1.Move(vertexDatas, netImpulse);
                        v2.Move(vertexDatas, netImpulse);
                        v3.Move(vertexDatas, netImpulse);

                        var newCenter = (v1.position + v2.position + v3.position) * 0.333333f;
                        interactiveObj.lastColTriangle = triangles[k];
                        interactiveObj.lastColRelativePos = newCenter - colisionPoint + normal * -interactiveObj.lastColSide * interactiveObj.localRadius;

                        clothActive.index = -1;
                        collided = true;
                    }
                }
            }
            return collided;
        }

        bool CubeIntersectSphere(Vector3 C1, Vector3 C2, Vector3 S, float R)
        {
            float dist_squared = R * R;
            /* assume C1 and C2 are element-wise sorted, if not, do that now */
            if (S.x < C1.x) dist_squared -= Mathf.Pow(S.x - C1.x, 2);
            else if (S.x > C2.x) dist_squared -= Mathf.Pow(S.x - C2.x, 2);
            if (S.y < C1.y) dist_squared -= Mathf.Pow(S.y - C1.y, 2);
            else if (S.y > C2.y) dist_squared -= Mathf.Pow(S.y - C2.y, 2);
            if (S.z < C1.z) dist_squared -= Mathf.Pow(S.z - C1.z, 2);
            else if (S.z > C2.z) dist_squared -= Mathf.Pow(S.z - C2.z, 2);
            return dist_squared > 0;
        }

        public bool Intersect(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 rayOrigin, Vector3 rayDirection, float rayLength, float radius, out float colDistance)
        {
            colDistance = -1;
            // Vectors from p1 to p2/p3 (edges)
            Vector3 e1, e2;

            Vector3 p, q, t;
            float det, invDet, u, v;


            //Find vectors for two edges sharing vertex/point p1
            e1 = p2 - p1;
            e2 = p3 - p1;

            // calculating determinant 
            p = Vector3.Cross(rayDirection, e2);

            //Calculate determinat
            det = Vector3.Dot(e1, p);

            //if determinant is near zero, ray lies in plane of triangle otherwise not
            if (det > -float.Epsilon && det < float.Epsilon) { return false; }
            invDet = 1.0f / det;

            //calculate distance from p1 to ray origin
            t = rayOrigin - p1;

            //Calculate u parameter
            u = Vector3.Dot(t, p) * invDet;

            //Check for ray hit
            if (u < 0 || u > 1) { return false; }

            //Prepare to test v parameter
            q = Vector3.Cross(t, e1);

            //Calculate v parameter
            v = Vector3.Dot(rayDirection, q) * invDet;

            //Check for ray hit
            if (v < 0 || u + v > 1) { return false; }

            //Get distance from hit Point
            float distance = (Vector3.Dot(e2, q) * invDet);

            if (distance > float.Epsilon && distance <= rayLength + radius)
            {
                colDistance = distance;
                //ray does intersect
                return true;
            }

            // No hit at all
            return false;
        }

        Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f)
                return a; // Barycentric coordinates are (1,0,0).

            // Check if P is in vertex region outside B.
            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3)
                return b; // Barycentric coordinates are (0,1,0).

            // Check if P is in edge region of AB, and if so, return the projection of P onto AB.
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float vu = d1 / (d1 - d3);
                return a + vu * ab; // The barycentric coordinates are (1-v, v, 0).
            }

            // Check if P is in vertex region outside C.
            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6)
                return c; // The barycentric coordinates are (0,0,1).

            // Check if P is in edge region of AC, and if so, return the projection of P onto AC.
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float wu = d2 / (d2 - d6);
                return a + wu * ac; // The barycentric coordinates are (1-w, 0, w).
            }

            // Check if P is in edge region of BC, and if so, return the projection of P onto BC.
            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && d4 - d3 >= 0f && d5 - d6 >= 0f)
            {
                float ww = (d4 - d3) / (d4 - d3 + d5 - d6);
                return b + ww * (c - b); // The barycentric coordinates are (0, 1-w, w).
            }

            // P must be inside the face region. Compute the closest point through its barycentric coordinates (u,v,w).
            float denom = 1.0f / (va + vb + vc);
            float v = vb * denom;
            float w = vc * denom;

            return a + ab * v + ac * w;
        }


        bool CheckLineBox(Vector3 B1, Vector3 B2, Vector3 L1, Vector3 L2)
        {
            Vector3 Hit = Vector3.zero;
            if (L2.x < B1.x && L1.x < B1.x) return false;
            if (L2.x > B2.x && L1.x > B2.x) return false;
            if (L2.y < B1.y && L1.y < B1.y) return false;
            if (L2.y > B2.y && L1.y > B2.y) return false;
            if (L2.z < B1.z && L1.z < B1.z) return false;
            if (L2.z > B2.z && L1.z > B2.z) return false;
            if (L1.x > B1.x && L1.x < B2.x &&
                L1.y > B1.y && L1.y < B2.y &&
                L1.z > B1.z && L1.z < B2.z)
            {
                Hit = L1;
                return true;
            }
            if ((GetIntersection(L1.x - B1.x, L2.x - B1.x, L1, L2, ref Hit) && InBox(Hit, B1, B2, 1))
              || (GetIntersection(L1.y - B1.y, L2.y - B1.y, L1, L2, ref Hit) && InBox(Hit, B1, B2, 2))
              || (GetIntersection(L1.z - B1.z, L2.z - B1.z, L1, L2, ref Hit) && InBox(Hit, B1, B2, 3))
              || (GetIntersection(L1.x - B2.x, L2.x - B2.x, L1, L2, ref Hit) && InBox(Hit, B1, B2, 1))
              || (GetIntersection(L1.y - B2.y, L2.y - B2.y, L1, L2, ref Hit) && InBox(Hit, B1, B2, 2))
              || (GetIntersection(L1.z - B2.z, L2.z - B2.z, L1, L2, ref Hit) && InBox(Hit, B1, B2, 3)))
                return true;

            return false;
        }

        bool GetIntersection(float fDst1, float fDst2, Vector3 P1, Vector3 P2, ref Vector3 Hit)
        {
            if ((fDst1 * fDst2) >= 0.0f) return false;
            if (fDst1 == fDst2) return false;
            Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
            return true;
        }

        bool InBox(Vector3 Hit, Vector3 B1, Vector3 B2, int Axis)
        {
            
            if (Axis == 1 && Hit.z > B1.z && Hit.z < B2.z && Hit.y > B1.y && Hit.y < B2.y) return true;
            if (Axis == 2 && Hit.z > B1.z && Hit.z < B2.z && Hit.x > B1.x && Hit.x < B2.x) return true;
            if (Axis == 3 && Hit.x > B1.x && Hit.x < B2.x && Hit.y > B1.y && Hit.y < B2.y) return true;
            return false;
         
        }



        float GetApproximatedMagnitude(Vector3 vector) {
            float x = vector.x > 0 ? vector.x : -vector.x;
            float y = vector.y > 0 ? vector.y : -vector.y;
            float z = vector.z > 0 ? vector.z : -vector.z;

            return Mathf.Max(x, Mathf.Max(y, z));
        }


        private void AutoWeld(ref Vector3[] verts, ref int[] tris, ref Vector2[] uvs, float threshold = 0.0001f)
        {
            List<Vector3> newVerts = new List<Vector3>();
            List<Vector2> newUVs = new List<Vector2>();

            int k = 0;

            foreach (Vector3 vert in verts)
            {
                foreach (Vector3 newVert in newVerts)
                    if (Vector3.Distance(newVert, vert) <= threshold)
                        goto skipToNext;

                newVerts.Add(vert);
                newUVs.Add(mesh.uv[k]);

            skipToNext: ;
                ++k;
            }

            for (int i = 0; i < tris.Length; ++i)
            {
                for (int j = 0; j < newVerts.Count; ++j)
                {
                    if (Vector3.Distance(newVerts[j], verts[tris[i]]) <= threshold)
                    {
                        tris[i] = j;
                        break;
                    }
                }
            }

            uvs = newUVs.ToArray();
            verts = newVerts.ToArray();
        }
    }


  
}
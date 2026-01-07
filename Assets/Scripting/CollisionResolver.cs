using UnityEngine;
using System.Collections.Generic;
public static class BoxCollisionResolver
{
    // 衝突情報構造体
    struct CollisionInfo
    {
        public bool HasCollision;
        public float Penetration;
        public Vector3 Normal; // AからBへのベクトル
    }

public static void ResolveCollision(MyRigidBody boxA, MyRigidBody boxB)
{
    CollisionInfo col = CheckSAT(boxA, boxB);
    if (!col.HasCollision) return;

    List<Vector3> contactPoints = FindContactPoints(boxA, boxB, col.Normal);
    // セーフティ：接触点がない場合は重心付近を仮の点にする
    if (contactPoints.Count == 0)
    {
        Vector3 fallback = boxB.Position + col.Normal * boxB.HalfSize.y;
        ApplyImpulse(boxA, boxB, fallback, col.Normal, col.Penetration, 1);
        return;
    }
    // ★重要：ここでは分割せず、ApplyImpulseに元のPenetrationを渡すか、
    // ApplyImpulse側で正しく処理する必要があります。
    foreach (var point in contactPoints)
    {
        // 今回はApplyImpulse側を「分割を前提としない」形に修正するため、そのまま渡します
        ApplyImpulse(boxA, boxB, point, col.Normal, col.Penetration, contactPoints.Count);
    }
}
// 2つの箱の重なりから、接触点リスト(マニフォールド)を生成する
static List<Vector3> FindContactPoints(MyRigidBody A, MyRigidBody B, Vector3 normal)
{
    // Bの面のうち、Aに向かっている面（底面）を取得
    // GetIncidentFaceは「入力ベクトルの逆」を探すので、そのままNormal(B->A)を渡すと
    // Bの「上向きの反対」＝底面が取れるはずです。
    List<Vector3> incidentFace = GetIncidentFace(B, normal);
    List<Vector3> clippedPolygon = new List<Vector3>(incidentFace);

    // --- クリッピング ---
    // Aのローカル空間に変換して、Aの箱の範囲内に収める
    Matrix4x4 worldToA = A.GetInverseTransformMatrix();
    Matrix4x4 aToWorld = A.GetTransformMatrix();

    for (int i = 0; i < clippedPolygon.Count; i++)
        clippedPolygon[i] = worldToA.MultiplyPoint3x4(clippedPolygon[i]);

    Vector3 sizeA = A.HalfSize;
    clippedPolygon = Clip(clippedPolygon, Vector3.right, sizeA.x);
    clippedPolygon = Clip(clippedPolygon, -Vector3.right, sizeA.x);
    clippedPolygon = Clip(clippedPolygon, Vector3.up, sizeA.y);
    clippedPolygon = Clip(clippedPolygon, -Vector3.up, sizeA.y);
    clippedPolygon = Clip(clippedPolygon, Vector3.forward, sizeA.z);
    clippedPolygon = Clip(clippedPolygon, -Vector3.forward, sizeA.z);

    for (int i = 0; i < clippedPolygon.Count; i++)
        clippedPolygon[i] = aToWorld.MultiplyPoint3x4(clippedPolygon[i]);

    // --- 最後のフィルタリング（ここが重要） ---
    List<Vector3> finalPoints = new List<Vector3>();

    // Aの表面(Reference Plane)を計算
    // NormalはB->A(下向き)なので、A(床)から見ると「-Normal(上向き)」の場所が表面。
    Vector3 surfaceNormalOfA = -normal; 
    Vector3 planePoint = A.Position + surfaceNormalOfA * GetProjectedRadius(A, surfaceNormalOfA);
    
    // 平面との距離を測る。Normal方向(B->A)にどれだけ進んでいるか。
    float planeD = Vector3.Dot(surfaceNormalOfA, planePoint);

// FindContactPointsの最後
float contactThreshold = 0.05f; // 5cmくらい浮いてても「当たってる」ことにする

foreach (var p in clippedPolygon) {
    float distance = Vector3.Dot(surfaceNormalOfA, p) - planeD;
    // distanceがプラス（浮いている）でも、閾値内なら「接地」とみなす
    if (distance <= contactThreshold) {
        finalPoints.Add(p);
    }
}

    return finalPoints;
}

    // ポリゴンを平面でカットする関数
    // normal: クリップ面の法線, distance: 原点からの距離(HalfSize)
    static List<Vector3> Clip(List<Vector3> polygon, Vector3 normal, float clipLimit)
    {
        List<Vector3> clipped = new List<Vector3>();
        if (polygon.Count == 0) return clipped;

        Vector3 v1 = polygon[polygon.Count - 1];
        // ここでの判定: 点は clipLimit より「内側」にあるべき
        // ローカル空間なので、 dot(p, normal) < clipLimit ならOK、とします
        // 例: normal=(1,0,0), limit=5.  xが5より小さければOK。
        
        float d1 = Vector3.Dot(v1, normal) - clipLimit; 

        for (int i = 0; i < polygon.Count; i++)
        {
            Vector3 v2 = polygon[i];
            float d2 = Vector3.Dot(v2, normal) - clipLimit;

            // d <= 0 なら「内側」
            if (d1 <= 0 && d2 <= 0) 
            {
                // 両方内側 -> v2を追加
                clipped.Add(v2);
            }
            else if (d1 <= 0 && d2 > 0)
            {
                // 内から外へ -> 交点を追加
                clipped.Add(CalculateIntersection(v1, v2, normal, clipLimit));
            }
            else if (d1 > 0 && d2 <= 0)
            {
                // 外から内へ -> 交点とv2を追加
                clipped.Add(CalculateIntersection(v1, v2, normal, clipLimit));
                clipped.Add(v2);
            }
            // 両方外側 -> 何もしない

            v1 = v2;
            d1 = d2;
        }

        return clipped;
    }

    static Vector3 CalculateIntersection(Vector3 v1, Vector3 v2, Vector3 n, float limit)
    {
        // 線分 v1-v2 と 平面(n, limit) の交点
        float dist1 = Vector3.Dot(v1, n) - limit;
        float dist2 = Vector3.Dot(v2, n) - limit;
        float t = dist1 / (dist1 - dist2);
        return v1 + (v2 - v1) * t;
    }

    // 指定された法線に最も逆らっている（向かい合っている）面を取得
    static List<Vector3> GetIncidentFace(MyRigidBody body, Vector3 normal)
    {
        // 1. 法線と最も「逆向き」に近いローカル軸を探す
        Vector3 localNormal = Quaternion.Inverse(body.Orientation) * normal; // 回転が必要なら
        // MyRigidBodyに回転が無い(Axis aligned)ならそのままnormal

        // もしRigidBodyが回転行列を持っているなら：
        // localNormal = body.InverseTransformDirection(normal);
        
        // 一番絶対値が大きい軸が、その面の法線軸
        Vector3 absN = new Vector3(Mathf.Abs(localNormal.x), Mathf.Abs(localNormal.y), Mathf.Abs(localNormal.z));
        
        Vector3 size = body.HalfSize;
        List<Vector3> face = new List<Vector3>();

        if (absN.x > absN.y && absN.x > absN.z)
        {
            // X軸方向の面
            bool positive = localNormal.x > 0; 
            // Incidentなので、法線と「逆」の面を選ぶ
            float sign = positive ? -1 : 1; 
            float x = size.x * sign;
            
            // YZ平面上の4点
            face.Add(new Vector3(x, size.y, -size.z));
            face.Add(new Vector3(x, size.y, size.z));
            face.Add(new Vector3(x, -size.y, size.z));
            face.Add(new Vector3(x, -size.y, -size.z));
        }
        else if (absN.y > absN.x && absN.y > absN.z)
        {
            // Y軸方向の面
            bool positive = localNormal.y > 0;
            float sign = positive ? -1 : 1;
            float y = size.y * sign;

            face.Add(new Vector3(-size.x, y, size.z));
            face.Add(new Vector3(size.x, y, size.z));
            face.Add(new Vector3(size.x, y, -size.z));
            face.Add(new Vector3(-size.x, y, -size.z));
        }
        else
        {
            // Z軸方向の面
            bool positive = localNormal.z > 0;
            float sign = positive ? -1 : 1;
            float z = size.z * sign;

            face.Add(new Vector3(-size.x, size.y, z));
            face.Add(new Vector3(size.x, size.y, z));
            face.Add(new Vector3(size.x, -size.y, z));
            face.Add(new Vector3(-size.x, -size.y, z));
        }

        // ローカル座標の頂点をワールド座標へ変換
        Matrix4x4 localToWorld = body.GetTransformMatrix(); // ※要自作
        for(int i=0; i<face.Count; i++)
        {
            face[i] = localToWorld.MultiplyPoint3x4(face[i]);
        }

        return face;
    }

    // ヘルパー：法線に最も近い面の法線を返す（今回は使用していないが概念として）
    static Vector3 GetFaceNormalMostParallelTo(MyRigidBody body, Vector3 normal)
    {
        // 省略（GetIncidentFaceと似たロジックで、符号判定が逆になるだけ）
        return Vector3.zero;
    }
    // SATの実装
    static CollisionInfo CheckSAT(MyRigidBody A, MyRigidBody B)
{
    float minPenetration = float.MaxValue;
    Vector3 bestAxis = Vector3.zero;

    Vector3[] axesToTest = new Vector3[6];
    for(int i=0; i<3; i++) axesToTest[i] = A.GetAxis(i);
    for(int i=0; i<3; i++) axesToTest[3+i] = B.GetAxis(i);

    for (int k = 0; k < axesToTest.Length; k++)
    {
        Vector3 axis = axesToTest[k];
        if (axis.sqrMagnitude < 0.001f) continue;
        axis.Normalize();

        float pA = GetProjectedRadius(A, axis);
        float pB = GetProjectedRadius(B, axis);
        float distance = Mathf.Abs(Vector3.Dot(B.Position - A.Position, axis));
        float overlap = (pA + pB) - distance;

        if (overlap < 0) return new CollisionInfo { HasCollision = false };

        // ★重要：バイアスの修正
        // 面の軸(k < 6)を優先し、辺の軸（今回は実装していませんが今後増える場合）を避けやすくする
        // 面の軸（最初の6本）はそのまま、それ以外（辺同士など）は少し「重なりを大きく見積もる」
        float bias = (k < 6) ? 1.0f : 1.5f; 
        
        if (overlap * bias < minPenetration)
        {
            minPenetration = overlap;
            bestAxis = axis;
        }
    }

    if (Vector3.Dot(bestAxis, B.Position - A.Position) > 0) bestAxis = -bestAxis;

    return new CollisionInfo { HasCollision = true, Penetration = minPenetration, Normal = bestAxis };
}

    // 箱を軸に投影したときの「半径（中心から端までの長さ）」を計算
    static float GetProjectedRadius(MyRigidBody box, Vector3 axis)
    {
        // 箱の「影」の長さは、各軸成分の投影の和になる
        // R = |dot(x, axis)| * halfX + |dot(y, axis)| * halfY + ...
        return 
            Mathf.Abs(Vector3.Dot(box.GetAxis(0), axis)) * box.HalfSize.x +
            Mathf.Abs(Vector3.Dot(box.GetAxis(1), axis)) * box.HalfSize.y +
            Mathf.Abs(Vector3.Dot(box.GetAxis(2), axis)) * box.HalfSize.z;
    }

    // インパルス応答（球のときと全く同じ式です）
static void ApplyImpulse(MyRigidBody A, MyRigidBody B, Vector3 contactPoint, Vector3 normal, float totalPenetration, int contactCount)
{
    // --- 1. 準備 ---
    Vector3 rA = contactPoint - A.Position;
    Vector3 rB = contactPoint - B.Position;
    Vector3 velA = A.LinearVelocity + Vector3.Cross(A.AngularVelocity, rA);
    Vector3 velB = B.LinearVelocity + Vector3.Cross(B.AngularVelocity, rB);
    Vector3 relVel = velA - velB;

    float velAlongNormal = Vector3.Dot(relVel, normal);
    if (velAlongNormal > 0) return;

    // --- 2. 垂直インパルス ---
    float e = Mathf.Min(A.Restitution, B.Restitution);
    if (velAlongNormal > -0.5f) e = 0.0f; 

    float invMassA = A.IsStatic ? 0f : 1.0f / A.Mass;
    float invMassB = B.IsStatic ? 0f : 1.0f / B.Mass;
    float invMassSum = invMassA + invMassB;

    Matrix4x4 invIA = A.GetInverseInertiaTensorWorld();
    Matrix4x4 invIB = B.GetInverseInertiaTensorWorld();

    Vector3 rAxN = Vector3.Cross(rA, normal);
    Vector3 rBxN = Vector3.Cross(rB, normal);
    float angularTerm = Vector3.Dot(Vector3.Cross(invIA.MultiplyVector(rAxN), rA) + 
                                    Vector3.Cross(invIB.MultiplyVector(rBxN), rB), normal);
    
    float j = -(1 + e) * velAlongNormal;
    j /= (invMassSum + angularTerm);
    j /= (float)contactCount; // 点数で力を割る

    Vector3 impulse = j * normal;

    // --- 3. 摩擦 (Friction) ---
    // (接線方向のインパルス計算 jt ... 省略なしで実装)
    Vector3 tangent = relVel - Vector3.Dot(relVel, normal) * normal;
    if (tangent.sqrMagnitude > 0.0001f)
    {
        tangent.Normalize();
        float jt = -Vector3.Dot(relVel, tangent);
        float angularTermT = Vector3.Dot(Vector3.Cross(invIA.MultiplyVector(Vector3.Cross(rA, tangent)), rA) + 
                                        Vector3.Cross(invIB.MultiplyVector(Vector3.Cross(rB, tangent)), rB), tangent);
        jt /= (invMassSum + angularTermT);
        jt /= (float)contactCount;

        float mu = Mathf.Sqrt(A.Friction * B.Friction);
        float maxFriction = Mathf.Abs(j) * mu; // 正しいクランプ
        jt = Mathf.Clamp(jt, -maxFriction, maxFriction);
        impulse += jt * tangent;
    }

    // --- 4. 適用 ---
    if (!A.IsStatic) {
        A.LinearVelocity += impulse * invMassA;
        A.AngularVelocity += invIA.MultiplyVector(Vector3.Cross(rA, impulse));
    }
    if (!B.IsStatic) {
        B.LinearVelocity -= impulse * invMassB;
        B.AngularVelocity -= invIB.MultiplyVector(Vector3.Cross(rB, impulse));
    }

    // --- 5. 位置補正 (すり抜け防止の肝) ---
    float percent = 0.2f; 
    float slop = 0.01f;
    // 補正は分割する前の「トータルのめり込み」で判定する
    float p = Mathf.Max(0, totalPenetration - slop);
    if (p > 0)
    {
        // 補正ベクトルを点数で割って分配
        Vector3 correction = normal * ((p / invMassSum) * percent / contactCount);
        if (!A.IsStatic) A.Position += correction * invMassA;
        if (!B.IsStatic) B.Position -= correction * invMassB;
    }
}
}
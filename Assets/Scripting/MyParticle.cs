using UnityEngine;

[System.Serializable]
public class MyRigidBody
{
    // --- 状態変数 (State Variables) ---
    public Vector3 Position;           // 位置 x
    public Quaternion Orientation = Quaternion.identity;     // 姿勢 q (回転)
    public Vector3 LinearVelocity;     // 速度 v
    public Vector3 AngularVelocity;    // 角速度 ω

    public float Friction; // 静止摩擦係数 μ

    // --- 定数・物理プロパティ (Constants) ---
    public float Mass = 1.0f;          // 質量 m
    
    public float Restitution = 0.2f;

    // --- 箱用の追加 ---
    public Vector3 HalfSize = Vector3.one * 0.5f; // 幅・高さ・奥行きの半分
    // 慣性テンソル (Local座標系での定数)
    // 直方体の場合、幅・高さ・奥行きから計算できる対角行列
    
    public Matrix4x4 InverseInertiaTensorLocal; 

// 【追加】半径
    public float Radius = 0.5f;      // 球としての半径

    // --- ワーク変数 (計算用) ---
    private Vector3 _forceAccumulator;  // 蓄積された力 F
    private Vector3 _torqueAccumulator; // 蓄積されたトルク τ

    public bool IsStatic = false;

    public float LinearDamping = 0.98f;
    public float AngularDamping = 0.96f;

    public float InverseMass 
    {
        get { return IsStatic ? 0.0f : 1.0f / Mass; }
    }

// 【変更】コンストラクタを球用に調整
public MyRigidBody(Vector3 position, Vector3 size, float mass, float friction)
    {
        Position = position;
        HalfSize = size * 0.5f;
        Mass = mass;
        Friction = friction;
        // 箱の慣性モーメント
        float w2 = size.x * size.x;
        float h2 = size.y * size.y;
        float d2 = size.z * size.z;
        Vector3 I = new Vector3(
            (mass / 12.0f) * (h2 + d2),
            (mass / 12.0f) * (w2 + d2),
            (mass / 12.0f) * (w2 + h2)
        );

        InverseInertiaTensorLocal = Matrix4x4.identity;
        InverseInertiaTensorLocal[0, 0] = 1.0f / I.x;
        InverseInertiaTensorLocal[1, 1] = 1.0f / I.y;
        InverseInertiaTensorLocal[2, 2] = 1.0f / I.z;
        InverseInertiaTensorLocal[3, 3] = 1.0f;
        
    }
    public Matrix4x4 GetTransformMatrix()
    {
        return Matrix4x4.TRS(Position, Orientation, Vector3.one);
    }

    // ワールド → ローカル変換行列 (逆行列)
    public Matrix4x4 GetInverseTransformMatrix()
    {
        return Matrix4x4.Inverse(GetTransformMatrix());
    }
    // ワールド空間での軸を取得するヘルパー
    public Vector3 GetAxis(int index) 
    {
        // 0:Right, 1:Up, 2:Forward
        Matrix4x4 R = Matrix4x4.Rotate(Orientation);
        return R.GetColumn(index); // 行列の列ベクトルがそのままローカル軸のワールド表現
    }

    // 頂点群をワールド座標で取得
    public Vector3[] GetVerticesWorld()
    {
        Vector3[] vertices = new Vector3[8];
        Vector3[] corners = {
            new Vector3(1, 1, 1), new Vector3(1, 1, -1), new Vector3(1, -1, 1), new Vector3(1, -1, -1),
            new Vector3(-1, 1, 1), new Vector3(-1, 1, -1), new Vector3(-1, -1, 1), new Vector3(-1, -1, -1)
        };
        
        Matrix4x4 trs = Matrix4x4.TRS(Position, Orientation, Vector3.one);
        for(int i=0; i<8; i++)
        {
            // Vector3.Scaleでサイズを適用してから回転・移動
            vertices[i] = trs.MultiplyPoint3x4(Vector3.Scale(corners[i], HalfSize));
        }
        return vertices;
    }
    // 力を加えるメソッド (重心以外の点に加えると回転が生じる)
    public void AddForceAtPoint(Vector3 force, Vector3 point)
    {
        _forceAccumulator += force;

        // 重心(Position)から力点(point)へのベクトル r
        Vector3 r = point - Position;

        // トルク τ = r × F (外積)
        _torqueAccumulator += Vector3.Cross(r, force);
    }

    // 積分ステップ (物理世界の時間を進める)
    public void Integrate(float dt)
    {
        if (IsStatic) return;
        // 1. 並進運動の更新 (F = ma)
        // 加速度 a = F / m
        Vector3 acceleration = _forceAccumulator / Mass;
        LinearVelocity += acceleration * dt;
        Position += LinearVelocity * dt;
        // ★追加: 空気抵抗のような減衰
        // 毎フレーム少しだけ速度を落とす
        LinearVelocity *= Mathf.Pow(LinearDamping, dt * 60.0f);
        // 2. 回転運動の更新 (τ = I * dω/dt)
        // ここが難しいポイント：慣性テンソルは「回っている物体」と一緒に回るため、
        // World座標系での慣性テンソル逆行列を毎フレーム計算し直す必要がある。
        // I_world^-1 = R * I_local^-1 * R^T (Rは回転行列)
        
        Matrix4x4 R = Matrix4x4.Rotate(Orientation); // クォータニオンから回転行列へ
        Matrix4x4 InverseInertiaTensorWorld = R * InverseInertiaTensorLocal * R.transpose;

        // 角加速度 α = I^-1 * τ
        // Matrix4x4.MultiplyVector で行列とベクトルの掛け算
        Vector3 angularAcceleration = InverseInertiaTensorWorld.MultiplyVector(_torqueAccumulator);
        
        AngularVelocity += angularAcceleration * dt;

        // ドラッグ（空気抵抗的な減衰）を入れないと無限に回転し続けるので少し入れる
        AngularVelocity *= AngularDamping; 

        // 3. 姿勢(クォータニオン)の更新
        // dq/dt = 0.5 * ω * q
        // 小さい回転を加算する処理
        Quaternion deltaQ = new Quaternion(
            AngularVelocity.x * dt * 0.5f,
            AngularVelocity.y * dt * 0.5f,
            AngularVelocity.z * dt * 0.5f,
            0 // w成分は近似的に0で扱う（微小回転のため）
        );
        
        // クォータニオンの掛け算で回転を合成
        // 注意: 単純な足し算ではなく、今の姿勢に微小回転を掛ける
        // ここでは実装簡易化のため、Unityの内部計算に近い形「q_new = q + (0.5 * w * q * dt)」を簡略化した以下を採用
        
        // ωをクォータニオン化して合成する一般的な手法
        // q_new = q_old + (dq/dt * dt)
        Quaternion qDot = deltaQ * Orientation; 
        
        Orientation.x += qDot.x;
        Orientation.y += qDot.y;
        Orientation.z += qDot.z;
        Orientation.w += qDot.w;

        // 正規化（大きさを1に保たないと計算誤差で形が歪む）
        Orientation.Normalize();

        // 4. 力をリセット
        _forceAccumulator = Vector3.zero;
        _torqueAccumulator = Vector3.zero;
    }

    // ワールド空間での逆慣性テンソルを取得するヘルパー
    public Matrix4x4 GetInverseInertiaTensorWorld()
    {
        if (IsStatic) return Matrix4x4.zero;
        Matrix4x4 R = Matrix4x4.Rotate(Orientation);
        return R * InverseInertiaTensorLocal * R.transpose;
    }
}
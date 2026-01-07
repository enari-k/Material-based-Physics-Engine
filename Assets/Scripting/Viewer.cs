using UnityEngine;
using System.Collections.Generic;

public class BoxSimulator : MonoBehaviour
{
    public GameObject CubePrefab;
    
    // 物理剛体と見た目のGameObjectをペアで管理
    private List<MyRigidBody> bodies = new List<MyRigidBody>();
    private List<GameObject> visuals = new List<GameObject>();

    // シミュレーション設定
    [Range(1, 20)] public int SolverIterations = 20; // 積み上げ安定のための反復回数

    void Start()
    {
        // ------------------------------
        // 1. 床 (Floor) を作成
        // ------------------------------
        Vector3 floorPos = new Vector3(0, -2, 0);
        Vector3 floorSize = new Vector3(20, 1, 20);
        
        // 質量無限大(float.MaxValue)で作成し、IsStaticフラグを立てる
        var floorBody = new MyRigidBody(floorPos, floorSize, float.MaxValue, 0.5f);
        floorBody.IsStatic = true; 
        floorBody.Orientation = Quaternion.identity;
        bodies.Add(floorBody);

        // 床の見た目も作る
        var floorGo = Instantiate(CubePrefab, floorPos, Quaternion.identity);
        floorGo.transform.localScale = floorSize;
        floorGo.GetComponent<Renderer>().material.color = Color.gray; // 色を変えておく
        visuals.Add(floorGo);

        // ------------------------------
        // 2. 積み上げる箱を作成
        // ------------------------------
        // タワーのように縦に積む
        for (int i = 0; i < 5; i++)
        {
            // Y座標を少しずつ上げていく (サイズ1.0の箱なので、少し余裕を持って1.2ずつ上げる)
            // X, Zをわずかにずらすと、崩れる挙動が確認できて面白いです
            float jitter = 0.05f; 
            float randomX = Random.Range(-jitter, jitter);
            float randomZ = Random.Range(-jitter, jitter);

            Vector3 pos = new Vector3(randomX, -1.0f + (i * 1.2f), randomZ); 
            CreateBox(pos, Vector3.zero, Vector3.zero);
        }
    }

    MyRigidBody CreateBox(Vector3 pos, Vector3 vel, Vector3 angularVel)
    {
        Vector3 size = Vector3.one; // 1x1x1の箱
        float mass = 1.0f;
        float friction = 0.5f;

        MyRigidBody rb = new MyRigidBody(pos, size, mass, friction);
        rb.LinearVelocity = vel;
        rb.AngularVelocity = angularVel;
        
        bodies.Add(rb);

        // 見た目の生成
        var go = Instantiate(CubePrefab, pos, Quaternion.identity);
        go.transform.localScale = size;
        // 分かりやすく色をランダムに
        go.GetComponent<Renderer>().material.color = Random.ColorHSV();
        visuals.Add(go);

        return rb;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // ------------------------------
        // 1. 力を加える (重力)
        // ------------------------------
        foreach (var b in bodies)
        {
            // 固定物体（床）には重力をかけない
            if (b.IsStatic) continue;

            // 重力 F = mg (下向き)
            Vector3 gravityForce = new Vector3(0, -9.81f * b.Mass, 0);
            
            // ★重要：力は「重心(b.Position)」に加えること！
            // Vector3.zeroに加えると、原点からのモーメントで箱が猛回転してしまいます。
            b.AddForceAtPoint(gravityForce, b.Position);
        }

        // ------------------------------
        // 2. 衝突判定と解決 (Solver Loop)
        // ------------------------------
        // 積み上げを安定させるため、解決プロセスを複数回繰り返します。
        // これにより、「下の箱が押し出された影響」を「上の箱」に伝えることができます。
        for (int k = 0; k < SolverIterations; k++)
        {
            // 全ペア総当たり判定
            for (int i = 0; i < bodies.Count; i++)
            {
                for (int j = i + 1; j < bodies.Count; j++)
                {
                    // 以前作成したマニフォールド対応版のResolverを呼ぶ
                    BoxCollisionResolver.ResolveCollision(bodies[i], bodies[j]);
                }
            }
        }

        // ------------------------------
        // 3. 積分 (位置・回転の更新)
        // ------------------------------
        foreach (var b in bodies)
        {
            b.Integrate(dt);
        }

        // ------------------------------
        // 4. 描画同期
        // ------------------------------
        for(int i=0; i<bodies.Count; i++)
        {
            // bodiesとvisualsのインデックスは一致させている前提
            visuals[i].transform.position = bodies[i].Position;
            visuals[i].transform.rotation = bodies[i].Orientation;
        }
    }
}
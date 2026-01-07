using System.Reflection;

public class Rigid
{
    public float Posx;
    public float Posy;
    public float Posz;
    public float qua0;
    public float quax;
    public float quay;
    public float quaz;
    public float linevelx;
    public float linevely;
    public float linevelz;
    public float angvelx;
    public float angvely;
    public float angvelz;
    public float Friction;
    public float Mass = 1.0f;
    public float Restitution = 0.2f;
    public float halfx = 0.5f;
    public float halfy = 0.5f;
    public float halfz = 0.5f;
    public float Invx;
    public float Invy;
    public float Invz;
    public float force;
    public float torqu;
    public Rigid(float x,float y,float z,float sizex,float sizey,float sizez,float mass,float friction)
    {
        Posx = x;
        Posy = y;
        Posz = z;
        halfx = sizex * 0.5f;
        halfy = sizey * 0.5f;
        halfz = sizez * 0.5f;
        Mass = mass;
        Friction = friction;
        float w2 = sizex * sizex;
        float h2 = sizey * sizey;
        float d2 = sizez * sizez;
        float Ix = mass / 12.0f * (h2 + d2);
        float Iy = mass / 12.0f * (w2 + d2);
        float Iz = mass / 12.0f * (w2 + h2);
        Invx = 1.0f / Ix;
        Invy = 1.0f / Iy;
        Invz = 1.0f / Iz;
    }
    
}
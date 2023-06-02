#include <stdlib.h>
#include "color.h"
#include "image.h"
#include "image_io.h"
#include "vec.h"
#include "materials.h"
#include "mesh_io.h"

// Point 3D : appartient au triangle
bool appartient_tetraedre(const Vector &d, const Point &o, const Point &a, const Point &b,const Point &c ){
    Vector norm1 = normalize( cross( Vector(a, b), Vector(o, a) ) );
    Vector norm2 = normalize( cross( Vector(b, c), Vector(o, b) ) );
    Vector norm3 = normalize( cross( Vector(c, a), Vector(o, c) ) );

    float s1 = dot(d, norm1);
    float s2 = dot(d, norm2);
    float s3 = dot(d, norm3);

    if( s1 >= 0 && s2 >= 0 && s3 >= 0) {
        return true;
    }

    return false;
}

Vector interpole(const Point &pp, const Point &o, const Point &p0, const Point &p1, const Point &p2) {
    
    /*std::vector<Vector> p;
    p.push_back(Vector(o, p0));
    p.push_back(Vector(o, p1));
    p.push_back(Vector(o, p2));*/
    Vector P[3] = {
        Vector(o, p0),
        Vector(o, p1),
        Vector(o, p2)};

    /*std::vector<Vector> n;
    std::vector<float> V;*/
    Vector N[3] = {};
    float V[3] = {};
    Vector d(o, pp);

    Vector lambda;
    /*
    for(int i = 0; i < 3; i++) {
        n.push_back(cross(p[(i+2)%3], p[(i+1)%3]));
        V.push_back(dot( n[i], d));
    } */
    for (int i = 0; i < 3; i++)
    {
        N[i] = cross(P[(i + 2) % 3], P[(i + 1) % 3]);
        V[i] = dot(N[i], d);
    }

    lambda.x = V[0] / ( V[0] + V[1] + V[2] );
    lambda.y = V[1] / ( V[0] + V[1] + V[2] );
    lambda.z = V[2] / ( V[0] + V[1] + V[2] );

    return lambda;
}

Point point_dans_lespace(const Vector &lambda, const Point &a, const Point &b, const Point &c) {
    return Point(a*lambda.x + b*lambda.y + c*lambda.z);
}

void color_zbuff(const int &px, const int &py, float &zp, Image &zbuff, Image &im, const Color&c) {
    if(zbuff(px, py).r < zp) {
        zbuff(px, py) = Color(zp);
        im(px, py) = c;
    }
}

int main() {
    // cree l'image resultat
    Image image(1024, 1024);
    //Image image(512, 512);

    //Triangle ABC
    Point a(-1, -1, -2);
    Point b(1, -1, -2);
    Point c(1, 1, -2);

    // Triangle DEF
    Point d(-8, 2, -5);
    Point e(-8, -1, -5);
    Point f(1, -1, -5);

    // Triangle GHI
    Point g(0, 20, -10);
    Point h(-20, -20, -10);
    Point i(20, -20, -10);

    // Part 2 : question 1
    /*for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // rayon centre camera
        float x = float(px)/image.width() * 2 - 1;
        float y = float(py)/image.height() * 2 - 1;
        float z = -1;       
        Point p = Point(x, y, z);

        Point o = Point(0, 0, 0);
        Vector d = Vector(o, p);    // ou Vector d= p - o; // si vous preferrez...


        // triangle
        if(appartient_tetraedre(d, o, a, b, c)) {   
            image(px, py) = Red();
        }
        //Vector n= normalize( cross( Vector(a, b), Vector(a, c) ) );
        else{
            image(px, py)= White();
        }
        
    }*/

    // Part 2 question 3
    /*Color ca = Red();
    Color cb = Blue();
    Color cc = Green();


    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // rayon centre camera
        float x = float(px)/image.width() * 2 - 1;
        float y = float(py)/image.height() * 2 - 1;
        float z = -1;       
        Point p = Point(x, y, z);

        Point o = Point(0, 0, 0);
        Vector d = Vector(o, p);    // ou Vector d= p - o; // si vous preferrez...


        // triangle
        if(appartient_tetraedre(d, o, a, b, c)) { 
            Vector lambda = interpole(p, o, a, b, c);  
            image(px, py) = lambda.x * ca + lambda.y * cb + lambda.z * cc;
            Point pe = point_dans_lespace(lambda, a, b, c);
            float z = pe.z;
        }
        //Vector n= normalize( cross( Vector(a, b), Vector(a, c) ) );
        else{
            image(px, py)= Color(0.5, 0.5, 0.5);
        }
        
    }*/

    

    // Test du zbuffer avec 3 triangles manuellement
    Image zbuff(image.width(), image.height(), Color(-1000));  

    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // rayon centre camera
        float x = float(px)/image.width() * 2 - 1;
        float y = float(py)/image.height() * 2 - 1;
        float z = -1;       
        Point p = Point(x, y, z);

        Point o = Point(0, 0, 0);
        Vector dist = Vector(o, p);    // ou Vector d= p - o; // si vous preferrez...


        // triangle
        if(appartient_tetraedre(dist, o, a, b, c) || appartient_tetraedre(dist, o, d, e, f) || appartient_tetraedre(dist, o, g, h, i)) {
            if(appartient_tetraedre(dist, o, a, b, c)) { 
                Vector lambda = interpole(p, o, a, b, c);  
                Point pe = point_dans_lespace(lambda, a, b, c);
                float z = pe.z;
                color_zbuff(px, py, z, zbuff, image, Red());
            }
            if(appartient_tetraedre(dist, o, d, e, f)) { 
                Vector lambda = interpole(p, o, d, e, f);  
                Point pe = point_dans_lespace(lambda, d, e, f);
                float z = pe.z;
                color_zbuff(px, py, z, zbuff, image, Green());
            }
            if(appartient_tetraedre(dist, o, g, h, i)) { 
                Vector lambda = interpole(p, o, g, h, i);  
                Point pe = point_dans_lespace(lambda, g, h, i);
                float z = pe.z;
                color_zbuff(px, py, z, zbuff, image, Blue());
            }
        }
        //Vector n= normalize( cross( Vector(a, b), Vector(a, c) ) );
        else{
            image(px, py)= Color(0.5, 0.5, 0.5);
        }
        
    }
    write_image(image, "triangles_zbuff.png"); // par defaut en .png
}

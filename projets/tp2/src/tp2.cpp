#include <stdlib.h>
#include "color.h"
#include "image.h"
#include "image_io.h"
#include "vec.h"
#include "materials.h"
#include "mesh_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <fstream>


struct Hit
{
    float t;
    float lambdax, lambday, lambdaz;
    int triangle_id;    // indice du triangle, pour retrouver la matiere associee au triangle, par exemple
     
    Point p;            // p= o + td
    Vector n;           // normale du triangle
};


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

// PARTIE 3
// Retourne vrai si il y a un point d'intersection entre le point origine du rayon et le plan du triangle faux sinon.
// Si il y a intersection entre ces point, on change la valeur du point t : coordonnées de l'intersection.
bool intersection_plan(const Point orayon, const Vector drayon, const Point &a, const Point &b, const Point &c, Point & t, float &tcoef) {
    Vector normPlan = normalize(cross( Vector(a,b), Vector(a,c) ));
    tcoef = dot(-normPlan,Vector(a, orayon)) / dot(normPlan, drayon);
    //std::cout << "tcoef = " << tcoef << std::endl;
    if(tcoef > 0) {
        t = orayon + (tcoef * drayon);
        return true;
    }
    else {
        return false;
    }
}

bool intersection_tetraedre(const Point orayon, const Vector drayon, const Point &a, const Point &b, const Point &c, Point & t, float &tcoef) {
    if(intersection_plan(orayon, drayon, a, b, c, t, tcoef)) {
        return appartient_tetraedre(drayon, orayon, a, b, c);
    }
    return false;
}

bool obstrue_lumiere(const Point oSoleil, Vector dirPtSoleil, const std::vector<Point>& positions) {
    bool collision = false;
    float tcoef;
    Point p;
    for(int i = 0; i < positions.size(); i+= 3) {
        // On ajoute un epsilon de 0.01 aux bornes de tcoef pour que le point ne se détecte pas lui-même en tant que collision
        if(intersection_tetraedre(oSoleil, dirPtSoleil, positions[i], positions[i+1], positions[i+2], p, tcoef) && tcoef > 0+0.0001 && tcoef < 1-0.0001) {
            collision = true;
        }
    }
    return collision;
}

// PARTIE 4
Hit intersect( const Point& o, const Vector& d, const std::vector<Point>& positions ) {
    Hit result;
    result.triangle_id = -1;
    Point pt;
    float tcoef;
    float tcoefmin = 1000000.0;
    // Pour savoir quel est le bon Hit on stoke le plus petit tcoef et si on en trouve un minimum on actualise le hit result
    for(int i=0; i < positions.size(); i+= 3) {
        if(intersection_tetraedre(o, d, positions[i], positions[i+1], positions[i+2], pt, tcoef) && tcoef < tcoefmin && tcoef >= 0) {
            tcoefmin = tcoef;
            result.p = pt;
            result.t = tcoef;
            //premier point du triangle
            result.triangle_id = i/3;
            result.n = cross(Vector(positions[i], positions[i+1]), Vector(positions[i], positions[i+2]));
            Vector lambda = interpole(result.p, o, positions[i], positions[i+1], positions[i+2]);
            result.lambdax = lambda.x;
            result.lambday = lambda.y;
            result.lambdaz = lambda.z;
        }
    }

    return result;
}


void color_zbuff(const int &px, const int &py, float &zp, Image &zbuff, Image &im, const Color&c) {
    if(zbuff(px, py).r < zp) {
        zbuff(px, py) = Color(zp);
        im(px, py) = c;
    }
}


int main(int argc, char **argv)
{
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


    // On utilise l'exécutable avec des arguments
    if(argc >=2) {
    // TRIANGLES AVEC ZBUFFER
        if(!strcmp(argv[1], "triangles")) {
            std::cout << "AFFICHAGE TRIANGLES" << std::endl;
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
            write_image(image, "projets/tp2/img/triangles_zbuff.png"); // par defaut en .png
        }

    // ROBOT EN 3D
        else if(!strcmp(argv[1], "robot")) {

            const char *filename= "data/robot.obj";
            std::vector<Point> positions;
            if(!read_positions(filename, positions))
                //return "erreur";       
                return 1;
            printf("%d triangles\n", int(positions.size() / 3));

            // Récupérer les materials et leurs indices
            Materials materials;
            std::vector<int> material_indices;   // indices des matieres
            if(!read_materials(filename, materials, material_indices))
                //return "erreur";       
                return 1;

            std::cout << "taille positions : " << positions.size() << std::endl;
            std::cout << "taille material_indices : " << material_indices.size() << std::endl;
            for(int y = 0; y < materials.count() ; y++)
                std::cout << "material de " << y << " :  " << materials(y).diffuse.r << " " << materials(y).diffuse.g << " " << materials(y).diffuse.b << " " << std::endl;
        
            // deplace tous les sommets devant la camera
            for(unsigned i= 0; i < positions.size(); i++)
                positions[i]= positions[i] + Vector(0, -2, -4);     // a ajuster en fonction de l'objet...
        
            // englobant des points, verifier qu'ils sont bien devant la camera...
            Point pmin= positions[0];
            Point pmax= positions[0];
            for(unsigned i= 1; i < positions.size(); i++)
            {
                pmin= min(pmin, positions[i]);
                pmax= max(pmax, positions[i]);
            }
            printf("bounds [%f %f %f]x[%f %f %f]\n", pmin.x, pmin.y, pmin.z, pmax.x, pmax.y, pmax.z);


        /// ROBOT : LANCER DE RAYONS
            if((argc == 3) && !strcmp(argv[2], "rayons")){
               
                // POUR PARCOURIR TOUS LES PIXELS DE L'IMAGE
                for(int py= 0; py < image.height(); py++)
                for(int px= 0; px < image.width(); px++)
                {
                    // Le pixel est noir par defaut
                    //image(px, py)= Black();
                    image(px, py)= Color(0.5, 0.5, 0.5);

                    // rayon centre camera
                    float x = float(px)/image.width() * 2 - 1;
                    float y = float(py)/image.height() * 2 - 1;
                    float z = -1;       
                    Point p = Point(x, y, z);

                    Point o = Point(0, 0, 0);
                    Vector dir = Vector(o, p);    // ou Vector d= p - o; // si vous preferrez...

                    // DEFINITION D'UN RAYON LUMINEUX : SOLEIL. Si le triangle est éclairé par la lumière, s couleur est normale, sinon noire.
                    Point origineSoleil(0.0, 0.0, 10.0);                     
                    

                    // cherche une collision avec le rayon origine-camera et un triangle
                    Hit intersection = intersect(o, dir, positions);
                    if(intersection.triangle_id != -1) {

                        int material_id= material_indices[ intersection.triangle_id ];
                        Material& material= materials( material_id );

                        // Calcul intersection entre le points sur le triangle et la lumière
                        Vector dirSoleil(origineSoleil, intersection.p);
                        Color col;
                        if(obstrue_lumiere(origineSoleil, dirSoleil, positions)) {
                            col = Black();
                        }
                        else {
                            col = material.diffuse * dot(normalize(intersection.n), normalize(Vector(intersection.p, o)));
                        }
                        
                        col = Color(col, 1);
                        image(px, py) = col;
                    }
                        
                }
                write_image(image, "projets/tp2/img/robot_rayons.png"); // par defaut en .png
            }
        /// ROBOT : ZBUFFER
            else {
                Image zbuff(image.width(), image.height(), Color(-1000));  

                // POUR PARCOURIR TOUS LES PIXELS DE L'IMAGE
                for(int py= 0; py < image.height(); py++)
                for(int px= 0; px < image.width(); px++)
                {
                    // Le pixel est noir par defaut
                    image(px, py)= Black();//Color(0.5, 0.5, 0.5);

                    // rayon centre camera
                    float x = float(px)/image.width() * 2 - 1;
                    float y = float(py)/image.height() * 2 - 1;
                    float z = -1;       
                    Point p = Point(x, y, z);

                    Point o = Point(0, 0, 0);
                    Vector dist = Vector(o, p);    // ou Vector d= p - o; // si vous preferrez...

                    // parcours tous les triangles
                    for(unsigned i= 0; i +2 < positions.size(); i+= 3)
                    {
                        Point t[3]= {
                            positions[ i    ],
                            positions[ i +1 ],
                            positions[ i +2 ]
                        };

                        //std::cout << t[0].x << t[0].y << t[0].z << std::endl;

                        Vector normal = normalize( cross( Vector(t[0], t[1]), Vector(t[0], t[2]) ) );
                        if(normal.z >= 0){
                            
                            // triangle
                            if(appartient_tetraedre(dist, o, t[0], t[1], t[2])) {
                                Vector lambda = interpole(p, o, t[0], t[1], t[2]);  
                                Point pe = point_dans_lespace(lambda, t[0], t[1], t[2]);
                                float z = pe.z;
                                Color col = Color(abs(normal.x), abs(normal.y), abs(normal.z), 1.0);
                                color_zbuff(px, py, z, zbuff, image, col);
                            }

                        }
                    }        
                }
                write_image(image, "projets/tp2/img/robot_zbuffer.png"); // par defaut en .png
            }
            
        }

    // GEOMETRY : SCENE AVEC LUMIERE DIFFUSE ET OMBRES
        else if(!strcmp(argv[1], "geometry")) {

            const char *filename= "data/geometry.obj";

            std::vector<Point> positions;
            if(!read_positions(filename, positions))
                //return "erreur";       
                return 1;
            printf("%d triangles\n", int(positions.size() / 3));


            // Récupérer les materials et leurs indices
            Materials materials;
            std::vector<int> material_indices;   // indices des matieres
            if(!read_materials(filename, materials, material_indices))
                //return "erreur";       
                return 1;

            std::cout << "taille positions : " << positions.size() << std::endl;
            std::cout << "taille material_indices : " << material_indices.size() << std::endl;
            for(int y = 0; y < materials.count() ; y++)
                std::cout << "material de " << y << " :  " << materials(y).diffuse.r << " " << materials(y).diffuse.g << " " << materials(y).diffuse.b << " " << std::endl;

        
            // deplace tous les sommets devant la camera
            for(unsigned i= 0; i < positions.size(); i++)
                //positions[i]= positions[i] + Vector(0, -2, -4);     // robot...
                positions[i]= positions[i] + Vector(0, -110, -550);     // geometry...
        
            // englobant des points, verifier qu'ils sont bien devant la camera...
            Point pmin= positions[0];
            Point pmax= positions[0];
            for(unsigned i= 1; i < positions.size(); i++)
            {
                pmin= min(pmin, positions[i]);
                pmax= max(pmax, positions[i]);
            }
            printf("bounds [%f %f %f]x[%f %f %f]\n", pmin.x, pmin.y, pmin.z, pmax.x, pmax.y, pmax.z);    
            

            // POUR PARCOURIR TOUS LES PIXELS DE L'IMAGE
            for(int py= 0; py < image.height(); py++)
            for(int px= 0; px < image.width(); px++)
            {

                // Le pixel est noir par defaut
                //image(px, py)= Black();
                image(px, py)= Color(0.5, 0.5, 0.5);

                // rayon centre camera
                float x = float(px)/image.width() * 2 - 1;
                float y = float(py)/image.height() * 2 - 1;
                float z = -1;       
                Point p = Point(x, y, z);

                Point o = Point(0, 0, 0);
                Vector dir = Vector(o, p);    // ou Vector d= p - o; // si vous preferrez...

                // DEFINITION D'UN RAYON LUMINEUX : SOLEIL. Si le triangle est éclairé par la lumière, s couleur est normale, sinon noire.
                Point origineSoleil(-100.0, 10.0, -200.0);
                if(argc == 3 && !strcmp(argv[2], "source2"))
                    origineSoleil = Point(100.0, 10.0, -150.0);
                else if(argc == 3 && !strcmp(argv[2], "source3"))
                    origineSoleil = Point(0.0, 10.0, -600.0);
                

                // cherche une collision avec le rayon origine-camera et un triangle
                Hit intersection = intersect(o, dir, positions);
                if(intersection.triangle_id != -1) {

                    int material_id= material_indices[ intersection.triangle_id ];
                    Material& material= materials( material_id );

                    // Calcul intersection entre le points sur le triangle et la lumière
                    Vector dirSoleil(origineSoleil, intersection.p);
                    Color col;
                    if(obstrue_lumiere(origineSoleil, dirSoleil, positions)) {
                        col = Black();
                    }
                    else {
                        col = material.diffuse * dot(normalize(intersection.n), normalize(Vector(intersection.p, o)));
                    }
                    
                    col = Color(col, 1);
                    image(px, py) = col;
                }
                    
            }
            write_image(image, "projets/tp2/img/geometry.png"); // par defaut en .png
        }   

    }

    // On utilise pas d'arguments pour lancer l'executable
    else {
        std::cout << "* Premier paramètre : ./bin/tp2" << std::endl;
        std::cout << "* Troisième paramètre : 'triangles' ou 'robot' ou 'geometry'" << std::endl;
        std::cout << "     - triangles :" << std::endl;
        
        std::cout << "     - robot :" << std::endl;
        std::cout << "          * Quatrième paramètre : 'zbuffer' ou 'rayons'" << std::endl;
        std::cout << "               - zbuffer :" << std::endl;
        std::cout << "               - rayons" << std::endl;
        std::cout << "     - geometry :" << std::endl;
        std::cout << "          * Quatrième paramètre : 'source1' ou 'source2' ou 'source3' " << std::endl;
        std::cout << "               - source1 :" << std::endl;
        std::cout << "               - source2 :" << std::endl;
        std::cout << "               - source3 :" << std::endl;


    }

    return 0;
}
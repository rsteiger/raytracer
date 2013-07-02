#include <cmath>
#include <algorithm>

template<typename Vec>
class VecComp {
   int index;
   public:
   VecComp(int i) : index(i) { }
   bool operator()(const Vec &a, const Vec &b) const {
      return a[index] < b[index];
   }
};

template <typename Vec3>
void KDSort(Vec3 *vecs, int numVecs) {
   if (numVecs <= 2)
      return;

   int v = numVecs - 1, split = 1;
   while (v >>= 1) split <<= 1;

   // Choose an axis
   Vec3 average(0,0,0), stdDev(0,0,0);
   for (int i = 0; i < numVecs; i++) { average = average + vecs[i] / numVecs; }
   for (int i = 0; i < numVecs; i++) { stdDev = stdDev + (vecs[i] - average).abs() / numVecs; }
   int axis = std::max_element(&stdDev[0],&stdDev[3]) - &stdDev[0];
   VecComp<Vec3> cmp(axis);

   // Sort on that axis
   std::nth_element(&vecs[0], &vecs[split], &vecs[numVecs], cmp);
   KDSort(&vecs[0], split);
   KDSort(&vecs[split], numVecs - split);
};


template <typename Vec3, typename precision>
struct BVH {
   struct Sphere {
      Vec3 position;
      precision radius; 
      static Sphere enclosingSphere(const Sphere &s1, const Sphere &s2) {
         // Check to see if one sphere completely surrounds the other
         float difference = (s2.position - s1.position).distance();
         if (difference + s2.radius <= s1.radius) {
            return Sphere(s1);
         }
         if (difference + s1.radius <= s2.radius) {
            return Sphere(s2);
         } 

         // Get the midpoint of their outermost edges
         Vec3 direction = (s2.position - s1.position).normalize();
         Vec3 rightEdge = s2.position + direction * s2.radius;
         Vec3 leftEdge = s1.position - direction * s1.radius;
         
         Sphere enclosing;
         enclosing.position = (rightEdge + leftEdge) / (precision)2; 
         enclosing.radius = (rightEdge - leftEdge).distance() / (precision)2;
         return enclosing;
      }

      static Sphere makeSphere(const Vec3 &position, const precision radius) {
         Sphere s;
         s.position = position;
         s.radius = radius;
         return s;
      }

      static Sphere minimumBoundingSphere(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) {
         Sphere s;
         // Calculate relative distances
         precision A = (p1 - p2).distance();
         precision B = (p2 - p3).distance();
         precision C = (p3 - p1).distance();
         const Vec3 *a = &p3, *b = &p1, *c = &p2;
         // Re-orient triangle
         if (A < B) swap(A, B), swap(a, b);
         if (B < C) swap(B, C), swap(b, c);
         if (A < B) swap(A, B), swap(a, b);
         // Check if obtuse
         if ((B*B) + (C*C) <= (A*A)) {
            // Use the longest diameter
            s.radius = A / (precision)2;
            s.position = (*b + *c) / (precision)2;
         } else {
            // Circumscribe
            precision cos_a = (B*B + C*C - A*A) / (B*C*2);
            s.radius = A / (sqrt(1 - cos_a*cos_a)*2);
            Vec3 alpha = *a - *c, beta = *b - *c;
            s.position = (beta * alpha.dot(alpha) - alpha * beta.dot(beta)).cross(alpha.cross(beta)) /
             (alpha.cross(beta).dot(alpha.cross(beta)) * (precision)2) + *c; 
         }
         return s;
      }
   };

   struct Node {
      bool isLeaf;
      Sphere sphere;
   };

   // The tree for our BVH
   Node *mNodes;
   int treeSize;

   BVH(Sphere *spheres, int numSpheres) {
      // Allocate space for the BVH
      int treeDepth = 1, s = numSpheres;
      while (s >>= 1) treeDepth++;
      treeSize = (2 << treeDepth);
      mNodes = new Node[treeSize];

      // Copy the spheres into the BVH
      int firstNode = (1 << treeDepth);
      for (int i = 0; i < firstNode; i++) {
         if (i < numSpheres)
            mNodes[firstNode + i].sphere = spheres[i];
         else {
            mNodes[firstNode + i].sphere.position = spheres[numSpheres - 1].position;
            mNodes[firstNode + i].sphere.radius = 0;
         }
         mNodes[firstNode + i].isLeaf = true;
        
         // Create parent sphere
         int node = firstNode + i;
         while (node % 2) {
            Node *lc = &mNodes[node - 1];
            Node *rc = &mNodes[node];
            mNodes[node >> 1].sphere = Sphere::enclosingSphere(lc->sphere, rc->sphere);
            mNodes[node >> 1].isLeaf = false;
            node >>= 1;
         }
      }
   }
};


/**
 * Testing stuff
 /
void test1();
void test2();
void test3();
void test4();
void test5();

int main(int argc, char **argv) {
   test1(); 
   test2();
   test3();
   test4();
   test5();
}


/***********
 * TEST BOILERPLATE
 /
#include "data.h"
#include <iostream>
#include <vector>
#include <fstream>
using namespace std;


void testEnclosingSphere(const vec3 &p1, float r1, const vec3 &p2, float r2, const vec3 &p3, float r3) {
   BVH<vec3, float>::Sphere sphere;
   sphere.position = p1;
   sphere.radius = r1;

   BVH<vec3, float>::Sphere sphere2;
   sphere2.position = p2;
   sphere2.radius = r2;

   BVH<vec3, float>::Sphere sphere3 = BVH<vec3, float>::Sphere::enclosingSphere(sphere, sphere2);
   cout << "position: " << sphere3.position.x << ", " << sphere3.position.y << ", " << sphere3.position.z
        << " (should be: " << p3.x << ", " << p3.y << ", " << p3.z << ")" << endl;
   cout << "radius: " << sphere3.radius << " (should be: " << r3 << ")" << endl;
}

void testMinimumBoundingSphere(const vec3 &p1, const vec3 &p2, const vec3 &p3, const vec3 &p, float r) {
   BVH<vec3, float>::Sphere sphere3 = BVH<vec3, float>::Sphere::minimumBoundingSphere(p1, p2, p3);
   cout << "position: " << sphere3.position.x << ", " << sphere3.position.y << ", " << sphere3.position.z
        << " (should be: " << p.x << ", " << p.y << ", " << p.z << ")" << endl;
   cout << "radius: " << sphere3.radius << " (should be: " << r << ")" << endl;
}
ostream &operator<<(ostream &os, const BVH<vec3, float>::Sphere &s) {
   return os << s.position.x << ", " << s.position.y << ", " << s.position.z << " (" << s.radius << ")";
}
ostream &operator<<(ostream &os, const vec3 &v) {
   return os << v.x << ", " << v.y << ", " << v.z;
}
void printBVH(BVH<vec3, float> bvh, int startingNode) {
   int s = startingNode;
   while (s >>= 1) cout << "   "; 
   cout << bvh.mNodes[startingNode].sphere << endl;
   if (!bvh.mNodes[startingNode].isLeaf) {
      printBVH(bvh, startingNode << 1);
      printBVH(bvh, (startingNode << 1) + 1); 
   }
}

float rf() {
   return (float)rand()/(float)RAND_MAX;
}

void randomSpheres(BVH<vec3, float>::Sphere *s, int numSpheres) {
   for (int i = 0; i < numSpheres; i++) {
      s[i].position = vec3(10 * rf(), 10 * rf(), 0);
      s[i].radius = rf() + 1;
   }
}

/***********
 * Actual tests
 /
// test that the enclosingSphere method does what we expect
void test1() {
   testEnclosingSphere(vec3(0,1,0),1,vec3(0,3,0),1,vec3(0,2,0),2);
   testEnclosingSphere(vec3(0,0,0),1,vec3(0,0,0),2,vec3(0,0,0),2);
   testEnclosingSphere(vec3(0,0,0),2,vec3(0,0,0),2,vec3(0,0,0),2);
}

// test that the bvh works
void test2() {
   char c;
   BVH<vec3, float>::Sphere v;
   v.radius = 0.01f;
   vector<BVH<vec3, float>::Sphere> spheres;
   ifstream iss("bunny.obj"); 
   iss >> c;
   while (c == 'v') {
      iss >> v.position.x >> v.position.y >> v.position.z;
      spheres.push_back(v);
      iss >> c;
   }

   BVH<vec3, float> bvh(&spheres[0], spheres.size());
   //printBVH(bvh, 1);
   float averageSize = 0;
   for (int i = 0; i < (bvh.treeSize/2); i++) {
      averageSize += bvh.mNodes[i].sphere.radius / (bvh.treeSize/2);
   }
   cout << "average sphere size before sort: " << averageSize << endl;
}

// test that minimumBoundingSphere for triangle works
void test3() {
   testMinimumBoundingSphere(vec3(0,0,0), vec3(0,0,0), vec3(0,0,0), vec3(0,0,0), 0);
   testMinimumBoundingSphere(vec3(0,0,0), vec3(0,8,0), vec3(6,0,0), vec3(3,4,0), 5);
   testMinimumBoundingSphere(vec3(0,0,-2), vec3(0,0,2), vec3(0,1,0), vec3(0,0,0), 2);
   testMinimumBoundingSphere(vec3(0,0,0), vec3(0,0,6), vec3(0,8,3), vec3(0,3.5,3), 4.5);
}

// test the KDSort
void test4() {
   vec3 points1[7] = { vec3(1,-2,0), vec3(1,-2, 1), vec3(-1,-2,0), vec3(-1,-2,1), vec3(-1,2,0), vec3(-1,2,1), vec3(1,2,0) };
   KDSort(&points1[0], 7);
   for (int i = 0; i < 7; i++) { cout << points1[i] << endl; }
}

// test that the bvh works
void test5() {
   char c;
   vec3 v;
   vector<vec3> points;
   ifstream iss("bunny.obj"); 
   iss >> c;
   while (c == 'v') {
      iss >> v.x >> v.y >> v.z;
      points.push_back(v);
      iss >> c;
   }
   KDSort(&points[0], points.size());

   BVH<vec3, float>::Sphere s;
   s.radius = 0.01f;
   vector<BVH<vec3, float>::Sphere> spheres;
   for(vector<vec3>::iterator it = points.begin(); it != points.end(); it++) {
      s.position = *it;
      spheres.push_back(s);
   }

   BVH<vec3, float> bvh(&spheres[0], spheres.size());
   //printBVH(bvh, 1);
   float averageSize = 0;
   for (int i = 0; i < (bvh.treeSize/2); i++) {
      averageSize += bvh.mNodes[i].sphere.radius / (bvh.treeSize/2);
   }
   cout << "average sphere size after sort: " << averageSize << endl;
}

*/

//kernelPBO.cu (Rob Farber)
#include <stdio.h>

#include <cstdlib>
#include <iostream>
#include <limits>

#include "BVH.h"

#define GPU __host__ __device__
#include "data.h"

#define WIDTH 1024
#define HEIGHT 1024
#define TILE_WIDTH 16
#define THRESHOLD 0.001f
#define MAX_SPHERES 2048
#define VIEW_ANGLE (3.14159f / 3.f)
#define RADIUS .3f


#define USE_CONSTANT_MEMORY
#define USE_BVH
//#define USE_LINE_POINTS
#define USE_BUNNY
//#define USE_SPHERE_TEST
#define ASSUME_ALL_CONST


#ifdef ASSUME_ALL_CONST
 #define USE_CONSTANT_MEMORY
#else
 #define USE_SOME_CONSTANT_MEMORY
#endif
 
#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__ ))

static void HandleError(cudaError_t err, const char *file, int line) {
   if(err != cudaSuccess) {
      printf("%s in %s at line %d\n", cudaGetErrorString(err), file, line);
      exit(EXIT_FAILURE);
   }
}

namespace cuda {

#ifdef USE_CONSTANT_MEMORY
__constant__ data::Sphere constSpheres[MAX_SPHERES];
#endif

// Calculates if and where a ray intersects a sphere
__device__ bool SphereIntersectionTest(data::Sphere s, data::Ray r, data::Hit *h) {
   data::vec3 o = r.start - s.center;
   data::vec3 dir = r.dir.normalize();

   float b = 2.f * dir.dot(o);
   float c = o.dot(o) - s.radius * s.radius;

   float determinant = b * b - 4 * c;   
   float dist = sqrtf(determinant) / 2.0f;

   if (determinant < THRESHOLD) {
      return false;
   } else {
      float d1 = -b / 2.f + dist,  d2 = -b / 2.f - dist;
      h->depth = (d1 < THRESHOLD || d2 < THRESHOLD) ? max(d1, d2) : min(d1, d2);
      h->position = r.start + dir * h->depth;
      h->normal = (h->position - s.center).normalize();
      return d1 >= THRESHOLD || d2 >= THRESHOLD;
   }
}

#ifdef USE_SPHERE_TEST
__device__ void CastRayBVH(data::Ray &r, data::Sphere *spheres, data::Hit *sh, int *t) {
#else
__device__ void CastRayBVH(data::Ray &r, data::Sphere *spheres, data::Hit *sh) {
#endif
   data::Hit h;
   int stack[20], *sp = &stack[1];
   stack[0] = 0, stack[1] = 1;
   while(sp >= &stack[1]) {
#ifdef USE_SPHERE_TEST
      (*t)++;
#endif
      bool hit = false;
#if defined(USE_CONSTANT_MEMORY) and defined(USE_SOME_CONSTANT_MEMORY)
      if(*sp < MAX_SPHERES) {
         hit = SphereIntersectionTest(constSpheres[*sp], r, &h);
      } else {
         hit = SphereIntersectionTest(spheres[*sp], r, &h);
      }    
#else
      hit = SphereIntersectionTest(spheres[*sp], r, &h);
#endif
      if(hit){
         if (spheres[*sp].draw && h.depth < sh->depth) {
            *sh = h;
            while (*sp % 2) sp--;
            (*sp)++;
         } else {
            int first_child = (*sp) << 1; 
            (*++sp) = first_child;
         }
      } else {
         while (*sp % 2) sp--;
         (*sp)++;
      }
   }
}

#ifdef USE_SPHERE_TEST
__device__ void CastRayLinear(data::Ray &r, data::Sphere *spheres, int numSpheres, data::Hit *sh, int *t) {
#else
__device__ void CastRayLinear(data::Ray &r, data::Sphere *spheres, int numSpheres, data::Hit *sh) {
#endif
   data::Hit h;
   for(int i = 0; i < numSpheres; i++) {
#ifdef USE_SPHERE_TEST
      (*t)++;
#endif
      bool hit = false;
#if defined(USE_CONSTANT_MEMORY) and defined(USE_SOME_CONSTANT_MEMORY)
      if(i < MAX_SPHERES) {
         hit = SphereIntersectionTest(constSpheres[i], r, &h);
      } else {
         hit = SphereIntersectionTest(spheres[i], r, &h);
      }
#else
      hit = SphereIntersectionTest(spheres[i], r, &h);
#endif
      if(hit) {
         if (spheres[i].draw && h.depth < sh->depth) {
            *sh = h;
         }
      }
   }
}

#ifdef USE_SPHERE_TEST
__device__ data::color TraceLights(data::Ray &primary, data::Hit &hit, data::Sphere *spheres, int numSpheres, data::Pointlight *lights, int numLights, int *t) {
#else
__device__ data::color TraceLights(data::Ray &primary, data::Hit &hit, data::Sphere *spheres, int numSpheres, data::Pointlight *lights, int numLights) {
#endif
   data::color col(0,0,0);
   for(int i = 0; i < numLights; i++) {
      data::Hit sh;
      data::Ray r;
      r.start = hit.position;
      r.dir = (lights[i].position - hit.position).normalize();
      float distance = (lights[i].position - hit.position).distance();

#ifdef USE_BVH
 #ifdef USE_SPHERE_TEST
      cuda::CastRayBVH(r, spheres, &sh, t);
 #else
      cuda::CastRayBVH(r, spheres, &sh);
 #endif
#else
 #ifdef USE_SPHERE_TEST
      cuda::CastRayLinear(r, spheres, numSpheres, &sh, t);
 #else
      cuda::CastRayLinear(r, spheres, numSpheres, &sh);
 #endif
#endif

      if (sh.depth > distance) {
         col += lights[i].pColor * r.dir.dot(hit.normal);
      }
      //if(r.dir.dot(hit.normal) < 0) {
      //   col += lights[i].pColor * -r.dir.dot(hit.normal);
      //}
   }
   return col;
}


// This tests multiple spheres
#ifdef USE_SPHERE_TEST
__device__ data::color CastRay(data::Ray &r, data::Sphere *spheres, int numSpheres, data::Pointlight *lights, int numLights, int *t) {
#else
__device__ data::color CastRay(data::Ray &r, data::Sphere *spheres, int numSpheres, data::Pointlight *lights, int numLights) {
#endif
   data::Hit h;
   float prevDepth = h.depth;
#ifdef USE_SPHERE_TEST
   *t = 0;
#endif

#ifdef USE_BVH
 #ifdef USE_SPHERE_TEST
   cuda::CastRayBVH(r, spheres, &h, t);
 #else
   cuda::CastRayBVH(r, spheres, &h);
 #endif
#else
 #ifdef USE_SPHERE_TEST
   cuda::CastRayLinear(r, spheres, numSpheres, &h, t);
 #else
   cuda::CastRayLinear(r, spheres, numSpheres, &h);
 #endif
#endif
   if(h.depth < 1.0f) {
      return data::color(1.0f, 0, 0);
   } else if(h.depth < prevDepth) {
      // Create a ray for each light and test to see if it hits      
#ifdef USE_SPHERE_TEST
      return TraceLights(r, h, spheres, numSpheres, lights, numLights,t);
#else
      return TraceLights(r, h, spheres, numSpheres, lights, numLights);
#endif     
   } else {
      return data::color(0.0f, 0.0f, 0.0f);
   }
}



#ifdef USE_SPHERE_TEST
__global__ void RayTrace(data::Sphere *spheres, const int numSpheres,
                         data::Pointlight *lights, const int numLights, 
                         int *tests, uchar4 *pos, data::Camera c) {
#else
__global__ void RayTrace(data::Sphere *spheres, const int numSpheres,
                         data::Pointlight *lights, const int numLights, 
                         uchar4 *pos, data::Camera c) {
#endif
   int row = blockIdx.y*blockDim.y + threadIdx.y;
   int col = blockIdx.x*blockDim.x + threadIdx.x; // thread id (index to vectors a,b,c)

#ifdef ASSUME_ALL_CONST
   spheres = constSpheres;
#endif

   if(col < WIDTH && row < HEIGHT) {
      data::color clr;
      data::Ray r;

      r.start = c.position;
      float yaw = (float)(col - WIDTH/2)/(float)WIDTH * VIEW_ANGLE + (3.14159f * c.mYaw /180.f);
      float pitch = (float)(row - HEIGHT/2)/(float)HEIGHT * VIEW_ANGLE + (3.14159f * c.mPitch /180.f);
      data::vec3 dirmod(cos(yaw), sin(pitch), sin(yaw));
      r.dir = (data::vec3(0, 0, 0) + dirmod).normalize();


#ifdef USE_SPHERE_TEST
      clr = CastRay(r, spheres, numSpheres, lights, numLights, &tests[row*WIDTH + col]);
#else
      clr = CastRay(r, spheres, numSpheres, lights, numLights);
#endif

      // Each thread writes one pixel location in the texture (textel)
      pos[row*WIDTH + col].w = 0;
      pos[row*WIDTH + col].x = clr[0]*255;
      pos[row*WIDTH + col].y = clr[1]*255;
      pos[row*WIDTH + col].z = clr[2]*255;
   }
}

}

float rf() {
   return (float)rand()/(float)RAND_MAX;
}

void randomLights(data::Pointlight *l, int numLights) {
   for (int i = 0; i < numLights; i++) {
      float latitude = (rf() - 0.5f) * 3.14159f;
      float longitude = (rf() * 2.f * 3.14159f); 
      l[i].position = data::vec3(longitude, latitude, longitude);
      l[i].pColor = data::color(rf() + .3f,rf() + .3f,rf() + .3f);
      l[i].period = (2.f*rf() + 1)/90.f;
   }
}


data::Pointlight *cLights;
data::Sphere *cSpheres;
#ifdef USE_SPHERE_TEST
int *cTests;
#endif

const int numLights = 2;
data::Pointlight lights[numLights];
int numSpheres;

data::Camera cam;

#include <fstream>
#include <vector>
std::vector<data::vec3> randomPoints() {
   // 100 random spheres
   std::vector<data::vec3> points;
   for(int i = 0; i < 100; i++)
      points.push_back(data::vec3(rf(), rf(), rf())*10);
   return points;
}

std::vector<data::vec3> bunnyPoints() {
   // Read the points into an array
   int i = 0;
   char c;
   data::vec3 v;
   std::vector<data::vec3> points;
   std::ifstream iss("bunny.obj"); 
   iss >> c;
   while (c == 'v') {
      iss >> v.x >> v.y >> v.z;
      v *= 40;
      v.y *= -1;
      v += data::vec3(3, 0, 3);
      if (!(++i%3))
         points.push_back(v);
      iss >> c;
   }
   return points;
}

std::vector<data::vec3> linePoints() {
   std::vector<data::vec3> points;
   for(int i = 1; i < 100; i++)
      points.push_back(data::vec3(10,0,i*10));
   return points;
}

void randomSpheresBVH(data::Sphere **s, int *numSpheres) {
#ifdef USE_BUNNY
   std::vector<data::vec3> points = bunnyPoints();
#elif defined(USE_LINE_POINTS)
   std::vector<data::vec3> points = linePoints();
#else
   std::vector<data::vec3> points = randomPoints();
#endif

   // Sort the points according to their spatial location
   KDSort(&points[0], points.size());

   // Convert points into BVH spheres
   BVH<data::vec3, float>::Sphere sp;
   sp.radius = RADIUS;
   std::vector<BVH<data::vec3, float>::Sphere> spheres;
   for(std::vector<data::vec3>::iterator it = points.begin(); it != points.end(); it++) {
      sp.position = *it;
      spheres.push_back(sp);
   }
 
   // Create BVH
   BVH<data::vec3, float> bvh(&spheres[0], spheres.size());

   // Convert BVH spheres into renderable spheres
   *numSpheres = bvh.treeSize;
   *s = new data::Sphere[*numSpheres];
   for (int i = 0; i < *numSpheres; i++) {
      (*s)[i].center = bvh.mNodes[i].sphere.position;
      (*s)[i].radius = bvh.mNodes[i].sphere.radius;
      (*s)[i].draw = bvh.mNodes[i].isLeaf;
   }

}

void randomSpheresLinear(data::Sphere **s, int *numSpheres) {
#ifdef USE_BUNNY
   std::vector<data::vec3> points = bunnyPoints();
#elif defined(USE_LINE_POINTS)
   std::vector<data::vec3> points = linePoints();
#else
   std::vector<data::vec3> points = randomPoints();
#endif
   *numSpheres = points.size();
   *s = new data::Sphere[*numSpheres];
   for (int i = 0; i < *numSpheres; i++) {
      (*s)[i].center = points[i];
      (*s)[i].radius = RADIUS;
      (*s)[i].draw = true;
   }
}

extern void moveCamera(int direction) {
   cam.move(direction);
}

extern void printCamera() {
   cam.print();
}

extern void rotateCamera(int x, int y) {
   cam.rotate(x, y);
}

extern void init_kernel() {
#ifdef USE_SPHERE_TEST
   // Used to check the number of sphere tests we do
   HANDLE_ERROR(cudaMalloc((void **) &cTests, HEIGHT*WIDTH*sizeof(int)));
#endif

   randomLights(&lights[0], numLights);
   HANDLE_ERROR(cudaMalloc((void **) &cLights, numLights*sizeof(data::Pointlight)));
   HANDLE_ERROR(cudaMemcpy(cLights, lights, numLights*sizeof(data::Pointlight), cudaMemcpyHostToDevice));

   data::Sphere *spheres;
#ifdef USE_BVH
   randomSpheresBVH(&spheres, &numSpheres);
#else
   randomSpheresLinear(&spheres, &numSpheres);
#endif
   HANDLE_ERROR(cudaMalloc((void **) &cSpheres, numSpheres*sizeof(data::Sphere)));
   HANDLE_ERROR(cudaMemcpy(cSpheres, spheres, numSpheres*sizeof(data::Sphere), cudaMemcpyHostToDevice));
#ifdef USE_CONSTANT_MEMORY
   HANDLE_ERROR(cudaMemcpyToSymbol(cuda::constSpheres, spheres, min(numSpheres,MAX_SPHERES)*sizeof(data::Sphere)));
#endif
}


extern void destroy_kernel() {
   HANDLE_ERROR(cudaFree(cLights));
   HANDLE_ERROR(cudaFree(cSpheres));
#ifdef USE_SPHERE_TEST
   HANDLE_ERROR(cudaFree(cTests));
#endif
}

int frame = 0;

// Wrapper for the __global__ call that sets up the kernel call
extern "C" void launch_kernel(uchar4* pos) {
   cudaEvent_t start, stop;

   dim3 dimGrid(WIDTH/TILE_WIDTH + 1, HEIGHT/TILE_WIDTH + 1, 1);
   dim3 dimBlock(TILE_WIDTH, TILE_WIDTH, 1);

   frame++;
   data::Pointlight plights[numLights];
   for(int i = 0; i < numLights; i++) {
      plights[i].position.x = 30*sin(lights[i].position.x + frame * lights[i].period);
      plights[i].position.y = 30*sin(lights[i].position.y + frame * lights[i].period);
      plights[i].position.z = 30*cos(lights[i].position.z + frame * lights[i].period);
      plights[i].pColor = lights[i].pColor;
   }
   HANDLE_ERROR(cudaMemcpy(cLights, plights, numLights*sizeof(data::Pointlight), cudaMemcpyHostToDevice));

   HANDLE_ERROR(cudaEventCreate(&start));
   HANDLE_ERROR(cudaEventCreate(&stop));
   HANDLE_ERROR(cudaEventRecord(start, 0));

#ifdef USE_SPHERE_TEST
   cuda::RayTrace<<<dimGrid, dimBlock>>>(cSpheres, numSpheres, cLights, numLights, cTests, pos, cam);
#else
   cuda::RayTrace<<<dimGrid, dimBlock>>>(cSpheres, numSpheres, cLights, numLights, pos, cam);
#endif

   HANDLE_ERROR(cudaEventRecord(stop, 0));
   HANDLE_ERROR(cudaEventSynchronize(stop));
   float elapsedTime;
   HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime, start, stop));
   HANDLE_ERROR(cudaEventDestroy(start));
   HANDLE_ERROR(cudaEventDestroy(stop));

#ifdef USE_SPHERE_TEST
   static int tests[WIDTH * HEIGHT];
   HANDLE_ERROR(cudaMemcpy(tests, cTests, HEIGHT*WIDTH*sizeof(int), cudaMemcpyDeviceToHost));
   double numTests = 0;
   for(int i = 0; i < HEIGHT*WIDTH; i++) {
      numTests = numTests + tests[i];
   }
   numTests /= HEIGHT*WIDTH;
   printf("Time to generate: %.1f ms --- Average sphere intersection tests per ray: %.1f\n", elapsedTime, numTests);
#else
   printf("Time to generate: %.1f ms\n", elapsedTime);
#endif
   cudaThreadSynchronize();
}

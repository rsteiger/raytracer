#pragma once 

#include <cmath>

#ifndef GPU
#define GPU
#endif

namespace data {
   template <typename t>
      GPU const t &max(const t &a, const t &b) { return a > b ? a : b; }
   template <typename t>
      GPU const t &min(const t &a, const t &b) { return a < b ? a : b; }
   template <typename t>
      GPU t &max(t &a, t &b) { return a > b ? a : b; }
   template <typename t>
      GPU t &min(t &a, t &b) { return a < b ? a : b; }


   // Stores color data for a single pixel
   struct color {
      float a[3];
      GPU float &operator[] (int i) {
         return a[i];
      }
      GPU color() {}
      GPU color(const color &c) { 
         a[0] = c.a[0];
         a[1] = c.a[1];
         a[2] = c.a[2];
      }
      GPU color &operator= (const color &c) {
         a[0] = c.a[0];
         a[1] = c.a[1];
         a[2] = c.a[2];
         return *this;
      }
      GPU color &operator+= (const color &c) {
         a[0] = max(min(a[0] + c.a[0],1.f),0.f);
         a[1] = max(min(a[1] + c.a[1],1.f),0.f);
         a[2] = max(min(a[2] + c.a[2],1.f),0.f);
         return *this;
      }
      GPU color operator*(float f) { return color(f*a[0], f*a[1], f*a[2]); }
      GPU color(float ax, float b, float c) { 
         a[0] = max(min(ax,1.f),0.f);
         a[1] = max(min(b,1.f),0.f);
         a[2] = max(min(c,1.f),0.f);
      }
      GPU color operator+(const color &f) const {
         return color(this->a[0] + f.a[0], this->a[1]+f.a[1], this->a[2]+f.a[2]);
      }
   };

   struct vec3 {
      union { float a[3]; struct { float x,y,z; }; };
      GPU vec3() { }
      GPU vec3(float d, float e, float f) { x = d, y = e, z = f; }
      GPU const float &operator[] (int i) const { return a[i]; }
      GPU vec3 operator*(float f) { return vec3(f*x, f*y, f*z); }
      GPU vec3 operator/(float f) { return vec3(x/f, y/f, z/f); }
      GPU vec3 operator+(const vec3 &f) const { return vec3(x + f.x, y + f.y, z + f.z); }
      GPU vec3 operator-(const vec3 &f) const { return vec3(x - f.x, y - f.y, z - f.z); }
      GPU vec3 &operator*=(float f) { x*=f, y*=f, z*=f; return *this; }
      GPU vec3 &operator+=(const vec3 &f) { return *this = *this + f; }
      GPU float dot(const vec3 &f) const { return x*f.x + y*f.y + z*f.z; }
      GPU float distance() const { return sqrtf(dot(*this)); }
      GPU vec3 cross(const vec3 &v) const { return vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
      GPU vec3 normalize() const { float d = distance(); return vec3(x/d, y/d, z/d); }
      GPU vec3 abs() const { return vec3(x>0?x:-x, y>0?y:-y, z>0?z:-z); }
   };

   struct Ray {
      vec3 start;
      vec3 dir;
   };

   struct Sphere {
      vec3 center;
      float radius;
      bool draw;
   };

   struct Pointlight {
      vec3 position;
      color pColor;
      float period;
   };

   struct Hit {
      float depth;
      vec3 position;
      vec3 normal;
      GPU Hit() : depth(2e10f), position(0,0,0) { }
   };

   struct Camera {
      enum { Left, Right, Up, Down, Forward, Backward };
      vec3 position;
      float mYaw;
      float mPitch;
      Camera() : position(13,-12,20), mPitch(24), mYaw(-124) { }

      void move(int dir) {
         float px = cos(3.14159 * mYaw / 180);
         float pz = sin(3.14159 * mYaw / 180);
         float pny = cos(3.14159 * mPitch / 180);
         float py = sin(3.14159 * mPitch / 180);

         switch (dir) {
            case Forward:
               position.x += px * pny;
               position.z += pz * pny;
               position.y += py;
               break;
            case Backward:
               position.x -= px * pny;
               position.z -= pz * pny;
               position.y -= py;
               break;
            case Right:
               position.x -= pz;
               position.z += px;
               break;
            case Left:
               position.x += pz;
               position.z -= px;
               break;
            default:
               break;
         }
      }

      void rotate(int dx, int dy) {
         mYaw += (float)dx / 10;
         mPitch += (float)dy / 10;
         if ( mPitch > 50 ) mPitch = 50; 
         if ( mPitch < -50) mPitch = -50;
      }

      void print() {
         printf("position(%.f,%.f,%.f), mPitch(%.f), mYaw(%.f)\n",position.x, position.y, position.z,mPitch, mYaw);
      }
   };
}


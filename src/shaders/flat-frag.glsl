#version 300 es
precision highp float;

uniform vec3 u_Eye, u_Ref, u_Up;
uniform vec2 u_Dimensions;
uniform float u_Time;
uniform float u_BG;
uniform float u_Illum;

in vec2 fs_Pos;
out vec4 out_Col;

const int MAX_RAY_STEPS = 255;
const float MIN_DIST = 0.0;
const float MAX_DIST = 100.0;
const float EPSILON = 0.01;

struct Bbox
{
  vec3 min;
  vec3 max;
};

// helper functions for sdf scene

float rand(float n){return fract(sin(n) * 43758.5453123);}

float sphere (vec3 p, float r, vec3 c)
{
    return distance(p,c) - r;
}

float sphere( vec3 p, float s )
{
  return length(p) - s;
}

float torus( vec3 p, vec2 t )
{
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

float box( vec3 p, vec3 b )
{
  vec3 d = abs(p) - b;
  return length(max(d,0.0))
         + min(max(d.x,max(d.y,d.z)),0.0);
}

float cylinder( vec3 p, vec2 h )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}

float unionSDF(float distA, float distB) {
    return min(distA, distB);
}

float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}

float sawtooth_wave(float x, float freq, float amplitude) {
  return (x * freq - floor(x * freq)) * amplitude - (amplitude / 2.0);
}

float triangle_wave(float x, float freq, float amplitude) {
  return abs(mod(x * freq, amplitude) - (0.5 * amplitude)) - (0.25 * amplitude);
}

float smoothblend(float d1, float d2, float a)
{
	return a * d1 + (1.0 - a) * d2;
}

mat4 rotateX(float theta) {
    float c = cos(theta);
    float s = sin(theta);

    return mat4(
        vec4(1, 0, 0, 0),
        vec4(0, c, -s, 0),
        vec4(0, s, c, 0),
        vec4(0, 0, 0, 1)
    );
}

mat4 rotateZ(float theta) {
    float c = cos(theta);
    float s = sin(theta);

    return mat4(
        vec4(c, -s, 0, 0),
        vec4(s, c, 0, 0),
        vec4(0, 0, 1, 0),
        vec4(0, 0, 0, 1)
    );
}

// scene

float sceneSDF(vec3 pos) {
  float d = smoothblend(
    sphere(pos, 3.0), 
    box(pos, vec3(3.0, 3.0, 3.0)),
    (sin(u_Time / 30.0) + 1.0) / 2.0
  );

  float c = cylinder(pos, vec2(1.5, 4.0));
  vec3 c2pos = (rotateX(-radians(90.0)) * vec4(pos, 1.0)).xyz;
  float c2 = cylinder(c2pos, vec2(1.5, 4.0));
  vec3 c3pos = (rotateZ(-radians(90.0)) * vec4(pos, 1.0)).xyz;
  float c3 = cylinder(c3pos, vec2(1.5, 4.0));
  
  float s = triangle_wave(u_Time / 100.0, 3.14159, 25.0);
  float sph = sphere(pos + vec3(s, 0.0, 0.0), 1.0);
  float sph2 = sphere(pos + vec3(0.0, s, 0.0), 1.0);
  float sph3 = sphere(pos + vec3(0.0, 0.0, -s), 1.0);

  s = sawtooth_wave(u_Time / 200.0, 3.14159, 20.0);
  float t = torus(c2pos, vec2(6.0, 0.5));
  float b = box(pos + vec3(0.0, s, 0.0), vec3(7.0, 1.0, 1.0));
  float b2 = box(pos + vec3(s, 0.0, 0.0), vec3(1.0, 7.0, 1.0));

  float cs = unionSDF(unionSDF(c, c2), c3);
  float ds = differenceSDF(d, cs);
  float ss = unionSDF(unionSDF(sph, sph2), sph3);
  float bb = unionSDF(b, b2);
  float bt = intersectSDF(t, bb);

  return unionSDF(unionSDF(ds, ss), bt);
}

// ray marching

Bbox calculateBbox() {
  return Bbox(vec3(-3.5, -3.5, -3.5), vec3(3.5, 3.5, 3.5));
}

bool rayCubeIntersect(vec3 eye, vec3 dir, out float tnear) {
  // test for intersection of ray and scene
  Bbox sceneBox = calculateBbox();
  tnear = -1000.0; 
  float tfar = 1000.0;

  for (int i = 0; i < 3; i++) {
    if (dir[i] == 0.0) {
      if (eye[i] < sceneBox.min[i] || eye[i] > sceneBox.max[i]) {
        return false;
      }
    }
    float t0 = (sceneBox.min[i] - eye[i]) / dir[i];
    float t1 = (sceneBox.max[i] - eye[i]) / dir[i];
    if (t0 > t1) {
      float temp = t0;
      t0 = t1;
      t1 = temp;
    }
    else if (t0 > tnear) {
      tnear = t0;
    }
    else if (t1 < tfar) {
      tfar = t1;
    }
  }

  if (tnear > tfar) return false;
  else return true;
}

float raymarch(vec3 eye, vec3 dir, float start, float end) {
  float depth = start;
  float t;
  bool hitScene = rayCubeIntersect(eye, dir, t);
  if (!hitScene) return end;

  depth = t;
  for (int i = 0; i < MAX_RAY_STEPS; i++) {
    float dist = sceneSDF(eye + depth * dir);
    if (dist < EPSILON) {
      return depth;
    }
    depth += dist;
    if (depth >= end) {
      return end;
    }
  }
  return end;
}

// lighting stuff

vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 lightPos, vec3 lightIntensity) {
  vec3 N = estimateNormal(p);
  vec3 L = normalize(lightPos - p);
  vec3 V = normalize(u_Eye - p);
  vec3 R = normalize(reflect(-L, N));

  float dotLN = dot(L, N);
  float dotRV = dot(R, V);

  if (dotLN < 0.0) {
    return vec3(0.0, 0.0, 0.0);
  }
  if (dotRV < 0.0) {
    return lightIntensity * (k_d * dotLN);
  }
  return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p) {
  const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
  vec3 color = ambientLight * k_a;

  vec3 light1Pos = vec3(4.0 * sin(u_Time / 10.0), 
                        2.0, 
                        4.0 * cos(u_Time / 10.0));
  vec3 light1Intensity = vec3(0.4, 0.4, 0.4);

  color += phongContribForLight(k_d, k_s, alpha, p, light1Pos, light1Intensity);
  
  return color;
}

// ray casting stuff

vec3 raycast(out vec3 screenpos) {
  float a = fs_Pos.x;
  float b = fs_Pos.y;
  float FOVY = radians(90.0);
  float aspect = u_Dimensions.x / u_Dimensions.y;
  float len = length(u_Ref - u_Eye);

  vec3 R = normalize(cross(u_Ref - u_Eye, u_Up));
  vec3 V = tan(FOVY / 2.0) * len * u_Up;
  vec3 H = aspect * len * tan(FOVY / 2.0) * R;

  screenpos = u_Ref + a * H + b * V;
  vec3 ray_dir = screenpos - u_Eye;

  return normalize(ray_dir);
}

// noise stuff

// perlin noise

vec2 falloff(vec2 t) {
  return t*t*t*(t*(t*6.0-15.0)+10.0);
  }
vec4 permute(vec4 x){return mod(((x*34.0)+1.0)*x, 289.0);}
vec3 permute(vec3 x) {
    return mod((34.0 * x + 1.0) * x, 289.0);
  }

float pnoise(vec2 P){
  vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
  vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
  Pi = mod(Pi, 289.0); // To avoid truncation effects in permutation
  vec4 ix = Pi.xzxz;
  vec4 iy = Pi.yyww;
  vec4 fx = Pf.xzxz;
  vec4 fy = Pf.yyww;
  vec4 i = permute(permute(ix) + iy);
  vec4 gx = 2.0 * fract(i * 0.0243902439) - 1.0; // 1/41 = 0.024...
  vec4 gy = abs(gx) - 0.5;
  vec4 tx = floor(gx + 0.5);
  gx = gx - tx;
  vec2 g00 = vec2(gx.x,gy.x);
  vec2 g10 = vec2(gx.y,gy.y);
  vec2 g01 = vec2(gx.z,gy.z);
  vec2 g11 = vec2(gx.w,gy.w);
  vec4 norm = 1.79284291400159 - 0.85373472095314 * 
    vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11));
  g00 *= norm.x;
  g01 *= norm.y;
  g10 *= norm.z;
  g11 *= norm.w;
  float n00 = dot(g00, vec2(fx.x, fy.x));
  float n10 = dot(g10, vec2(fx.y, fy.y));
  float n01 = dot(g01, vec2(fx.z, fy.z));
  float n11 = dot(g11, vec2(fx.w, fy.w));
  vec2 fade_xy = falloff(Pf.xy);
  vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
  float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
  return 2.3 * n_xy;
}

float applyPerlinNoise(vec2 worldPos) {
  vec2 p = worldPos;
  float x = pnoise(p);
  return x;
}

void generateTerrain(vec3 pos, out float height) {  
  // two recursions of perlin noise
  height = applyPerlinNoise(vec2(pos.x, pos.y)) + applyPerlinNoise(vec2(pos.x, pos.y));
  height = pow(height, 2.0);
  height = applyPerlinNoise(vec2(pos.x, height)) + applyPerlinNoise(vec2(pos.x, height));
  height = pow(height, 2.0);
}

vec3 terrainBG(float height) {
  float heightScale = height / 4.0;

  vec3 lowCol = vec3(0.0, 0.0, 0.0);
    vec3 highCol = vec3(1.0, 1.0, 1.0);

    if (u_BG == 0.0) { // blue
        lowCol = vec3(34.0, 85.0, 165.0) / 255.0;
        highCol = vec3(66.0, 134.0, 244.0) / 255.0;
    }
    else if (u_BG == 1.0) { // red
        lowCol = vec3(200.0, 71.0, 71.0) / 255.0;
        highCol = vec3(165.0, 34.0, 34.0) / 255.0;
    }
    else if (u_BG == 2.0) { // gray
        lowCol = vec3(113.0, 113.0, 113.0) / 255.0;
        highCol = vec3(25.0, 25.0, 25.0) / 255.0;
    }
    else { // something went wrong
    }

  vec3 height_Col = mix(lowCol, highCol, heightScale);

  return height_Col;
}

void main() {
  
  vec3 screenpos = vec3(0.0);
  vec3 ray_dir = raycast(screenpos);
  float dist = raymarch(u_Eye, ray_dir, MIN_DIST, MAX_DIST);

  // out_Col = vec4((0.5 * (ray_dir + vec3(1.0, 1.0, 1.0))), 1.0);

  if (dist > MAX_DIST - EPSILON) {
    float height = 0.0;
    generateTerrain(screenpos, height);
    vec3 color = terrainBG(height);
    out_Col = vec4(color, 1.0);
    return;
  }

  vec3 p = u_Eye + dist * ray_dir;
  
  vec3 K_a = vec3(0.2, 0.2, 0.2);
  vec3 K_d = vec3(0.7, 0.2, 0.2);
  vec3 K_s = vec3(1.0, 1.0, 1.0);
  float shininess = u_Illum;

  vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p);

  out_Col = vec4(color, 1.0);
}

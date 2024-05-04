#pragma once
#include<cmath>

class vec2f
{
public:
	vec2f(float _x = 0.f, float _y = 0.f) :x(_x), y(_y) {}
	float x;
	float y;
};

inline float length(const vec2f& v) {
	return sqrt(v.x * v.x + v.y * v.y);
}

inline vec2f normalize(const vec2f& v) {
	if (v.x == 0.f && v.y == 0.f) return v;
	float l = length(v);
	return vec2f(v.x / l, v.y / l);
}

inline float dot(const vec2f& a, const vec2f& b) {
	return a.x * b.x + a.y * b.y;
}

inline float cosine(const vec2f& a, const vec2f& b) {
	return dot(a, b) / (length(a) * length(b));
}

inline float angleBetweenTwoVec2f(const vec2f& a, const vec2f& b) {
	return std::acos(cosine(a, b));
}

inline vec2f operator-(const vec2f& a, const vec2f& b) {
	return vec2f(a.x - b.x, a.y - b.y);
}

inline vec2f operator+(const vec2f& a, const vec2f& b) {
	return vec2f(a.x + b.x, a.y + b.y);
}

inline vec2f operator/(const vec2f& a, const float b) {
	return vec2f(a.x / b, a.y / b);
}

inline vec2f operator*(const vec2f& a, const float b) {
	return vec2f(a.x * b, a.y * b);
}

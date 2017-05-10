#pragma once

#include <glm/glm.hpp>

// populates intersects with intersect data
// can consider having another variable passed for reflection and refraction
void findIntersect(
	Ray ray, 
	std::vector<Intersect> &intersects // empty vector that gets populated by function
);

// called by colorThePixel. populates the color field based on specular and diffuse lighting
void traceLightRays(
	Light &light, // single light source
	Intersect &intersect, // intersect of primary ray with an object
	Ray &cameraRay, // primary ray
	glm::vec3 &color, // this field gets populated with resulting color
);

// populates the color field by calling traceLightRays iteratively on all lights and adds color based on ambient lighting
void colorThePixel(
	Intersect &intersect, // intersect with primary ray with an object
	Ray cameraRay, // primary ray 
	glm::vec3 &color, // this field gets populated with resulting color
	const glm::vec3 & ambient, // value for ambient lighting
	const std::list<Light *> &lights // vector of all light sources
);

/* function that renders the image
 */ 
void render(
		// What to render
		Objects * root,

		// Image to write to, set to a given width and height
		Image & image,

		// Viewing parameters
		const glm::vec3 & eye,
		const glm::vec3 & view,
		const glm::vec3 & up,
		double fovy,

		// Lighting parameters
		const glm::vec3 & ambient,
		const std::list<Light *> & lights
);

struct Light {
	glm::vec3 colour;
	glm::vec3 origin;
};

struct Intersect {
	bool hit = false;
	float distanceToEye;
	glm::vec3 point;
	Material *material = NULL;
	glm::vec3 normal;
};

struct Ray {
	glm::vec3 origin;
	glm::vec3 direction;
};
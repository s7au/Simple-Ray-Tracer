#include <glm/ext.hpp>

#include <algorithm>
#include <vector>
#include <cmath>
#include <map>
#include <deque>

#define PI 3.14159265
#define MAX_DEPTH 4

/* getReflectionDirection, getRefractionDirection, and fresnel are derived from examples on 
 * https://www.scratchapixel.com/ -> highly recommend going through it
 * these functions aren't used but might be a passable reference
 */ 
glm::vec3 getReflectionDirection(glm::vec3 rayDirection, glm::vec3 normal)
{
	return rayDirection - 2 * glm::dot(rayDirection, normal) * normal; 
}

glm::vec3 getRefractionDirection(glm::vec3 rayDirection, glm::vec3 normal, float refractiveIndex)
{
	float cosi = std::min(std::max(-1.0f, glm::dot(rayDirection, normal)),1.0f); 
    float outsideRI = 1, insideRI = refractiveIndex; 
    if (cosi < 0) {
    	cosi = -cosi;
    } else {
    	std::swap(outsideRI, insideRI); 
    	normal = - normal; 
    } 
    float ratioRI = outsideRI / insideRI; 
    float n = 1 - ratioRI * ratioRI * (1 - cosi * cosi); 
    glm::vec3 zero = {0,0,0};
    return (n < 0.0f) ? zero : ratioRI * rayDirection + (ratioRI * cosi - sqrtf(n)) * normal; 
}

// this is not my function. taken from website and adds a bit of both reflective and refractive properties
void fresnel(glm::vec3 rayDirection, glm::vec3 normal, float refractiveIndex, float &kr) 
{ 
    float cosi = std::min(std::max(-1.0f, glm::dot(rayDirection, normal)),1.0f); 
    float outsideRI = 1, insideRI = refractiveIndex; 
    if (cosi > 0) {
    	std::swap(outsideRI, insideRI); 
    }
    // Compute sini using Snell's law
    float sint = outsideRI / insideRI * sqrtf(std::max(0.f, 1.0f - cosi * cosi)); 
    // Total internal reflection
    if (sint >= 1) { 
        kr = 1; 
    } else { 
        float cost = sqrtf(std::max(0.f, 1.0f - sint * sint)); 
        cosi = fabsf(cosi); 
        float Rs = ((insideRI * cosi) - (outsideRI * cost)) / ((insideRI * cosi) + (outsideRI * cost)); 
        float Rp = ((outsideRI * cosi) - (insideRI * cost)) / ((outsideRI * cosi) + (insideRI * cost)); 
        kr = (Rs * Rs + Rp * Rp) / 2; 
    } 
} 

void findIntersect(Ray ray, std::vector<Intersect> &intersects) {
	// want to go through objects in scene to find if ray intersect objects
	// somehow must have access to Objects var that is passed to render function
	// left unimplemented here
}

void traceLightRays(
	Light& light, 
	Intersect &intersect, 
	Ray &cameraRay, 
	glm::vec3 &color, 
)
{
	glm::vec3 intersectNormal(intersect.normal);
	glm::vec3 rayDirection(cameraRay.direction);
	glm::vec3 tempColor = {0,0,0};

	glm::vec3 lightRay.direction = glm::normalize(intersect.point - lightRay.origin);

	Ray lightRay;
	std::vector<Intersect> lightIntersects;

	findIntersect(lightRay, lightIntersects);

	// we only want the single point where the light intersects the same point that the primary point intersects so
	// if findIntersect does not already return a single Intersect it should be modified to return one in this case
	Intersect &lightIntersect = lightIntersects[0];

	if (lightIntersect.hit && lightIntersect.distanceToEye < glm::distance(intersect.point, lightRay.origin)-.1) {
		return;
	} else {
		// these formulas are taken from various sources on the internet. Lots of graphics lectures
		// have these formulas
		
		// for whatever reason there is a specular and diffuse portion when lighting 

		//specular
		glm::vec3 reflectedRay = glm::normalize(
			2*glm::dot(-lightRay.direction, intersectNormal)*intersectNormal + 
			lightRay.direction
		);
		float dotSpecular = glm::dot(reflectedRay, -rayDirection);
		if (dotSpecular > 0) {
			tempColor += intersect.material->getKS()*pow(dotSpecular, intersect.material->getShininess())*light.colour;
		}

		//diffuse
		float dotDiffuse = glm::dot(-lightRay.direction, intersectNormal);
		if (dotDiffuse > 0) {
			tempColor += intersect.material->getKD()*dotDiffuse*light.colour;
		}
	}
	// you also have the option to include falloff in the calculations somewhere in this function
	color = tempColor;
}

void colorThePixel(
	Intersect &intersect, 
	Ray cameraRay, 
	glm::vec3 &color, 
	const glm::vec3 & ambient,
	const std::list<Light> &lights
) {
	// if not hit then background
	if (!intersect.hit) {
		// do whatever you want for background here
	} else {
		intersect.normal = glm::normalize(intersect.normal);
		for (std::list<Light>::const_iterator it = lights.begin(); it != lights.end(); it++) {
			traceLightRays((*it), intersect, cameraRay, color, raysPerSource);
		}
		color += intersect.material->getKD()*ambient;
	}
}

void render(
		// What to render
		Objects * root,

		// Image to write to, set to a given width and height
		Image & image,

		// Viewing parameters
		const glm::vec3 & eye,
		const glm::vec3 & view,
		const glm::vec3 & up,
		double fov, // field of view in degrees

		// Lighting parameters
		const glm::vec3 & ambient,
		const std::list<Light> & lights
) {
	std::cout << "Calling render(\n" <<
		  "\t" << *root <<
		  "\t" << "Image(width:" << image.width() << ", height:" << image.height() << ")\n"
		  "\t" << "eye:  " << glm::to_string(eye) << std::endl <<
		  "\t" << "view: " << glm::to_string(view) << std::endl <<
		  "\t" << "up:   " << glm::to_string(up) << std::endl <<
		  "\t" << "fovy: " << fovy << std::endl <<
		  "\t" << "ambient: " << glm::to_string(ambient) << std::endl <<
		  "\t" << "lights{" << std::endl;

	for(const Light light : lights) {
		std::cout << "\t\t" <<  light << std::endl;
	}
	std::cout << "\t}" << std::endl;
	std:: cout <<")" << std::endl;

	size_t h = image.height();
	size_t w = image.width();

	// data structure containing ray information
	// this ray represents the primary ray
	Ray ray;
	ray.origin = eye;
	
	float fovRadius = fov*PI/360; // this is where we convert fov to ... radians divided by 2
	glm::vec3 side = glm::normalize(glm::cross(view,up));
	Intersect intersect;

	/* initialize random seed: */
  	srand (time(NULL));

	for (float y = 0; y < h; y++) {
		for (float x = 0; x < w; x++) {
			glm::vec3 color(0,0,0);
			
			// get normalized ray vector for pixel
			// think of this as z + x + y but instead view + side + up coordinates
			glm::vec3 rayDirection(
				glm::normalize(
					view + 
					(x / (float)w * 2 - 1) * tan(fovRadius) * (float)w/(float)h * side +
					(1 - y / (float)h * 2) * tan(fovRadius) * up
				)
			);

			// the origin of the ray will not change but the direction will change with every pixel
			ray.direction = rayDirection;

			// if you want to implement something interesting it might be useful to return multiple intersects
			intersects.clear();
			findIntersect(ray, intersects, 0);

			// continually add color to the pixel with all the intersects
			for (Intersect intersect : intersects) {
				colorThePixel(intersect, ray, color, ambient, lights);
			}

			// pixel is colored here according to x and y
			for (int i = 0; i < 3; i++) {
				image(x,y,i) = color[i]; // this function is not implemented
			}
		}
	}
}

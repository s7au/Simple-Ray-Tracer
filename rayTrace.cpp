#include <glm/ext.hpp>

#include "A4.hpp"
#include "GeometryNode.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <map>
#include <deque>

#define PI 3.14159265
#define MAX_DEPTH 4

// #define ANTI_ALIASING
#define SOFT_SHADOW
// #define DOV
// #define REFLECTION_REFRACTION
// #define NO_ACCEL

struct Cell {
	std::vector<GeometryNode*> *nodes;
	Cell() {
		nodes = new std::vector<GeometryNode*>;
	}
	~Cell() {
		delete nodes;
	}
};

static float focalLength = 40;
static float convergingRays = 100;
static int raysPerSource = 5;

static glm::vec3 gridBBoxMin;
static glm::vec3 gridBBoxMax;
static glm::vec3 cellDimension;
static glm::vec3 gridResolution;
Cell** cells;
SceneNode* flattenedRoot;

glm::vec4 vec3ToVec4(glm::vec3 vector, int fourthArray)
{
	glm::vec4 newVector;
	for (int i = 0; i < 3; i++) {
		newVector[i] = vector[i];
	}
	newVector[3] = fourthArray;
	return newVector;
}

void flattenRoot(const glm::mat4 trans, SceneNode* flattenedRoot, SceneNode* node) {
	node->set_transform(trans*node->get_transform());

	if (node->m_nodeType == NodeType::GeometryNode) {
		GeometryNode* geoNode = (GeometryNode*)node;
		geoNode->transformBoundingBox(node->get_transform());
	}

	flattenedRoot->add_child(node);

	for (std::list<SceneNode*>::const_iterator it = node->children.begin(); it != node->children.end(); it++) {
		flattenRoot(node->get_transform(), flattenedRoot, (*it));
	}
	node->children.clear();
}

glm::vec4 getReflectionDirection(glm::vec4 rayDirection, glm::vec4 normal)
{
	return rayDirection - 2 * glm::dot(rayDirection, normal) * normal; 
}

glm::vec4 getRefractionDirection(glm::vec4 rayDirection, glm::vec4 normal, float refractiveIndex)
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
    glm::vec4 zero = {0,0,0,0};
    return (n < 0.0f) ? zero : ratioRI * rayDirection + (ratioRI * cosi - sqrtf(n)) * normal; 
}

// this is not my function. taken from website and adds a bit of both reflective and refractive properties
void fresnel(glm::vec4 rayDirection, glm::vec4 normal, float refractiveIndex, float &kr) 
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

// this uses DDA which is admittedly not entirely my code
void gridIntersect(Ray ray, std::vector<Intersect> &intersects, int depth)
{ 
	Intersect intersect;
	// initialization step
	glm::vec3 exit, step, cell; 
	glm::vec3 deltaT, nextCrossingT; 
	for (int i = 0; i < 3; i++) { 
		// convert ray starting point to cell coordinates
		// (ray.point[i] + ray.direction[i] * ray.tmin???) -> if we want to denote a minimum ray intersection distance
		double rayOrigCell = ray.point[i] - gridBBoxMin[i]; // min of bounding box
		// cell[i] = std::clamp(std::floor(rayOrigCell / cellDimension[i]), 0, gridResolution[i] - 1); // need to calculate grid Reso and cell dime
		cell[i] = std::max(std::min(std::floor(rayOrigCell / cellDimension[i]), (double)gridResolution[i]-1), 0.0);
		// think of deltaT as how many cellDimensions needed to purchase a ray direction
		if (ray.direction[i] < 0) { 
			deltaT[i] = -cellDimension[i] / ray.direction[i]; 
			nextCrossingT[i] = (cell[i] * cellDimension[i] - rayOrigCell) / ray.direction[i]; 
			exit[i] = -1; 
			step[i] = -1; 
		} 
		else { 
			deltaT[i] = cellDimension[i] / ray.direction[i]; 
			nextCrossingT[i] = ((cell[i] + 1)  * cellDimension[i] - rayOrigCell) / ray.direction[i]; 
			exit[i] = gridResolution[i]; 
			step[i] = 1; 
		} 
	}
 
	// walk through each cell of the grid and test for an intersection if
	// current cell contains geometry
	intersect.hit = false;
	std::map<SceneNode*, bool> visited;
	float map[8] = {2, 1, 2, 1, 2, 2, 0, 0}; 
	while (1) { 
		// use grid and cell information to determine next cell crossing
		// int o = cell[2] * gridResolution[0] * gridResolution[1] + cell[1] * gridResolution[0] + cell[0]; 
		// check grid to see whether there is object in cell
		int cellPos = cell[2] * gridResolution[0] * gridResolution[1] + cell[1] * gridResolution[0] + cell[0]; 
		if (cells[cellPos] != NULL) {
			for (GeometryNode* node : *(cells[cellPos]->nodes)) {
				if (visited.count(node)) continue;
				Intersect tempIntersect = node->intersectCheck(ray);
				if (tempIntersect.hit && (!intersect.hit || tempIntersect.distanceToEye < intersect.distanceToEye)) {
					intersect = tempIntersect;
				}
				visited[node] = true;
			}
			// if (intersect.hit) return;
		} 
		// fancy dumb way of finding which one is the min and returning the index...
		int k = 
			((nextCrossingT[0] < nextCrossingT[1]) << 2) + 
			((nextCrossingT[0] < nextCrossingT[2]) << 1) + 
			((nextCrossingT[1] < nextCrossingT[2])); 
		int axis = map[k]; 
		// if (ray.tmax < nextCrossingT[axis]) break; // we don't have a max distance for ray
		cell[axis] += step[axis]; 
		if (cell[axis] == exit[axis]) {
			// this is our exit condition
#ifdef REFLECTION_REFRACTION
			if (intersect.hit && intersect.material->isGlassy() && depth < MAX_DEPTH) {
				// i have some error with intersect.normal[3] != 0... debug this in the future;
				ray.direction[3] = 0;
				intersect.normal[3] = 0;
				ray.direction = glm::normalize(ray.direction);
				intersect.normal = glm::normalize(intersect.normal);

				Ray reflectRay;
				// float offset = 2*std::pow(intersect.distanceToEye,2)/10000; 
				float offset = .00001;
				// std::cout << offset << " ";
				reflectRay.point = intersect.point;
		        for (int i = 0; i < 3; i++) {
		        	reflectRay.point[i] += offset;
		        }
		        reflectRay.direction = getReflectionDirection(ray.direction, intersect.normal); 
		        // recurse
		        std::vector<Intersect> reflectedIntersect;
		        gridIntersect(reflectRay, reflectedIntersect, depth + 1); 
		        Ray refractRay; 
		        refractRay.point = intersect.point;
		        for (int i = 0; i < 3; i++) {
		        	refractRay.point[i] += offset;
		        }
		        refractRay.direction = getRefractionDirection( 
		            ray.direction, 
		            intersect.normal,
		            intersect.material->getRefractiveIndex());
		        // recurse
		        std::vector<Intersect> refractedIntersect;
		        gridIntersect(refractRay, refractedIntersect, depth + 1); 
		        float Kr;
		        fresnel(
		        	ray.direction, 
		            intersect.normal,
		            intersect.material->getRefractiveIndex(),
		            Kr);
		        for (Intersect rIntersect: reflectedIntersect) {
		        	if (rIntersect.material == NULL) continue;
		        	rIntersect.kr *= Kr;
		        	intersects.push_back(rIntersect);
		        }
		        for (Intersect rIntersect: refractedIntersect) {
		        	if (rIntersect.material == NULL) continue;
		        	// refraction picks up some abnormal brightness so this deals with it
		        	rIntersect.kr *= (1-Kr)*.8;
		        	intersects.push_back(rIntersect);
		        }
			}
#endif
		    intersects.push_back(intersect);
			// if .hit and is glass call gridIntersect again...
			return;
		}
		// increment next crossing with our 'purchase' of a deltaT
		nextCrossingT[axis] += deltaT[axis]; 
	}
}

void subdivideScene(SceneNode* root)
{
	for (SceneNode* node : root->children) {
		GeometryNode* geoNode;
		// if (node->m_nodeType == NodeType::GeometryNode) {
		geoNode = (GeometryNode*)node;
		// } else continue;
		// convert to cell coordinates
		glm::vec3 min;
		glm::vec3 max;
	    for (int i = 0; i < 3; i++) {
		    min[i] = (geoNode->bboxMin[i] - gridBBoxMin[i]) / cellDimension[i]; 
		    max[i] = (geoNode->bboxMax[i] - gridBBoxMin[i]) / cellDimension[i]; 
	    }

	    // to handle the case where the max is the maximum possible value resulting in an invalid cell coordinate due to not rounding down
	    int zmin = std::min(std::floor(min[2]), gridResolution[2] - 1); 
	    int zmax = std::min(std::floor(max[2]), gridResolution[2] - 1); 
	    int ymin = std::min(std::floor(min[1]), gridResolution[1] - 1); 
	    int ymax = std::min(std::floor(max[1]), gridResolution[1] - 1); 
	    int xmin = std::min(std::floor(min[0]), gridResolution[0] - 1); 
	    int xmax = std::min(std::floor(max[0]), gridResolution[0] - 1); 
	    // loop over all the cells the triangle overlaps and insert
	    for (int z = zmin; z <= zmax; z++) { 
	        for (int y = ymin; y <= ymax; y++) { 
	            for (int x = xmin; x <= xmax; x++) { 
	            	int cellPos = z * gridResolution[0] * gridResolution[1] + y * gridResolution[0] + x; 
	                if (cells[cellPos] == NULL) cells[cellPos] = new Cell(); 
	                cells[cellPos]->nodes->push_back(geoNode); 
	            } 
	        } 
		}
	} 
}

void traceLightRays(Light* light, Intersect &intersect, Ray &cameraRay, glm::vec3 &color, int raysPerSource)
{
	glm::vec3 intersectNormal3(intersect.normal);
	glm::vec3 rayDirection3(cameraRay.direction);
	glm::vec3 tempColor = {0,0,0};

	float hitRays = 0;
#ifdef SOFT_SHADOW
	for (int s = 0; s < raysPerSource; s++) {
#endif
		Ray lightRay;
		std::vector<Intersect> lightIntersects;

		glm::vec4 lightPoint4(light->position, 1);
		lightRay.point = lightPoint4;
#ifdef SOFT_SHADOW
		float xld = (float)rand()/RAND_MAX*8;
		float yld = (float)rand()/RAND_MAX*8;
		float zld = (float)rand()/RAND_MAX*8;
		lightRay.point[0] = lightPoint4[0]-4+xld;
		lightRay.point[1] = lightPoint4[1]-4+yld;
		lightRay.point[2] = lightPoint4[2]-4+zld;
#endif

		glm::vec3 lightDirection3(
			intersect.point[0] - lightRay.point[0],
			intersect.point[1] - lightRay.point[1],
			intersect.point[2] - lightRay.point[2]);
		lightDirection3 = glm::normalize(lightDirection3);
		glm::vec4 lightDirection4(lightDirection3, 0);
		lightRay.direction = lightDirection4;
#ifdef NO_ACCEL
		Intersect newIntersect;
		newIntersect = flattenedRoot->intersectCheck(lightRay);
		lightIntersects.push_back(newIntersect);
#else
		gridIntersect(lightRay, lightIntersects, 4);
#endif
		if (lightIntersects.size() != 1) {
			std::cerr << "depth not utilized properly" << std::endl;
			exit(1);
		}

		Intersect &lightIntersect = lightIntersects[0];

		if (lightIntersect.hit && lightIntersect.distanceToEye < glm::distance(intersect.point, lightRay.point)-.1) {
#ifdef SOFT_SHADOW
			continue;
#else
			return;
#endif
		} else {
			
			//specular
			glm::vec3 reflectedRay = glm::normalize(
				2*glm::dot(-lightDirection3, intersectNormal3)*intersectNormal3 + lightDirection3);
			float dotSpecular = glm::dot(reflectedRay, -rayDirection3);
			if (dotSpecular > 0) {
				tempColor += intersect.material->getKS()*pow(dotSpecular, intersect.material->getShininess())*light->colour*intersect.kr;
			}

			//diffuse
			float dotDiffuse = glm::dot(-lightDirection3, intersectNormal3);
			if (dotDiffuse > 0) {
				// std::cout << intersect.normal[0] << " " << intersect.normal[1] << " " << intersect.normal[2] << std::endl;
				if (intersect.material->isTexture() != 0) {
					tempColor += intersect.material->getKD(intersect.xProportion, intersect.yProportion)*dotDiffuse*light->colour*intersect.kr;
				} else {
				tempColor += intersect.material->getKD()*dotDiffuse*light->colour*intersect.kr;
				}
			}
			if (dotSpecular > 0 || dotDiffuse > 0) hitRays++;
		}
#ifdef SOFT_SHADOW
	}
	if (hitRays > 0) tempColor /= raysPerSource;
	// tempColor /= raysPerSource;x
#endif
	color += tempColor;
}

void colorThePixel(
	float x, 
	float y, 
	size_t w, 
	size_t h, 
	Intersect &intersect, 
	Ray ray, 
	glm::vec3 &color, 
	const glm::vec3 & ambient,
	const std::list<Light *> &lights
#ifdef ANTI_ALIASING
	,glm::vec3 *colorGrid
#endif
) {
	// if not hit then background
	if (!intersect.hit) {
		#ifdef ANTI_ALIASING
		colorGrid[(int)x] = {y / h, 0, (x+y) / (w+h)};
		#elif defined DOV
		color[0] += y / h;
		color[1] += 0;
		color[2] += (x+y) / (w+h);
		// #elif defined SOFT_SHADOW
		// color[0] = (float)y / h * raysPerSource*2/3;
		// // Green: increasing from left to right
		// color[1] = (float)0 * raysPerSource*2/3;
		// // Blue: in lower-left and upper-right corners
		// color[2] = (float)(x+y) / (w+h) * raysPerSource*2/3;
		#else
		color[0] = (float)y / h;
		// Green: increasing from left to right
		color[1] = (float)0;
		// Blue: in lower-left and upper-right corners
		color[2] = (float)(x+y) / (w+h);
		#endif
		// image(x, y, 2) = ((y < h/2 && x < w/2)
		// 			  || (y >= h/2 && x >= w/2)) ? 1.0 : 0.0;
	} else {
		glm::vec3 normalisedIntersectNormal(intersect.normal);
		intersect.normal = vec3ToVec4(glm::normalize(normalisedIntersectNormal), 0);
		for (std::list<Light *>::const_iterator it = lights.begin(); it != lights.end(); it++) {
			traceLightRays((*it), intersect, ray, color, raysPerSource);
		}

		color += intersect.material->getKD()*ambient*intersect.kr;
		#ifdef ANTI_ALIASING
		colorGrid[(int)x] = color;
		#endif
		// std::cout << "hello" << std::endl;
	}
}

glm::vec3 getAverageAround(int x, std::deque<glm::vec3*> twoRows)
{
	glm::vec3 returnVector(0,0,0);
	for (int i = 0; i < 2; i++) {
		returnVector += twoRows[i][x];
		returnVector += twoRows[i][x+1];
	}
	for (int i = 0; i < 3; i++) {
		returnVector[i] = returnVector[i]/4;
	}

	// returnVector = threeRows[2][x];

	return returnVector;
}

void A4_Render(
		// What to render
		SceneNode * root,

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
) {

  // Fill in raytracing code here...

  std::cout << "Calling A4_Render(\n" <<
		  "\t" << *root <<
		  "\t" << "Image(width:" << image.width() << ", height:" << image.height() << ")\n"
		  "\t" << "eye:  " << glm::to_string(eye) << std::endl <<
		  "\t" << "view: " << glm::to_string(view) << std::endl <<
		  "\t" << "up:   " << glm::to_string(up) << std::endl <<
		  "\t" << "fovy: " << fovy << std::endl <<
		  "\t" << "ambient: " << glm::to_string(ambient) << std::endl <<
		  "\t" << "lights{" << std::endl;

	for(const Light * light : lights) {
		std::cout << "\t\t" <<  *light << std::endl;
	}
	std::cout << "\t}" << std::endl;
	std:: cout <<")" << std::endl;
#ifdef NO_ACCEL
	std::cout << "\t" << "Using Acceleration: false" << std::endl;
#else
	std::cout << "\t" << "Using Acceleration: true" << std::endl;
#endif

	#ifdef ANTI_ALIASING
	size_t h = image.height()*2;
	size_t w = image.width()*2;
	#else
	size_t h = image.height();
	size_t w = image.width();
	#endif

	Ray ray;
	float fovRadius = fovy*PI/360; // this is where we convert fov to ... whatever
	glm::vec3 side = glm::normalize(glm::cross(view,up));
	Intersect intersect;
	std::vector<Intersect> intersects;
	ray.point[0] = eye[0];
	ray.point[1] = eye[1];
	ray.point[2] = eye[2];
	ray.point[3] = 1;

	SceneNode* realTreeRoot = root->clone();

	flattenedRoot = new SceneNode("flattenedRoot");
	for (std::list<SceneNode*>::const_iterator it = realTreeRoot->children.begin(); it != realTreeRoot->children.end(); it++) {
		flattenRoot(realTreeRoot->get_transform(), flattenedRoot, (*it));
	}

	// creating great lord bounding box
	for (std::list<SceneNode*>::const_iterator it = flattenedRoot->children.begin(); it != flattenedRoot->children.end(); it++) {
		if ((*it)->m_nodeType == NodeType::GeometryNode) {
			GeometryNode* geoNode = (GeometryNode*)(*it);
			for (int i = 0; i < 3; i++) {
				gridBBoxMin[i] = std::min(gridBBoxMin[i], geoNode->bboxMin[i]);
				gridBBoxMax[i] = std::max(gridBBoxMax[i], geoNode->bboxMax[i]);
			}
		}
	}
	int objectNum = flattenedRoot->children.size();
	int factor = 4;
	float volume = 1;
	for (int i = 0; i < 3; i++) {
		volume *= gridBBoxMax[i]-gridBBoxMin[i];
	}
	std::cout << "\t" << "Grid Resolution: ";
	for (int i = 0; i < 3; i++) {
		gridResolution[i] = std::floor((gridBBoxMax[i]-gridBBoxMin[i])*std::cbrt(factor*objectNum/volume)+1);
		std::cout << gridResolution[i] << " ";
	}
	std::cout << std::endl;
	int gridSize = gridResolution[0]*gridResolution[1]*gridResolution[2];
	for (int i = 0; i < 3; i++) {
		cellDimension[i] = (gridBBoxMax[i] - gridBBoxMin[i]) / gridResolution[i];
	}

	// Cell * tempCell[gridSize];
	cells = new Cell*[gridSize];
	// cells = tempCell;
	for (int i = 0; i < gridSize; i++) {
		cells[i] = NULL;
	}

	// create cells
	subdivideScene(flattenedRoot);
	#ifdef ANTI_ALIASING
	// code of anti-aliasing
	std::deque<glm::vec3*> twoRows;
	#endif

	/* initialize random seed: */
  	srand (time(NULL));

	for (float y = 0; y < h; y++) {
		#ifdef ANTI_ALIASING
		glm::vec3 *colorGrid = new glm::vec3[w];
		#endif
		for (float x = 0; x < w; x++) {
			glm::vec3 color(0,0,0);
			
			// get normalized ray vector for pixel
			// think of this as z + x + y but instead view + side + up
			glm::vec3 rayDirection3(glm::normalize(view + (x / (float)w * 2 - 1) * tan(fovRadius) * (float)w/(float)h * side +
				(1 - y / (float)h * 2) * tan(fovRadius) * up));

			#ifdef DOV
			glm::vec3 focalPoint = eye + focalLength*rayDirection3;
			for (int f = 0; f < convergingRays; f++) {

				float xd = (float)rand()/RAND_MAX*.2;
				float yd = (float)rand()/RAND_MAX*.2;
				float zd = (float)rand()/RAND_MAX*.2;
				glm::vec3 jitteredPoint = {
					eye[0]-.1+xd,
					eye[1]-.1+yd,
					eye[2]-.1+zd
				};
				for (int i = 0; i < 3; i++) {
					ray.point[i] = jitteredPoint[i];
				}

				rayDirection3 = glm::normalize(focalPoint-jitteredPoint);
			#endif
				glm::vec4 rayDirection4(rayDirection3, 0);

				ray.direction = rayDirection4;

				// return intersection data whether ray vector hits root
				// intersect = root->intersectCheck(ray);
				// if (bboxIntersect(gridBBoxMin, gridBBoxMax, ray)) {
					// intersect = flattenedRoot->intersectCheck(ray);
				// } else {
				// 	intersect.hit = false;
				// }
#ifdef NO_ACCEL
				intersect = flattenedRoot->intersectCheck(ray);
				intersects.clear();
				intersects.push_back(intersect);
#else
				intersects.clear();
				gridIntersect(ray, intersects, 0);
#endif
				for (Intersect intersect : intersects) {
					colorThePixel(x, y, w, h, intersect, ray, color, ambient, lights
#ifdef ANTI_ALIASING
						,colorGrid
#endif
					);
				}
			#ifdef DOV
			}
			image(x, y, 0) = color[0]/convergingRays;
			// Green: increasing from left to right
			image(x, y, 1) = color[1]/convergingRays;
			// Blue: in lower-left and upper-right corners
			image(x, y, 2) = color[2]/convergingRays;
			// #elif defined SOFT_SHADOW
			// image(x, y, 0) = color[0];
			// // Green: increasing from left to right
			// image(x, y, 1) = color[1];
			// // Blue: in lower-left and upper-right corners
			// image(x, y, 2) = color[2];
			#elif !defined ANTI_ALIASING
			image(x, y, 0) = color[0];
			// Green: increasing from left to right
			image(x, y, 1) = color[1];
			// Blue: in lower-left and upper-right corners
			image(x, y, 2) = color[2];
			#endif
		}
		#ifdef ANTI_ALIASING
		twoRows.push_back(colorGrid);
		if (twoRows.size() == 2) {
			for (int xt = 0; xt < w; xt+=2) {
				glm::vec3 colorPixel = getAverageAround(xt, twoRows);
				for (int i = 0; i < 3; i++) {
					// std::cout << xt << " " << y << " ";
					image(xt/2,y/2-1,i) = colorPixel[i];
				}
			}
			delete [] twoRows[0];
			delete [] twoRows[1];
			twoRows.pop_front();
			twoRows.pop_front();
		}
		#endif
	}

	delete flattenedRoot;

	for (int i = 0; i < gridSize; i++) {
		delete cells[i];
	}
	delete [] cells;
}

#pragma once

#include "film.h"
#include "linagl.h"
#include "ray.h"
#include "intersection.h"

class Camera {
public:
	Camera(const std::shared_ptr<Film> &pFilm) {
		film = pFilm;
	}
	virtual Ray GenerateRay(int pixelX, int pixelY, Vec u, real offset = 0.0) const = 0;
	virtual Vec We(const Ray &ray) const = 0;
	virtual Vec Sample_Wi(const Intersection &isect, real *pdfW, Vec *wi, Vec u) const = 0;
	virtual real PdfPos() const = 0;
	virtual real PdfDir(const Ray &cameraRay) const = 0;
	virtual std::shared_ptr<Film> GetFilm() const { return film; }
protected:
	std::shared_ptr<Film> film;
};

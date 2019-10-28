#ifndef _TRIANGULATION_COST_H_
#define _TRIANGULATION_COST_H_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

typedef std::vector<std::vector<std::array<double, 2>>> CorrPtsSet;
typedef std::vector<std::array<double, 3>> Pts3dSet;

struct CameraParameters {
	int index;
	double ax[3];
	double trans[3];
	double fx, fy;
	double cx, cy;
	double k1, k2, k3;
	double p1, p2;

	CameraParameters(int index_, double ax1_, double ax2_, double ax3_, double t1_, double t2_, double t3_, double fx_, double fy_, double cx_, double cy_, double k1_, double k2_, double p1_, double p2_, double k3_)
		:index(index_), ax{ ax1_, ax2_, ax3_ }, trans{ t1_, t2_, t3_ }, fx(fx_), fy(fy_), cx(cx_), cy(cy_), k1(k1_), k2(k2_), k3(k3_), p1(p1_), p2(p2_) {
		data_array = new double[15];
		data_array[0] = ax1_;
		data_array[1] = ax2_;
		data_array[2] = ax3_;
		data_array[3] = t1_;
		data_array[4] = t2_;
		data_array[5] = t3_;
		data_array[6] = fx_;
		data_array[7] = fy_;
		data_array[8] = cx_;
		data_array[9] = cy_;
		data_array[10] = k1_;
		data_array[11] = k2_;
		data_array[12] = p1_;
		data_array[13] = p2_;
		data_array[14] = k3_;
	}
	CameraParameters() {}
	~CameraParameters() {
		delete[] data_array;
	}
	double* GetDataArray() {
		return data_array;
	}

	void Print() {
		printf("Camera[%d] -------\nax: (%.4f, %.4f, %.4f), t: (%.4f, %.4f, %.4f),\nf: (%.4f, %.4f), c: (%.4f, %.4f), k: (%.4f, %.4f, %.4f), p: (%.4f, %.4f)\n\n",
			index,
			ax[0], ax[1], ax[2],
			trans[0], trans[1], trans[2],
			fx, fy,
			cx, cy,
			k1, k2, k3,
			p1, p2);
	}

	int static loadCameraParameters(const char* xml_path, CameraParameters** cam_params, size_t numCams) {
		FILE* camparam_fptr = fopen(xml_path, "r");
		if (camparam_fptr == NULL) {
			std::cerr << "ERROR: unable to open file: " << xml_path << "\n";
			std::getchar();
			return 0;
		}

		for (int cam_idx = 0; cam_idx < numCams; cam_idx++) {
			double ax1, ax2, ax3, t1, t2, t3, fx, fy, c1, c2, k1, k2, p1, p2, k3;
			int cam_idx_confirm;
			fscanf(camparam_fptr, "%d", &cam_idx_confirm);
			fscanf(camparam_fptr, "%lf", &ax1);
			fscanf(camparam_fptr, "%lf", &ax2);
			fscanf(camparam_fptr, "%lf", &ax3);

			fscanf(camparam_fptr, "%lf", &t1);
			fscanf(camparam_fptr, "%lf", &t2);
			fscanf(camparam_fptr, "%lf", &t3);

			fscanf(camparam_fptr, "%lf", &fx);
			fscanf(camparam_fptr, "%lf", &fy);

			fscanf(camparam_fptr, "%lf", &c1);
			fscanf(camparam_fptr, "%lf", &c2);

			fscanf(camparam_fptr, "%lf", &k1);
			fscanf(camparam_fptr, "%lf", &k2);
			fscanf(camparam_fptr, "%lf", &p1);
			fscanf(camparam_fptr, "%lf", &p2);
			fscanf(camparam_fptr, "%lf", &k3);

			cam_params[cam_idx] = new CameraParameters(cam_idx_confirm, ax1, ax2, ax3, t1, t2, t3, fx, fy, c1, c2, k1, k2, p1, p2, k3);
		}
		//std::cout << "Done parsing: " << xml_path << std::endl;
		return 1;
	}

private:
	double* data_array;
};

struct ReprojectionErrorNoUndist {

	ReprojectionErrorNoUndist(double observed_xi, double observed_yi) 
		: observed_x(observed_xi), observed_y(observed_yi) 
	{
	}

	template<typename T>
	bool operator()(const T* const world_point, T* residuals) const {
		const T ang_axis[3] = { T(cp->ax[0]), T(cp->ax[1]),T(cp->ax[2]) };
		const T trans[3] = { T(cp->trans[0]),T(cp->trans[1]),T(cp->trans[2]) };
		const T f[2] = { T(cp->fx), T(cp->fy) };
		const T c[2] = { T(cp->cx), T(cp->cy) };

		T cam_point[3];

		// angle-axis rotation
		ceres::AngleAxisRotatePoint(ang_axis, world_point, cam_point);

		// translation
		cam_point[0] += trans[0];
		cam_point[1] += trans[1];
		cam_point[2] += trans[2];

		// center of distortion
		T xp = cam_point[0] / cam_point[2];
		T yp = cam_point[1] / cam_point[2];

		// projected point position
		T predicted_x = f[0] * xp + c[0];
		T predicted_y = f[1] * yp + c[1];

		// error
		T dx = predicted_x - observed_x;
		T dy = predicted_y - observed_y;
		// output
		residuals[0] = dx;
		residuals[1] = dy;

		return true;
	}

	static ceres::CostFunction* Create(double observed_x, double observed_y, CameraParameters* cp_in) {
		ReprojectionErrorNoUndist* re = new ReprojectionErrorNoUndist(observed_x, observed_y);
		re->cp = cp_in;
		return (new ceres::AutoDiffCostFunction<ReprojectionErrorNoUndist, 2, 3>(re));
	}

	double observed_x, observed_y; // size 8
	CameraParameters* cp = NULL;
};

struct ReprojectionErrorUndist {
	double observed_x, observed_y; // size 8
	CameraParameters* cp = NULL;
	bool k3Off = true;

	ReprojectionErrorUndist(double observed_xi, double observed_yi, bool k3OFF = true) : observed_x(observed_xi), observed_y(observed_yi), k3Off(k3OFF) {
	}

	template<typename T>
	bool operator()(const T* const world_point, T* residuals) const {
		const T ang_axis[3] = { T(cp->ax[0]), T(cp->ax[1]),T(cp->ax[2]) };
		const T trans[3] = { T(cp->trans[0]),T(cp->trans[1]),T(cp->trans[2]) };
		const T f[2] = { T(cp->fx), T(cp->fy) };
		const T k[3] = { T(cp->k1), T(cp->k2) , T(cp->k3) };
		const T p[2] = { T(cp->p1), T(cp->p2) };
		const T c[2] = { T(cp->cx), T(cp->cy) };

		T cam_point[3];
		// angle-axis rotation
		ceres::AngleAxisRotatePoint(ang_axis, world_point, cam_point);

		// translation
		cam_point[0] += trans[0];
		cam_point[1] += trans[1];
		cam_point[2] += trans[2];

		// center of distortion
		T xp = cam_point[0] / cam_point[2];
		T yp = cam_point[1] / cam_point[2];

		// distortions
		T r2 = xp * xp + yp * yp;
		T radial_dist(0.);
		if (k3Off) {
			radial_dist = 1.0 + r2 * (k[0] + k[1] * r2);
		}
		else {
			radial_dist = 1.0 + r2 * (k[0] + k[1] * r2 + k[2] * r2 * r2);
		}
		T tangential_dist_x = T(2.0) * p[0] * xp * yp + p[1] * (r2 + T(2.0) * xp * xp);
		T tangential_dist_y = p[0] * (r2 + T(2.0) * yp * yp) + T(2.0) * p[1] * xp * yp;

		xp = xp * radial_dist + tangential_dist_x;
		yp = yp * radial_dist + tangential_dist_y;

		// projected point position
		T predicted_x = f[0] * xp + c[0];
		T predicted_y = f[1] * yp + c[1];

		// error
		T dx = predicted_x - observed_x;
		T dy = predicted_y - observed_y;
		// output
		residuals[0] = dx;
		residuals[1] = dy;
		return true;
	}
	static ceres::CostFunction* Create(double observed_x, double observed_y, CameraParameters* cp_in, bool k3Off=true) {
		ReprojectionErrorUndist* re = new ReprojectionErrorUndist(observed_x, observed_y, k3Off);
		re->cp = cp_in;
		return (new ceres::AutoDiffCostFunction<ReprojectionErrorUndist, 2, 3>(re));
	}
};

#endif